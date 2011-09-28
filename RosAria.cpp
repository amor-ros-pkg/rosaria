#include <stdio.h>
#include <math.h>
#include <Aria.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h>     //for sonar data
#include "nav_msgs/Odometry.h"
#include "ROSARIA/BumperState.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"	//for tf::getPrefixParam
#include "tf/transform_datatypes.h"

#include <sstream>

class ActionPause : public ArAction
{
public:
  // constructor, sets myMaxSpeed and myStopDistance
  ActionPause(double time);
  // destructor. does not need to do anything
  virtual ~ActionPause(void) {};
  // called by the action resolver to obtain this action's requested behavior
  virtual ArActionDesired *fire(ArActionDesired currentDesired);
protected:
  double pauseTime;
  ArActionDesired myDesired;
};

ActionPause::ActionPause(double time) :
  ArAction("Pause")
{
  pauseTime = time;
  myDesired.reset();
}

ArActionDesired *ActionPause::fire(ArActionDesired currentDesired)
{
  ros::Duration(pauseTime).sleep();
  return &myDesired;
}


class RosAriaNode
{
  public:
    RosAriaNode(ros::NodeHandle n);
    virtual ~RosAriaNode();
    
  public:
    int Setup();
    void cmdvel_cb( const geometry_msgs::TwistConstPtr &);
    void spin();
    void publish();
    void sonarConnectCb();

  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher bumpers_pub;
    ros::Publisher sonar_pub;
    ros::Subscriber cmdvel_sub;

    ros::Time veltime;

    std::string serial_port;

    ArSimpleConnector *conn;
    ArRobot *robot;
    ActionPause *pause;
    nav_msgs::Odometry position;
    ROSARIA::BumperState bumpers;
    ArPose pos;
    ArFunctorC<RosAriaNode> myPublishCB;

    //for resolving tf names.
    std::string tf_prefix;
    std::string frame_id_odom;
    std::string frame_id_bumper;
    std::string frame_id_sonar;

    //Sonar support
    bool use_sonar;		// enable and publish sonars
};

void RosAriaNode::sonarConnectCb()
{
  if (sonar_pub.getNumSubscribers() == 0)
  {
    robot->disableSonar();
    use_sonar = false;
  }
  else
  {
    robot->enableSonar();
    use_sonar = true;
  }
}

RosAriaNode::RosAriaNode(ros::NodeHandle nh) : 
  myPublishCB(this, &RosAriaNode::publish), use_sonar(false)
{
  // read in config options
  n = nh;

  // !!! port !!!
  n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
  ROS_INFO( "using serial port: [%s]", serial_port.c_str() );

  /*
   * Figure out what frame_id's to use. if a tf_prefix param is specified,
   * it will be added to the beginning of the frame_ids.
   *
   * e.g. rosrun ... _tf_prefix:=MyRobot (or equivalently using <param>s in
   * roslaunch files)
   * will result in the frame_ids being set to /MyRobot/odometry_frame etc,
   * rather than /odometry_frame. This is useful for Multi Robot Systems.
   * See ROS Wiki for further details.
   */
  tf_prefix = tf::getPrefixParam(n);
  frame_id_odom = tf::resolve(tf_prefix, "odometry_frame");
  frame_id_bumper = tf::resolve(tf_prefix, "bumpers_frame");
  frame_id_sonar = tf::resolve(tf_prefix, "sonar_frame");

  // advertise services
  pose_pub = n.advertise<nav_msgs::Odometry>("pose",1000);
  bumpers_pub = n.advertise<ROSARIA::BumperState>("bumper_state",1000);
  sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar", 50, boost::bind(&RosAriaNode::sonarConnectCb, this),
    boost::bind(&RosAriaNode::sonarConnectCb, this));
  
  // subscribe to services
  cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function < void(const geometry_msgs::TwistConstPtr&)>) boost::bind( &RosAriaNode::cmdvel_cb, this, _1 ));
  
  veltime = ros::Time::now();
}

RosAriaNode::~RosAriaNode()
{
  //disable motors and sonar.
  robot->disableMotors();
  robot->disableSonar();

  robot->stopRunning();
  robot->waitForRunExit();
  Aria::shutdown();
}

int RosAriaNode::Setup()
{
  ArArgumentBuilder *args;
  args = new ArArgumentBuilder();
  args->add("-rp"); //pass robot's serial port to Aria
  args->add(serial_port.c_str());
  args->add("-rlpr"); //log received packets
  args->add("-rlps"); //log sent packets
  args->add("-rlvr"); //log received velocities
  conn = new ArSimpleConnector(args);

  robot = new ArRobot();
  pause = new ActionPause(4e-3);

  ArLog::init(ArLog::File, ArLog::Verbose, "aria.log", true);

  // Connect to the robot
  if (!conn->connectRobot(robot)) {
    ArLog::log(ArLog::Terse, "rotate: Could not connect to robot! Exiting.");
    return 1;
  }

  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->disableSonar();

  robot->addSensorInterpTask("PublishingTask", 100, &myPublishCB);
//  robot->addAction(pause, 10);
  robot->runAsync(true);

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  return 0;
}

void RosAriaNode::spin()
{
  ros::spin();
}

void RosAriaNode::publish()
{
//  robot->lock();
  pos = robot->getPose();
//  robot->unlock();
  tf::poseTFToMsg(tf::Pose(tf::Quaternion(pos.getTh()*M_PI/180, 0, 0), tf::Vector3(pos.getX()/1000, pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel()/1000; //Aria returns velocity in mm/s.
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  
  position.header.frame_id = frame_id_odom;
  position.header.stamp = ros::Time::now();
  pose_pub.publish(position);
  ROS_INFO("rcv: %f %f %f", position.header.stamp.toSec(), (double) position.twist.twist.linear.x, (double) position.twist.twist.angular.z);

  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = frame_id_bumper;
  bumpers.header.stamp = ros::Time::now();

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
    bumper_info << " " << (front_bumpers & (1 << (i+1)));
  }
  ROS_INFO("Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  ROS_INFO("Rear bumpers:%s", bumper_info.str().c_str());
  
  bumpers_pub.publish(bumpers);

  /*
   * Publish sonar information, if necessary.
   */
  if (use_sonar) {
    sensor_msgs::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = position.header.stamp;	//copy time.
    //sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;
    
    std::stringstream sonar_debug_info;
    sonar_debug_info << "Sonar readings: ";
    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
	      ROS_WARN("Did not receive a sonar reading.");
	      continue;
      }
      
      //getRange() will return an integer between 0 and 5000 (5m)
      sonar_debug_info << reading->getRange() << " ";

      /*
       * local (x,y). Appears to be from the centre of the robot, since values may
       * exceed 5000. This is good, since it means we only need 1 transform.
       * x & y seem to be swapped though, i.e. if the robot is driving north
       * x is north/south and y is east/west.
       */
      //ArPose sensor = reading->getSensorPosition();	//position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX() 
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", " 
      //                  << sensor.getY() << ") ;; " ;
      
      //add to cloud
      geometry_msgs::Point32 p;
      p.x = reading->getLocalX() / 1000.0;
      p.y = reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    ROS_DEBUG_STREAM(sonar_debug_info.str());
    
    sonar_pub.publish(cloud);
  }

  ros::Duration(1e-3).sleep();
}

void
RosAriaNode::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
{
  veltime = ros::Time::now();
  ROS_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );

//  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
//  robot->unlock();
  ROS_INFO("snd: %f %f %f", veltime.toSec(), (double) msg->linear.x, (double) msg->angular.z);
}


int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria");
  ros::NodeHandle n(std::string("~"));
  Aria::init();

  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    printf( "setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  printf( "\nQuitting... \n" );
  return 0;
  
}
