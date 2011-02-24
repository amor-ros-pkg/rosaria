#include <stdio.h>
#include <math.h>
#include <Aria.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"

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

  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Subscriber cmdvel_sub;

    ros::Time veltime;

    std::string serial_port;

    ArSimpleConnector *conn;
    ArRobot *robot;
    ActionPause *pause;
    nav_msgs::Odometry position;
    ArPose pos;
    ArFunctorC<RosAriaNode> myPublishCB;
};

RosAriaNode::RosAriaNode(ros::NodeHandle nh) : 
  myPublishCB(this, &RosAriaNode::publish)
{
  // read in config options
  n = nh;

  // !!! port !!!
  n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
  ROS_INFO( "using serial port: [%s]", serial_port.c_str() );

  // advertise services
  pose_pub = n.advertise<nav_msgs::Odometry>("pose",1000);

  // subscribe to services
  cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function < void(const geometry_msgs::TwistConstPtr&)>) boost::bind( &RosAriaNode::cmdvel_cb, this, _1 ));
  
  veltime = ros::Time::now();
}

RosAriaNode::~RosAriaNode()
{
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

  // Enable the motors, disable sonar
  robot->enableMotors();
  robot->comInt(ArCommands::SONAR, 0);

  robot->addSensorInterpTask("PublishingTask", 100, &myPublishCB);
//  robot->addAction(pause, 10);
  robot->runAsync(true);

  return 0;
}

void RosAriaNode::spin()
{
  while( n.ok() )
  {
    ros::spinOnce();
  }
}

void RosAriaNode::publish()
{
//  robot->lock();
  pos = robot->getPose();
//  robot->unlock();
  tf::poseTFToMsg(tf::Pose(tf::Quaternion(pos.getTh()*M_PI/180, 0, 0), tf::Vector3(pos.getX()/1000, pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel();
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;

  position.header.stamp = ros::Time::now();
  pose_pub.publish(position);
  ROS_INFO("rcv: %f %f %f", position.header.stamp.toSec(), (double) position.twist.twist.linear.x, (double) position.twist.twist.angular.z);
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
