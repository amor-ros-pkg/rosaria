#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h>  //for sonar data
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> // can optionally publish sonar as new type pointcloud2
#include "nav_msgs/Odometry.h"
#include "rosaria/BumperState.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include <rosaria/RosAriaConfig.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

#include <sstream>


const boost::array<double, 36ul> ODOM_POSE_COVARIANCE   = {{1e-3, 0, 0, 0, 0, 0, //Covariance while the robot is moving
                                                            0, 1e-3, 0, 0, 0, 0,
                                                            0, 0, 1e6, 0, 0, 0,
                                                            0, 0, 0, 1e6, 0, 0,
                                                            0, 0, 0, 0, 1e6, 0,
                                                            0, 0, 0, 0, 0, 1e3}};

const boost::array<double, 36ul> ODOM_POSE_COVARIANCE_NM= {{1e-9, 0, 0, 0, 0, 0,  //Covariance while the robot is not moving
                                                            0, 1e-3, 0, 0, 0, 0,
                                                            0, 0, 1e6, 0, 0, 0,
                                                            0, 0, 0, 1e6, 0, 0,
                                                            0, 0, 0, 0, 1e6, 0,
                                                            0, 0, 0, 0, 0, 1e-9}};

const boost::array<double, 36ul>  ODOM_TWIST_COVARIANCE = {{1e-3, 0, 0, 0, 0, 0,
                                                            0, 1e-3, 0, 0, 0, 0,
                                                            0, 0, 1e6, 0, 0, 0,
                                                            0, 0, 0, 1e6, 0, 0,
                                                            0, 0, 0, 0, 1e6, 0,
                                                            0, 0, 0, 0, 0, 1e3}};
const boost::array<double, 36ul> ODOM_TWIST_COVARIANCE_NM= {{1e-9, 0, 0, 0, 0, 0,
                                                            0, 1e-3, 0, 0, 0, 0,
                                                            0, 0, 1e6, 0, 0, 0,
                                                            0, 0, 0, 1e6, 0, 0,
                                                            0, 0, 0, 0, 1e6, 0,
                                                            0, 0, 0, 0, 0, 1e-9}};


// Node that interfaces between ROS and mobile robot base features via ARIA library. 
//
// RosAria uses the roscpp client library, see http://www.ros.org/wiki/roscpp for
// information, tutorials and documentation.
class RosAriaNode
{
  public:
    RosAriaNode(ros::NodeHandle n);
    virtual ~RosAriaNode();
    


  public:
    int Setup();
    void cmdvel_cb( const geometry_msgs::TwistConstPtr &);
    //void cmd_enable_motors_cb();
    //void cmd_disable_motors_cb();
    void spin();
    void publish();
    void sonarConnectCb();
    void dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level);
    void readParameters();

  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher bumpers_pub;
    ros::Publisher sonar_pub;
    ros::Publisher sonar_pointcloud2_pub;
    ros::Publisher voltage_pub;

    ros::Publisher recharge_state_pub;
    std_msgs::Int8 recharge_state;

    ros::Publisher state_of_charge_pub;

    ros::Publisher motors_state_pub;
    std_msgs::Bool motors_state;
    bool published_motors_state;

    ros::Subscriber cmdvel_sub;

    ros::ServiceServer enable_srv;
    ros::ServiceServer disable_srv;
    bool enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    ros::Time veltime;

    std::string serial_port;
    int serial_baud;

    ArRobotConnector *conn;
    ArRobot *robot;
    nav_msgs::Odometry position;
    rosaria::BumperState bumpers;
    ArPose pos;
    ArFunctorC<RosAriaNode> myPublishCB;
    //ArRobot::ChargeState batteryCharge;

    //for odom->base_link transform
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    //for resolving tf names.
    std::string tf_prefix;
    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_bumper;
    std::string frame_id_sonar;

    // flag indicating whether sonar was enabled or disabled on the robot
    bool sonar_enabled; 

    // enable and publish sonar topics. set to true when first subscriber connects, set to false when last subscriber disconnects. 
    bool publish_sonar; 
    bool publish_sonar_pointcloud2;

    // Debug Aria
    bool debug_aria;
    std::string aria_log_filename;
    
    // Robot Parameters
    int TicksMM, DriftFactor, RevCount;  // Odometry Calibration Settings
    
    // dynamic_reconfigure
    dynamic_reconfigure::Server<rosaria::RosAriaConfig> *dynamic_reconfigure_server;
};

void RosAriaNode::readParameters()
{
  // Robot Parameters  
  robot->lock();
  ros::NodeHandle n_("~");
  if (n_.hasParam("TicksMM"))
  {
    n_.getParam( "TicksMM", TicksMM);
    ROS_INFO("Setting TicksMM from ROS Parameter: %d", TicksMM);
    robot->comInt(93, TicksMM);
  }
  else
  {
    TicksMM = robot->getOrigRobotConfig()->getTicksMM();
    n_.setParam( "TicksMM", TicksMM);
    ROS_INFO("Setting TicksMM from robot controller stored configuration: %d", TicksMM);
  }
  
  if (n_.hasParam("DriftFactor"))
  {
    n_.getParam( "DriftFactor", DriftFactor);
    ROS_INFO("Setting DriftFactor from ROS Parameter: %d", DriftFactor);
    robot->comInt(89, DriftFactor);
  }
  else
  {
    DriftFactor = robot->getOrigRobotConfig()->getDriftFactor();
    n_.setParam( "DriftFactor", DriftFactor);
    ROS_INFO("Setting DriftFactor from robot controller stored configuration: %d", DriftFactor);
  }
  
  if (n_.hasParam("RevCount"))
  {
    n_.getParam( "RevCount", RevCount);
    ROS_INFO("Setting RevCount from ROS Parameter: %d", RevCount);
    robot->comInt(88, RevCount);
  }
  else
  {
    RevCount = robot->getOrigRobotConfig()->getRevCount();
    n_.setParam( "RevCount", RevCount);
    ROS_INFO("Setting RevCount from robot controller stored configuration: %d", RevCount);
  }
  robot->unlock();
}

void RosAriaNode::dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level)
{
  //
  // Odometry Settings
  //
  robot->lock();
  if(TicksMM != config.TicksMM and config.TicksMM > 0)
  {
    ROS_INFO("Setting TicksMM from Dynamic Reconfigure: %d -> %d ", TicksMM, config.TicksMM);
    TicksMM = config.TicksMM;
    robot->comInt(93, TicksMM);
  }
  
  if(DriftFactor != config.DriftFactor)
  {
    ROS_INFO("Setting DriftFactor from Dynamic Reconfigure: %d -> %d ", DriftFactor, config.DriftFactor);
    DriftFactor = config.DriftFactor;
    robot->comInt(89, DriftFactor);
  }
  
  if(RevCount != config.RevCount and config.RevCount > 0)
  {
    ROS_INFO("Setting RevCount from Dynamic Reconfigure: %d -> %d ", RevCount, config.RevCount);
    RevCount = config.RevCount;
    robot->comInt(88, RevCount);
  }
  
  //
  // Acceleration Parameters
  //
  int value;
  value = config.trans_accel * 1000;
  if(value != robot->getTransAccel() and value > 0)
  {
    ROS_INFO("Setting TransAccel from Dynamic Reconfigure: %d", value);
    robot->setTransAccel(value);
  }
  
  value = config.trans_decel * 1000;
  if(value != robot->getTransDecel() and value > 0)
  {
    ROS_INFO("Setting TransDecel from Dynamic Reconfigure: %d", value);
    robot->setTransDecel(value);
  } 
  
  value = config.lat_accel * 1000;
  if(value != robot->getLatAccel() and value > 0)
  {
    ROS_INFO("Setting LatAccel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatAccel() > 0 )
      robot->setLatAccel(value);
  }
  
  value = config.lat_decel * 1000;
  if(value != robot->getLatDecel() and value > 0)
  {
    ROS_INFO("Setting LatDecel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatDecel() > 0 )
      robot->setLatDecel(value);
  }
  
  value = config.rot_accel * 180/M_PI;
  if(value != robot->getRotAccel() and value > 0)
  {
    ROS_INFO("Setting RotAccel from Dynamic Reconfigure: %d", value);
    robot->setRotAccel(value);
  }
  
  value = config.rot_decel * 180/M_PI;
  if(value != robot->getRotDecel() and value > 0)
  {
    ROS_INFO("Setting RotDecel from Dynamic Reconfigure: %d", value);
    robot->setRotDecel(value);
  } 
  robot->unlock();
}

void RosAriaNode::sonarConnectCb()
{
  publish_sonar = (sonar_pub.getNumSubscribers() > 0);
  publish_sonar_pointcloud2 = (sonar_pointcloud2_pub.getNumSubscribers() > 0);
  robot->lock();
  if (publish_sonar || publish_sonar_pointcloud2)
  {
    robot->enableSonar();
    sonar_enabled = false;
  }
  else if(!publish_sonar && !publish_sonar_pointcloud2)
  {
    robot->disableSonar();
    sonar_enabled = true;
  }
  robot->unlock();
}

RosAriaNode::RosAriaNode(ros::NodeHandle nh) : 
  myPublishCB(this, &RosAriaNode::publish), serial_port(""), serial_baud(0),
  sonar_enabled(false), publish_sonar(false), publish_sonar_pointcloud2(false)
{
  // read in runtime parameters
  n = nh;

  // port and baud
  n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
  ROS_INFO( "RosAria: using port: [%s]", serial_port.c_str() );

  n.param("baud", serial_baud, 0);
  if(serial_baud != 0)
  ROS_INFO("RosAria: using serial port baud rate %d", serial_baud);

  // handle debugging more elegantly
  n.param( "debug_aria", debug_aria, false ); // default not to debug
  n.param( "aria_log_filename", aria_log_filename, std::string("Aria.log") );

  // Figure out what frame_id's to use. if a tf_prefix param is specified,
  // it will be added to the beginning of the frame_ids.
  //
  // e.g. rosrun ... _tf_prefix:=MyRobot (or equivalently using <param>s in
  // roslaunch files)
  // will result in the frame_ids being set to /MyRobot/odom etc,
  // rather than /odom. This is useful for Multi Robot Systems.
  // See ROS Wiki for further details.
  tf_prefix = tf::getPrefixParam(n);
  frame_id_odom = tf::resolve(tf_prefix, "odom");
  frame_id_base_link = tf::resolve(tf_prefix, "base_link");
  frame_id_bumper = tf::resolve(tf_prefix, "bumpers_frame");
  frame_id_sonar = tf::resolve(tf_prefix, "sonar_frame");

  // advertise services for data topics
  // second argument to advertise() is queue size.
  // other argmuments (optional) are callbacks, or a boolean "latch" flag (whether to send current data to new
  // subscribers when they subscribe).
  // See ros::NodeHandle API docs.
  pose_pub = n.advertise<nav_msgs::Odometry>("pose",1000);
  bumpers_pub = n.advertise<rosaria::BumperState>("bumper_state",1000);
  sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar", 50, 
      boost::bind(&RosAriaNode::sonarConnectCb, this),
      boost::bind(&RosAriaNode::sonarConnectCb, this));
  sonar_pointcloud2_pub = n.advertise<sensor_msgs::PointCloud2>("sonar_pointcloud2", 50,
      boost::bind(&RosAriaNode::sonarConnectCb, this),
      boost::bind(&RosAriaNode::sonarConnectCb, this));

  voltage_pub = n.advertise<std_msgs::Float64>("battery_voltage", 1000);
  recharge_state_pub = n.advertise<std_msgs::Int8>("battery_recharge_state", 5, true /*latch*/ );
  recharge_state.data = -2;
  state_of_charge_pub = n.advertise<std_msgs::Float32>("battery_state_of_charge", 100);

  motors_state_pub = n.advertise<std_msgs::Bool>("motors_state", 5, true /*latch*/ );
  motors_state.data = false;
  published_motors_state = false;
  
  // subscribe to services
  cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
    boost::bind(&RosAriaNode::cmdvel_cb, this, _1 ));

  // advertise enable/disable services
  enable_srv = n.advertiseService("enable_motors", &RosAriaNode::enable_motors_cb, this);
  disable_srv = n.advertiseService("disable_motors", &RosAriaNode::disable_motors_cb, this);
  
  veltime = ros::Time::now();
}

RosAriaNode::~RosAriaNode()
{
  // disable motors and sonar.
  robot->disableMotors();
  robot->disableSonar();

  robot->stopRunning();
  robot->waitForRunExit();
  Aria::shutdown();
}

int RosAriaNode::Setup()
{
  // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
  // called once per instance, and these objects need to persist until the process terminates.

  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

  // Now add any parameters given via ros params (see RosAriaNode constructor):

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  // for wireless serial connection. Otherwise, interpret it as a serial port name.
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-robotPort"); // pass robot's serial port to Aria
    args->add(serial_port.c_str());
  }

  // if a baud rate was specified in baud parameter
  if(serial_baud != 0)
  {
    args->add("-robotBaud");
    char tmp[100];
    snprintf(tmp, 100, "%d", serial_baud);
    args->add(tmp);
  }
  
  if( debug_aria )
  {
    // turn on all ARIA debugging
    args->add("-robotLogPacketsReceived"); // log received packets
    args->add("-robotLogPacketsSent"); // log sent packets
    args->add("-robotLogVelocitiesReceived"); // log received velocities
    args->add("-robotLogMovementSent");
    args->add("-robotLogMovementReceived");
    ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
  }


  // Connect to the robot
  conn = new ArRobotConnector(argparser, robot); // warning never freed
  if (!conn->connectRobot()) {
    ROS_ERROR("RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device.)");
    return 1;
  }

  // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
  if(!Aria::parseArgs())
  {
    ROS_ERROR("RosAria: ARIA error parsing ARIA startup parameters!");
    return 1;
  }

  readParameters();

  // Start dynamic_reconfigure server
  dynamic_reconfigure_server = new dynamic_reconfigure::Server<rosaria::RosAriaConfig>;
  
  // Setup Parameter Minimums
  rosaria::RosAriaConfig dynConf_min;
  dynConf_min.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000;
  dynConf_min.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000;
  // TODO: Fix rqt dynamic_reconfigure gui to handle empty intervals
  // Until then, set unit length interval.
  dynConf_min.lat_accel = ((robot->getAbsoluteMaxLatAccel() > 0.0) ? robot->getAbsoluteMaxLatAccel() : 0.1) / 1000;
  dynConf_min.lat_decel = ((robot->getAbsoluteMaxLatDecel() > 0.0) ? robot->getAbsoluteMaxLatDecel() : 0.1) / 1000;
  dynConf_min.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180;
  dynConf_min.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180;
  
  // I'm setting these upper bounds relitivly arbitrarily, feel free to increase them.
  dynConf_min.TicksMM     = 10;
  dynConf_min.DriftFactor = -200;
  dynConf_min.RevCount    = -32760;
  
  dynamic_reconfigure_server->setConfigMin(dynConf_min);
  
  
  rosaria::RosAriaConfig dynConf_max;
  dynConf_max.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000;
  dynConf_max.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000;
  // TODO: Fix rqt dynamic_reconfigure gui to handle empty intervals
  // Until then, set unit length interval.
  dynConf_max.lat_accel = ((robot->getAbsoluteMaxLatAccel() > 0.0) ? robot->getAbsoluteMaxLatAccel() : 0.1) / 1000;
  dynConf_max.lat_decel = ((robot->getAbsoluteMaxLatDecel() > 0.0) ? robot->getAbsoluteMaxLatDecel() : 0.1) / 1000;
  dynConf_max.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180;
  dynConf_max.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180;
  
  // I'm setting these upper bounds relitivly arbitrarily, feel free to increase them.
  dynConf_max.TicksMM     = 200;
  dynConf_max.DriftFactor = 200;
  dynConf_max.RevCount    = 32760;
  
  dynamic_reconfigure_server->setConfigMax(dynConf_max);
  
  
  rosaria::RosAriaConfig dynConf_default;
  dynConf_default.trans_accel = robot->getTransAccel() / 1000;
  dynConf_default.trans_decel = robot->getTransDecel() / 1000;
  dynConf_default.lat_accel   = robot->getLatAccel() / 1000;
  dynConf_default.lat_decel   = robot->getLatDecel() / 1000;
  dynConf_default.rot_accel   = robot->getRotAccel() * M_PI/180;
  dynConf_default.rot_decel   = robot->getRotDecel() * M_PI/180;

  dynConf_default.TicksMM     = TicksMM;
  dynConf_default.DriftFactor = DriftFactor;
  dynConf_default.RevCount    = RevCount;
  
  dynamic_reconfigure_server->setConfigDefault(dynConf_max);
  
  dynamic_reconfigure_server->setCallback(boost::bind(&RosAriaNode::dynamic_reconfigureCB, this, _1, _2));

  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->disableSonar();

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  // Run ArRobot background processing thread
  robot->runAsync(true);

  return 0;
}

void RosAriaNode::spin()
{
  ros::spin();
}

void RosAriaNode::publish()
{
  // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
  pos = robot->getPose();
  tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTh()*M_PI/180), tf::Vector3(pos.getX()/1000,
    pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel()/1000; //Aria returns velocity in mm/s.
  position.twist.twist.linear.y = robot->getLatVel()/1000.0;
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  
  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = ros::Time::now();

  if(position.twist.twist.linear.x == 0 && 
    position.twist.twist.linear.y == 0 && 
    position.twist.twist.angular.z ==0 )
  {
    position.pose.covariance = ODOM_POSE_COVARIANCE_NM;
    position.twist.covariance = ODOM_TWIST_COVARIANCE_NM;
  }
  else
  {
    position.pose.covariance = ODOM_POSE_COVARIANCE;
    position.twist.covariance = ODOM_TWIST_COVARIANCE;
  }

  pose_pub.publish(position);

  ROS_DEBUG("RosAria: publish: (time %f) pose x: %f, y: %f, angle: %f; linear vel x: %f, y: %f; angular vel z: %f", 
    position.header.stamp.toSec(), 
    (double)position.pose.pose.position.x,
    (double)position.pose.pose.position.y,
    (double)position.pose.pose.orientation.w,
    (double) position.twist.twist.linear.x,
    (double) position.twist.twist.linear.y,
    (double) position.twist.twist.angular.z
  );


  // publishing transform odom->base_link
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;
  
  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.getTh()*M_PI/180);
  
  odom_broadcaster.sendTransform(odom_trans);
  
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
  ROS_DEBUG("RosAria: Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  ROS_DEBUG("RosAria: Rear bumpers:%s", bumper_info.str().c_str());
  
  bumpers_pub.publish(bumpers);

  //Publish battery information
  // TODO: Decide if BatteryVoltageNow (normalized to (0,12)V)  is a better option
  std_msgs::Float64 batteryVoltage;
  batteryVoltage.data = robot->getRealBatteryVoltageNow();
  voltage_pub.publish(batteryVoltage);

  if(robot->haveStateOfCharge())
  {
    std_msgs::Float32 soc;
    soc.data = robot->getStateOfCharge()/100.0;
    state_of_charge_pub.publish(soc);
  }

  // publish recharge state if changed
  char s = robot->getChargeState();
  if(s != recharge_state.data)
  {
    ROS_INFO("RosAria: publishing new recharge state %d.", s);
    recharge_state.data = s;
    recharge_state_pub.publish(recharge_state);
  }

  // publish motors state if changed
  bool e = robot->areMotorsEnabled();
  if(e != motors_state.data || !published_motors_state)
  {
	ROS_INFO("RosAria: publishing new motors state %d.", e);
	motors_state.data = e;
	motors_state_pub.publish(motors_state);
	published_motors_state = true;
  }

  // Publish sonar information, if enabled.
  if (publish_sonar || publish_sonar_pointcloud2)
  {
    sensor_msgs::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = position.header.stamp;	//copy time.
    // sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;
  

    std::stringstream sonar_debug_info; // Log debugging info
    sonar_debug_info << "Sonar readings: ";

    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
        ROS_WARN("RosAria: Did not receive a sonar reading.");
        continue;
      }
    
      // getRange() will return an integer between 0 and 5000 (5m)
      sonar_debug_info << reading->getRange() << " ";

      // local (x,y). Appears to be from the centre of the robot, since values may
      // exceed 5000. This is good, since it means we only need 1 transform.
      // x & y seem to be swapped though, i.e. if the robot is driving north
      // x is north/south and y is east/west.
      //
      //ArPose sensor = reading->getSensorPosition();  //position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX() 
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", " 
      //                  << sensor.getY() << ") ;; " ;
    
      //add sonar readings (robot-local coordinate frame) to cloud
      geometry_msgs::Point32 p;
      p.x = reading->getLocalX() / 1000.0;
      p.y = reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    ROS_DEBUG_STREAM(sonar_debug_info.str());
    
    // publish topic(s)

    if(publish_sonar_pointcloud2)
    {
      sensor_msgs::PointCloud2 cloud2;
      if(!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2))
      {
        ROS_WARN("Error converting sonar point cloud message to point_cloud2 type before publishing! Not publishing this time.");
      }
      else
      {
        sonar_pointcloud2_pub.publish(cloud2);
      }
    }

    if(publish_sonar)
    {
      sonar_pub.publish(cloud);
    }
  } // end if sonar_enabled
}

bool RosAriaNode::enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("RosAria: Enable motors request.");
    robot->lock();
    if(robot->isEStopPressed())
        ROS_WARN("RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
    robot->enableMotors();
    robot->unlock();
	// todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}

bool RosAriaNode::disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("RosAria: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
	// todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}

void
RosAriaNode::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
{
  veltime = ros::Time::now();
  ROS_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );

  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  if(robot->hasLatVel())
    robot->setLatVel(msg->linear.y*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
  robot->unlock();
  ROS_DEBUG("RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.toSec(),
    (double) msg->linear.x * 1e3, (double) msg->linear.y * 1.3, (double) msg->angular.z * 180/M_PI);
}


int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria");
  ros::NodeHandle n(std::string("~"));
  Aria::init();

  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  ROS_INFO( "RosAria: Quitting... \n" );
  return 0;
  
}
