#include "mw_udp_communicator/udp_communicator.hpp"


UdpCommunicator::UdpCommunicator(ros::NodeHandle *nh) : nh_(*nh)
{
  ROS_INFO("Initialising node mw_udp_communicator.");
  initialiseVariables();
  initialiseParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseTimers();
}


void UdpCommunicator::initialiseTimers()
{
  tmr_udp_ = nh_.createTimer(ros::Duration(0.005), &UdpCommunicator::callbackUdp, this);
  ROS_INFO("UDP connection is established.");
  tmr_mon_ = nh_.createTimer(ros::Duration(0.05), &UdpCommunicator::callbackMon, this);
  ROS_INFO("Topic monitoring is alive.");
}


void UdpCommunicator::initialiseVariables()
{
  ROS_INFO("Initialising UDP parameters.");
  sock_ = 0;
  header_in_ = recvbuf_;
  data_in_ = &recvbuf_[2];
  header_out_ = sendbuf_;
  data_out_ = &sendbuf_[2];

  cmd_mon_.linear.x = 0.0;
  cmd_mon_.linear.y = 0.0;
  cmd_mon_.linear.z = 0.0;
  cmd_mon_.angular.x = 0.0;
  cmd_mon_.angular.y = 0.0;
  cmd_mon_.angular.z = 0.0;

  tst_mon_.linear.x = 0.0;
  tst_mon_.linear.y = 0.0;
  tst_mon_.linear.z = 0.0;
  tst_mon_.angular.x = 0.0;
  tst_mon_.angular.y = 0.0;
  tst_mon_.angular.z = 0.0;
}


void UdpCommunicator::initialiseSubscribers()
{
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 200, &UdpCommunicator::callbackOdom, this);
  sub_cmd_ = nh_.subscribe<geometry_msgs::Twist>("/middleware/cmd_vel", 200, &UdpCommunicator::callbackCmd, this);
}


void UdpCommunicator::initialisePublishers()
{
  pub_mon_cmd_ = nh_.advertise<geometry_msgs::Twist>("/middleware/monitoring_vel_cmd", 200);
  pub_mon_tst_ = nh_.advertise<geometry_msgs::Twist>("/middleware/monitoring_twist", 200);
}


void UdpCommunicator::callbackOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  tst_mon_.linear.x = msg->twist.twist.linear.x;
  tst_mon_.linear.y = msg->twist.twist.linear.y;
  tst_mon_.linear.z = msg->twist.twist.linear.z;
  tst_mon_.angular.x = msg->twist.twist.angular.x;
  tst_mon_.angular.y = msg->twist.twist.angular.y;
  tst_mon_.angular.z = msg->twist.twist.angular.z;
}


void UdpCommunicator::callbackCmd(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_mon_.linear.x = msg->linear.x;
  cmd_mon_.angular.z = msg->angular.z;
}


void UdpCommunicator::initialiseParams()
{
  std::string rtc_ip;
  std::string nuc_ip;
  std::string rtc_port;
  std::string nuc_port;
  std::string cmd_type_tmp;

  if (!nh_.getParam("rtc_ip", rtc_ip))
  {
    ros::param::param<std::string>("rtc_ip", rtc_ip, "10.10.10.3");
  }
  if (!nh_.getParam("nuc_ip", nuc_ip))
  {
    ros::param::param<std::string>("nuc_ip", nuc_ip, "10.10.10.100");
  }
  if (!nh_.getParam("rtc_port", rtc_port))
  {
    ros::param::param<std::string>("rtc_port", rtc_port, "25000");
  }
  if (!nh_.getParam("nuc_port", nuc_port))
  {
    ros::param::param<std::string>("nuc_port", nuc_port, "25001");
  }
  if (!nh_.getParam("cmd_type", cmd_type_tmp))
  {
    ros::param::param<std::string>("cmd_type", cmd_type_tmp, "0");
  }

  cmd_type_ = std::stoi(cmd_type_tmp);
  ROS_INFO("Parameter set rtc_ip -> %s", rtc_ip.c_str());
  ROS_INFO("Parameter set nuc_ip -> %s", nuc_ip.c_str());
  ROS_INFO("Parameter set rtc_port -> %s", rtc_port.c_str());
  ROS_INFO("Parameter set nuc_port -> %s", nuc_port.c_str());
  sock_ = udp_init_client(rtc_ip.c_str(), rtc_port.c_str(), nuc_ip.c_str(), nuc_port.c_str());
}


void UdpCommunicator::dummyControl(const cassie_out_t *out, cassie_user_in_t *in)
{
  // Initialise the array with zeros
  memset(in, 0, sizeof(cassie_user_in_t));

  // Command types:
  //   0     -> send null values (zeros)
  //   other -> send original values
  if (cmd_type_ != 0)
  {
    in->torque[0] = tst_mon_.linear.x;
    in->torque[1] = tst_mon_.linear.y;
    in->torque[2] = tst_mon_.linear.z;
    in->torque[3] = tst_mon_.angular.x;
    in->torque[4] = tst_mon_.angular.y;
    in->torque[5] = tst_mon_.angular.z;
    in->torque[6] = cmd_mon_.linear.x;
    in->torque[7] = cmd_mon_.angular.z;
  }
}


void UdpCommunicator::callbackUdp(const ros::TimerEvent& evt)
{
  // Poll for a new packet (uncomment to enable the receiver)
  // wait_for_packet(sock, recvbuf, sizeof recvbuf, NULL, NULL);
  process_packet_header(&header_info_, header_in_, header_out_);  // Process incoming header and write outgoing header
  unpack_cassie_out_t(data_in_, &cassie_out_);  // Unpack received data into Cassie output struct
  dummyControl(&cassie_out_, &cassie_in_);  // Run controller
  pack_cassie_user_in_t(&cassie_in_, data_out_);  // Pack Cassie input struct into outgoing packet

  int nbites = 0;
  nbites = send_packet(sock_, sendbuf_, sizeof sendbuf_, NULL, 0);  // Send response
}


void UdpCommunicator::callbackMon(const ros::TimerEvent& evt)
{
  pub_mon_cmd_.publish(cmd_mon_);
  pub_mon_tst_.publish(tst_mon_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mw_udp_communicator");
  ros::NodeHandle nh("~");
  UdpCommunicator uc(&nh);
  ros::Rate r(200);
  ROS_INFO("Node mw_udp_communicator is alive.");

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
