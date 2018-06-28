#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <iomanip>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>
#include <sstream>

#define foreach BOOST_FOREACH

//输入起止时间，找出goal和输出bag

int main(int argc, char ** argv)
{
  if(argc<5)
  {
    ROS_ERROR("args number < 4!!!!!!!!!!");
    return 0;
  }
  
//   for(int k=0; k<argc; k++)
//   {
//     std::cout<<argv[k]<<std::endl;
//   }
  
  std::string arg1 = argv[1];
  std::string arg2 = argv[2];
  std::string arg3 = argv[3];
  std::string arg4 = argv[4];
  
  
  std::stringstream ss;
  int start_frame;
  int end_frame;
  std::string bag_path = "/home/sun/rosbags/1-synchronized/";
  std::string bag_save_path = "/home/sun/rosbags/2-processed/";
  
  std::string bag_name = argv[3];
  std::string bag_save_name = argv[4];
  
  ss<<arg1;
  ss>>start_frame;
  ss.clear();
  ss<<arg2;
  ss>>end_frame;
  ss.clear();
   
  bag_path = bag_path + bag_name;
  bag_save_path = bag_save_path + bag_save_name;
  
  std::cout<<"Start frame: "<<start_frame<<", end frame: "<<end_frame<<std::endl;
  std::cout<<"Input bag file: "<<bag_path<<std::endl;
  std::cout<<"Output bag file: "<<bag_save_path<<std::endl;
  
  ros::init(argc, argv, "rosbag_process");
 
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  rosbag::Bag bag_save;
  bag_save.open(bag_save_path, rosbag::bagmode::Write);
  
  std::vector<std::string> topic_names;
  topic_names.push_back(std::string("/robot"));
  topic_names.push_back(std::string("/vel"));
  topic_names.push_back(std::string("/people"));
  topic_names.push_back(std::string("/obstacles_new"));
  topic_names.push_back(std::string("/robot_map_new")); 
  topic_names.push_back(std::string("/people/people_arrow_markers"));
  topic_names.push_back(std::string("/people/people_cylinders_markers"));
  topic_names.push_back(std::string("/tf"));
  
  std::vector<geometry_msgs::PoseStamped> v_robot;
  std::vector<geometry_msgs::Twist> v_vel;
  std::vector<upo_msgs::PersonPoseArrayUPO> v_people;
  std::vector<sensor_msgs::PointCloud2> v_obstacles;
  std::vector<geometry_msgs::PoseStamped> v_robot_map;
  std::vector<visualization_msgs::MarkerArray> v_people_arrow;
  std::vector<visualization_msgs::MarkerArray> v_people_cylinders;
  std::vector<tf2_msgs::TFMessage> v_tf;
  
  rosbag::View view(bag, rosbag::TopicQuery(topic_names));
  
  foreach(rosbag::MessageInstance const m, view)
  {
    if(m.getTopic()=="/robot")
    {
      geometry_msgs::PoseStamped::Ptr robot = m.instantiate<geometry_msgs::PoseStamped>();
      v_robot.push_back(*robot.get());
    }
    else if(m.getTopic()=="/vel")
    {
      geometry_msgs::Twist::Ptr vel = m.instantiate<geometry_msgs::Twist>();
      v_vel.push_back(*vel.get());
    }
    else if(m.getTopic()=="/people")
    {
      upo_msgs::PersonPoseArrayUPO::Ptr people = m.instantiate<upo_msgs::PersonPoseArrayUPO>();
      v_people.push_back(*people.get());
    }
    else if(m.getTopic()=="/obstacles_new")
    {
      sensor_msgs::PointCloud2::Ptr obstacles= m.instantiate<sensor_msgs::PointCloud2>();
      v_obstacles.push_back(*obstacles.get());
    }
    else if(m.getTopic()=="/robot_map_new")
    {
      geometry_msgs::PoseStamped::Ptr robot_map = m.instantiate<geometry_msgs::PoseStamped>();
      v_robot_map.push_back(*robot_map.get());
    }
    else if(m.getTopic()=="/people/people_arrow_markers")
    {
      visualization_msgs::MarkerArray::Ptr people_arrow = m.instantiate<visualization_msgs::MarkerArray>();
      v_people_arrow.push_back(*people_arrow.get());
    }
    else if(m.getTopic()=="/people/people_cylinders_markers")
    {
      visualization_msgs::MarkerArray::Ptr people_cylinders = m.instantiate<visualization_msgs::MarkerArray>();
      v_people_cylinders.push_back(*people_cylinders.get());
    }
    else if(m.getTopic()=="/tf")
    {
      tf2_msgs::TFMessage::Ptr tfs = m.instantiate<tf2_msgs::TFMessage>();
      v_tf.push_back(*tfs.get());
    }
  }
  
  
  
  
  ///tf /tf_static /people/people_arrow_markers /people/people_cylinders_markers
  
  std::cout<<"----------------------------------"<<std::endl;
  std::cout<<v_robot.size()<<std::endl;
  std::cout<<v_vel.size()<<std::endl;
  std::cout<<v_people.size()<<std::endl;
  std::cout<<v_obstacles.size()<<std::endl;
  std::cout<<v_robot_map.size()<<std::endl;
  std::cout<<v_people_arrow.size()<<std::endl;
  std::cout<<v_people_cylinders.size()<<std::endl;
  std::cout<<v_tf.size()<<std::endl;
  
  if(start_frame < 0)
  {
    ROS_ERROR("Start frame number < 0!!!");
    return 0;
  }
  
  if(end_frame > v_robot.size())
  {
    ROS_ERROR("End frame number > size!!!");
    return 0;
  }
  
  //找到vector里最后一个pose，作为goal，加入goal的vector
  geometry_msgs::PoseStamped goal_pose;
  goal_pose = v_robot[end_frame];
  
  double t_start = v_robot[start_frame].header.stamp.toSec();
  double t_end = v_robot[end_frame].header.stamp.toSec();
  
  for(int i=start_frame;i<=end_frame;i++)
  {
    ros::Time t = v_robot[i].header.stamp;
//     std::cout<<std::setprecision(20)<<t.toSec()<<std::endl;
//     std::cout<<t.sec<<":"<<t.nsec<<std::endl;
    bag_save.write("/robot", t, v_robot[i]);
    bag_save.write("/vel", t, v_vel[i]);
    bag_save.write("/people", t, v_people[i]);
    bag_save.write("/obstacles_new", t, v_obstacles[i]);
    bag_save.write("/robot_map_new", t, v_robot_map[i]);
    bag_save.write("/people/people_arrow_markers", t, v_people_arrow[i]);
    bag_save.write("/people/people_cylinders_markers", t, v_people_cylinders[i]);
    
    goal_pose.header.stamp = t;
    bag_save.write("/goal", t, goal_pose);
  }
  
  for(int j=0; j<v_tf.size(); j++)
  {
    if((v_tf[j].transforms[0].header.stamp.toSec() >= t_start)&&(v_tf[j].transforms[0].header.stamp.toSec() <= t_end))
    {
      ros::Time t = v_tf[j].transforms[0].header.stamp;
      bag_save.write("/tf", t, v_tf[j]);
    }
  }

  
  
  bag.close();
  bag_save.close();
  
  return 0;
}
