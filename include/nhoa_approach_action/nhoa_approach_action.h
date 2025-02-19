#ifndef APPROACH_ACTION_H_
#define APPROACH_ACTION_H_

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <angles/angles.h>
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nhoa_approach_action/ApproachAction.h>
//#include <people_msgs/People.h>
#include <std_msgs/Int16.h>
#include <hri_msgs/IdsList.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
//#include <indires_macro_actions/NavigateHomeAction.h>
//#include <indires_macro_actions/ExplorationAction.h>
//#include <indires_macro_actions/TeleoperationAction.h>

// Probably it is not required
//#include <adapted_move_base/move_base.h>

//#include <upo_navigation_macro_actions/Yield.h>

#include <mutex> //Mutex

// Dynamic reconfigure
/*#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <indires_macro_actions/NavigationMacroActionsConfig.h>
*/

// namespace macroactions {

class Nhoa_approach_action {
public:
  // enum datatype{INT_TYPE=1, DOUBLE_TYPE=2, BOOL_TYPE=3, STRING_TYPE=4,
  // GROUP_TYPE=5};

  Nhoa_approach_action(tf2_ros::Buffer *tf);
  ~Nhoa_approach_action();

  void
  approachCallback(const nhoa_approach_action::ApproachGoal::ConstPtr &goal);
  // void peopleCallback(const people_msgs::People::ConstPtr &msg);
  void idListCallback(const hri_msgs::IdsList::ConstPtr &msg);

  // void changeParametersNarrowPlaces();
  // void changeParametersNarrowPlaces2();
  // bool reconfigureParameters(std::string node, std::string param_name,
  // std::string value, const datatype type);

private:
  // Dynamic reconfigure
  // boost::recursive_mutex configuration_mutex_;
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>
  // *dsrv_;
  // void
  // reconfigureCB(upo_navigation_macro_actions::NavigationMacroActionsConfig
  // &config, uint32_t level);

  bool getPersonFromHRI(const std::string id,
                        geometry_msgs::TransformStamped &p);
  bool getPerson(const std::string frame_id,
                 geometry_msgs::TransformStamped &p);

  // geometry_msgs::PoseStamped computeRotationGoal(const people_msgs::Person
  // &p);
  geometry_msgs::PoseStamped
  computeRotationGoal(const geometry_msgs::TransformStamped &p);
  void computeApproachGoal(const geometry_msgs::TransformStamped &p,
                           geometry_msgs::PoseStamped &pose_goal);

  void fixFrame(std::string &cad);
  float normalizeAngle(float val, float min, float max);
  geometry_msgs::PoseStamped
  transformPoseTo(const geometry_msgs::PoseStamped &pose_in,
                  std::string frame_out);
  geometry_msgs::PointStamped
  transformPointTo(const geometry_msgs::PointStamped &point_in,
                   std::string frame_out);

  // CLIENT FOR THE MOVE_BASE SERVER
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      moveBaseClient;
  std::shared_ptr<moveBaseClient> moveBaseClient_;

  tf2_ros::Buffer *tf_;
  tf2_ros::TransformListener *tflistener_;

  ros::NodeHandle nh_;

  typedef actionlib::SimpleActionServer<nhoa_approach_action::ApproachAction>
      approachActionServer;
  std::shared_ptr<approachActionServer> ApprActionServer_;

  nhoa_approach_action::ApproachFeedback apprFeedback_;
  nhoa_approach_action::ApproachResult apprResult_;

  //
  double control_frequency_;
  //
  double person_max_dist_;
  //
  double th_min_tf_diff_;
  //
  double person_max_angle_;
  //
  std::string robot_base_frame_;
  std::string global_frame_;

  // Used to also perform approach
  bool move_closer_;
  float person_distance_;

  // ros::Subscriber people_sub_;
  // people_msgs::People people_;
  // std::mutex pmutex_;
  bool test_;
  ros::Subscriber hri_ids_sub_;
  std::vector<std::string> id_list_;
  std::mutex lmutex_;
};
//};
#endif
