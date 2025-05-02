#include <nhoa_approach_action/nhoa_approach_action.h>

/*
Status can take this values:
uint8 PENDING         = 0   # The goal has yet to be processed by the action
server
uint8 ACTIVE          = 1   # The goal is currently being processed by
the action server
uint8 PREEMPTED       = 2   # The goal received a cancel
request after it started executing #   and has since completed its execution
(Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved
successfully by the action server (Terminal State)
uint8 ABORTED         = 4   #
The goal was aborted during execution by the action server due #    to some
failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by
the action server without being processed, #    because the goal was
unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal
received a cancel request after it started executing #    and has not yet
completed execution
uint8 RECALLING       = 7   # The goal received a cancel
request before it started executing, #    but the action server has not yet
confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal
received a cancel request before it started executing #    and was successfully
cancelled (Terminal State)
uint8 LOST            = 9   # An action client can
determine that a goal is LOST. This should not be #    sent over the wire by an
action server
*/

// namespace macroactions {

Nhoa_approach_action::Nhoa_approach_action(tf2_ros::Buffer *tf) {
  tf_ = tf;
  tflistener_ = new tf2_ros::TransformListener(*tf_);

  ros::NodeHandle n("~");

  // Dynamic reconfigure
  // dsrv_ = new
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>(n);
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>::CallbackType
  // cb = boost::bind(&Upo_navigation_macro_actions::reconfigureCB, this, _1,
  // _2); dsrv_->setCallback(cb);

  n.param<bool>("test_without_hri", test_, false);
  n.param<double>("person_max_dist", person_max_dist_, 2.0); // meters
  n.param<double>("th_min_tf_diff", th_min_tf_diff_, 0.25); // meters

  n.param<bool>("move_close", move_closer_, false);
  n.param<double>("person_max_angle_diff", person_max_angle_, 0.4);
  n.param<double>("control_frequency", control_frequency_, 1.0);
  // std::string people_topic = "";
  // n.param<std::string>("people_topic", people_topic, "people");
  n.param<std::string>("robot_frame", robot_base_frame_, "base_link");
  n.param<std::string>("global_frame", global_frame_, "map");

  person_distance_ = 100.0;

  ros::NodeHandle nh;
  if (!test_) {
    std::string hri_id_topic = "";
    n.param<std::string>("hri_ids_topic", hri_id_topic,
                         "humans/bodies/tracked");
    hri_ids_sub_ = nh.subscribe<hri_msgs::IdsList>(
        hri_id_topic.c_str(), 1, &Nhoa_approach_action::idListCallback, this);
  }
  // Dynamic reconfigure
  // dsrv_ = new
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>(n);
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>::CallbackType
  // cb = boost::bind(&Upo_navigation_macro_actions::reconfigureCB, this, _1,
  // _2); dsrv_->setCallback(cb);

  // people_sub_ = nh.subscribe<people_msgs::People>(
  //    people_topic.c_str(), 1, &Nhoa_approach_action::peopleCallback, this);

  moveBaseClient_ = std::make_shared<moveBaseClient>(
      "move_base", true); // true-> do not need ros::spin()
  ROS_INFO("Waiting for move_base action server to start...");
  moveBaseClient_->waitForServer();
  ROS_INFO("\n\n\nMove_base action client connected!\n\n\n");
  //ros::Duration(10.0).sleep();

  // Initialize action server
  ROS_INFO("Initializing APPROACH action server...");
  ApprActionServer_ = std::make_shared<approachActionServer>(
      nh_, "Approach",
      boost::bind(&Nhoa_approach_action::approachCallback, this, _1),
      false); // boost::bind

  // start action server
  ApprActionServer_->start();
  ROS_INFO("\n\n\n\nAPPROACH action server started!\n\n\n\n");
}

Nhoa_approach_action::~Nhoa_approach_action() {}

/*
void
Upo_navigation_macro_actions::reconfigureCB(upo_navigation_macro_actions::NavigationMacroActionsConfig
&config, uint32_t level){

    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    control_frequency_ = config.control_frequency;
    secs_to_check_block_ = config.secs_to_check_block;
  block_dist_ = config.block_dist;
  secs_to_wait_ = config.secs_to_wait;
  social_approaching_type_ = config.social_approaching_type;
  secs_to_yield_ = config.secs_to_yield;
  //use_leds_ = config.use_leds;
  //leds_number_ = config.leds_number;

}*/

/*
MoveBase server:
-----------------
Action Subscribed topics:
move_base/goal		[move_base_msgs::MoveBaseActionGoal]
move_base/cancel 	[actionlib_msgs::GoalID]

Action Published topcis:
move_base/feedback	[move_base_msgs::MoveBaseActionFeedback]
move_base/status	[actionlib_msgs::GoalStatusArray]
move_base/result	[move_base_msgs::MoveBaseAcionResult]
*/

bool Nhoa_approach_action::getPersonFromHRI(
    const std::string id, geometry_msgs::TransformStamped &p) {

  // Get the id list
  lmutex_.lock();
  std::vector<std::string> list = id_list_;
  lmutex_.unlock();
  std::string id_frame = "body_";
  // first, check the target_id
  // if (id == "-1") {
  //   // we use the first person if exists
  //   if (!list.empty()) {
  //     id_frame = id_frame + list[0];
  //   } else
  //     return false;
  // } else {
  //   // we use directly the id to get the TF
  //   id_frame = id_frame + id;
  // }
  // Now, we look for the frame in robot base frame
  sleep(5);
  return getPerson("target", p);
}
  // TODO: add checking if frame_id == -1 to take the first one.

bool Nhoa_approach_action::getPerson(const std::string frame_id,
                                     geometry_msgs::TransformStamped &p) {
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped p_prev;
  bool found = false;
  bool timeout = false;
  const double tolerance = th_min_tf_diff_;//0.15; // 15 cm tolerance for stabilization
  ros::Rate r(5); // 5 Hz rate to check the position
  ros::Time start_time = ros::Time::now();
  const double timeout_duration = 10.0; // Timeout after 10 seconds

  while (!found && !timeout) {
    try {
      transformStamped =
          tf_->lookupTransform(robot_base_frame_, frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Approach. Error getPerson: %s", ex.what());
      ROS_INFO("Approach - ERROR");
      return false;
    }

    // Check distance between current and previous transform
    if (p_prev.header.stamp != ros::Time(0)) { // Ensure p_prev is initialized
      double dx = transformStamped.transform.translation.x - p_prev.pose.position.x;
      double dy = transformStamped.transform.translation.y - p_prev.pose.position.y;
      double dz = transformStamped.transform.translation.z - p_prev.pose.position.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (distance <= tolerance) {
        found = true; // Position has stabilized
      }
    }

    // Update p_prev with the current transform
    p_prev.pose.position.x = transformStamped.transform.translation.x;
    p_prev.pose.position.y = transformStamped.transform.translation.y;
    p_prev.pose.position.z = transformStamped.transform.translation.z;
    p_prev.header.stamp = transformStamped.header.stamp;

    // Check timeout
    if ((ros::Time::now() - start_time).toSec() > timeout_duration) {
      timeout = true;
      ROS_WARN("Approach. Timeout while waiting for position to stabilize.");
    }

    r.sleep();
  }

  if (found) {
    p = transformStamped;
    return true;
  }

  return false; // Return false if timed out or failed to stabilize
}

geometry_msgs::PoseStamped Nhoa_approach_action::computeRotationGoal(
    const geometry_msgs::TransformStamped &p) {

  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = 0.0;
  goal.pose.position.y = 0.0;
  goal.pose.position.z = 0.0;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  goal.header.frame_id = robot_base_frame_;
  goal.header.stamp = ros::Time::now();
  float angle_diff =
      std::atan2(p.transform.translation.y, p.transform.translation.x);
  if (fabs(angle_diff) >= person_max_angle_) {
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(angle_diff);
  }
  return goal;
}

void Nhoa_approach_action::computeApproachGoal(
    const geometry_msgs::TransformStamped &p,
    geometry_msgs::PoseStamped &pose_goal) {
  float dist = std::sqrt(p.transform.translation.x * p.transform.translation.x +
                         p.transform.translation.y * p.transform.translation.y);
  // ROS_INFO("\n\n\nPEOPLE DISTANCE: %.2f\n\n\n", dist);
  person_distance_ = dist;
  if (dist >= person_max_dist_) {
    float angle =
        std::atan2(p.transform.translation.y, p.transform.translation.x);
    pose_goal.pose.position.x = dist / 2.0 * cos(angle);
    pose_goal.pose.position.y = dist / 2.0 * sin(angle);
    pose_goal.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  }
}

// geometry_msgs::PoseStamped
// Nhoa_approach_action::computeRotationGoal(const people_msgs::Person &p) {
//   geometry_msgs::PoseStamped movebase_goal;
//   movebase_goal.pose.position.x = 0.0;
//   movebase_goal.pose.position.y = 0.0;
//   movebase_goal.pose.position.z = 0.0;
//   movebase_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
//   movebase_goal.header.frame_id = robot_base_frame_;
//   movebase_goal.header.stamp = ros::Time::now();
//   geometry_msgs::PointStamped in;
//   pmutex_.lock();
//   in.header = people_.header;
//   pmutex_.unlock();
//   in.point = p.position;
//   geometry_msgs::PointStamped point = transformPointTo(in,
//   robot_base_frame_); float angle_diff = std::atan2(point.point.y,
//   point.point.x); if (fabs(angle_diff) >= person_max_angle_) {
//     angle_diff = normalizeAngle(angle_diff, -M_PI, M_PI);
//     movebase_goal.pose.orientation =
//     tf::createQuaternionMsgFromYaw(angle_diff);
//   }
//   // Transform goal to "map" frame
//   movebase_goal = transformPoseTo(movebase_goal, global_frame_);
//   return movebase_goal;
// }

void Nhoa_approach_action::approachCallback(
    const nhoa_approach_action::ApproachGoal::ConstPtr &goal) {
  printf("¡¡¡¡¡¡¡Action APPROACH started!!!!!!\n");

  // people_msgs::Person person;
  std::string id = goal->target_id;
  bool follow_mode = goal->mode == goal->FOLLOW;
  geometry_msgs::TransformStamped tfperson;
  bool found;
  if (test_)
    found = getPerson(goal->target_id, tfperson);
  else
    found = getPersonFromHRI(goal->target_id, tfperson);
  // Abort action if person not found!!!
  if (!found) {
    ROS_INFO("Approach - ABORTED state - no person found");
    apprResult_.result = "no person found";
    apprResult_.value = 1;
    ApprActionServer_->setAborted(apprResult_, "Approach aborted");
    moveBaseClient_->cancelAllGoals();
    // is_navigating = false;
    return;
  }

  // cancel any other move base action
  // moveBaseClient_->cancelAllGoals();

  // Now compute the goal
  // First check the orientation
  move_base_msgs::MoveBaseGoal g;
  // goal in robot frame
  geometry_msgs::PoseStamped pose_goal_prev;
  geometry_msgs::PoseStamped pose_goal = computeRotationGoal(tfperson);
  // check the distance
  if (move_closer_) {
    computeApproachGoal(tfperson, pose_goal);
  }
  // transform goal from robot base to global frame
  g.target_pose = transformPoseTo(pose_goal, global_frame_);
  moveBaseClient_->sendGoal(g);
  pose_goal_prev = g.target_pose;
  // is_navigating = true;

  // actionlib::SimpleClientGoalState state = moveBaseClient_->getState();
  // if (moveBaseClient_->getState() ==
  //     actionlib::SimpleClientGoalState::SUCCEEDED) {
  //   ROS_INFO("Success!!!");
  // } /*else {
  //   ROS_INFO("Failed!");
  // }*/

  ros::Rate r(control_frequency_);
  bool exit = false;
  ros::Time time_init = ros::Time::now();
  bool first = true;
  // nav_msgs::Odometry pose_init;
  // ros::WallTime startt;
  while (nh_.ok()) {
    // startt = ros::WallTime::now();

    if (ApprActionServer_->isPreemptRequested()) {
      if (ApprActionServer_->isNewGoalAvailable()) {
        nhoa_approach_action::ApproachGoal new_goal =
            *ApprActionServer_->acceptNewGoal();
        id = new_goal.target_id;
      } else {
        // Cancel?????
        // notify the ActionServer that we've successfully preempted
        apprResult_.result = "Preempted";
        apprResult_.value = 2;
        ROS_DEBUG_NAMED("nhoa_approach_action", "preempting the current goal");
        ApprActionServer_->setPreempted(apprResult_, "Approach preempted");
        // we'll actually return from execute after preempting
        return;
      }
    }

    if (test_)
      found = getPerson(goal->target_id, tfperson);
    else
      found = getPersonFromHRI(goal->target_id, tfperson);
    // Abort action if person not found!!!
    if (!found) {
      ROS_INFO("Approach - ABORTED state - no person found");
      apprResult_.result = "no person found";
      apprResult_.value = 1;
      ApprActionServer_->setAborted(apprResult_, "Approach aborted");
      moveBaseClient_->cancelAllGoals();
      return;
    }

    pose_goal = computeRotationGoal(tfperson);


      // check the distance
      if (move_closer_) {
        computeApproachGoal(tfperson, pose_goal);
      }
      // transform goal from robot base to global frame
      g.target_pose = transformPoseTo(pose_goal, global_frame_);

      if ( sqrt( pow(g.target_pose.pose.position.x-pose_goal_prev.pose.position.x, 2) + pow(g.target_pose.pose.position.y-pose_goal_prev.pose.position.y, 2)) > 0.1  ) { 
        moveBaseClient_->sendGoal(g);
        pose_goal_prev = g.target_pose;

        ROS_INFO_STREAM("Sent new goal: " << g.target_pose.pose);
    }
    // Posible states:
    // PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    actionlib::SimpleClientGoalState state = moveBaseClient_->getState();

    // If movebase SUCCEEDED OR ACTIVE, return ok for our action
    // if (state == actionlib::SimpleClientGoalState::SUCCEEDED ||
    //    state == actionlib::SimpleClientGoalState::ACTIVE ||
    //    state == actionlib::SimpleClientGoalState::PREEMPTED) {
    apprFeedback_.text = "Running";
    //} else {
    // Otherwise, communicate a movebase error
    //  apprFeedback_.text = "Movebase error";
    //}

    apprFeedback_.person_distance = person_distance_;

    // push the feedback out
    // geometry_msgs::PoseStamped aux;
    // aux.header = new_pose.header;
    // aux.pose.position = new_pose.pose.pose.position;
    // aux.pose.orientation = new_pose.pose.pose.orientation;
    // nwfeedback_.base_position = aux;
    ApprActionServer_->publishFeedback(apprFeedback_);

    if(!follow_mode && apprFeedback_.person_distance < person_max_dist_) // Approach person -> action completed successfully
    {
      apprResult_.result = "Person approached";
      apprResult_.value = 0;
      ApprActionServer_->setSucceeded(apprResult_, "Approach succeded");
      moveBaseClient_->cancelAllGoals();
      return;
    }

    // ros::WallDuration dur = ros::WallTime::now() - startt;
    // printf("Loop time: %.4f secs\n", dur.toSec());

    r.sleep();
  }

  ROS_INFO("Approach - ABORTED state");
  apprResult_.result = "Aborted. System is shuting down";
  apprResult_.value = 3;
  ApprActionServer_->setAborted(
      apprResult_, "Approach aborted because the node has been killed");
}

/*
bool Indires_macro_actions::reconfigureParameters(std::string node, std::string
param_name, std::string value, const datatype type)
{
  //printf("RECONFIGURE PARAMETERS METHOD\n");
  dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter param1;
    dynamic_reconfigure::BoolParameter param2;
    dynamic_reconfigure::DoubleParameter param3;
    dynamic_reconfigure::StrParameter param4;
    dynamic_reconfigure::Config conf;

    switch(type)
    {
    case INT_TYPE:
      param1.name = param_name.c_str();
      param1.value = stoi(value);
      conf.ints.push_back(param1);
      break;

    case DOUBLE_TYPE:
      param3.name = param_name.c_str();
      //printf("type double. Value: %s\n", param3.name.c_str());
      param3.value = stod(value);
      //printf("conversion to double: %.3f\n", param3.value);
      conf.doubles.push_back(param3);
      break;

    case BOOL_TYPE:
      param2.name = param_name.c_str();
      param2.value = stoi(value);
      conf.bools.push_back(param2);
      break;

    case STRING_TYPE:
      param4.name = param_name.c_str();
      param4.value = value;
      conf.strs.push_back(param4);
      break;

    default:
      ROS_ERROR("indires_macro_actions. ReconfigureParameters. datatype not
valid!");
  }
    srv_req.config = conf;

    std::string service = node + "/set_parameters";
    if (!ros::service::call(service, srv_req, srv_resp)) {
      ROS_ERROR("Could not call the service %s reconfigure the param %s to %s",
service.c_str(), param_name.c_str(), value.c_str());
      return false;
    }
    return true;
}
*/

// void Nhoa_approach_action::peopleCallback(
//     const people_msgs::People::ConstPtr &msg) {
//   pmutex_.lock();
//   people_ = *msg;
//   pmutex_.unlock();
// }

void Nhoa_approach_action::idListCallback(
    const hri_msgs::IdsList::ConstPtr &msg) {
  lmutex_.lock();
  id_list_ = msg->ids;
  lmutex_.unlock();
}

geometry_msgs::PoseStamped
Nhoa_approach_action::transformPoseTo(const geometry_msgs::PoseStamped &pose_in,
                                      std::string frame_out) {
  geometry_msgs::PoseStamped in = pose_in;
  geometry_msgs::PoseStamped pose_out = pose_in;
  pose_out.header.stamp = ros::Time();
  in.header.stamp = ros::Time();
  try {
    tf_->transform(in, pose_out, frame_out);
  } catch (tf2::TransformException ex) {
    ROS_WARN("\n\nApproachAction. TransformException in method "
             "transformPoseTo: %s\n\n",
             ex.what());
  }
  return pose_out;
}

geometry_msgs::PointStamped Nhoa_approach_action::transformPointTo(
    const geometry_msgs::PointStamped &point_in, std::string frame_out) {
  geometry_msgs::PointStamped in = point_in;
  in.header.stamp = ros::Time();
  geometry_msgs::PointStamped point_out;
  try {
    tf_->transform(in, point_out, frame_out.c_str());
  } catch (tf2::TransformException ex) {
    ROS_WARN("\n\nApproachAction. TransformException in method "
             "transformPointTo: %s\n\n",
             ex.what());
    point_out.header = in.header;
    point_out.header.stamp = ros::Time::now();
    point_out.point.x = 0.0;
    point_out.point.y = 0.0;
    point_out.point.z = 0.0;
  }

  return point_out;
}

// This method removes the initial slash from the frame names
// in order to compare the string names easily
void Nhoa_approach_action::fixFrame(std::string &cad) {
  if (cad[0] == '/') {
    cad.erase(0, 1);
  }
}

float Nhoa_approach_action::normalizeAngle(float val, float min, float max) {
  float norm = 0.0;
  if (val >= min)
    norm = min + fmod((val - min), (max - min));
  else
    norm = max - fmod((min - val), (max - min));

  return norm;
}
