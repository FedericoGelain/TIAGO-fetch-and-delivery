#include <ros/ros.h>
#include <ros/topic.h>

#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>

#include <actionlib/server/simple_action_server.h>

#include <assignment2/ManipulationAction.h>
#include <assignment2/ManipulationActionResult.h>

#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <gazebo_ros_link_attacher/Attach.h>

#include <tiago_iaslab_simulation/Objs.h>

// Our Action interface pointer type for moving TIAGo's arm or gripper, provided as a typedef for convenience
typedef boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_trajectory_Ptr;

// Constant values of table dimensions 
constexpr float TABLE_HEIGHT = 0.755;
constexpr float TABLE_THICKNESS = 0.04;

// Constant values of cylinders dimensions 
constexpr float CYLINDERS_HEIGHT = 0.69;
constexpr float CYLINDERS_RADIUS = 0.21;

class ManipulationAction{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<assignment2::ManipulationAction> as_;
  std::string action_name_;
  assignment2::ManipulationFeedback feedback_;
  assignment2::ManipulationResult result_;

	ros::ServiceClient client_detach;
	ros::ServiceClient client_attach;

  // Variables to deal with the transformation matrix that allows to convert the pose from "map" frame into the robot frame "base_footprint"
  tf::TransformListener tf_;
  tf::StampedTransform transform;
	
  // Variables that manage the movements of the TIAGO's gripper 
  follow_trajectory_Ptr GripperClient;

  // Move group interfaces that manage the movements of the TIAGO's arm and armtorso
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm;
  moveit::planning_interface::MoveGroupInterface move_group_interface_armtorso;

  // Interface that manages the creation/elimination of the collisions
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Vectors to store the positions of the joints for intermediate and secure pose 
  std::vector<double> joint_tucked_arm_positions;
  std::vector<double> joint_extended_arm_positions;
	
public:
  // Constructor
  ManipulationAction(std::string name);

  // Destructor
  ~ManipulationAction(void){}
  
  /**
   * @brief Utility function that sends feedback to the action client
   * 
   * @param feed message string to send as feedback
   */
  void sendFeedback(std::string feed);

  /**
   * @brief Function that tries to connect to the action server (maximum 3 attempts) 
   * 
   * @param actionClient action interface pointer 
   * @param controller_name name of the specific controller that we want to connect
   * @return true If the connection with the server is successful 
   * @return false If the connection with the server fails 
   */
  bool createControllerClient(follow_trajectory_Ptr& actionClient, const std::string& controller_name);

  /**
   * @brief Function that tries to place the object on top of the relative cylinder. 
   *        It manages the arm pose, the gripper and the detaching of the picked object
   * 
   * @param goal Pointer to the ManipulationAction goal, where all the information sent by the client is stored
   * @return true If the object is placed correctly on top of the cylinder
   * @return false If the object is not placed correctly on top of the cylinder
   */
  bool placeObject(const assignment2::ManipulationGoalConstPtr &goal);
  
  /**
   * @brief Function that generates the collision of the cylinder where we want to place the picked object.
   *        It is useful in order to avoid collisions between the arm and cylinder during the place routine.
   * 
   * @param cylinder_pose Pose of the cylinder 
   */
  void generateCylinderCollision(geometry_msgs::Pose cylinder_pose);

  /**
   * @brief Function that tries to detach the object from TIAGO's gripper based on the object identifier.
   * 
   * @param obj_id The object identifier 
   * @return true If the object is detached correctly
   * @return false If the object is not detached correctly
   */
  bool detachObject(int obj_id);
  
  /**
   * @brief Function that performs the right reset movements of the TIAGO's arm based on the integer parameter. 
   *        If TIAGO has taken the red or green object, then the function tries to perform two reset movements 
   *        of the arm: intermediate and secure arm pose. Else it only performs the movement to reach the secure 
   *        pose. 
   *        Note: the default integer parameter is -1. 
   * 
   * @param obj_id Object identifier or -1 during the place routine
   * @return true If the arm is reset correctly to the secure pose
   * @return false If the arm is not reset correctly to the secure pose
   */
  bool resetArmPose(int obj_id);

  /**
   * @brief Function that tries to attach the object to the TIAGO's gripper based on the object identifier.
   * 
   * @param obj_id The object identifier
   * @return true If the object is attached correctly
   * @return false If the object is not attached correctly
   */
  bool attachObject(int obj_id);

  /**
   * @brief Function that tries to move the gripper's joints according to the input parameter, waiting util it 
   *        either succeeds or not.
   * 
   * @param joints_value The desired joints value of the TIAGO's gripper
   * @return true If the gripper's joints reach correctly the input joints value 
   * @return false If the gripper's joints fail to reach the input joints value 
   */
  bool moveGripper(float joints_value);

  /**
   * @brief Function that tries to move the input planning move group in order to reach the input desired pose.
   * 
   * @param target_pose The desired pose to reach
   * @param move_group Interface to plan and execute the plan movements
   * @return true If the move group succeeds to reach the input pose
   * @return false If the move group fails to reach the input pose
   */
  bool movePlanningGroup(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface& move_group);

  /**
   * @brief Function that tries to move the joints in order to reach the desired positions defined as input.
   * 
   * @param joints_value Vector of double that contains the desired joints values
   * @return true If the joints reach correctly the input values 
   * @return false If the joints fail to reach the input values
   */
  bool performJointsMotion(std::vector<double> joints_values);

  /**
   * @brief Function that manages the picking routine: reaching intermediate arm pose if it is needed, moving arm 
   *        on top of the object, moving arm on the object, closing gripper, attaching the object, removing 
   *        collision, raising arm and reset arm pose
   * 
   * @param goal Pointer to the ManipulationAction goal, where all the information sent by the client is stored
   * @param pick_height The height of the object that we want to pick
   * @return true If tiago succeeds correctly in taking the object
   * @return false If tiago fails to take the object
   */
  bool pickObject(const assignment2::ManipulationGoalConstPtr &goal, float& pick_height);

  /**
   * @brief Function that adds all collisions based on the objects seen by TIAGO and saves in the 'pick_height' variable the 
   *        height of the object to be picked
   * 
   * @param goal Pointer to the ManipulationAction goal, where all the information sent by the client is stored
   * @param pick_height The variable where the height of the object to be fetched will be saved (useful for later functions)
   */
  void generateCollisions(const assignment2::ManipulationGoalConstPtr &goal, float& pick_height);

  /**
   * @brief Function that generates the collision of the table where TIAGO will pick up the objects 
   */
  void buildTable();

  /**
  * @brief Callback function that handles the communication between ManipulationAction server and client
  * @param goal Pointer to the ManipulationAction goal, where all the information sent by the client is stored
  */
  void executeCB(const assignment2::ManipulationGoalConstPtr &goal);
};

