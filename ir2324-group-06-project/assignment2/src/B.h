#include <ros/ros.h>
#include <ros/topic.h>

#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>

#include <actionlib/server/simple_action_server.h>

#include <assignment2/DetectionAction.h>
#include <assignment2/DetectionActionResult.h>

#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

/**
 * @brief class of the server action that will deal with the detection of the objects placed on the table
 */
class DetectionAction {

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2::DetectionAction> as_;
    std::string action_name_;
    assignment2::DetectionFeedback feedback_;
    assignment2::DetectionResult result_;
    ros::Publisher orientation_pub; // publisher that manages the orientation of the robot head
    //we want to retrieve the transformation matrix that allows to convert the pose in "xtion_rgb_optical_frame" frame into the robot frame "base_footprint"
	tf::TransformListener tf_;
	tf::StampedTransform transform;

public:
    DetectionAction(std::string name) :
        as_(nh_, name, boost::bind(&DetectionAction::executeCB, this, _1), false),
        action_name_(name)
    {
    	orientation_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);
        as_.start();
    }

    /**
     * @brief Destructor of DetectionAction class 
     */
    ~DetectionAction(void){}

    /**
     * @brief Callback function that handles the communication between DetectionAction server and client
     * @param goal pointer to the DetectionAction goal, where all the information sent by the client is stored
     */
    void executeCB(const assignment2::DetectionGoalConstPtr &goal);
  
    /**
     * @brief Function that, given the transformation matrix between two frames, converts the coordinates of a pose in one frame to the other
     * @param transform transformation matrix used to convert the coordinates from one frame to the other
     * @param obj_pose pose that we want to convert in the other frame
     * @return geometry_msgs::Pose transformed pose in the wanted frame
     */
    geometry_msgs::Pose transform_pose(tf::StampedTransform transform, geometry_msgs::PoseWithCovarianceStamped obj_pose);
};
