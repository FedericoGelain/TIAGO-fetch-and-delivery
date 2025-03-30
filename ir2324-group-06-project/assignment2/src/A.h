#include <ros/ros.h>
#include <ros/topic.h>

#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/ref.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <assignment1/InfoAction.h>
#include <assignment1/InfoActionResult.h>
#include <assignment2/DetectionAction.h>
#include <assignment2/DetectionActionResult.h>
#include <assignment2/ManipulationAction.h>
#include <assignment2/ManipulationActionResult.h>

#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <gazebo_ros_link_attacher/Attach.h>
#include <image_transport/image_transport.h>

#include <tiago_iaslab_simulation/Objs.h>

// constant values of the place cylinder used to determine the correct place pose of the object
constexpr float CYLINDERS_HEIGHT = 0.69;
constexpr float CYLINDERS_RADIUS = 0.21;

// topics used to retreive and show what the camera robot is seeing
static const std::string IMAGE_TOPIC = "/xtion/rgb/image_raw";

class NodeA {
protected:
	ros::NodeHandle n;
	ros::ServiceClient client;
	tiago_iaslab_simulation::Objs srv;
	actionlib::SimpleActionClient<assignment1::InfoAction> ac;

	// Create the action client for the detection
	actionlib::SimpleActionClient<assignment2::DetectionAction> ac_detection;

	// Create the action client for the manipulation
	actionlib::SimpleActionClient<assignment2::ManipulationAction> ac_manipulation;

	// publisher used to specify the velocities to move the robot (in this case the linear velocity along the x axis)
	ros::Publisher cmdvel_pub;

    // publisher used to specify how the robot head will rotate
	ros::Publisher head_pub;

    // variables to look at and store the correct transformation matrix to convert a pose from a certain frame to another
	tf::TransformListener tf_;
	tf::StampedTransform transform;

	//this vector will store all the waypoints the robot takes to reach a certain pose, so that
	//it can backtrack to safe positions while moving around the table
	std::vector<std::tuple<float, float, float>> waypoints;

	//vector that stores the poses the robot has to safely reach each side of the table
	std::vector<std::tuple<float, float, float>> table_sides;
	
	//id of the object that has to currently be picked and placed
	int service_id;

	//flag that allows to start the extra points routine
	int extra_part;


public:
    /**
     * @brief Constructor of class NodeA
     * 
     * @param name 
     */
    NodeA(std::string name, int extra);

    /**
     * @brief Destructor of class NodeA
     * 
     */
    ~NodeA(void){}

    /**
     * @brief main method that handles all the communication with human node service
     * and the various action servers for robot movement, detection and manipulation
     */
    void start();

    /**
     * @brief helper function that initializes all the waypoints to reach safe positions and all the
     * table sides
     */
    void initialize_waypoints_and_table_sides();

    /**
     * @brief function that allows the robot to move to the last side of the table, next to the wall,
     * publishing to the cmd_vel topic
     * 
     * @param destination y coordinate that the robot has to reach before stopping
     */
    void move_cmd_vel(float destination);

    /**
     * @brief function that allows the robot to safely move away from the table after the detection of the last
     * table side
     * 
     * @param destination y coordinate that the robot has to reach before stopping
     */
    void goingBackCmdVel(float destination);

    /**
     * @brief functions that sends to C the goal to perform the placing of the object on top of the cylinder
     * 
     * @param cylinder_position position of the cylinder on top of which to place the object
     * @param res result of the action server after performing the place routine
	 * @param positionOfCylinder index of the image sector in which the cylinder was found
     * @return true if the place routine worked successfully
     * @return false if the place routine failed
     */
    bool placeObject(tf::Vector3 cylinder_position, assignment2::ManipulationResult& res, int positionOfCylinder);
    
    /**
     * @brief function that tries to find the coordinates of the cylinder in which to place the object
     * and stores them in the corresponding function argument
     * 
     * @param cylinder_position vector in which we store the coordinates of the place cylinder
     * @return true if the cylinder coordinates are correctly found
     * @return false if the function fails at any point and the coordinates can't be found
     */
    bool findPlaceCylinder(tf::Vector3 &cylinder_position, int& positionOfCylinder);

    /**
     * @brief function that allows to move the robot head in both directions to correctly see the place cylinders
     * 
     * @param roll angle that specifies the sideways rotation of the head
     * @param pitch angle that specifies the longitudinal rotation of the head
     */
    void moveHead(double roll, double pitch);

    /**
     * @brief function that computes the average color of an image and returns it
     * 
     * @param image input image to calculate the average color of
     * @return cv::Vec3f average color of the input image
     */
    cv::Vec3f computeAverageColor(const cv::Mat& image);

    /**
     * @brief function that finds the correct index of the array in which the cylinder positions found by scan are stored, 
     * given the corresponding image and the color of the object picked by TIAGO
     * 
     * @param COLOR_TO_PLACE_BGR color of the object picked by TIAGO
     * @param image input image that shows all 3 cylinders
     * @return int array index of the correct cylinder position
     */
    int whereIsTheDesiredCylinder(cv::Vec3f COLOR_TO_PLACE_BGR, cv::Mat image);

    /**
     * @brief function that sends a goal request to the assignment1 action server to reach a certain pose on the map either using move_base or
     * the motion control law and then move_base
     * 
     * @param ac action client that creates the goal request to send to the action server
     * @param x x coordinate to reach
     * @param y y coordinate to reach
     * @param yaw final orientation of the robot
     * @param control_law flag that specifies if the robot will use the motion control law to navigate through the narrow corridor or not
     * @return assignment1::InfoResult result of the action server
     */
    assignment1::InfoResult goToPosition(actionlib::SimpleActionClient<assignment1::InfoAction>& ac, float x, float y, float yaw, bool control_law);
};

/**
 * @brief function that prints the assignment1 action server feedback while it elaborates the action client goal request
 * 
 * @param feedback string message to send as feedback
 */
void feedbackCb(const assignment1::InfoFeedbackConstPtr& feedback);

/**
 * @brief function that prints the detection action server feedback while it elaborates the action client goal request
 * 
 * @param feedback string message to send as feedback
 */
void feedbackCb_detection(const assignment2::DetectionFeedbackConstPtr& feedback);

/**
 * @brief function that prints the detection action server feedback while it elaborates the action client goal request
 * 
 * @param feedback string message to send as feedback
 */
void feedbackCb_manipulation(const assignment2::ManipulationFeedbackConstPtr& feedback);
