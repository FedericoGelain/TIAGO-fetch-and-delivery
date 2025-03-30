#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment1/InfoAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/Twist.h"

#include "boost/ref.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

class InfoAction{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<assignment1::InfoAction> as_;
  std::string action_name_;
  assignment1::InfoFeedback feedback_;
  assignment1::InfoResult result_;
  
  // Global vector for color map
  std::vector<cv::Scalar> colorMap = {
    cv::Scalar(0, 0, 255),   // 0: Red
    cv::Scalar(0, 255, 0),   // 1: Green
    cv::Scalar(255, 0, 0),   // 2: Blue
    cv::Scalar(0, 255, 255), // 3: Yellow
    cv::Scalar(255, 0, 255), // 4: Magenta
    cv::Scalar(255, 255, 0), // 5: Cyan
    cv::Scalar(0, 0, 128),   // 6: Dark Red
    cv::Scalar(0, 128, 0),   // 7: Dark Green
    cv::Scalar(128, 0, 0),   // 8: Navy
    cv::Scalar(0, 128, 128), // 9: Olive
    cv::Scalar(128, 0, 128), // 10: Purple
    cv::Scalar(128, 128, 0), // 11: Teal
    cv::Scalar(128, 128, 128),// 12: Gray
    cv::Scalar(192, 192, 192),// 13: Silver
    cv::Scalar(0, 165, 255),  // 14: Orange
    cv::Scalar(42, 42, 165),  // 15: Brown
    cv::Scalar(128, 128, 0),  // 16: Olive (128)
    cv::Scalar(127, 255, 0),  // 17: Spring Green
    cv::Scalar(0, 215, 255),  // 18: Gold
    cv::Scalar(0, 69, 255)    // 19: Red-Orange
  };
  
  // Global vector for mapping colors to their names
  std::vector<std::string> colorName = {
  	"Red",
  	"Green",
  	"Blue",
  	"Yellow",
  	"Magenta",
  	"Cyan",
  	"Dark Red",
  	"Dark Green",
  	"Navy",
  	"Olive",
  	"Purple",
  	"Teal",
  	"Gray",
  	"Silver",
  	"Orange",
  	"Brown",
  	"Olive",
  	"Spring Green",
  	"Gold",
  	"Orange"
  };


public:
  InfoAction(std::string name) :
      as_(nh_, name, boost::bind(&InfoAction::executeCB, this, _1), false),
      action_name_(name)
  {
    as_.start();
  }
  ~InfoAction(void){}

  	// Structure to store all needed information for each point during the clustering process
	struct Point {
	    double x, y; // cartesian coordinates of the point
	    bool visited; // flag that indicates if the point has been visited or not during the clustering process
	    int clusterId; // ID of the cluster the point has been assigned to

		//constructor to assign the corresponding coordinates of the point (plus the default values needed for clustering)
	    Point(double x, double y) : x(x), y(y), visited(false), clusterId(-1) {}
	};

	/**
	 * @brief Function that returns the euclidean distance between the two given points
	 * @param point1 first point
	 * @param point2 second point
	 * @return the euclidean distance between first and second point
	 */
	double euclideanDistance(const Point& point1, const Point& point2);

	/**
	 * @brief Function that returns the indices of the neighbors of the given input point
	 * @param center input point
	 * @param points all points
	 * @param epsilon euclidean distance threshold
	 * @return vector that contains all the indices of the neighbors of the input point
	 */
	std::vector<int> computeNeighbours(const Point& center, const std::vector<Point>& points, double epsilon);
		
		
	/**
	 * @brief Function to calculate the variance of points within a cluster
	 * @param cluster the cluster of points
	 * @return the variance calculated
	 */
	double calculateClusterVariance(const std::vector<Point>& cluster);
	
	
	/**
	 * @brief Function to build the clusters given all points
	 * @param points the vector of all points
	 * @return the vector containing all clusters (vectors of points)
	 */
	std::vector<std::vector<Point>> buildClusters(const std::vector<Point>& points);
	
	/**
	 * @brief Function to save the plot of the clusters in an image
	 * @param clusters the vector of clusters (vectors of points)
	 * @param imageSize the image size
	 * @param filename the path where to save the image
	 * @return
	 */
	void plotClusters(const std::vector<std::vector<Point>>& clusters, const cv::Size& imageSize, const std::string& filename);
	
	/**
	 * Function to detect the obstacles using global thresholding on the cluster variances
	 * @param variances the vector of variances calculated for each cluster
	 * @param threshold the global threshold on the variances
	 * @return the vector of indices corresponding to the clusters which are obstacles
	 */
	std::vector<int> obstaclesDetectionThresholding(const std::vector<double>& variances, const double& threshold);
	
	/**
     * @brief Callback function to retrieve the current pose of the robot and determine if it reached the end of the corridor or not 
     * @param msg pointer to the message written in the /robot_pose topic
     * @param end_corridor flag that is true if the robot has reached the end of the corridor, false otherwise
     */
	void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, bool& end_corridor);

	/**
	 * @brief Callback function to analyze the data registered by the laser
	 * @param msg pointer to the message written in the /scan topic
	 * @param res reference to the InfoAction result, to write the positions of the objects detected
	 */			 
	void scanCallback(const sensor_msgs::LaserScanConstPtr& msg, assignment1::InfoResult& res);

	/**
	 * @brief Callback function to analyze the data registered by the laser using the motion control law
	 * @param msg pointer to the message written in the /scan topic
	 * @param res reference to the InfoAction result, to write the positions of the objects detected
	 * @param goal_angle current angle of the goal position
	 * @param z_rot_clockwise current clockwise angular velocity
	 * @param z_rot_counterclockwise current counterclockwise angular velocity
	 */			 
	void scanWithCMDCallback(const sensor_msgs::LaserScanConstPtr& msg, assignment1::InfoResult& res, float& goal_angle, float& z_rot_clockwise, float& z_rot_counterclockwise);

    /**
     * @brief function that handles the motion control law to move the robot through the corridor
     * @param kp proportional gain: to drive the orientation in the direction of the goal
     * @param kd derivative gain: to reduce the sudden change in the angular velocity
     */
    void cmd_vel_controller(float kp, float kd);
	
    /**
     * @brief Callback function that handles the communication between InfoAction server and client
     * @param goal pointer to the InfoAction goal, where all the information sent by the client is stored
     */
    void executeCB(const assignment1::InfoGoalConstPtr &goal);
};

