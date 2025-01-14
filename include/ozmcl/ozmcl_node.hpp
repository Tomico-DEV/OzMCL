/*----------------------------------------------------------------------------
 /--\        ==   == ----- ==
| -- | ----- | \./ | |   | ||
| || | |   / |  |  | |     ||
| -- |   /   |  |  | |     ||
 \--/  /---| |  |  | ----| |====|
----------------------------------OzMCL---------------------------------------
ozmcl_node.hpp
ROS2 node for OzMCL

Theory: 
The node starts in the "PREDICTION" state and listens for a message from 
"odom_topic". 'u' (the control) is taken from "odom_topic" and used to predict 
the present predicted particle state 'Xbar_now'. After a prediction has been 
made, the node swithces into the "CORRECTION" state and listens for a message 
from "scan_topic". Once a message is received, 'scan_data' (the measurement) is
updated and a correction is made, with the results stored in 'X_now'. The 
centroid of X_now is taken and published as the robot's corrected pose in the 
specified transform frame ("robot_frame")

Input: /scan, /odom
Output: tf from "map" to "robot_frame"

parameters:

--NODE PARAMS--

"scan_topic": default "/scan"
    topic containing data (measurements) from lidar
"odom": default "/odom"
    odometry topic
"map_server": default "/scan"
    map server namespace
"map_frame": default "map"
    map transform frame
"n_map_subsample": default 4
    interval of map subsampling. Resulting number of scan points becomes 
    (n_map_points) / n_map_subsample
"robot_frame": default "base_link"
    robot transform frame.

--MCL PARAMS--

"n_particles": default 75
    number of particles to represent state probability
"start_x": default 0
    starting x position (in map frame)
"start_y": default 0
    starting y position (in map frame)
"start_theta": default 0
    starting orientation (in map frame, rel to +x)
"start_pos_variance": 0.0025
    variance for expected translational deviation from specified starting 
    coordinates
"start_angular_variance"L 0.03046176808
    variance for expected angular deviation from specified starting coordinates
     
--ODOMETRY PARAMS--

"angular_variance_per_m": default 0.1
    expected angular variance (radians^2) per meter of translational movement
"angular_variance_per_rad": default 0.2
    expected angular variance (radians^2) per radian of rotational movement
"xy_variance_per_m": default 0.01
    expected translational variance (meters^2) per meter of translational 
    movement

--LIDAR PARAMS--

"n_scan_subsample": default 2
    interval of beamskip. Resulting number of scan points becomes 
    (scan_points) / n_scan_subsample
"lidar_variance": default 0.03
    variance expected from lidar data

------------------------------------------------------------------------------
*/

#ifndef OZMCL_NODE_HPP_
#define OZMCL_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <random>

#include <csignal>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

#include "ozmcl/structs_n_stuff.hpp"
#include "ozmcl/visualizer.hpp"
#include "ozmcl/icp.hpp"
#include "ozmcl.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

class OzMCLNode : public rclcpp::Node
{
public:
    explicit OzMCLNode();
private:
    //------------------------ROS OVERHEAD---------------------------

    rclcpp::TimerBase::SharedPtr    timer_;
    rclcpp::TimerBase::SharedPtr    viz_timer_;
    rclcpp::TimerBase::SharedPtr    test_timer_;

    tf2_ros::Buffer                 tf_buffer_1;
    tf2_ros::TransformListener      tf_listener_1;

    tf2_ros::Buffer                 tf_buffer_2;
    tf2_ros::TransformListener      tf_listener_2;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr            scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                odom_subscription_;
    rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr   map_lifecycle_subscription_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   map_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   scan_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   association_marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     particles_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   test_pose_publisher_;
    
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> pose_tf_broadcaster_;


    void declare_parameters();
    void create_publishers();
    void create_subscriptions();
    
    void get_map();
    void get_tfs();
    

    void odom_sub_callback    (nav_msgs::msg::Odometry::UniquePtr msg);
    void scan_sub_callback    (sensor_msgs::msg::LaserScan::UniquePtr msg);
    void map_service_callback (rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture response);

    void send_pose();
    void send_marker();

    //----------------------------MATH------------------------------

    // all of these vectors are assumed to be the same size at runtime
    std::vector<OzMCL::x> X_prev;
    std::vector<OzMCL::x> Xbar_now;
    std::vector<OzMCL::x> X_now;
    
    std::vector<double> Weights_now;
    std::vector<double> Weights_prev;

    OzMCL::x odom_prev;

    OzMCL::u u_now;
    OzMCL::x xbar_now; 
    OzMCL::x x_now;


    OzMCL::x pose_density_estimation(std::vector<OzMCL::x>& X);

    void predict();
    void correct();
    
    //-------------------------UTIL---------------------

    std::default_random_engine gen;

    
    pcl::PointCloud<pcl::PointXYZ> scan_data;
    pcl::PointCloud<pcl::PointXYZ> reference_points;

    int                              map_res[2];
    float                            map_size[2];
    float                            map_origin[2];
    std::vector<std::vector<int8_t>> map_data;

    
    tf2::Transform lidar_1_tf;
    tf2::Transform lidar_2_tf;

    geometry_msgs::msg::PoseWithCovarianceStamped prev_odom;

    OzMCL::STATE state = OzMCL::STATE::PREDICTION;

    void generate_particles(
        int n,
        OzMCL::x mean,
        OzMCL::x_var variance
    );
};


#endif