/*----------------------------------------------------------------------------
 /--\        ==   == ----- ==
| -- | ----- | \./ | |   | ||
| || | |   / |  |  | |     ||
| -- |   /   |  |  | |     ||
 \--/  /---| |  |  | ----| |====|
----------------------------------OzMCL---------------------------------------
visualizer.hpp
Visualizer - class with methods to send marker messages and etc to RViZ
to better visualizer data
------------------------------------------------------------------------------
*/

#ifndef OZMCL_VIZ
#define OZMCL_VIZ

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include "tf2/LinearMath/Quaternion.hpp"

#include "ozmcl/structs_n_stuff.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace OzMCL
{
    class Visualizer 
    {
    public:
        /**
         * @brief Publishes a point cloud as a marker
         */
        static void pointcloud2marker(
            rclcpp::Clock::SharedPtr clock,
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
            std::string frame_id,
            int id,
            pcl::PointCloud<pcl::PointXYZ>& point_cloud, 
            visualization_msgs::msg::Marker& marker, 
            std_msgs::msg::ColorRGBA color
        );

         /**
         * @brief Publishes particles (a collection of states) as a marker
         */
        static void particles2marker(
            rclcpp::Clock::SharedPtr clock,
            rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher,
            std::string frame_id,
            std::vector<OzMCL::x>& X
        );

        /**
         * @brief Publishes the pose of a state
         */
        static void pose(
            rclcpp::Clock::SharedPtr clock,
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher,
            std::string frame_id,
            OzMCL::x& pose
        );

        /**
         * @brief Publishes the "association" (line list) for two point clouds.
         */
        static void point_association(
            rclcpp::Clock::SharedPtr clock,
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
            std::string frame_id,
            int id,
            pcl::PointCloud<pcl::PointXYZ>& source,
            pcl::PointCloud<pcl::PointXYZ>& target, // reference
            std::vector<std::pair<int, int>> pairs
        );
    };
}

#endif