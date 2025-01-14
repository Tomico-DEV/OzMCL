/*----------------------------------------------------------------------------
 /--\        ==   == ----- ==
| -- | ----- | \./ | |   | ||
| || | |   / |  |  | |     ||
| -- |   /   |  |  | |     ||
 \--/  /---| |  |  | ----| |====|
----------------------------------OzMCL---------------------------------------
visualizer.cpp
source file for OzMCL's visualizer. See visualizer.hpp for more details
------------------------------------------------------------------------------
*/


#include "ozmcl/visualizer.hpp"

using namespace OzMCL;

void Visualizer::pointcloud2marker(
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
    std::string frame_id,
    int id,
    pcl::PointCloud<pcl::PointXYZ>& point_cloud, 
    visualization_msgs::msg::Marker& marker, 
    std_msgs::msg::ColorRGBA color
)
{
    marker.header.stamp = clock->now();
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.01; 
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color = color;

    std::vector<geometry_msgs::msg::Point> points;

    for (pcl::PointXYZ point : point_cloud.points)
    {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0f; // For 2D points, z is zero
        points.push_back(p);
    }

    marker.points = points;

    publisher->publish(marker);
}

void Visualizer::particles2marker(
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher,
    std::string frame_id,
    std::vector<OzMCL::x>& X
)
{
    geometry_msgs::msg::PoseArray poseArr;

    poseArr.header.stamp = clock->now();
    poseArr.header.frame_id = frame_id;
    
    for (OzMCL::x xm : X)
    {
        geometry_msgs::msg::Pose pose;
        
        pose.position.x = xm.x;
        pose.position.y = xm.y;
        pose.position.z = 0.05f;

        tf2::Quaternion q;
        q.setRPY(0, 0, xm.theta);

        pose.orientation.x = q.getX();
        pose.orientation.y = q.getY();
        pose.orientation.z = q.getZ();
        pose.orientation.w = q.getW();

        poseArr.poses.push_back(pose);
    }

    publisher->publish(poseArr);
}

void Visualizer::pose(
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher,
    std::string frame_id,
    OzMCL::x& pose
)
{
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.stamp = clock->now();
    pose_msg.header.frame_id = frame_id;

    pose_msg.pose.position.x = pose.x;
    pose_msg.pose.position.y = pose.y;
    pose_msg.pose.position.z = 0.05f;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);

    pose_msg.pose.orientation.x = q.getX();
    pose_msg.pose.orientation.y = q.getY();
    pose_msg.pose.orientation.z = q.getZ();
    pose_msg.pose.orientation.w = q.getW();

    publisher->publish(pose_msg);
}

void Visualizer::point_association(
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
    std::string frame_id,
    int id,
    pcl::PointCloud<pcl::PointXYZ>& source,
    pcl::PointCloud<pcl::PointXYZ>& target, // reference
    std::vector<std::pair<int, int>> pairs
)
{
    visualization_msgs::msg::Marker marker;
    
    marker.header.stamp = clock->now();
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.01f; // width of line
    marker.color.r = 1.0f;
    marker.color.g = 0.9f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration( 1, 0 );

    std::vector<geometry_msgs::msg::Point> points;

    for (std::pair<int, int> pair : pairs)
    {
        geometry_msgs::msg::Point p_start;
        p_start.x = source.points.at(pair.first).x;
        p_start.y = source.points.at(pair.first).y;

        geometry_msgs::msg::Point p_end;
        p_end.x = target.points.at(pair.second).x;
        p_end.y = target.points.at(pair.second).y;

        points.push_back(p_start);
        points.push_back(p_end);
    }

    marker.points = points;

    publisher->publish(marker);
}

