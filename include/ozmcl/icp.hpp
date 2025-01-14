/*----------------------------------------------------------------------------
 /--\        ==   == ----- ==
| -- | ----- | \./ | |   | ||
| || | |   / |  |  | |     ||
| -- |   /   |  |  | |     ||
 \--/  /---| |  |  | ----| |====|
----------------------------------OzMCL---------------------------------------
icp.hpp
ICP - class for handling and generating point cloud data
------------------------------------------------------------------------------
*/
#ifndef OZMCL_ICP
#define PZMCL_ICP

#include "rclcpp/rclcpp.hpp"

#include "ozmcl/structs_n_stuff.hpp"

#include <vector>
#include <memory>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace OzMCL
{
    class ICP {
    public:
        /**
         * @brief Takes in an OccupancyGrid map and generates a point cloud of the border. 
         * @note map_res should have a length of 2, in width height. Points are in map space
         * @param logger pointer to logger
         * @param map occupancy map, represented by a int8 2x2 vector
         * @param n_subsample how many intervals to subsample by
         * @param map_res map resultion (pixels, cell count)
         * @param map_size map actual size (in meters)
         * @param origin origin offset of the map (meters)
         * @param point_cloud reference to point_cloud where the result will be stored    
         */
        static void generate_point_cloud(
            const rclcpp::Logger &logger, 
            std::vector<std::vector<int8_t>> map, 
            int n_subsample,
            int* map_res, float* map_size, float* origin,
            pcl::PointCloud<pcl::PointXYZ>& point_cloud
            );
        
        /**
         * @note Associate landmarks by using ICP. z should have already been transformed
         * into map space
         * @brief calculates the distance difference in landmarks, sqaures them, and returns their sum
         * @param pairs index pairs for a point in the source cloud and a point in the measurement cloud
         * @param z measurement cloud
         * @param aligned_cloud point cloud to store the aligned cloud (from pcl) to
         * @param reference_points target (ground truth, map) cloud
         * @returns the sum of squared distances if ICP converged; -1.0 otherwise (fail)
         */
        static double get_dist_squared(
            //const rclcpp::Logger& logger,
            std::vector<std::pair<int, int>>& pairs, 
            pcl::PointCloud<pcl::PointXYZ>& z,
            pcl::PointCloud<pcl::PointXYZ>& aligned_cloud, 
            pcl::PointCloud<pcl::PointXYZ>& reference_points
        );
    };
}

#endif