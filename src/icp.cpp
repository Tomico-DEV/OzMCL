/*----------------------------------------------------------------------------
 /--\        ==   == ----- ==
| -- | ----- | \./ | |   | ||
| || | |   / |  |  | |     ||
| -- |   /   |  |  | |     ||
 \--/  /---| |  |  | ----| |====|
----------------------------------OzMCL---------------------------------------
icp.cpp
Source file for handling scan data. See icp.hpp for more details
------------------------------------------------------------------------------
*/


#include "ozmcl/icp.hpp"

using namespace OzMCL;

void ICP::generate_point_cloud(
    const rclcpp::Logger &logger,
    std::vector<std::vector<int8_t>> map, 
    int n_subsample,
    int* map_res, float* map_size, float* origin,
    pcl::PointCloud<pcl::PointXYZ>& point_cloud
    )
{
    RCLCPP_INFO(logger, "Generating map boundary...");

    float cell_size = map_size[0] / (float)map_res[0];
    RCLCPP_INFO(logger, "Cell size is %f meters per cell", cell_size);

    int n = 0;
    for (int i = 1; i < map_res[0] - 1; i++)
    {
        for (int j = 1; j < map_res[1] - 1; j++)
        {
            // Generates points at corners of cells
            // Coords are measured from top-left
            // Map is assumed to be centered
            
            
            // Checks are applied to vacant cells
            // If occupied, skip

            /*
                Image coord frame:
                ----> + x
                |
                |
                V
                -y


                Ros coord frame:
                        +x
                        ^
                        |
                        |
                +y <----
            
            */ 

            if (map[i][j] > 75) continue;
            
            // left
            if (map[i - 1][j] > 75)
            {
                n++;
                pcl::PointXYZ p;
                p.y = i * cell_size + origin[0];
                p.x = ((float)j + 0.5f) * cell_size + origin[1];
                point_cloud.points.push_back(p);
            }

            // right
            if (map[i + 1][j] > 75)
            {
                n++;
                pcl::PointXYZ p;
                p.y = (i + 1) * cell_size + origin[0];
                p.x = ((float)j + 0.5f) * cell_size + origin[1];
                point_cloud.points.push_back(p);
            }

            // top
            if (map[i][j - 1] > 75)
            {
                n++;
                pcl::PointXYZ p;
                p.y = ((float)i + 0.5f) * cell_size + origin[0];
                p.x =  j * cell_size + origin[1];
                point_cloud.points.push_back(p);
            }

            // bottom
            if (map[i][j + 1] > 75)
            {
                n++;
                pcl::PointXYZ p;
                p.y = ((float)i + 0.5f) * cell_size + origin[0];
                p.x = (j + 1) * cell_size + origin[1];
                point_cloud.points.push_back(p);
            }
        }
    }

    // subsample
    RCLCPP_INFO(logger, "Generation done. Made %d points", n);

    RCLCPP_INFO(logger, "Sampling by %d...", n_subsample);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(point_cloud.points.begin(), point_cloud.points.end(), gen); 
    point_cloud.points.resize((int)(point_cloud.points.size() / n_subsample)); 
}

double ICP::get_dist_squared(
    std::vector<std::pair<int, int>>& pairs, 
    pcl::PointCloud<pcl::PointXYZ>& z, 
    pcl::PointCloud<pcl::PointXYZ>& aligned_cloud,
    pcl::PointCloud<pcl::PointXYZ>& reference_points
    )
{
    // First, we will use ICP to align the measurement cloud to the reference cloud
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(15);

    auto ref_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(reference_points);
    auto z_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(z);

    icp.setInputSource(z_ptr);
    icp.setInputTarget(ref_cloud_ptr);

    icp.align(aligned_cloud);
    
    if (!icp.hasConverged())
    {
        //RCLCPP_WARN(logger, "ICP did not converge. Robot delocalized?");
        return -1.0f;
    } 

    // Now we associate the points using closest neighbors
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(ref_cloud_ptr);

    for (int i = 0; i < aligned_cloud.points.size(); i++) {
        pcl::PointXYZ search_point = aligned_cloud.points[i];

        // Perform nearest neighbor search
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        if (kdtree.nearestKSearch(search_point, 1, indices, distances) > 0) {
            if (distances[0] < 0.05) // if distance difference is less than 5 cm, associate
            {
                pairs.emplace_back(i, indices[0]);
            }
        }
    }
    
    // sum of squared distances
    int n_associated = pairs.size();
    float ratio_associated = n_associated / (float)z.points.size();
    if (ratio_associated > 0.7) // associate only if % of associations > 70%
    {   
        //RCLCPP_INFO(logger, "%d points associated!", n_associated);
        double accum_dist = 0.0;
        for (std::pair<int, int> pair : pairs)
        {
            accum_dist +=
                pow(z.points.at(pair.first).x - reference_points.points.at(pair.second).x, 2)
                + pow(z.points.at(pair.first).y - reference_points.points.at(pair.second).y, 2) +
                + pow(z.points.at(pair.first).z - reference_points.points.at(pair.second).z, 2)
            ;
        } 
        return accum_dist;
    }

    // not enough points associated

    //RCLCPP_WARN(logger, "Too few points were associated!");
    return -1.0;
}
