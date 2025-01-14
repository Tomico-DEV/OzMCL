/*----------------------------------------------------------------------------
 /--\        ==   == ----- ==
| -- | ----- | \./ | |   | ||
| || | |   / |  |  | |     ||
| -- |   /   |  |  | |     ||
 \--/  /---| |  |  | ----| |====|
----------------------------------OzMCL---------------------------------------
ozmcl_node.cpp
Source file for main OzMCL node. See ozmcl_node.hpp for more details
------------------------------------------------------------------------------
*/

#include "ozmcl/ozmcl_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief normal_distribution function
 */
float normal_distribution(float x, float mu, float variance)
{
    const float sqrt_2_pi = std::sqrt(2.0 * M_PI);
    return /*(1.0 / (sqrt(variance) * sqrt_2_pi)) * */ exp(-0.5 * std::pow((x - mu), 2) / variance);
}

/**
 * @brief generates particles at a specified mean with the specified variance
 * @param n number of particles to generate
 * @param mean location/center of particles
 * @param variance variance of particles
 */
void OzMCLNode::generate_particles(
    int n,
    OzMCL::x mean,
    OzMCL::x_var variance
)
{
    RCLCPP_INFO(get_logger(), "Generating %d particles", n);
    X_prev.clear();
    
    std::normal_distribution<float> x(mean.x, sqrt(variance.x));
    std::normal_distribution<float> y(mean.y, sqrt(variance.y));
    std::normal_distribution<float> theta(mean.theta, sqrt(variance.theta));

    for (int i = 0; i < n; i++)
    {
        X_prev.push_back(
            (OzMCL::x){ x(gen), y(gen), theta(gen) }
        );
    }
    
    OzMCL::Visualizer::particles2marker(
        get_clock(),
        particles_pose_publisher_,
        this->get_parameter("map_frame").as_string(),
        X_prev
    );
}

/**
 * @brief takes in a set of particles and returns a state based
 * on a density estimation of the particle cloud
 * @param X set of particles to perform density estimation on
 * @return estimated pose
 */
OzMCL::x OzMCLNode::pose_density_estimation(std::vector<OzMCL::x>& X)
{
    float x = 0.0f;
    float y = 0.0f;
    float heading_x = 0.0f;
    float heading_y = 0.0f;
    
    for (OzMCL::x particle : X)
    {
        x += particle.x;
        y += particle.y;

        heading_x += cos(particle.theta);
        heading_y += sin(particle.theta);
    }

    return (OzMCL::x){
        x / X.size(),
        y / X.size(),
        atan2f(heading_y, heading_x)
    };
}

/**
 * @brief updates the particle state using control (u)
 */
void OzMCLNode::predict()
{
    RCLCPP_INFO(get_logger(), "Predicting with variances %f %f %f", u_now.variance_x, u_now.variance_y, u_now.variance_theta);

    OzMCL::MCL::predict(X_prev, u_now, Xbar_now);
    
    state = OzMCL::STATE::CORRECTION;

    // update current predicted pose from particle cloud
    xbar_now = pose_density_estimation(Xbar_now);

    OzMCL::Visualizer::particles2marker(
        get_clock(),
        particles_pose_publisher_,
        this->get_parameter("map_frame").as_string(),
        Xbar_now
    );
}

/**
 * @brief calculates the variance of a data set of numbers
 * @param data the dataset. vector of doubles
 */
double calculate_variance(std::vector<double>& data)
{
    int n = 0;
    if (data.size() == 0)
    {
        return 0.0;
    }
    double mean = 0.0;
    for (double value : data)
    {
        if (value >= 0.0)
            mean += value;
            n++;
    }

    if (mean == 0.0)
        return 0.0;

    
    // Calculate the variance
    double variance = 0.0;
    for (double value : data) {
        if (value > 0.0)
            variance += (value - mean) * (value - mean);
    }
    variance /= n;

    // Return the standard deviation
    return variance;
}

/**
 * @brief gets the index of the smallest value that is not negative
 * in a vector containing the sum of distance squares
 * @note negative values are considered to be corrupted, and as such
 * will be skipped
 * @param sqdst vector of sum of distance squares 
 * @return index of the smallest value
 */
int get_i_smin(const std::vector<double>& sqdst)
{
    int i = 0;
    double min = -1.0;

    for (int n = 0; n < sqdst.size(); n++)
    {
        // if min is negative
        if (min < 0.0)
        {
            if (sqdst.at(n) >= 0.0)
            {
                min = sqdst.at(n);
                i = n;
            }
            else {
                continue;
            }
        }
        if (sqdst.at(n) < 0.0) continue; //can't be negative
        if (sqdst.at(n) < min)
        {
            min = sqdst.at(n);
            i = n;
        }
    }
    return i;
}

/**
 * @brief gets the index of the max value in a certain data set
 * @param data the data set. vector of doubles
 * @return index of the largest value
 */
int get_i_max(const std::vector<double>& data)
{
    int i = 0;
    double max = data.at(0);

    for (int n = 0; n < data.size(); n++)
    {
        if (data.at(n) > max)
        {
            max = data.at(n);
            i = n;
        }
    }
    return i;
}

/**
 * @brief corrects the predicted state generated by predict() using
 * measurements from "scan_topic"
 */
void OzMCLNode::correct()
{
    RCLCPP_INFO(get_logger(), "Correcting");

    X_now.clear();

    Weights_now.clear();
    std::vector<double> Squares_dist;

    // adding particle variety if moving
    if (abs(u_now.dx) > 0.0000001 or abs(u_now.dy) > 0.0000001 or abs(u_now.dtheta) > 0.0000001)
    {

        Xbar_now.resize(Xbar_now.size() - 10);
        std::normal_distribution<float> x(xbar_now.x, 0.1);
        std::normal_distribution<float> y(xbar_now.y, 0.1);
        std::normal_distribution<float> theta(xbar_now.theta, 0.3);

        for (int i = 0; i < 10; i++)
        {
            Xbar_now.push_back(
                (OzMCL::x){ x(gen), y(gen), theta(gen) }
            );
        }
    }
        
    for (OzMCL::x xm : Xbar_now)
    {
        // transform measurment from measurement frame into xm

        // Create the transformation matrix
        float x_translation = xm.x - xbar_now.x; 
        float y_translation = xm.y -xbar_now.y;
        float rotation_angle = OzMCL::MCL::angle_difference(xm.theta, xbar_now.theta);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << x_translation, y_translation, 0.0; 

        // Create a rotation matrix around the Z-axis
        Eigen::Matrix3f rotation_matrix = Eigen::AngleAxisf(rotation_angle, Eigen::Vector3f::UnitZ()).toRotationMatrix();

        // Combine translation and rotation
        transform.linear() = rotation_matrix;

        scan_data.width = scan_data.points.size();
        scan_data.height = 1;
        
        // Transform the point cloud
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud (scan_data, transformed_cloud, transform);

        // get fitness
        std::vector<std::pair<int, int>> pairs;
        pcl::PointCloud<pcl::PointXYZ> aligned_clouds( 3.0, 3.0 );

        double sum_square_dist = OzMCL::ICP::get_dist_squared(
            //this->get_logger(),
            pairs,
            transformed_cloud,
            aligned_clouds,
            reference_points
        );

        //RCLCPP_INFO(get_logger(), "Fitness: %f", sum_square_dist);
        
        if (sum_square_dist >= 0.0 and sum_square_dist / pairs.size() < 1) {
            Squares_dist.push_back(sum_square_dist);
        } else {
            // data is corrupt
            Squares_dist.push_back(-1.0);
        }
    }

    double square_dist_variance = calculate_variance(Squares_dist);


    // calculate weights of each particle, using some of square distances for each
    // scan point in the particle's frame
    Weights_now.clear();
    for (double sum_square_dist : Squares_dist)
    {
        if (sum_square_dist >= 0.0)
        {
            Weights_now.push_back(
                
            OzMCL::MCL::p_measurement(
                sum_square_dist, 
                square_dist_variance
                )
            );
        } else {
            Weights_now.push_back(0.0);
        }
    }

    if (Weights_now.size() > 0)
    {   
        if (Weights_prev.empty())
        {
            Weights_prev = std::vector<double>(Weights_now.size(), 1.0);
        }

        if (abs(u_now.dx) > 0.0000001 or abs(u_now.dy) > 0.0000001 or abs(u_now.dtheta) > 0.0000001)
        {
            // moving, resample
            RCLCPP_INFO(get_logger(), "Resampling");
            OzMCL::MCL::importance_sample(
                Xbar_now,
                Weights_now,
                X_now
            );
        } else {
            // still, accumulate weights
            for (int i = 0; i < Weights_now.size(); i++)
            {
                Weights_now.at(i) *= Weights_prev.at(i);
            }
            X_now = Xbar_now;
        }
    } else {
        // no associations made
        X_now = Xbar_now;
    }
   
    X_prev = X_now;

    x_now = pose_density_estimation(X_now);
    
    this->state = OzMCL::STATE::PREDICTION;

    OzMCL::Visualizer::particles2marker(
        get_clock(),
        particles_pose_publisher_,
        this->get_parameter("map_frame").as_string(),
        X_now
    );
}

/**
 * @brief send pose tf (from "map_frame" to "robot_frame")
 */
void OzMCLNode::send_pose()
{
    //RCLCPP_INFO(this->get_logger(), "Setting pose");
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x_now.theta);
    
    geometry_msgs::msg::TransformStamped t;
    
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = this->get_parameter("map_frame").as_string();
    t.child_frame_id = this->get_parameter("robot_frame").as_string();
    
    t.transform.translation.x = x_now.x;
    t.transform.translation.y = x_now.y;
    t.transform.translation.z = 0.0f;
    t.transform.rotation.x = q.getX();
    t.transform.rotation.y = q.getY();
    t.transform.rotation.z = q.getZ();
    t.transform.rotation.w = q.getW();

    pose_tf_broadcaster_->sendTransform(t);

}

/**
 * @brief callback to send markers to rviz
 */
void OzMCLNode::send_marker()
{
    RCLCPP_INFO(this->get_logger(), 
        "Sending marker with %ld points",
        this->reference_points.points.size());
    // publish markers to rviz
    visualization_msgs::msg::Marker marker_msg;

    builtin_interfaces::msg::Duration lifetime;
    lifetime.sec = 1;
    lifetime.nanosec = 0;

    std_msgs::msg::ColorRGBA cyan;
    cyan.g = 1.0f;
    cyan.b = 0.75f;
    cyan.a = 1.0f;
    

    marker_msg.lifetime = lifetime;
    OzMCL::Visualizer::pointcloud2marker(
        this->get_clock(), this->map_marker_publisher_,
        get_parameter("map_frame").as_string(),
        1,
        this->reference_points, 
        marker_msg,
        cyan
    );
}

/**
 * @brief laser scan callback
 */
void OzMCLNode::scan_sub_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "scan_callback");
    if (state != OzMCL::STATE::CORRECTION)
    {
        return;
    }

    scan_data.clear();

    tf2::Quaternion q;
    q.setRPY( 0.0f, 0.0f, x_now.theta );

    tf2::Transform t_movbase_to_map(
        q, 
        tf2::Vector3( xbar_now.x, xbar_now.y, 0.0f )
    );

    for (int i = 0; i < msg->ranges.size(); i += this->get_parameter("n_scan_subsample").as_int())
    {
        // check for corrupt data
        if (msg->ranges[i] < msg->range_min) continue;
        if (msg->ranges[i] > msg->range_max) continue;

        // relative to +x
        float theta = (i * msg->angle_increment + msg->angle_min);

        // polar to cartesian
        // vector in lidar space
        tf2::Vector3 point_coords(
            msg->ranges[i] * cos(theta),
            msg->ranges[i] * sin(theta),
            0.0f
        );


        // now we transform into map space
        if (msg->header.frame_id == "lidar_1")
        {
            point_coords = t_movbase_to_map * lidar_1_tf * point_coords;
        }
        else if (msg->header.frame_id == "lidar_2")
        {
            point_coords = t_movbase_to_map * lidar_2_tf * point_coords;
        }

        // movbase space to map space
        //point_coords = t_movbase_to_map * point_coords;

        // add to point cloud  
        scan_data.points.push_back(
            pcl::PointXYZ(point_coords.x(), point_coords.y(), 0.0f)
        );
    }

    // visualize
    RCLCPP_INFO(this->get_logger(), "Sending scan of %d points", this->scan_data.size());
    // publish markers to rviz
    visualization_msgs::msg::Marker marker_msg;
    
    builtin_interfaces::msg::Duration lifetime;
    lifetime.sec = 0;
    lifetime.nanosec = 200000;

    std_msgs::msg::ColorRGBA scan_color;
    scan_color.r = 1.0f;
    scan_color.a = 1.0f;
    

    marker_msg.lifetime = lifetime;
    OzMCL::Visualizer::pointcloud2marker(
        this->get_clock(), this->map_marker_publisher_,
        get_parameter("map_frame").as_string(),
        2,
        this->scan_data, 
        marker_msg,
        scan_color
    );

    this->correct();
}

/**
 * @brief odometry subscription callback
 */
void OzMCLNode::odom_sub_callback(nav_msgs::msg::Odometry::UniquePtr msg)
{
    if (this->state != OzMCL::STATE::PREDICTION)
    {
        return;
    }
    
    u_now.dx = msg->pose.pose.position.x - odom_prev.x;
    u_now.dy = msg->pose.pose.position.y - odom_prev.y;

    tf2::Transform transform;
    tf2::fromMsg(msg->pose.pose, transform);

    double _roll, _pitch, yaw;
    transform.getBasis().getRPY(_roll, _pitch, yaw);

    u_now.theta = yaw;
    u_now.dtheta = yaw - odom_prev.theta;

    odom_prev.x = msg->pose.pose.position.x;
    odom_prev.y = msg->pose.pose.position.y;
    odom_prev.theta = yaw;

    // set control variances
    u_now.variance_x = 
        abs(u_now.dx * std::stof(this->get_parameter("xy_variance_per_m").as_string()));
    u_now.variance_y = abs(u_now.dy * std::stof(this->get_parameter("xy_variance_per_m").as_string()));
    u_now.variance_theta = abs(u_now.dtheta * std::stof(this->get_parameter("angular_variance_per_rad").as_string()))
    + abs(std::stof(this->get_parameter("angular_variance_per_m").as_string()) * u_now.dy) + abs(std::stof(this->get_parameter("angular_variance_per_m").as_string()) * u_now.dx);
    
    Xbar_now.clear();
    predict();
}

// map service callback; used to generate boundary points
void OzMCLNode::map_service_callback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture response)
{  
    if (response.get())
    {
        auto msg = response.get()->map;
        this->map_res[0] = msg.info.width;
        this->map_res[1] = msg.info.height;
        this->map_size[0] = float(this->map_res[0]) * msg.info.resolution;
        this->map_size[1] = float(this->map_res[1]) * msg.info.resolution;
        this->map_origin[0] = msg.info.origin.position.x;
        this->map_origin[1] = msg.info.origin.position.y;
        RCLCPP_INFO(this->get_logger(), 
            "Got map of resolution %d %d pixels, size %f %f meters, origin at %f %f",
            this->map_res[0], this->map_res[1], this->map_size[0], this->map_size[1],
            this->map_origin[0], this->map_origin[1]
        );
        std::vector<int8_t> map_raw_data = msg.data;
        for (int i = 0; i < this->map_res[1]; i++)
        {
            this->map_data.push_back(
                std::vector<int8_t>(
                    map_raw_data.begin() + i * this->map_res[1], 
                    map_raw_data.begin() + (i + 1) * this->map_res[1]
                    )
                );
        }
        
        OzMCL::ICP::generate_point_cloud(
            get_logger(), 
            map_data, 
            get_parameter("n_map_subsample").as_int(),
            this->map_res, this->map_size, this->map_origin,
            this->reference_points
            );
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get a response from the map server.");
    }
}

void OzMCLNode::declare_parameters()
{
    // ROS2
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("map_server", "/map_server");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("scan_topic", "/scan");

    // self localization    
    this->declare_parameter("n_particles", 75);
    this->declare_parameter("n_scan_subsample", 2);
    this->declare_parameter("n_map_subsample", 4);
    this->declare_parameter("start_x", "0.0");
    this->declare_parameter("start_y", "0.0");
    this->declare_parameter("start_theta", "0.0");
    this->declare_parameter("start_pos_variance", "0.0025");
    this->declare_parameter("start_angular_variance", "0.03046176808");
    this->declare_parameter("xy_variance_per_m", "0.01");
    this->declare_parameter("angular_variance_per_rad", "0.2");
    this->declare_parameter("angular_variance_per_m", "0.1");
    this->declare_parameter("lidar_variance", "0.03");
}

void OzMCLNode::get_map()
{
    map_client_ = this->create_client<nav_msgs::srv::GetMap>(this->get_parameter("map_server").as_string() + "/map");

    while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
        if ( !rclcpp::ok() ) 
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for map service...");
    }

    auto map_request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto map_future = map_client_->async_send_request(map_request, std::bind(&OzMCLNode::map_service_callback, this, std::placeholders::_1));

    auto map_lifecycle_callback = [this, map_request](lifecycle_msgs::msg::TransitionEvent::UniquePtr msg) -> void
    {
        if (msg->goal_state.label == "active")
        {
            this->map_client_->async_send_request(map_request, std::bind(&OzMCLNode::map_service_callback, this, std::placeholders::_1));
        }
    };

    map_lifecycle_subscription_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        this->get_parameter("map_server").as_string() + "/transition_event", 10, map_lifecycle_callback
    );
}

/**
 * @brief get transforms from "robot_frame" to "lidar_1" and "lidar_2"
 * only works with team A's robot. Too lazy to fix 
 */
void OzMCLNode::get_tfs()
{
    pose_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    tf2_ros::CreateTimerInterface::SharedPtr cti1 = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface()
    );

    tf2_ros::CreateTimerInterface::SharedPtr cti2 = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface()
    );

    tf_buffer_1.setCreateTimerInterface(cti1);
    tf_buffer_2.setCreateTimerInterface(cti2);

    // get transforms for lidar_1 and lidar_2 to movebase
    tf_buffer_1.waitForTransform(
        "movebase",
        "lidar_1",
        tf2::TimePointZero,
        std::chrono::minutes(2min),
        [this](const std::shared_future<geometry_msgs::msg::TransformStamped> & tf) 
        {
            try {
                geometry_msgs::msg::Quaternion quat = tf.get().transform.rotation;
                geometry_msgs::msg::Vector3 trans = tf.get().transform.translation;

                tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
                tf2::Vector3 tf_origin(trans.x, trans.y, trans.z);

                lidar_1_tf.setOrigin(tf_origin);
                lidar_1_tf.setRotation(tf_quat);
                RCLCPP_INFO(get_logger(), "lidar_1 transform OK, bassis %f %f %f",
                    lidar_1_tf.getOrigin().x(), lidar_1_tf.getOrigin().y(), lidar_1_tf.getOrigin().z());
            } catch (tf2::TimeoutException & e)
            {
                RCLCPP_ERROR(get_logger(), "couldn't get lidar_1's tf innit bruv");
            }
        }
        );
    tf_buffer_2.waitForTransform(
        "movebase",
        "lidar_2",
        tf2::TimePointZero,
        std::chrono::minutes(2min),
        [this](const std::shared_future<geometry_msgs::msg::TransformStamped> & tf) 
        {
            try {
                geometry_msgs::msg::Quaternion quat = tf.get().transform.rotation;
                geometry_msgs::msg::Vector3 trans = tf.get().transform.translation;

                tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
                tf2::Vector3 tf_origin(trans.x, trans.y, trans.z);

                lidar_2_tf.setOrigin(tf_origin);
                lidar_2_tf.setRotation(tf_quat);
                RCLCPP_INFO(get_logger(), "lidar_2 transform OK");
            } catch (tf2::TimeoutException & e)
            {
                RCLCPP_ERROR(get_logger(), "couldn't get lidar_2's tf innit bruv");
            }
        }
        );
}

void OzMCLNode::create_publishers()
{
    map_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("map_boundary", 10);
    scan_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("transformed_scan", 10);
    association_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("points_association", 10);
    particles_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", 10);
    test_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("test_pose", 10);
}

void OzMCLNode::create_subscriptions()
{
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odom_topic").as_string(), 10, std::bind(&OzMCLNode::odom_sub_callback, this, std::placeholders::_1)
    );

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        this->get_parameter("scan_topic").as_string(), 10, std::bind(&OzMCLNode::scan_sub_callback, this, std::placeholders::_1)
    );
}

OzMCLNode::OzMCLNode() 
: Node("ozmcl"), 
tf_buffer_1(this->get_clock()), tf_listener_1(tf_buffer_1),
tf_buffer_2(this->get_clock()), tf_listener_2(tf_buffer_2),
gen(std::random_device{}())
{
    declare_parameters();

    x_now.x = std::stof(this->get_parameter("start_x").as_string());
    x_now.y = std::stof(this->get_parameter("start_y").as_string());
    x_now.theta = std::stof(this->get_parameter("start_theta").as_string());

    xbar_now = x_now;

    RCLCPP_INFO(get_logger(), "x_now: %f %f %f", x_now.x, x_now.y, x_now.theta);

    state = OzMCL::STATE::PREDICTION;

    // request for the map
    get_map();
    create_publishers();
    create_subscriptions();
    get_tfs();
    
    
    timer_ = this->create_wall_timer(10ms, std::bind(&OzMCLNode::send_pose, this));
    viz_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&OzMCLNode::send_marker, this));

    generate_particles(
        this->get_parameter("n_particles").as_int(), 
        (OzMCL::x){ 
            std::stof(this->get_parameter("start_x").as_string()),
            std::stof(this->get_parameter("start_y").as_string()),
            std::stof(this->get_parameter("start_theta").as_string())    
        },  
        (OzMCL::x_var){
            std::stof(this->get_parameter("start_pos_variance").as_string()),
            std::stof(this->get_parameter("start_pos_variance").as_string()),
            std::stof(this->get_parameter("start_angular_variance").as_string())
        }
        
    );
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OzMCLNode>());
    rclcpp::shutdown();
    return 0;
}