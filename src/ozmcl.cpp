#include "ozmcl/ozmcl.hpp"

using namespace OzMCL;

OzMCL::x MCL::g(OzMCL::x x_prev, OzMCL::u u_t)
{
    OzMCL::x x_t;
    x_t.x = x_prev.x + u_t.dx * cos(x_prev.theta - u_t.theta) - u_t.dy * sin(x_prev.theta - u_t.theta);
    x_t.y = x_prev.y + u_t.dx * sin(x_prev.theta - u_t.theta) + u_t.dy * cos(x_prev.theta - u_t.theta);
    x_t.theta = x_prev.theta + u_t.dtheta;

    return x_t;
}

void MCL::predict(
    std::vector<OzMCL::x>& X_prev, 
    OzMCL::u u, 
    std::vector<OzMCL::x>& Xbar_now
)
{
    std::default_random_engine gen(std::random_device{}());
    std::normal_distribution<float> dx_prime(u.dx, sqrt(u.variance_x));
    std::normal_distribution<float> dy_prime(u.dy, sqrt(u.variance_y));
    std::normal_distribution<float> dtheta_prime(u.dtheta, sqrt(u.variance_theta));

    Xbar_now.clear();

    for (OzMCL::x x_prev : X_prev)
    {
        OzMCL::u u_prime;
        // add noise to our control
        u_prime.dx = dx_prime(gen);
        u_prime.dy = dy_prime(gen);
        u_prime.dtheta = dtheta_prime(gen);

        // predict
        Xbar_now.push_back(g(x_prev, u_prime));
    }
}

double MCL::angle_difference(double angle1, double angle2) {
    double diff = angle2 - angle1;
    while (diff > M_PI) {
        diff -= 2 * M_PI;
    }
    while (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return diff;
}

double normal_distribution(double x, double mu, double variance)
{
    const double sqrt_2_pi = sqrt(2.0 * M_PI);
    return (1.0 / (sqrt(variance) * sqrt_2_pi)) * exp(-0.5 * pow((x - mu), 2) / variance);
}

/*
double MCL::p_measurement(OzMCL::z z, OzMCL::x x_m)
{
    // get diff in pose and its probability
    double dist_diff = sqrt(pow(z.x - x_m.x, 2) + pow(z.y - x_m.y, 2));
    double angle_diff = angleDifference(z.theta, x_m.theta);

    return (
        normal_distribution(dist_diff, 0.0f, z.variance_dist) 
        * normal_distribution(angle_diff, 0.0f, z.variance_angle)
    );
}
*/

double MCL::p_measurement(
    double sum_square_dist,
    double variance_sum_squares
)
{
    /*
    // transform measurment from reference frame into xm
    std::vector<pcl::PointXYZ> transformed_points;
    for (pcl::PointXYZ p : measurement.points)
    {
        pcl::PointXYZ tp;
        tp.x = (p.x + ref_to_xm.x) + (p.x * cosf(ref_to_xm.theta) - p.y * sinf(ref_to_xm.theta));
        tp.y += (p.y + ref_to_xm.y) + (p.x * sinf(ref_to_xm.theta) + p.y * cosf(ref_to_xm.theta));
        transformed_points.push_back(tp);
    }

    float accum_dist = 0.0;
    for (std::pair<int, int> pair : pairs)
    {
        accum_dist += pcl::squaredEuclideanDistance(transformed_points.at(pair.first), reference.points.at(pair.second));
    }
    */

    return normal_distribution(sum_square_dist, 0, variance_sum_squares);
}

void MCL::importance_sample(
    std::vector<OzMCL::x>& particles, 
    std::vector<double>& weights, 
    std::vector<OzMCL::x>& result
)
{
    double w_sum = std::accumulate(weights.begin(), weights.end(), 0);

    // accumulate
    std::vector<double> integrand(weights.size());
    std::partial_sum(weights.begin(), weights.end(), integrand.begin());

    // sample
    result.clear();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, w_sum);

    for (size_t i = 0; i < particles.size(); i++) {
        double u = dis(gen);
        auto it = std::lower_bound(integrand.begin(), integrand.end(), u);
        auto index = std::distance(integrand.begin(), it);
        result.push_back(particles.at(index));
    }

    //result = particles;
}