/*----------------------------------------------------------------------------
 /--\        ==   == ----- ==
| -- | ----- | \./ | |   | ||
| || | |   / |  |  | |     ||
| -- |   /   |  |  | |     ||
 \--/  /---| |  |  | ----| |====|
----------------------------------OzMCL---------------------------------------
ozmcl.hpp
MCL - class containing a collection of OzMCL's mathematical functions
------------------------------------------------------------------------------
*/

#ifndef OZMCL_HPP_
#define OZMCL_HPP_

#include <random>
#include <vector>
#include "ozmcl/structs_n_stuff.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>

namespace OzMCL
{
    class MCL 
    {
    public:
        /**
         * @brief predicts the next state based on the previous state and the control
         * @param X_prev previous particles
         * @param u control
         * @param Xbar_now particle vector to store the predicted state to
         */
        static void predict(
            std::vector<OzMCL::x>& X_prev, 
            OzMCL::u u, 
            std::vector<OzMCL::x>& Xbar_now
        );

        /**
         * @brief importance sampling function
         * @param particles particles to sample
         * @param weights weights of particles
         * @param result particle vector to store result to
         */
        // all points in frame of map
        static void importance_sample(
            std::vector<OzMCL::x>& particles, 
            std::vector<double>& weights, 
            std::vector<OzMCL::x>& result
        );

        /**
         * @brief p(z | x) - function for the probability of a measurement given a statem x
         * @note actual implementation is in ozmcl_node.... I think
         * @param sum_square_dist sum of square distances for the measurement in question
         * @param variance_noise the variance of the sum_square_dist
         */
        static double p_measurement(
            double sum_square_dist,
            double variance_noise
        );
        

        static double angle_difference(double angle1, double angle2);
    private:
        /**
         * @brief state prediction function
         * @param x_prev previous state
         * @param u_t control of present
         */
        static OzMCL::x g(OzMCL::x x_prev, OzMCL::u u_t);
    };

};



#endif