/*----------------------------------------------------------------------------
 /--\        ==   == ----- ==
| -- | ----- | \./ | |   | ||
| || | |   / |  |  | |     ||
| -- |   /   |  |  | |     ||
 \--/  /---| |  |  | ----| |====|
----------------------------------OzMCL---------------------------------------
structs_n_stuff.hpp
Common header for structs used throughout OzMCL
------------------------------------------------------------------------------
*/

#ifndef OZMCL_STRUCTS
#define OZMCL_STRUCTS

namespace OzMCL {
    struct indexPair {
        int a;
        int b;
    };
    // control state. Make sure these are gained from odometry
    struct u {
        float dx;
        float dy;
        float dtheta;
        float theta;
        // variances for each variable
        // note to self: take from pose_covariance
        float variance_x;
        float variance_y;
        float variance_theta;
    };

    // particle state
    // relative to world
    struct x {
        float x;
        float y;
        float theta;
    };

    // measurement state
    // unused, I think
    struct z {
        float x;
        float y;
        float theta;
    };

    // state variances
    struct x_var {
        float x;
        float y;
        float theta;
    };

    // main node state machine's states
    enum STATE {
        PREDICTION,
        CORRECTION,
    };
}

#endif