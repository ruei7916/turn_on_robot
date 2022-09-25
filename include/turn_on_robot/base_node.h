// covariance matrix for odometry
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
                                              0, 1e-9,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };
                                              
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
                                              0, 1e-9,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9 };
