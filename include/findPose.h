
#ifndef FIND_POSE
#define FIND_POSE

/***********************************************\
 * findPose
 * Matthew Bailey
 *
 * This file defines a 3D pose as
 *     [x, y, z, ψ, θ, φ]
 * where ψ, θ, and φ are rotations
 * about the x, y, and z axes respectively
 * and defines a function to retrieve the 3D
 * pose given valid rotation and translation
 * vectors.
 * Assumes that R = RzRyRx
 *
 *     |cosθcosφ -sinψsinθcosφ − cosψsinφ  -cosψsinθcosφ + sinθsinφ |
 * R = |cosθsinφ -sinψsinθsinφ + cosψcosφ  -cosψsinθsinφ − sinθcosφ |
 *     |  sinθ            sinψcosθ                 cosψcosθ         |
 *
\***********************************************/

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "opencv2/core/core.hpp"

struct Pose3D {
  float x;
  float y;
  float z;

  float psi;
  float theta;
  float phi;
};


std::ostream& operator<< (std::ostream& out, Pose3D& pose) {
    out << "[" << pose.x << ", " << pose.y << ", " << pose.z << ", "
        << pose.psi << ", " << pose.theta << ", " << pose.phi << "]";
        
    return out;
}

#define R(i,j) R.at<double>((i-1), (j-1))
#define epsilon (1E-10)

bool find3DPose(cv::Mat& R, cv::Mat& t, Pose3D& pose) {
  
    pose.x = t.at<double>(0);
    pose.y = t.at<double>(1);
    pose.z = t.at<double>(2);

    // If |R(3,1)| = |-sin(theta)| == 1, then theta = 0 or PI
    // and cos(theta) = 0 epsilon = 0.000001;
    if(fabs(1 - fabs(R(3,1)) < epsilon)) {
        int s = ((R(3,1) < 0)? -1 : 1);
        pose.theta = CV_PI/2 * s;
        pose.phi = 0; // set gimbal-locked theta_z to 0

        if(fabs(R(1,3)) < epsilon)
          pose.psi = CV_PI/2 * ((R(1,2) < 0)? -1 : 1);
        else
          pose.psi = atan2(R(1,2), R(1,3));
      }
    else { 
        double theta_y = asin(R(3,1));
        if(theta_y > CV_PI/2)
          theta_y = CV_PI - theta_y;
        
        int s = ((cos(theta_y) < 0)? -1 : 1);
        double theta_x = 0;
        double theta_z = 0;

        if(fabs(R(3,3)) < epsilon)
           theta_x = CV_PI/2 * ((R(3,2) < 0)? -1 : 1);
        else
           theta_x = atan2(R(3,2), R(3,3));
        
        if(fabs(R(1,1)) < epsilon)
           theta_z = CV_PI/2 * ((R(2,1) < 0)? -1 : 1);
        else
           theta_z = atan2(R(2,1), R(1,1));

        pose.psi = theta_x * s;
        pose.theta = theta_y;
        pose.phi = theta_z * s;
      }

    return true;
}

#endif
