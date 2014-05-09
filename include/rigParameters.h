/*********************************************
 * rigParameters.h
 * Matthew Bailey
 *
 * This file builds the rig based on its
 * physical parameters.
 *********************************************/

// Other Includes
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;

#define NOTDEBUG

// struct point2D
// defines a two dimensional point
struct point2D {
    double x, y;
};

// struct point3D
// defines a three dimensional point
struct point3D {
    double x, y, z;
};

// connectionPoint
// defines a point of connection
// on an element of the rig
typedef point3D connectionPoint;

// struct dimension
// defines the dimension of
// an element of the rig
struct dimension {
    double width, length, height;
};

// struct rotationAxis
// defines an axis of rotation
// by the joint to rotate and
// the direction of rotatation
struct rotationAxis {
    int joint;
    double x, y, z;
    bool rotates;
};

// struct connection
// defines a connection
// between two connection points
// with a joint (perhaps immobile)
// between them
struct connection {
//    point3D t;
    rotationAxis axis;
    connectionPoint* from;
    connectionPoint* to;
};

// Description of the joints of the rig
enum joint { BASE_TO_BRACKET, BRACKET_TO_ARM, ARM1_TO_ARM2, ARM_TO_SENSOR_HEAD };
double theta0[4] = {0, 0, 0, 0};

// Demension of rig components
dimension base, bracket1, bracket2, arm1, arm2, armS, plate;

// Points of connection between components
connectionPoint base_con, brac_con1, brac_con2, brac_weld1, brac_weld2,
                arm1_con1, arm1_con2, arm2_con1, arm2_con2, armS_con1,
                armS_con2, plate_con, ground;

// Connections of rig components
connection ground_base, base_brac1, brac1_brac2, brac2_arm1, arm1_arm2,
           arm2_armS, armS_plate;

double bushing_width;

// Constructs the transform matrix for the current
// postion of the sensor platform.
// (Note, this assumes the points have already been
//        transformed into the coordinate system of
//        the 'origin' of the sensor plate.)
void constructTransform(cv::Mat& ri, cv::Mat& ti, cv::Mat& rf, cv::Mat& tf, double angle[4]) {

//	for(int i = 0; i < 4; i++) theta0[i] = 0.0;

    double a1 = /*CV_PI*/ 0.0 + (angle[BASE_TO_BRACKET] + theta0[BASE_TO_BRACKET]),
           a2 = /*CV_PI*/ 0.0 + (angle[BRACKET_TO_ARM] + theta0[BRACKET_TO_ARM]),
           a3 = /*CV_PI*/ 0.0 + (angle[ARM1_TO_ARM2] + theta0[ARM1_TO_ARM2]),
           a4 = /*CV_PI*/ 0.0 + (angle[ARM_TO_SENSOR_HEAD] + theta0[ARM_TO_SENSOR_HEAD]);

    Mat r1, r2, r3, r4,
        t1, t2, t3, t4, d;
	
	d = (Mat_<double>(3,1) << ti.at<double>(0) + armS.width   + 10.0,
                              ti.at<double>(1) + plate.height + 0.5*armS.height,
                              ti.at<double>(2) + plate.length + 3.61);
    
	r4 = (Mat_<double>(3,1) << a4, 0.0, 0.0);
    t4 = (Mat_<double>(3,1) << -bushing_width - arm2.width, 0, -arm2.length + 3.1 + 2.8);

    r3 = (Mat_<double>(3,1) << a3, 0.0, 0.0);
    t3 = (Mat_<double>(3,1) << -bushing_width - arm1.width, 0, arm1.length - 1.5 - 2.8);

    r2 = (Mat_<double>(3,1) << -a2, 0.0, 0.0);
    t2 = (Mat_<double>(3,1) << 0.5*arm1.width, -2.9, 0);

    r1 = (Mat_<double>(3,1) << 0.0, a1, 0.0);
    t1 = (Mat_<double>(3,1) << 0.5*base.width, -base.height, 0.5*base.length);

//    composeRT(ri, tf, r4, t4, rf, tf);
    composeRT(r4, t4, r3, t3, rf, tf);
    composeRT(rf, tf, r2, t2, rf, tf);
    composeRT(rf, tf, r1, t1, rf, tf);
    composeRT(ri,  d, -rf, -tf, rf, tf);

    #ifdef DEBUG
    Mat t2f, r2f;
    composeRT(r1, t1, ri, t2, r2f, t2f);

    std::cout << "r2, t2" << r2f << " " << t2f << std::endl;
    #endif 
}

// Initialize test rig parameters from configuration file.
void initRig() {

    // Distance from end to center of connection
    // for the first arm segment
    double d11 = 1.5;
    double d12 = 2.5;

    // for the second arm segment
    double d21 = 2.5;
    double d22 = 1.5;

    // for the sensor arm
    double ds1 = 2.5;
    double ds2 = 3;

    // for the sensor plate
    double dp1 = 2;

    // Dimensions
    // Base
    base.width  = 30.5;
    base.length = 30.5;
    base.height = 14;

    // L-Bracket
    bracket1.width  = 5;
    bracket1.length = 7;
    bracket1.height = 0.5;

    bracket2.width  = 0.5;
    bracket2.length = 6;
    bracket2.height = 6;

    // First Arm Segment
    arm1.width  = 2.5;
    arm1.length = 124;
    arm1.height = 2.5;

    // Second Arm Segment
    arm2.width  = 2.5;
    arm2.length = 118.5;
    arm2.height = 2.5;

    // Sensor Head
    armS.width  = 2;
    armS.length = 11.5;
    armS.height = 2.5;

    plate.width  = 25.5;
    plate.length = 16;
    plate.height = 0.25;

    // Connection Points, from center of element

    bushing_width = 0.5;

    // Connection point on ground;
    ground.x = 0;
    ground.y = 0;
    ground.z = 0;

    // Connection point on base
    base_con.x = -0.5*base.length;
    base_con.y = base.height; //0.5*base.height;
    base_con.z = -0.5*base.width;

    // Connection point on bracket (connects to base)
    brac_con1.x = 0;
    brac_con1.y = 0.5*bracket1.height;
    brac_con1.z = 0;

    // 'Weld' for first bracket piece to connect to second
    brac_weld1.x = 0;
    brac_weld1.y = 0.5*bracket1.height;
    brac_weld1.z = 0.5*bracket1.width;

    // 'Weld' for second bracket piece to connect to first
    brac_weld2.x = 0;
    brac_weld2.y = 0.5*bracket2.height;
    brac_weld2.z = -0.5*bracket2.width;

    // Connection point on bracket (connects to arm)
    brac_con2.x = 0;
    brac_con2.y = 0;
    brac_con2.z = -0.5*bracket2.width;

    // Connection point on first arm segment (connects to bracket)
    arm1_con1.x = -0.5*arm1.length + d11;
    arm1_con1.y = 0;
    arm1_con1.z = -0.5*arm1.width;

    // Connection point on first arm segment (connects to second segment)
    arm1_con2.x = -(0.5*arm1.length - d12);
    arm1_con2.y = 0;
    arm1_con2.z = -0.5*arm1.width - 0.5*bushing_width;

    // Connection point on second arm segment (connects to first segment)
    arm2_con1.x = -(-0.5*arm2.length + d21);
    arm2_con1.y = 0;
    arm2_con1.z = -0.5*arm2.width - 0.5*bushing_width;

    // Connection point on second arm segment (connects to sensor platform)
    arm2_con2.x = 0.5*arm2.length - d22;
    arm2_con2.y = 0;
    arm2_con2.z = -0.5*arm2.width - 0.5*bushing_width;

    // Connection point on arm portion of sensor head (connects to arm)
    armS_con1.x = -(-0.5*armS.length + ds1);
    armS_con1.y = 0;
    armS_con1.z = -0.5*armS.width - 0.5*bushing_width;

    // Connection point on arm portion of sensor head (connects to sensor plate)
    armS_con2.x = -(0.5*armS.length - ds2);
    armS_con2.y = 0.5*armS.height;
    armS_con2.z = 0;

    // Connection point on sensor plate (connects to arm portion of sensor head)
    plate_con.x = 14;//0.5*plate.width;
    plate_con.y = 0.5*plate.height;
    plate_con.z = armS.width;

    // Initial angles
    theta0[BASE_TO_BRACKET]    = 0;
    theta0[BRACKET_TO_ARM]     = -asin(14.5/(arm1.length - d11 - d12));
    theta0[ARM1_TO_ARM2]       = asin((1 + 0.5*arm1.height)/(arm2.length - d21 - d22));
    theta0[ARM_TO_SENSOR_HEAD] = -(-theta0[BRACKET_TO_ARM] + theta0[ARM1_TO_ARM2]);

    // Base
    ground_base.axis.rotates = 0;
    ground_base.from = &ground;
    ground_base.to   = &base_con;  

    // L-Bracket: Base
    base_brac1.axis.rotates = 1;
    base_brac1.axis.joint = BASE_TO_BRACKET;
    base_brac1.axis.x = 0;
    base_brac1.axis.y = 1;
    base_brac1.axis.z = 0;

    base_brac1.from = &base_con;
    base_brac1.to   = &brac_con1;

    // L-Bracket: Side
    brac1_brac2.axis.rotates = 0;
    brac1_brac2.from = &brac_weld1;
    brac1_brac2.to   = &brac_weld2;

    // First Arm Segment
    brac2_arm1.axis.rotates = 1;
    brac2_arm1.axis.joint = BRACKET_TO_ARM;
    brac2_arm1.axis.x = 1;
    brac2_arm1.axis.y = 0;
    brac2_arm1.axis.z = 0;

    brac2_arm1.from = &brac_con2;
    brac2_arm1.to   = &arm1_con1;

    // Second Arm Segment
    arm1_arm2.axis.rotates = 1;
    arm1_arm2.axis.joint = ARM1_TO_ARM2;
    arm1_arm2.axis.x = 1;
    arm1_arm2.axis.y = 0;
    arm1_arm2.axis.z = 0;

    arm1_arm2.from = &arm1_con2;
    arm1_arm2.to   = &arm2_con1;

    // Sensor Head: Arm
    arm2_armS.axis.rotates = 1;
    arm2_armS.axis.joint = ARM_TO_SENSOR_HEAD;
    arm2_armS.axis.x = 1;
    arm2_armS.axis.y = 0;
    arm2_armS.axis.z = 0;

    arm2_armS.from = &arm2_con2;
    arm2_armS.to   = &armS_con1;

    // Sensor Head: Plate
    armS_plate.axis.rotates = 0;
    armS_plate.from = &armS_con2;
    armS_plate.to   = &plate_con;
}

/* EOF */

