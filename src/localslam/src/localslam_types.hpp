#ifndef LOCAL_SLAM_TYPES_H
#define LOCAL_SLAM_TYPES_H

/*---------------------------------------------------
Rodrigo Munguía 2020.

Get data from data base
-----------------------------------------------------
*/
#include <string>
#include "opencv2/opencv.hpp"
#include <fstream>
#include <algorithm>
#include <sstream>
#include <armadillo>
//#include "parameters.hpp"

using namespace std;
using namespace cv;



// gps data
struct GPS
{
   long int time;
   double lat;
   double lon;
   double alt;
   int sat;     
};

// barometer (raw) data
struct BAR 
{
    long int time;
    double pressure;
    double temp;
};
// Altitude  data
struct ALT
{
    long int time;
    double altitude; //  (meters)
};


// Range (sonar) data
struct RANGE
{
    long int time;
    double range;
    double volt;
};

// Visual data
struct FRAME
{
    long int time;
    string image_file;
    cv::Mat image;
    double range;
    string range_type;
};

struct ATT
{
    long int time;
    float roll; //  
    float pitch;
    float yaw;
};

struct SPD
{
    long int time;
    float speedX; //  
    float speedY;
    float speedZ;
};

struct ODOD
{
    long int time;
    int TicksRight;
    int TicksLeft;
};

struct ODOV
{
    long int time;
    float linear_vel;
    float angular_vel;
    
};



// struct for storing data from sensors
struct DATA
{   
    string data_type;     
    long int time;
    GPS gps;
    BAR bar;
    RANGE range;
    FRAME frame;
    ALT alt;
    ATT att;
    SPD spd;
    ODOD odod;
    ODOV odov;
};

struct POINT3d
{
    double x;
    double y;
    double z;
};

struct LOCALSLAM_DATA
{
    vector<double> robot_state;
    arma::vec::fixed<3> t_c2r;  // camera to robot position vector
    arma::mat::fixed<3,3> Rr2c; // robot to camera rotation matrix 
    arma::vec::fixed<3> r_N; // position of the robot, expressed in the navigation frame 
    arma::mat::fixed<3,3> Rn2r; // navigation to robot rotation matrix
    vector<POINT3d> map_state;
    vector<POINT3d> map_anchors;
    vector<POINT3d> matched_img_feats;
    vector<POINT3d> unmatched_img_feats;
    vector<POINT3d> matched_img_anchors;
    vector<POINT3d> unmatched_img_anchors;

};

struct POS_UPDATE
{
    bool pos_update_available;
    arma::vec::fixed<3> delta_pos_update;
    string type;

};

    


#endif
