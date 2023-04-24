#include "localslam_component.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "localslam_types.hpp"
#include <mutex>
#include <cv_bridge/cv_bridge.h>

using namespace std;


namespace localslam
{

void EKFslam::Alt_callback(const interfaces::msg::Alt & msg) const
{
   //cout << "EKFslam: Alt" << endl;
  /* 
  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;
  */

  DATA dat;
  //dat.alt.time = ms;
  dat.data_type = "alt";  
  dat.alt.altitude = msg.altitude; 
  dat.alt.time = msg.time;
  dat.time = msg.time;  
   
  mutex_dat.lock(); 
   ManageBuffer(dat);
  mutex_dat.unlock();  

}

void EKFslam::Att_callback(const interfaces::msg::Att & msg) const
{  
  //cout << "EKFslam: Att" << endl;
  /*
  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;
  */

  DATA dat;
  //dat.att.time = ms;
  dat.data_type = "attitude";
  dat.att.roll = msg.roll;
  dat.att.pitch = msg.pitch;
  dat.att.yaw = msg.yaw; 
  dat.att.time = msg.time;
  dat.time = msg.time; 

   mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock();

}

void EKFslam::Gps_callback(const interfaces::msg::Gps & msg) const
{
 //cout << "EKFslam: Gps" << endl;
 /*
 struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;
  */

  DATA dat;
 // dat.att.time = ms;
  dat.data_type = "gps";
  dat.gps.lat = msg.lat;
  dat.gps.lon = msg.lon;
  dat.gps.alt = msg.alt;
  dat.gps.sat = msg.sat; 
  dat.gps.time = msg.time;
  dat.time = msg.time; 

   mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock();

}

void EKFslam::Range_callback(const interfaces::msg::Range & msg) const
{

 //cout << "EKFslam: Range" << endl;
  /*
 struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;
  */
  
  DATA dat;
  dat.range.time = msg.time;
  dat.data_type = "range";
  dat.range.range = msg.range;
  dat.range.volt = msg.volt;
  dat.time = msg.time; 

  mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock(); 


}

void EKFslam::Frame_callback(const interfaces::msg::Frame & msg) const
{
  //cout << "EKFslam: Frame" << endl;
  /*
  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;
  */
  

  DATA dat;
  dat.frame.time = msg.time;
  dat.data_type = "frame";
  
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg.img,sensor_msgs::image_encodings::MONO8 );
  
  dat.frame.image = cv_ptr->image;
  
  dat.frame.range = -1; // by default 
  dat.time = msg.time; 
  
  //cv::imshow( "FAST Detections", dat.frame.image);

  mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock();    

  


}
 
void EKFslam::Speed_callback(const interfaces::msg::Spd & msg) const
{

  //cout << "EKFslam: Speed" << endl;
  /*
  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;
  */
  
  DATA dat;
  dat.spd.time = msg.time;
  dat.data_type = "speed";
  dat.spd.speedX = msg.speed_x;
  dat.spd.speedY = msg.speed_y;
  dat.spd.speedZ = msg.speed_z;
  dat.time = msg.time; 

  mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock(); 

}

//-------------------------------------------------
void EKFslam::OdoDiff_callback(const interfaces::msg::Ododiff &msg) const
{
  DATA dat;
  dat.odod.time = msg.time;
  dat.data_type = "odometry_diff";
  dat.odod.TicksLeft = msg.ticks_left;
  dat.odod.TicksRight = msg.ticks_right;
  dat.time = msg.time;

   mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock(); 

}

//-------------------------------------------------
void EKFslam::OdoVW_callback(const interfaces::msg::Odovw &msg) const
{
  DATA dat;
  dat.odov.time = msg.time;
  dat.data_type = "odometry_vw";
  dat.odov.angular_vel= msg.angular_vel;
  dat.odov.linear_vel = msg.linear_vel;
  dat.time = msg.time;

   mutex_dat.lock(); 
    ManageBuffer(dat);
   mutex_dat.unlock(); 

}

//--------------------------------------------------

 void EKFslam::ManageBuffer(DATA &dat)
 {
   
   string data_type = dat.data_type;

 for (int i=0; i < DataBuffer.size(); i++)
 {
   if (DataBuffer[i].data_type == data_type)
   {
      DataBuffer.erase(DataBuffer.begin() + i);
      i--;
   }
 }

   DataBuffer.push_back(dat);
   
   
 }

//----------------------------------------------------

DATA EKFslam::getData()
 {
    static double last_range;
    static long int last_range_time;
    static bool range_available = false;
    static bool range_available_alt = false ;
    static double last_alt = 0; 
    
  mutex_dat.lock();   
    
    DATA dat;

    for (int i= 0 ; i <DataBuffer.size() ; i++)
    {
      
      //--------Speed measurement
      if (DataBuffer[i].data_type == "speed")
      {   
          dat =  DataBuffer[i];        
          DataBuffer.erase(DataBuffer.begin() + i);          
          break;
      }
      //-------- Odometry diff
      if (DataBuffer[i].data_type == "odometry_diff")
      {   
          dat =  DataBuffer[i];        
          DataBuffer.erase(DataBuffer.begin() + i);          
          break;
      }
      //-------- Odometry linear-angular velocity
      if (DataBuffer[i].data_type == "odometry_vw")
      {   
          dat =  DataBuffer[i];        
          DataBuffer.erase(DataBuffer.begin() + i);          
          break;
      }
      //-------- Attitude
      if (DataBuffer[i].data_type == "attitude")
      {   
          dat =  DataBuffer[i];        
          DataBuffer.erase(DataBuffer.begin() + i);          
          break;
      }
      //------- altitude measurement
      if (DataBuffer[i].data_type == "alt")
      {   
          dat =  DataBuffer[i];        
          DataBuffer.erase(DataBuffer.begin() + i);
          last_alt = dat.alt.altitude;
          range_available_alt = true;
          break;
      }
      //-------- range measurement
      if (DataBuffer[i].data_type == "range")
      {   
          dat =  DataBuffer[i];
          DataBuffer.erase(DataBuffer.begin() + i);

          if (dat.range.range > .4 && dat.range.range < 6) // if sonar measurement is valid
          {
            last_range = dat.range.range;
            last_range_time = dat.range.time;
            range_available = true;           
            break;
          }
          else
          {
            dat.data_type = "null";
            break;
          }  
      }
      //-------- camera measurement
      if (DataBuffer[i].data_type == "frame")
      {   
          dat =  DataBuffer[i];                 
          DataBuffer.erase(DataBuffer.begin() + i);          

          if (range_available == true)
          {
            dat.frame.range = last_range; // if a range masurement has just "received" then associate it to the frame
            dat.frame.range_type = "sonar";
            range_available = false;  
          }  
          long int dt = dat.frame.time - last_range_time; //ms
          // if no range (from sensor) has been received fro a while, try to use alt measurements
          if ((range_available_alt == true) && (dt > 100) && (last_alt > 0) )
          {
            dat.frame.range = last_alt;
            dat.frame.range_type = "alt";
            range_available_alt = false;
          }
          break;
      }


    }  
  mutex_dat.unlock();

  return dat;
 }

 //-------------------------------------------------



}