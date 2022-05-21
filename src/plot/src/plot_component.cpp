#include "plot_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cinttypes>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/viz.hpp>
#include <cv_bridge/cv_bridge.h>
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/lslamstate.hpp"
#include "../../common/Transforms/quat2R.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"

using namespace cv;
using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace plot
{

// Create a Talker "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
PLOTscene::PLOTscene(const rclcpp::NodeOptions & options)
: Node("PLOTscene", options)
{
  setParameters();

  // Set services
  srv_plot_run_ = this->create_service<interfaces::srv::SimpleServ>("plot_service",std::bind(&PLOTscene::Handle_plot_run_service, this,_1,_2));
  
   // Set subscribers
  sub_Frame_ = this->create_subscription<interfaces::msg::Frame>("Frame_topic", 10, std::bind(&PLOTscene::Frame_callback, this, _1)); 
  sub_lslam_data_ = this->create_subscription<interfaces::msg::Lslam>("lslam_data_topic", 10, std::bind(&PLOTscene::lslam_data_callback, this, _1)); 
  sub_gmap_data_ = this->create_subscription<interfaces::msg::Gmap>("gmap_data_topic",10,std::bind(&PLOTscene::gmap_data_callback, this, _1));

  // plot thread
  viewer_active = true;
  new_robot_pose_flag = false;
  new_clear_scene = false;
  init_scene = false;
  plot_loop_ = thread(&PLOTscene::PLOT_LOOP,this);

  

}

void PLOTscene::draw_drone()
{
  cv::viz::WCoordinateSystem robot_ref(.15);          
  viewer.showWidget("robot", robot_ref, robot_pose);          
  cv::viz::WCube robot_cube(cv::Point3d(-.1,-.035,-.025),cv::Point3d(.1,.035,.025),true,cv::viz::Color::white());	
  viewer.showWidget("robot_cube",robot_cube,robot_pose);
  cv::viz::WCylinder prop1(cv::Point3d(.12,.12,.01),cv::Point3d(.12,.12,-.01),.04,20,cv::viz::Color::white()); 
  viewer.showWidget("prop1",prop1,robot_pose);
  cv::viz::WCylinder prop2(cv::Point3d(.12,-.12,.01),cv::Point3d(.12,-.12,-.01),.04,20,cv::viz::Color::white()); 
  viewer.showWidget("prop2",prop2,robot_pose);
  cv::viz::WCylinder prop3(cv::Point3d(-.12,.12,.01),cv::Point3d(-.12,.12,-.01),.04,20,cv::viz::Color::white()); 
  viewer.showWidget("prop3",prop3,robot_pose);
  //cv::viz::WCircle prop4(.04,cv::Point3d(-.12,-.12,0),cv::Vec3d(0,0,1),.01, cv::viz::Color::white());
  cv::viz::WCylinder prop4(cv::Point3d(-.12,-.12,.01),cv::Point3d(-.12,-.12,-.01),.04,20,cv::viz::Color::white()); 
  viewer.showWidget("prop4",prop4,robot_pose);
  
  cv::viz::WCameraPosition robot_cam_ref(.1);
  viewer.showWidget("robot_cam_ref",robot_cam_ref,robot_pose*robot_camera_pose);
  cv::viz::WCameraPosition robot_cam(Vec2f(0.889484, 0.523599),.1,cv::viz::Color::cyan());
  viewer.showWidget("robot_cam",robot_cam,robot_pose*robot_camera_pose);



}
//-------------------------------------------
void PLOTscene::draw_localslam_trajectory()
{
  //for(int i = 1; i<LocalSlam_trajectory.size(); i++ )
   //     {
        if(LocalSlam_trajectory.size() > 1)
        {
          int i = LocalSlam_trajectory.size()-1;
          string id = to_string(i);
          viz::WLine segment(LocalSlam_trajectory[i-1], LocalSlam_trajectory[i]);
          segment.setRenderingProperty(viz::LINE_WIDTH, 1.0);
          segment.setRenderingProperty(viz::OPACITY,0.8);
          viewer.showWidget(id, segment);
        }  
    //    }


}

//---------------------------------------------
void PLOTscene::set_camera_view()
{
  if(Cam_view == 1)
  {
    // x-y view  
    cv::Mat Rxy = (Mat_<double>(3,3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);   
    cv::Vec3f t_c;        
          t_c(0) = Cam_pos_init_x;
          t_c(1) = Cam_pos_init_y;
          t_c(2) = -Cam_distance;        
    cv::Affine3d Pose(Rxy, t_c);
    viewer.setViewerPose(Pose);
  }
  if(Cam_view == 2)
  {
    // x-z view  
    cv::Mat Rxz = (Mat_<double>(3,3) << 1, 0, 0, 0, 0, -1, 0, 1, 0);   
    cv::Vec3f t_c;        
          t_c(0) = Cam_pos_init_x;
          t_c(1) = Cam_distance;
          t_c(2) = Cam_pos_init_z;        
    cv::Affine3d Pose(Rxz, t_c);
    viewer.setViewerPose(Pose);
  }
  if(Cam_view == 3)
  {
    // y-z view
    //cv::Mat R_phi = (Mat_<double>(3,3) << 1, 0, 0, 0, 0, 1, 0, -1, 0);    
    //cv::Mat R_theta = (Mat_<double>(3,3) << 0, 0, 1, 0, -1, 0, -1, 0, 0);
    // Ryz = R_phi*R_theta 
    cv::Mat Ryz = (Mat_<double>(3,3) << 0, 0, 1, -1, 0, 0, 0, 1, 0);   
    cv::Vec3f t_c;        
          t_c(0) = -Cam_distance;
          t_c(1) = Cam_pos_init_y;
          t_c(2) = Cam_pos_init_z;        
    cv::Affine3d Pose(Ryz, t_c);
    viewer.setViewerPose(Pose);
  }


  //------------ 
}
void PLOTscene::initialize_scene()
{
  
  // draw navigation frame
  cv::viz::WCoordinateSystem origin_ref(.15);          
  viewer.showWidget("origin", origin_ref); 
  viewer.spinOnce(1, true);
  
  // draw grid
  cv::viz::WGrid grid_xy(cv::Vec2i::all(30), cv::Vec2d::all(1.0),cv::viz::Color::white());
   grid_xy.setRenderingProperty(viz::OPACITY,0.2);
   viewer.showWidget("grid_xy", grid_xy);
  
  cv::viz::WGrid grid_xz( cv::Point3d(0.0,0.0,0.0),cv::Vec3d(-0.0000001,1.0,0.0),cv::Vec3d(0.0,1.0,0.0), cv::Vec2i::all(10), cv::Vec2d::all(1.0),cv::viz::Color::blue());
  grid_xz.setRenderingProperty(viz::OPACITY,0.2);
  viewer.showWidget("grid_xz", grid_xz);
  
  
  
}

//------------------------------------------------------------------------
//  Main Plot loop (this function runs in a separate thread)
void PLOTscene::PLOT_LOOP()
{
  
  cout << "-> Plot thread running... " << endl;    
  //viz::Viz3d viewer("Viewer");  
  viewer = viz::Viz3d("Viewer");
  viewer.setWindowSize(cv::Size(Viewer_width,Viewer_height));
  //viewer.setBackgroundColor(viz::Color::black(),viz::Color::not_set());
  viewer.spinOnce(1, true);
  const auto pose_0 = viewer.getViewerPose();

    // Set the robot_camera pose
   /*
   double Rr2c_a[9];
   Euler_to_Ra2b_row_major(Robot_cam_axis_x, Robot_cam_axis_y, Robot_cam_axis_z, Rr2c_a);  
   cv::Vec3f  t_c;
   t_c(0) = Robot_cam_pos_x;
   t_c(1) = Robot_cam_pos_y;
   t_c(2) = Robot_cam_pos_z;  
   cv::Mat Rr2c = cv::Mat(3,3,CV_64FC1,Rr2c_a);  // opencv is row-major
   cv::Mat Rc2r = Rr2c.t();
   robot_camera_pose = cv::Affine3d(Rc2r, t_c);
   */
   //---------------------------------------------
   
   //cout << "plot R" << endl;
   //cout << Robot_cam_axis_x << " " << Robot_cam_axis_y << " " << Robot_cam_axis_z << endl;
   //cout << Rc2r << endl;
   
   


  
  

  while(!viewer.wasStopped()&&viewer_active == true)
  {   
        
        if(init_scene == false)
        { 
          // add default widgets
          set_camera_view();  
          initialize_scene();          
          init_scene = true;
        }        
        //-------------------------------------------
        mutex_clear_scene.lock();
          if(new_clear_scene == true)
          {
            reset_scene();
            new_clear_scene = false;            
          }
         mutex_clear_scene.unlock();           
        //------------------------------------------
        if(!frame.empty())
        {
          cv::Size viewer_size = viewer.getWindowSize();
          Viewer_height = viewer_size.height;      
          // upper-left location
          mutex_frame.lock();
            cv::Mat frame_rgb(frame.size(), CV_8UC3);
            // convert grayscale to color image
            cv::cvtColor(frame, frame_rgb, CV_GRAY2RGB);
          mutex_frame.unlock();

            for( int i = 0 ; i < Img_matched_feats.size(); i++)
            {
              circle(frame_rgb, Img_matched_feats[i], 1, Scalar(0, 0, 255), 2, 8, 0);            
            }
            for( int i = 0 ; i < Img_unmatched_feats.size(); i++)
            {
              circle(frame_rgb, Img_unmatched_feats[i], 2, Scalar(0, 0, 128), 1, 8, 0);            
            }
            for( int i = 0 ; i < Img_matched_anchors.size(); i++)
            {
              circle(frame_rgb, Img_matched_anchors[i], 1, Scalar(0, 255, 255), 2, 8, 0);            
            }
            for( int i = 0 ; i < Img_unmatched_anchors.size(); i++)
            {
              circle(frame_rgb, Img_unmatched_anchors[i], 2, Scalar(0, 128, 128), 1, 8, 0);            
            }
            cv::Rect frame_rect(0,Viewer_height-frame_rgb.rows,frame_rgb.cols,frame_rgb.rows);
            cv::viz::WImageOverlay frame_s(frame_rgb,frame_rect);
           
          viewer.showWidget("frame",frame_s);
        }
        //------------------------ 
        mutex_get_state.lock();
          if(new_robot_pose_flag == true)
          {                    
            draw_drone();          

            if(Draw_local_slam_trajectory)draw_localslam_trajectory(); 

            //viewer.removeWidget("Cloud");
            if(!EKFmap.empty())
            {
              cv::viz::WCloud cloud_local_map(EKFmap,EKFmap_color);
              cloud_local_map.setRenderingProperty(cv::viz::POINT_SIZE, 2);
              viewer.showWidget( "Cloud_feats", cloud_local_map );
            }
            if(!ANCHORSmap.empty())
            {
              cv::viz::WCloud cloud_local_map_anchors(ANCHORSmap,ANCHORSmap_color);
              cloud_local_map_anchors.setRenderingProperty(cv::viz::POINT_SIZE, 2);
              viewer.showWidget( "Cloud_anchors", cloud_local_map_anchors );
            }
            new_robot_pose_flag = false;
          }
        mutex_get_state.unlock();
        //------------------------------
        mutex_set_cam.lock();
          if(New_cam_view == true )
          {
            set_camera_view();
            New_cam_view = false;
          }
        mutex_set_cam.unlock(); 
        //------------------------------
        // plot gmap
        
        
        mutex_clear_get_gmap.lock();
         
          for(int i = 0; i < kf_poses.size(); i++)
          {             
            string id = "kf" + to_string(i);       
            
                cv::viz::WCameraPosition kf_i_pos(Vec2f(0.889484, 0.523599),.05,cv::viz::Color::cyan());
                  viewer.showWidget(id,kf_i_pos,kf_poses[i]);
                       
          }  
          if(!Gmap.empty()&&!Gmap_color.empty())
            {
              cv::viz::WCloud cloud_local_map(Gmap,Gmap_color);
              cloud_local_map.setRenderingProperty(cv::viz::POINT_SIZE, 1);
              viewer.showWidget( "Gmap_anchors", cloud_local_map );
            }       
       
        mutex_clear_get_gmap.unlock();   
       


        viewer.spinOnce(1, true);
    }
  
  
  

}  
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
void PLOTscene::on_timer()
{

  //RCLCPP_INFO(this->get_logger(),"x");
    //cout << "xxx";
    //viewer.spinOnce(30, false);

}    

//--------------------------------------------------------------------------
PLOTscene::~PLOTscene()
{  
    viewer_active = false;
    viewer.close();
    plot_loop_.join();
}
//-----------------------------------------------------------------------------
void PLOTscene::reset_scene()
{ 
  /*
  if(LocalSlam_trajectory.size() > 1 )
  {
    viewer.getWidget() 
    for(int i = 1; i<LocalSlam_trajectory.size(); i++ )
          {          
            string id = to_string(i);
            viewer.removeWidget(id);    
        }
   
    LocalSlam_trajectory.clear();
  } 
  */
  LocalSlam_trajectory.clear();
  viewer.removeAllWidgets();
  kf_poses.clear();    
  init_scene = false;


}


//----------------------------------------------------------------------------
void PLOTscene::Handle_plot_run_service(const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,std::shared_ptr<interfaces::srv::SimpleServ::Response> response) 
    { 
      if (request->cmd == 'c')
      { 
        cout << "plot-> request received: clear plot" << endl;
        mutex_clear_scene.lock();
          new_clear_scene = true;
        mutex_clear_scene.unlock();   
         
      }        
      if (request->cmd == 'r')
      {          
        cout << "plot-> request received: run plot" << endl;        
        
      } 
      if (request->cmd == 's')      
      {          
         
        cout << "plot-> request received: stop plot" << endl;        
        
      }
      if (request->cmd == 45)
      {
        cout << "plot-> zoom out" << endl;
        mutex_set_cam.lock();
          Cam_distance = Cam_distance+Cam_zoom_delta;
          New_cam_view = true;
        mutex_set_cam.unlock();  
      }
      if (request->cmd == 43)
      {
        cout << "plot-> zoom in" << endl;
        mutex_set_cam.lock();
          Cam_distance = Cam_distance-Cam_zoom_delta;
          if(Cam_distance < 1)Cam_distance = 1;
          New_cam_view = true;
        mutex_set_cam.unlock();  
      }
      if (request->cmd == '1')
      {
        cout << "plot-> view x-y" << endl;
        mutex_set_cam.lock();          
          New_cam_view = true;
          Cam_view = 1;
        mutex_set_cam.unlock();  
      }
      if (request->cmd == '2')
      {
        cout << "plot-> view x-z" << endl;
        mutex_set_cam.lock();          
          New_cam_view = true;
          Cam_view = 2;
        mutex_set_cam.unlock();  
      }
      if (request->cmd == '3')
      {
        cout << "plot-> view y-z" << endl;
        mutex_set_cam.lock();          
          New_cam_view = true;
          Cam_view = 3;
        mutex_set_cam.unlock();  
      }
      if (request->cmd == '8')
      {
        cout << "plot-> view up" << endl;
        mutex_set_cam.lock();          
          New_cam_view = true;          
          if(Cam_view == 1)
          {
            Cam_pos_init_x = Cam_pos_init_x -Cam_move_delta ;
          }
          if(Cam_view == 2)  // x-z view
          {
            Cam_pos_init_z = Cam_pos_init_z +Cam_move_delta ;
          }
          if(Cam_view == 3)  // y-z view
          {
            Cam_pos_init_z = Cam_pos_init_z +Cam_move_delta ;
          }          

        mutex_set_cam.unlock();  
      }
      if (request->cmd == '5')
      {
        cout << "plot-> view down" << endl;
        mutex_set_cam.lock();          
          New_cam_view = true;         
          if(Cam_view == 1)
          {
            Cam_pos_init_x = Cam_pos_init_x +Cam_move_delta ;
          } 
          if(Cam_view == 2)  // x-z view
          {
            Cam_pos_init_z = Cam_pos_init_z -Cam_move_delta ;
          }
          if(Cam_view == 3)  // x-z view
          {
            Cam_pos_init_z = Cam_pos_init_z -Cam_move_delta ;
          }        
        mutex_set_cam.unlock();  
      }
      if (request->cmd == '4')
      {
        cout << "plot-> view left" << endl;
        mutex_set_cam.lock();          
          New_cam_view = true;          
          if(Cam_view == 1)  // x-y view
          {
            Cam_pos_init_y = Cam_pos_init_y +Cam_move_delta ;
          }
          if(Cam_view == 2)  // x-z view
          {
            Cam_pos_init_x = Cam_pos_init_x +Cam_move_delta ;
          }
          if(Cam_view == 3)  // y-z view
          {
            Cam_pos_init_y = Cam_pos_init_y +Cam_move_delta ;
          }     
           
        mutex_set_cam.unlock();  
      }
      if (request->cmd == '6')
      {
        cout << "plot-> view right" << endl;
        mutex_set_cam.lock();          
          New_cam_view = true;          
          if(Cam_view == 1)
          {
            Cam_pos_init_y = Cam_pos_init_y -Cam_move_delta ;
          }
           if(Cam_view == 2)  // x-z view
          {
            Cam_pos_init_x = Cam_pos_init_x -Cam_move_delta ;
          } 
          if(Cam_view == 3)  // y-z view
          {
            Cam_pos_init_y = Cam_pos_init_y -Cam_move_delta ;
          }      
        mutex_set_cam.unlock();  
      }
      

      response->response = true;
    };

//------------------------------------------------------------------------------------------------------    
void PLOTscene::Frame_callback(const interfaces::msg::Frame & msg) const
{
    
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg.img,sensor_msgs::image_encodings::MONO8 );
    
  mutex_frame.lock(); 
    frame = cv_ptr->image;
  mutex_frame.unlock();    
  
}
//-----------------------------------------------------------------------------------------------------------
void PLOTscene::gmap_data_callback(const interfaces::msg::Gmap & msg)
{
  
  mutex_clear_get_gmap.lock();

  kf_poses.clear();
  Gmap.release();
  Gmap_color.release();
  //cv::Vec3f  t_l;
  for(int i = 0; i < msg.n2c.size();i++)
    {
      double quat[4];
      quat[0] = msg.n2c[i].rotation.w;
      quat[1] = msg.n2c[i].rotation.x;
      quat[2] = msg.n2c[i].rotation.y;
      quat[3] = msg.n2c[i].rotation.z;  
      double Rn2c_a[9];
      quat2R_row_major(quat,Rn2c_a);
      cv::Mat Rn2c = cv::Mat(3,3,CV_64FC1,Rn2c_a);
      cv::Mat Rc2n = Rn2c.t(); 
      cv::Vec3f  t_c;
      t_c(0) = msg.n2c[i].translation.x;//cv::Mat pt(1, 3, CV_64FC3); 
      t_c(1) = msg.n2c[i].translation.y;
      t_c(2) = msg.n2c[i].translation.z;
      cv::Affine3d kf_i_pos = cv::Affine3d(Rc2n, t_c);      
      
      kf_poses.push_back(kf_i_pos);
      //t_l = t_c;   
    }
    //cout << "plot last kf pos: "  << t_l << endl;

    for(int i = 0; i < msg.gmap_anchors.size();i++)
    {
      cv::Mat pt(1, 3, CV_64FC3);
      pt.at<double>(0,0) = msg.gmap_anchors[i].x;
      pt.at<double>(0,1) = msg.gmap_anchors[i].y;
      pt.at<double>(0,2) = msg.gmap_anchors[i].z;
      Gmap.push_back(pt);
      cv::Mat col(1, 3, CV_8UC3);
      col.at<char>(0,0) = 0;  // B
      col.at<char>(0,1) = 255;  // G
      col.at<char>(0,2) = 0;  // R   
      Gmap_color.push_back(col); 
    }    
   

   mutex_clear_get_gmap.unlock(); 

}
//--------------------------------------------------------------------------------------------------------
void  PLOTscene::lslam_data_callback(const interfaces::msg::Lslam & msg) 
{
  /*
  double phi = msg.robot_state[1];
  double theta = msg.robot_state[2];
  double psi = msg.robot_state[3];
  // Position Equations
   double Rn2r_a[9];
   Euler_to_Ra2b_row_major(phi, theta, psi, Rn2r_a);
  */  
  
  double quat[4];
  quat[0] = msg.n2r.rotation.w;
  quat[1] = msg.n2r.rotation.x;
  quat[2] = msg.n2r.rotation.y;
  quat[3] = msg.n2r.rotation.z;  
  double Rn2r_a[9];
  quat2R_row_major(quat,Rn2r_a);
  cv::Mat Rn2r = cv::Mat(3,3,CV_64FC1,Rn2r_a);
  cv::Mat Rb = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
  cv::Mat Rr2n = Rn2r.t();  
 
  cv::Vec3f  t_r;
  t_r(0) = msg.n2r.translation.x;//cv::Mat pt(1, 3, CV_64FC3); 
  t_r(1) = msg.n2r.translation.y;
  t_r(2) = msg.n2r.translation.z;

  LocalSlam_trajectory.push_back(t_r);
  
  cv::Vec3f  t_c;
  cv::Mat Rbc = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1); 
  double quatc[4];
  double Rr2c_a[9]; 
  if (Plot_actual_cam_pos == true)
  {    
    quatc[0] = msg.r2c.rotation.w;
    quatc[1] = msg.r2c.rotation.x;
    quatc[2] = msg.r2c.rotation.y;
    quatc[3] = msg.r2c.rotation.z;     
    quat2R_row_major(quatc,Rr2c_a);    
    t_c(0) = msg.r2c.translation.x;//cv::Mat pt(1, 3, CV_64FC3); 
    t_c(1) = msg.r2c.translation.y;
    t_c(2) = msg.r2c.translation.z;
  }
  else if(Plot_actual_cam_pos == false)
  {     
    Euler_to_Ra2b_row_major(Robot_cam_axis_x, Robot_cam_axis_y, Robot_cam_axis_z, Rr2c_a);     
    t_c(0) = Robot_cam_pos_x;
    t_c(1) = Robot_cam_pos_y;
    t_c(2) = Robot_cam_pos_z;    
  } 
  cv::Mat Rr2c = cv::Mat(3,3,CV_64FC1,Rr2c_a);
  cv::Mat Rc2r = Rr2c.t();     

  EKFmap.release();
  EKFmap_color.release();
  ANCHORSmap.release();
  ANCHORSmap_color.release();
  Img_matched_feats.clear();
  Img_unmatched_feats.clear();
  Img_matched_anchors.clear();
  Img_unmatched_anchors.clear();


  mutex_get_state.lock();
    robot_pose = cv::Affine3d(Rb*Rr2n, t_r);
    robot_camera_pose = cv::Affine3d(Rbc*Rc2r, t_c);
    new_robot_pose_flag = 1;
    
    for(int i = 0; i < msg.map_state.size();i++)
    {
      cv::Mat pt(1, 3, CV_64FC3);
      pt.at<double>(0,0) = msg.map_state[i].x;
      pt.at<double>(0,1) = msg.map_state[i].y;
      pt.at<double>(0,2) = msg.map_state[i].z;
      EKFmap.push_back(pt);
      cv::Mat col(1, 3, CV_8UC3);
      col.at<char>(0,0) = 0;  // B
      col.at<char>(0,1) = 0;  // G
      col.at<char>(0,2) = 255;  // R   
      EKFmap_color.push_back(col); 
    }
    for(int i = 0; i < msg.map_anchors.size();i++)
    {
      cv::Mat pt(1, 3, CV_64FC3);
      pt.at<double>(0,0) = msg.map_anchors[i].x;
      pt.at<double>(0,1) = msg.map_anchors[i].y;
      pt.at<double>(0,2) = msg.map_anchors[i].z;
      ANCHORSmap.push_back(pt);
      cv::Mat col(1, 3, CV_8UC3);
      col.at<char>(0,0) = 0;  // B
      col.at<char>(0,1) = 255;  // G
      col.at<char>(0,2) = 255;  // R   
      ANCHORSmap_color.push_back(col); 
    }
  
  mutex_get_state.unlock();


    mutex_frame.lock(); 
      for(int i = 0; i < msg.matched_img_feats.size();i++)
      {
        cv::Point2d pm;
        pm.x = msg.matched_img_feats[i].x;
        pm.y = msg.matched_img_feats[i].y;
        Img_matched_feats.push_back(pm);      
      } 
      for(int i = 0; i < msg.unmatched_img_feats.size();i++)
      {
        cv::Point2d pu;
        pu.x = msg.unmatched_img_feats[i].x;
        pu.y = msg.unmatched_img_feats[i].y;
        Img_unmatched_feats.push_back(pu);      
      }
      for(int i = 0; i < msg.matched_img_anchors.size();i++)
      {
        cv::Point2d pm;
        pm.x = msg.matched_img_anchors[i].x;
        pm.y = msg.matched_img_anchors[i].y;
        Img_matched_anchors.push_back(pm);      
      } 
      for(int i = 0; i < msg.unmatched_img_anchors.size();i++)
      {
        cv::Point2d pu;
        pu.x = msg.unmatched_img_anchors[i].x;
        pu.y = msg.unmatched_img_anchors[i].y;
        Img_unmatched_anchors.push_back(pu);      
      }  
    mutex_frame.unlock();  

}
//-----------------------------------------------------------

void PLOTscene::setParameters()
{  
  
   //  Declare node parameters (and default values)
   this->declare_parameter<int>("Viewer_width", 1200);
   this->declare_parameter<int>("Viewer_height", 600);
   this->declare_parameter<double>("Cam_distance",1.0);
   this->declare_parameter<double>("Cam_pos_init_x",0.0);
   this->declare_parameter<double>("Cam_pos_init_y",0.0);
   this->declare_parameter<double>("Cam_pos_init_z",0.0);
   this->declare_parameter<double>("Cam_zoom_delta",1.0);
   this->declare_parameter<double>("Cam_move_delta",1.0);
   this->declare_parameter<bool>("Draw_local_slam_trajectory",true);
   this->declare_parameter<bool>("Plot_actual_cam_pos",true);
   this->declare_parameter<double>("Robot_cam_pos_x",0.0);
   this->declare_parameter<double>("Robot_cam_pos_y",0.0);
   this->declare_parameter<double>("Robot_cam_pos_z",0.0);
   this->declare_parameter<double>("Robot_cam_axis_x",0.0);
   this->declare_parameter<double>("Robot_cam_axis_y",0.0);
   this->declare_parameter<double>("Robot_cam_axis_z",0.0);
     
  
   // Set parameter struct
   this->get_parameter("Viewer_width", Viewer_width);
   this->get_parameter("Viewer_height", Viewer_height);
   this->get_parameter("Cam_distance",Cam_distance);
   this->get_parameter("Cam_pos_init_x",Cam_pos_init_x);
   this->get_parameter("Cam_pos_init_y",Cam_pos_init_y);
   this->get_parameter("Cam_zoom_delta",Cam_zoom_delta);
   this->get_parameter("Cam_move_delta",Cam_move_delta);
   this->get_parameter("Draw_local_slam_trajectory",Draw_local_slam_trajectory);
   this->get_parameter("Plot_actual_cam_pos",Plot_actual_cam_pos);
   this->get_parameter("Robot_cam_pos_x",Robot_cam_pos_x);
   this->get_parameter("Robot_cam_pos_y",Robot_cam_pos_y);
   this->get_parameter("Robot_cam_pos_z",Robot_cam_pos_z);
   this->get_parameter("Robot_cam_axis_x",Robot_cam_axis_x);
   this->get_parameter("Robot_cam_axis_y",Robot_cam_axis_y);
   this->get_parameter("Robot_cam_axis_z",Robot_cam_axis_z);
   //cout << "data origin: " << PAR.Data_origin << endl; 
   
  
}


}  // namespace composition



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(plot::PLOTscene)

