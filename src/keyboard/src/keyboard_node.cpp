#include <cstdio>
#include "iostream"
#include "string"
#include "vpKeyboard.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/simple_serv.hpp"

using namespace std;


///---------------------------------
 void printcommands()
 {
    std::cout << "\n| Dataset commands :\n"             
                   "|   'q' to quit.\n"
                   "|   'p'-> Play/Pause  r-> Reset\n "
                   "|   '-'-> zoom out '+' ->  zoom in '1'-> x-y view '2' -> x-z view  '3'->y-z view\n"
                   "|   '8'-> view up '5' ->  view down '4'-> view left '6' -> view right  'c'-> clear plot\n"
                   "|   '9'-> save screenshot 'l' -> log statistics\n"                      
                 << std::endl;
   
 }   


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  vpKeyboard keyboard;
  int k = 0;

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("keyboard_dataset");
  
  // create client services

  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_dataset =
    node->create_client<interfaces::srv::SimpleServ>("dataset_service");

  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_ekf_run_ =
    node->create_client<interfaces::srv::SimpleServ>("ekf_run_service");
  
  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_globalslam_run_ =
    node->create_client<interfaces::srv::SimpleServ>("globalslam_run_service");
  // --------------
  
 auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
           request->cmd = 1;

 while (!client_dataset->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }          

  auto result = client_dataset->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  { 
    if (result.get()->response == true)
    {
      cout << "keyboard->  Dataset component response: Ok" << endl;
    }
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %ld", result.get()->response);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "keyboard->  Failed to call service dataset_service");
  }

//----------------------------------------------------------------
//  check for local slam component
rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_localslam =
    node->create_client<interfaces::srv::SimpleServ>("ekf_run_service");

auto request_2 = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request_2->cmd = 'r';           
auto result_2 = client_localslam->async_send_request(request_2);

if (rclcpp::spin_until_future_complete(node, result_2) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                  { 
                    if (result_2.get()->response == true) cout << "keyboard->  Local slam component response: Ok" << endl;
                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %ld", result.get()->response);
                  } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "keyboard->  Failed to call service ekf_run_service");
                  } 
//-----------------------------------------------------------------
//  check for plot component
rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_plot =
    node->create_client<interfaces::srv::SimpleServ>("plot_service");

auto request_3 = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request_3->cmd = 'r';           
auto result_3 = client_plot->async_send_request(request_3);

if (rclcpp::spin_until_future_complete(node, result_3) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                  { 
                    if (result_3.get()->response == true) cout << "keyboard->  Plot component response: Ok" << endl;
                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %ld", result.get()->response);
                  } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "keyboard->  Failed to call plot_service");
                  } 

printcommands();

while (k != 'q') 
      {
 
         k = '0'; // If no key is hit, we send a non-assigned key
         if (keyboard.kbhit()) 
         {
           k = keyboard.getchar();
          
           
           if (k == 'p' || k == 'r')
           {  
              //cout << k << endl;
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;           
              result = client_dataset->async_send_request(request);

              if (rclcpp::spin_until_future_complete(node, result) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                  { 
                    if (result.get()->response == true) cout << "keyboard-> Dataset response: OK" << endl;
                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %ld", result.get()->response);
                  } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service dataset");
                  }                   
            }
            if ( k == 'l')
            {
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_ekf_run_->async_send_request(request); 
              result = client_globalslam_run_->async_send_request(request); 

            }
            if ( k == 45)
            {
              // '-' ->  plot zoom out
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            if ( k == 43)
            {
              // '+' ->  plot zoom in
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request);
            }
            if ( k == '1')
            {
              // '1' ->  x-y view 
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request);
            }
            if ( k == '2')
            {
              // '2' ->  x-z view
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            if ( k == '3')
            {
              // '3' ->  y-z view 
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            if ( k == '8')
            {
              // '8' ->  cam up 
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            if ( k == '4')
            {
              // '4' ->  cam left 
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            if ( k == '6')
            {
              // '6' ->  cam right 
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            if ( k == '5')
            {
              // '5' ->  cam down 
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            if ( k == 'c')
            {
              // 'c' ->  clear plot
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            if (k == '9')
            {
              request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              result = client_plot->async_send_request(request); 
            }
            
            //cout << k << endl;

            printcommands();
            

         }
         //running = handleKeyboardInput(drone, k,ekf,locks,par,gmap,cloop,control,stop_control);
         

         
       }

  return 0;
}
