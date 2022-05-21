#include "gmap.hpp"



// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols

#include <gtsam/inference/Symbol.h>
// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.

#include <gtsam/slam/ProjectionFactor.h>
// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
// trust-region method known as Powell's Degleg
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

//#include <gtsam/slam/GeneralSFMFactor.h>

void GMAP::Bundle_adjustment()
{

    double fc1 = cam_parameters.fc[0];
    double fc2 = cam_parameters.fc[1];
    double cc1 = cam_parameters.cc[0];
    double cc2 = cam_parameters.cc[1];
    double alpha_c = cam_parameters.alpha_c;
    // Define the camera calibration parameters
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2( fc1, fc2, alpha_c, cc1, cc2) );
     // Define the camera observation noise model
    auto measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, PAR.BA_sigma_uv); // pixel error in (x,y)
    // Create a factor graph
    gtsam::NonlinearFactorGraph graph;

    gtsam::Values initial;
     
    
    std::vector<int64> idx_points;
    
    //------ define kf (and associated points) to be optimized
        
        

       // idx_kf_min_ref_for_search_matches = Gmap.KeyFDATA.size() - PAR.BA_max_n_kf_optimized;    
       // if( idx_kf_min_ref_for_search_matches < 0)idx_kf_min_ref_for_search_matches = 0;
        
        std::vector<int> idx_kf_to_optimize;
        int n_kf_to_optimize = 0;
        if(Gmap.KeyFDATA.size() < PAR.BA_max_n_kf_optimized)
        {
          n_kf_to_optimize = Gmap.KeyFDATA.size();
        }
        else
        {
          n_kf_to_optimize = PAR.BA_max_n_kf_optimized;
        }
        
       
        // consider at least the n-last keyframes to optimize
        // the difference between BA_max_n_kf_optimized and BA_min_n_latest_kf_to_optimize is the number of "oldest" kf
        // that will be tried to be included for optimization
        
        // to ensure that PAR.BA_min_n_latest_kf_to_optimize is not larger than PAR.BA_min_n_latest_kf_to_optimize
        if (PAR.BA_min_n_latest_kf_to_optimize > PAR.BA_max_n_kf_optimized)PAR.BA_min_n_latest_kf_to_optimize = PAR.BA_max_n_kf_optimized;
        
        int n_latest_kf = PAR.BA_min_n_latest_kf_to_optimize ;
        
        if (n_latest_kf < PAR.BA_max_n_kf_optimized)
        {
          std::vector<int> vl_kf = Get_n_oldest_visually_linked_kf(Gmap.KeyFDATA.size()-1,PAR.BA_max_n_kf_optimized-(n_latest_kf+1), 2 );     
          idx_kf_to_optimize = vl_kf;
        } 
             
        
        int idx_kf_t = Gmap.KeyFDATA.size()-1;
        while(idx_kf_to_optimize.size() < n_kf_to_optimize )
        {      
          if( find(idx_kf_to_optimize.begin(), idx_kf_to_optimize.end(), idx_kf_t) == idx_kf_to_optimize.end() )
          {
            idx_kf_to_optimize.push_back(idx_kf_t);
          }   
          idx_kf_t--;
        }
        sort(idx_kf_to_optimize.begin(), idx_kf_to_optimize.end());
    //----------------------------------------------------------------------------------------

   // for (int i = idx_kf_min_ref_for_search_matches; i < Gmap.KeyFDATA.size(); i++)
    for (int k = 0 ; k < idx_kf_to_optimize.size(); k++)
    {   
        int idx_kf = idx_kf_to_optimize[k];
        arma::mat::fixed<3,3> Rc2n = Gmap.KeyFDATA[idx_kf].Rn2c.t();

        gtsam::Rot3 R(
            Rc2n.at(0,0),
            Rc2n.at(0,1),
            Rc2n.at(0,2),
            Rc2n.at(1,0),
            Rc2n.at(1,1),
            Rc2n.at(1,2),
            Rc2n.at(2,0),
            Rc2n.at(2,1),
            Rc2n.at(2,2)
        );
        
        gtsam::Point3 t;
        t(0) = Gmap.KeyFDATA[idx_kf].t_c2n[0];
        t(1) = Gmap.KeyFDATA[idx_kf].t_c2n[1];
        t(2) = Gmap.KeyFDATA[idx_kf].t_c2n[2];

        gtsam::Pose3 pose(R,t);

        // Add prior for the first and second Kf
        double sigma_att;
        double sigma_xy;
        double sigma_z;
        
        if ((k == 0)) 
         {   
          // Add a prior on pose x0. This indirectly specifies where the origin is.            
          sigma_att = PAR.BA_sigma_kf_att/10000;
          sigma_xy = PAR.BA_sigma_kf_xy/10000;
          sigma_z = PAR.BA_sigma_kf_z/10000;
         } 
         else if(k==1)
         {
          sigma_att = PAR.BA_sigma_kf_att/10;
          sigma_xy = PAR.BA_sigma_kf_xy/10;
          sigma_z = PAR.BA_sigma_kf_z/10;
         }         
         else
         { 
          sigma_att = PAR.BA_sigma_kf_att;
          sigma_xy = PAR.BA_sigma_kf_xy;
          sigma_z = PAR.BA_sigma_kf_z;
         } 
            auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << sigma_att,sigma_att,sigma_att,sigma_xy,sigma_xy,sigma_z).finished());
            //graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('x', 0), pose, pose_noise); // add directly to graph
            graph.addPrior(gtsam::Symbol('x', idx_kf), pose, pose_noise);  // add directly to graph
       // }        
        
        initial.insert(gtsam::Symbol('x', idx_kf), pose);

        // ------------  add visual measurements
        for ( int j = 0; j < Gmap.KeyFDATA[idx_kf].Idx_Matched_points.size(); j++)
        {
          int64 idx_pt = Gmap.KeyFDATA[idx_kf].Idx_Matched_points[j];
          
         cv::Point2f ud_d = Gmap.KeyFDATA[idx_kf].UV_Matched_points[j];

          cv::Point2f uv_u;
          uv_u = Undistort_a_point( ud_d,cam_parameters,1 );  // undistort point
          
          gtsam::Point2 pt;
          pt(0) =  uv_u.x;
          pt(1) =  uv_u.y;
          
          //graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(pt, measurement_noise, gtsam::Symbol('x', idx_kf), gtsam::Symbol('l', i), gtsam::Symbol('K', 0));
          graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(pt, measurement_noise, gtsam::Symbol('x', idx_kf), gtsam::Symbol('l', idx_pt), K);
          
         // cout << "add visual measurement, kf: " << i << " with point: " << idx_pt << endl;
          auto it = std::find(idx_points.begin(), idx_points.end(), idx_pt);

            if(it==idx_points.end())
            {
              // if index of point has not been already added then
              idx_points.push_back(idx_pt);
            }          

        } // for ( int j = 0; j < Gmap.KeyFDATA[i].Idx_Matched_points.size(); j++)
        //--------------------------------------------------------------
       
      // cout << "Kf: " << idx_kf << " n matches: " << Gmap.KeyFDATA[idx_kf].Idx_Matched_points.size()  << endl;

    } // for (int i = idx_Fixed_kf_ref; i < Gmap.KeyFDATA.size(); i++)
    //cout << endl;
    //--- add prior of point positions
     bool init_prior = false;
    for( int i = 0; i < idx_points.size(); i++)
    {
        int64 idx_pt = idx_points[i];

        gtsam::Point3 Pt;
         Pt(0) = Gmap.AnchorsDATA[idx_pt].AnchorState[0];
         Pt(1) = Gmap.AnchorsDATA[idx_pt].AnchorState[1]; 
         Pt(2) = Gmap.AnchorsDATA[idx_pt].AnchorState[2]; 
         
         if (!init_prior) 
         {
              init_prior = true;
               gtsam::noiseModel::Isotropic::shared_ptr point_noise = gtsam::noiseModel::Isotropic::Sigma(3, PAR.BA_sigma_kf_pt);
              graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(gtsam::Symbol('l', idx_pt), Pt, point_noise);
         }  
         //cout << "add point to prior: " << idx_pt << endl;
         initial.insert<gtsam::Point3>(gtsam::Symbol('l', idx_pt), Pt);


    }

  
    /* Optimize the graph and print results */
  //gtsam::Values result = gtsam::DoglegOptimizer(graph, initial).optimize();
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  //result.print("Final results:\n");
  
  /*
  cout << "---- bundle adjustment results ---------------- " << endl;
  cout << "initial error = " << graph.error(initial) << endl;
  cout << "final error = " << graph.error(result) << endl;
  cout << "number of kf: " << Gmap.KeyFDATA.size() << endl;
  cout << "last kf pose:           " <<Gmap.KeyFDATA[Gmap.KeyFDATA.size()-1].t_c2n.t() << endl;
  //cout << "last kf optimized pose: " <<result.at<gtsam::Pose3>(gtsam::Symbol('x', Gmap.KeyFDATA.size()-1)).translation().transpose() << endl;
  cout << "last kf optimized pose: " <<result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf_to_optimize[idx_kf_to_optimize.size()-1]) ).translation().transpose() << endl;
 */
  
  // --------------- Update results to the global map------------------------------------------------------
    
    //--- check for large innovations
    int idx_last_kf = Gmap.KeyFDATA.size()-1;
    bool f_valid_update = true;
    for (int k = 0 ; k < idx_kf_to_optimize.size(); k++)
    {   
      int idx_kf = idx_kf_to_optimize[k];
      Eigen::Matrix<double, 3, 3> R;
      Eigen::Matrix<double, 3, 1> t_o;            
      R = result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf)).rotation().matrix();
      t_o = result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf)).translation();      
      arma::mat R_arma_t = arma::mat(R.data(), R.rows(), R.cols(),false, false);
      arma::mat::fixed<3,3> R_o = R_arma_t.t();
      arma::vec::fixed<3> D_kf_n;      
      D_kf_n[0] = t_o[0] - Gmap.KeyFDATA[idx_kf].t_c2n[0];
      D_kf_n[1] = t_o[1] - Gmap.KeyFDATA[idx_kf].t_c2n[1];
      D_kf_n[2] = t_o[2] - Gmap.KeyFDATA[idx_kf].t_c2n[2];         
      //double Delta_norm = arma::norm(D_kf_n, 2);
      double Delta_norm = sqrt(pow(D_kf_n[0],2) + pow(D_kf_n[1],2)); 
      if (Delta_norm > PAR.BA_max_delta_kf_pos)
      {
        f_valid_update = false;
        cout << "Delta_norm for n-kf optimized too large!! " << endl ;
      }      
    }    
    //-----------------------------------



    //for (int i = idx_kf_min_ref_for_search_matches; i < Gmap.KeyFDATA.size(); i++)
    for (int k = 0 ; k < idx_kf_to_optimize.size(); k++)
    {   
      int idx_kf = idx_kf_to_optimize[k];   

      Eigen::Matrix<double, 3, 3> R;
      Eigen::Matrix<double, 3, 1> t_o;
            
      R = result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf)).rotation().matrix();
      t_o = result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_kf)).translation();
      
      arma::mat R_arma_t = arma::mat(R.data(), R.rows(), R.cols(),false, false);
      arma::mat::fixed<3,3> R_o = R_arma_t.t();
      
      if(idx_kf == idx_last_kf)
      {
        Delta_kf_n[0] = t_o[0] - Gmap.KeyFDATA[idx_last_kf].t_c2n[0];
        Delta_kf_n[1] = t_o[1] - Gmap.KeyFDATA[idx_last_kf].t_c2n[1];
        Delta_kf_n[2] = t_o[2] - Gmap.KeyFDATA[idx_last_kf].t_c2n[2];
      }  
            
      /*
      cout << R_o << endl;
      cout << Gmap.KeyFDATA[i].Rn2c << endl;
      cout << t_o << endl;
      cout << Gmap.KeyFDATA[i].t_c2n << endl;
      */
      if(PAR.BA_update_kf_att == true && f_valid_update == true)
      {
        Gmap.KeyFDATA[idx_kf].Rn2c = R_o;
      }  
      
      if(PAR.BA_update_kf_pos == true && f_valid_update == true)
      {
        Gmap.KeyFDATA[idx_kf].t_c2n[0] = t_o[0];
        Gmap.KeyFDATA[idx_kf].t_c2n[1] = t_o[1];
        Gmap.KeyFDATA[idx_kf].t_c2n[2] = t_o[2];
      }         
     
    }  

  if(PAR.BA_update_map== true && f_valid_update == true)
  {
    for( int i = 0; i < idx_points.size(); i++)
    {
        int64 idx_pt = idx_points[i];
        
        Eigen::Vector3d pt = result.at<gtsam::Point3>(gtsam::Symbol('l', idx_pt));
        
       Gmap.AnchorsDATA[idx_pt].AnchorState[0] = pt[0];
       Gmap.AnchorsDATA[idx_pt].AnchorState[1] = pt[1];
       Gmap.AnchorsDATA[idx_pt].AnchorState[2] = pt[2];

    }    
  }
  //----------------------------------------------------------------------
   //cout << "last kf optimized pose:      " <<Gmap.KeyFDATA[Gmap.KeyFDATA.size()-1].t_c2n.t() << endl;


}


//------------------------------------------------------------------------------------------



