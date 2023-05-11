#include "cloop.hpp"

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>


void CLOOP::Pose_SLAM(arma::vec::fixed<3> &pos,std::vector<arma::vec::fixed<3>> &delta_kf_pos,std::vector<int> &idx_kf_opt)
{

    // Create a factor graph
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    
    /*
    double sigma_att = PAR.BA_sigma_kf_att/10000;
    double sigma_xy = PAR.BA_sigma_kf_xy/10000;
    double sigma_z = PAR.BA_sigma_kf_z/10000;
    auto ref_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << sigma_att,sigma_att,sigma_att,sigma_xy,sigma_xy,sigma_z).finished());
    */

    int idx_kf_ref = Gmap.idx_ref_pose_slam;
    
     //  Create the data structure to hold the initialEstimate estimate to the solution
    for(int i = idx_kf_ref ; i < Gmap.KeyFDATA.size() ; i++)
    {
        gtsam::Rot3 R(1,0,0,0,1,0,0,0,1);
        
        gtsam::Point3 t;
        t(0) = Gmap.KeyFDATA[i].t_c2n[0];
        t(1) = Gmap.KeyFDATA[i].t_c2n[1];
        t(2) = Gmap.KeyFDATA[i].t_c2n[2];

        gtsam::Pose3 pose(R,t);

        initial.insert(gtsam::Symbol('x', i), pose);

        if(i == idx_kf_ref)
        {
            double ref_sigma_att = PAR.CL_clo_sigma_kf_att/10000;
            double ref_sigma_xy = PAR.CL_clo_sigma_kf_xy/10000;
            double ref_sigma_z = PAR.CL_clo_sigma_kf_z/10000;
            auto ref_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << ref_sigma_att,ref_sigma_att,ref_sigma_att,ref_sigma_xy,ref_sigma_xy,ref_sigma_z).finished()); 
            graph.addPrior(gtsam::Symbol('x', i), pose, ref_noise);  // add directly to graph
        } 
    }
    
    double odo_sigma_att = PAR.CL_odo_sigma_kf_att;
    double odo_sigma_xy = PAR.CL_odo_sigma_kf_xy;
    double odo_sigma_z = PAR.CL_odo_sigma_kf_z;
    auto odo_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << odo_sigma_att,odo_sigma_att,odo_sigma_att,odo_sigma_xy,odo_sigma_xy,odo_sigma_z).finished()); 
    // Add odometry factors
    for(int i = idx_kf_ref+1 ; i < Gmap.KeyFDATA.size() ; i++)
    {
        arma::vec::fixed<3> odo_i = Gmap.KeyFDATA[i].t_c2n - Gmap.KeyFDATA[i-1].t_c2n;
        
        gtsam::Rot3 R(1,0,0,0,1,0,0,0,1);
        
        gtsam::Point3 t;
        t(0) = odo_i[0];
        t(1) = odo_i[1];
        t(2) = odo_i[2];

        gtsam::Pose3 pose_odo(R,t);

        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('x', i-1), gtsam::Symbol('x', i), pose_odo, odo_noise);    
    
    }
   
    // Add the loop closure constraint
    double lc_sigma_att = PAR.CL_clo_sigma_kf_att;
    double lc_sigma_xy = PAR.CL_clo_sigma_kf_xy;
    double lc_sigma_z = PAR.CL_clo_sigma_kf_z;
    auto lc_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << lc_sigma_att,lc_sigma_att,lc_sigma_att,lc_sigma_xy,lc_sigma_xy,lc_sigma_z).finished()); 
    arma::vec::fixed<3> clo_c = Gmap.KeyFDATA[idx_kf_ref].t_c2n - pos;
    gtsam::Rot3 R(1,0,0,0,1,0,0,0,1);
    gtsam::Point3 t;
    t(0) = clo_c[0];
    t(1) = clo_c[1];
    t(2) = clo_c[2];
    gtsam::Pose3 pose_cl(R,t);
    int idx_c_kf = Gmap.KeyFDATA.size() - 1;
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('x', idx_c_kf), gtsam::Symbol('x', idx_kf_ref), pose_cl, lc_noise);    


    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    cout << "cloop-> ---- Close loop results ---------------- " << endl;
    cout << "cloop-> initial error = " << graph.error(initial) << endl;
    cout << "cloop-> final error = " << graph.error(result) << endl;
    cout << "cloop-> number of kf: " << Gmap.KeyFDATA.size() << endl;
    cout << "cloop-> last kf pose:           " << Gmap.KeyFDATA[Gmap.KeyFDATA.size()-1].t_c2n.t() << endl;
    cout << "cloop-> last kf optimized pose: " << result.at<gtsam::Pose3>(gtsam::Symbol('x', idx_c_kf) ).translation().transpose() << endl;
 

    for(int i = idx_kf_ref ; i < Gmap.KeyFDATA.size() ; i++)
    {
        Eigen::Matrix<double, 3, 1> t_o;      
        t_o = result.at<gtsam::Pose3>(gtsam::Symbol('x', i)).translation();       
        
        arma::vec::fixed<3> dt_kf;
        dt_kf[0] = t_o[0] -  Gmap.KeyFDATA[i].t_c2n[0];
        dt_kf[1] = t_o[1] -  Gmap.KeyFDATA[i].t_c2n[1];
        if (PAR.CL_update_z == true)
        {
            dt_kf[2] = t_o[2] -  Gmap.KeyFDATA[i].t_c2n[2];
        }
        else
        {
           dt_kf[2] = 0; 
        }    
        
        idx_kf_opt.push_back(i);
        delta_kf_pos.push_back(dt_kf);

        if(PAR.Stats){
          store.cloop_stats.back().error_correction = arma::norm(dt_kf);
        }   


    }    
    

}