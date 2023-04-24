#include "ekf.hpp"
#include "../../common/anms/anms.h"



//--------------------------------------------------------------------------
// 
arma::vec::fixed<3> EKF::Triangulate_sigle_3d_point(cv::Point2f &uv1d,cv::Point2f &uv2d,arma::mat::fixed<3,3> &Rn2c_1,arma::vec::fixed<3> &t_c2n_1,arma::mat::fixed<3,3> &Rn2c_2,arma::vec::fixed<3> &t_c2n_2 )
{   
    arma::mat::fixed<3,2> dhc_duvd;  
    arma::vec::fixed<3> hc_1 = Inverse_projection_model(uv1d,1,false, cam_parameters,dhc_duvd); // compute inverse projection model
    arma::vec::fixed<3> hc_2 = Inverse_projection_model(uv2d,1,false, cam_parameters,dhc_duvd); // compute inverse projection model
    
    arma::vec::fixed<3> hn_1 = Rn2c_1.t()*hc_1;
    arma::vec::fixed<3> hn_2 = Rn2c_2.t()*hc_2;
    
    arma::vec::fixed<3> el2 = t_c2n_1 - t_c2n_2;
    arma::vec::fixed<3> el1 = t_c2n_2 - t_c2n_1;

    float hn_1_norm = arma::norm(hn_1);
    float hn_2_norm = arma::norm(hn_2);
    float el_norm = arma::norm(el2);

    hn_1 = hn_1/hn_1_norm; // normalized vector (in the nav frame) pointing in the direction of the feature
    hn_2 = hn_2/hn_2_norm; // normalized vector (in the nav frame) pointing in the direction of the feature

    float gama =  acos( arma::as_scalar(hn_1.t()*(el1)) /el_norm   ); 
    float beta =  acos( arma::as_scalar(hn_2.t()*(el2)) /el_norm   ); 
    
    float alfa = 3.141592653589793 - (beta + gama);
    
    float d_i = (el_norm*sin(gama))/sin(alfa);
    
    arma::vec::fixed<3> pt; 
    if (d_i > 1)
    { 
       pt = t_c2n_2 + d_i*hn_2;
    }
    else
    {
       pt = {0,0,0};
    }    
    return pt;

}



//--------------------------------------------------------------------------------------
void EKF::Get_img_points_for_init_feats(FRAME *frame, vector<cv::KeyPoint>& Points, cv::Mat &Descriptors, vector<cv::Mat> &Patches_init, vector<cv::Mat> &Patches_match)
{

    vector<cv::KeyPoint> keyPoints;

    int fastThresh = 10; // Fast threshold. Usually this value is set to be in range [10,35]   
      
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(2000, 1.2f, 8, 16,0,2, cv::ORB::FAST_SCORE,31, 5);
    detector->detect(frame->image, keyPoints);    
    
    /*
    cv::Mat fastDetectionResults; //draw FAST detections
    cv::drawKeypoints(frame->image,Points,fastDetectionResults, cv::Scalar(94.0, 206.0, 165.0, 0.0));
    cv::namedWindow("FAST Detections", cv::WINDOW_AUTOSIZE);     
    cv::imshow( "FAST Detections", fastDetectionResults);
    int k = waitKey(1);
    */
    
    //----- Select strongest and spatially distributed keypoints 
        vector<float> responseVector;
        for (unsigned int i =0 ; i<keyPoints.size(); i++) responseVector.push_back(keyPoints[i].response);
        vector<int> Indx(responseVector.size()); 
        std::iota (std::begin(Indx), std::end(Indx), 0);
        cv::sortIdx(responseVector, Indx, cv::SORT_DESCENDING);
        vector<cv::KeyPoint> keyPointsSorted;
        for (unsigned int i = 0; i < keyPoints.size(); i++) keyPointsSorted.push_back(keyPoints[Indx[i]]);

        int numRetPoints = PAR.Images_number_candidate_points_detected; //choose exact number of return points
    
        float tolerance = 0.6; // tolerance of the number of return points

        vector<cv::KeyPoint> sscKP = Ssc(keyPointsSorted,numRetPoints,tolerance,frame->image.cols,frame->image.rows);
    //-----------------------------------    
    
    /*
    cv::Mat anmsResults; //draw FAST detections
    cv::drawKeypoints(frame->image,sscKP,anmsResults, cv::Scalar(94.0, 206.0, 165.0, 0.0));
    cv::namedWindow("anms keypoints", cv::WINDOW_AUTOSIZE);     
    cv::imshow( "anms keypoints", anmsResults);
    int k = waitKey(1);
    */

    Remove_close_points(Points, sscKP); // remove visual points that are to close to already mapped visual features

    detector->compute(frame->image, Points, Descriptors); // compute descriptors for Keypoints

     
    int hp_init = PAR.Images_half_patch_size_when_initialized;
    int hp_match = PAR.Images_half_patch_size_when_matching;
    
    //cv::Mat rectResults; //
    //cv::drawKeypoints(frame->image,sscKP,rectResults, cv::Scalar(94.0, 206.0, 165.0, 0.0));
    
    /*
    for(int i = 0 ; i < Points.size(); i ++)
    {
       cv::Rect rect_init(Points[i].pt.x - hp_init, Points[i].pt.y - hp_init, hp_init*2, hp_init*2);
       cv::Rect rect_match(Points[i].pt.x - hp_match, Points[i].pt.y - hp_match, hp_match*2, hp_match*2);       
       
       cv::Mat patch_init(frame->image,rect_init);
       cv::Mat patch_match(frame->image,rect_match);

       Patches_init.push_back(patch_init);
       Patches_match.push_back(patch_match);
       //cv::rectangle(rectResults, rect_init, cv::Scalar(0, 255, 0));
    } 
    */
    //cv::namedWindow("patch keypoints", cv::WINDOW_AUTOSIZE);     
    //cv::imshow( "patch keypoints", rectResults);
    //int k = waitKey(1);

   

}



//---------------------------------------------------------------------------------------------------------
void EKF::Remove_close_points(vector<cv::KeyPoint>& output_KP, vector<cv::KeyPoint>&  input_KP)
{
    //-- remove visual points close to the image border
    int e_b = PAR.Images_half_patch_size_when_initialized + 1;
    for(int i = 0; i < input_KP.size(); i++)
    {
        if( (input_KP[i].pt.x < e_b)||(input_KP[i].pt.y < e_b)||(input_KP[i].pt.x > PAR.Mono_cam_img_cols-e_b)||(input_KP[i].pt.y > PAR.Mono_cam_img_rows-e_b)) 
        {

           input_KP.erase(input_KP.begin()+i);
           i--;  

        }            

    }

 


 //---- remove visual points close to already mapped visual features (or anchors)
 if (FeatsDATA.size() > 0 )
    {
        for(int i = 0; i < input_KP.size(); i++) // check each candidate point to be at a minimun distance from predicted features 
        {
            
            bool add = true;
            for(int j = 0; j < FeatsDATA.size(); j++)
            {
                double d =  sqrt( pow(input_KP[i].pt.x - FeatsDATA[j].PredictedPoint.x, 2) + pow(input_KP[i].pt.y - FeatsDATA[j].PredictedPoint.y, 2) );

                if (d < PAR.Images_minimun_distance_new_points)
                {
                    add = false;
                    break;
                }
            }
            for(int j = 0; j < AnchorsDATA.size(); j++ )
            {
                double d =  sqrt( pow(input_KP[i].pt.x - AnchorsDATA[j].PredictedPoint.x, 2) + pow(input_KP[i].pt.y -AnchorsDATA[j].PredictedPoint.y, 2) );
                if (d < PAR.Images_minimun_distance_new_points)
                {
                    add = false;
                    break;
                }

            }     

            if(add == true)
            {
                output_KP.push_back(input_KP[i]);
            }
        
        }

    
    }
    else
    {
        output_KP = input_KP;
    } 


}
//----------------------------------------------------------------------------------------------------------