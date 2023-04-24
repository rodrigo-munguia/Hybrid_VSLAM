#include "ekf.hpp"
#include <algorithm>



//--------------------------------------------------------------------------------------------------
// Aux function for removing a single anchor from the AnchorsDATA structure
void delete_i_anchor_(int idx, std::vector<FEAT> & AnchorsDATA)
{
    // remove anchor from table
    AnchorsDATA.erase(AnchorsDATA.begin()+idx);    
}


//------------------------------------------------------------------------------------------------
// Aux function for removing a single feature from the system state vector x and covariance matrix P
void EKF::Delete_i_feat_(int idx)
{

     int idx_i = FeatsDATA[idx].idx_i_state;
     int idx_f = FeatsDATA[idx].idx_f_state;
                            
    // remove feature from state
    x.shed_rows(idx_i,idx_f);
    P.shed_rows(idx_i,idx_f);
    P.shed_cols(idx_i,idx_f);
    FeatsDATA.erase(FeatsDATA.begin()+idx);
                            
    // Update FeatsData
    for(int j = idx; j<FeatsDATA.size(); j++)
    {
        FeatsDATA[j].idx_i_state = FeatsDATA[j].idx_i_state - 3;
        FeatsDATA[j].idx_f_state = FeatsDATA[j].idx_f_state - 3; 
    }


}

// aux func for sort
struct FeatsNotCons
{
    int feat_idx;
    int times_not_considered;
};

bool compareByLength(const FeatsNotCons &a, const FeatsNotCons &b)
{
    return a.times_not_considered > b.times_not_considered;
}


//----------------------------------------------------------------------------------------------------
// Function for deleting visual features (and anchors) from the local map to mantain stable computation time
// Rodrigo M. 2021
void EKF::Visual_delete_feats()
{
    
    std::vector<FeatsNotCons> FN;
    // Delete weak visual features
    // The criteria for defining a "weak" feature is the ratio between matched and not matched times
    if (FeatsDATA.size() > PAR.Visual_delete_minimun_feats_in_state_before_delete)
    {

        for(int i = 0; i < FeatsDATA.size(); i++ )
        {

            int Ttimes = FeatsDATA[i].times_mathed + FeatsDATA[i].times_not_mathed;
            float RatioMatched = (float)FeatsDATA[i].times_mathed/(float)FeatsDATA[i].times_not_mathed;
                //if((Ttimes > 10))
                    //if((RatioMatched < PAR.Visual_delete_minimun_RatioMatched)||(FeatsDATA[i].times_not_considered > PAR.Visual_delete_feats_times_not_considered_before_delete))
                    if((RatioMatched < PAR.Visual_delete_minimun_RatioMatched)&&Ttimes > 10)
                        {
                            
                            Delete_i_feat_(i);
                            //cout << "Weak feat deleted: " << i << endl;
                            i--;
                            
                        }
            FeatsNotCons fn;
            fn.feat_idx = i;
            fn.times_not_considered =  FeatsDATA[i].times_not_considered;           
            FN.push_back(fn) ;               
            
        }

        
    }
    
    // Delete old features
    // the criteria for defining an "old" feature is the times not considered (not predicted to appear in the image)
    // sort struct "FN" in descending order of times_not_considered; 
    std::sort(FN.begin(), FN.end(), compareByLength);
    
    if(FeatsDATA.size() > PAR.Visual_delete_Maximun_number_feats_allowed)
    {
        int n_old_feats_to_del = FeatsDATA.size() - PAR.Visual_delete_Maximun_number_feats_allowed;

        // delete the oldest features.  The oldest feature is the one with the higher numer of times not considered        
        for(int i = 0; i < n_old_feats_to_del; i++)
        {
            int idx_feat_to_del = FN[i].feat_idx;
            Delete_i_feat_(idx_feat_to_del);
            for(int j = i; j < n_old_feats_to_del; j++)
            {
                if(FN[j].feat_idx > idx_feat_to_del)FN[j].feat_idx--;
            }
            int q = 10;
            //cout << "Old feat deleted: " << idx_feat_to_del << endl; 
        }

    }
    
    
    // delete oldest anchors to mantain stable computation time

    std::vector<FeatsNotCons> AN;
    for(int i = 0; i < AnchorsDATA.size(); i++ )
    {
        FeatsNotCons an;
        an.feat_idx = i;
        an.times_not_considered =  AnchorsDATA[i].times_not_considered;           
        AN.push_back(an) ;  
    } 
    
    std::sort(AN.begin(), AN.end(), compareByLength);
    
    if(AnchorsDATA.size() > PAR.Visual_delete_Maximun_number_anchors_allowed)
    {
        int n_old_anchors_to_del = AnchorsDATA.size() - PAR.Visual_delete_Maximun_number_anchors_allowed;

        // delete the oldest features.  The oldest feature is the one with the higher numer of times not considered        
        for(int i = 0; i < n_old_anchors_to_del; i++)
        {
            int idx_anchor_to_del = AN[i].feat_idx;
            delete_i_anchor_(idx_anchor_to_del,  AnchorsDATA);
            for(int j = i; j < n_old_anchors_to_del; j++)
            {
                if(AN[j].feat_idx > idx_anchor_to_del)AN[j].feat_idx--;
            }
            //cout << "Old Anchor deleted: " << idx_anchor_to_del << endl; 
        }

    }



    int q = 10;
    
    

}
//-------------------------------------------------------------------------------------------------------------------
