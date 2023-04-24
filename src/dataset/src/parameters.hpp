/*---------------------------------------------------
Rodrigo Munguía 2021.

System parameters
-----------------------------------------------------
*/
#include <string>

#ifndef PARAMETERS_H
#define PARAMETERS_H

using namespace std;



struct parameters
{
    
    string Dataset;
    double run_time;
    string Dataset_path;
    double x_vel_run_time;
    bool restart;
    long int init_time;
    long int end_time;


} ;   





#endif /*PARAMETERS_H*/