#ifndef ANMS_H
#define ANMS_H

#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "nanoflann.hpp"
#include "range-tree/ranget.h"

using namespace std;
using namespace nanoflann;
typedef unsigned long t_r;

vector<cv::KeyPoint> TopN(vector<cv::KeyPoint> keyPoints, int numRetPoints);







vector<cv::KeyPoint> BrownANMS(vector<cv::KeyPoint> keyPoints, int numRetPoints); 



vector<cv::KeyPoint> Sdc(vector<cv::KeyPoint> keyPoints, int numRetPoints, float tolerance, int cols, int rows);

/*kdtree algorithm*/
template <typename T>
struct PointCloud
{
    struct Point
    {
        T  x,y;
    };
    std::vector<Point>  pts;
    inline size_t kdtree_get_point_count() const { return pts.size(); } // Must return the number of data points
    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
    {
        const T d0=p1[0]-pts[idx_p2].x;
        const T d1=p1[1]-pts[idx_p2].y;
        return d0*d0+d1*d1;
    }
    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim==0) return pts[idx].x;
        else if (dim==1) return pts[idx].y;
        return 0;
    }
    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};


template <typename T>
void generatePointCloud(PointCloud<T> &point, vector<cv::KeyPoint> keyPoints);



vector<cv::KeyPoint> KdTree(vector<cv::KeyPoint> keyPoints, int numRetPoints,float tolerance,int cols,int rows);

vector<cv::KeyPoint> RangeTree(vector<cv::KeyPoint> keyPoints, int numRetPoints,float tolerance, int cols, int rows);

vector<cv::KeyPoint> Ssc(vector<cv::KeyPoint> keyPoints, int numRetPoints,float tolerance, int cols, int rows);

void VisualizeAll(cv::Mat Image, vector<cv::KeyPoint> keyPoints, string figureTitle);

#endif // ANMS_H
