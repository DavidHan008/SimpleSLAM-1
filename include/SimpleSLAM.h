#ifndef _SIMPLESLAM_H_
#define _SIMPLESLAM_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <set>

using namespace std;


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "Vector.h"
#include "Feature.h"
#include "Tool.h"

const double myPI = acos(-1);
const double MIN_FEATURE_THRES = myPI / 6;
const double MAX_FEATURE_THRES = myPI * 11 / 20;

// #define DOWN_SAMPLE_ENABLED

class SimpleSLAM
{
    public:
        SimpleSLAM() = delete;
        SimpleSLAM(string path_base, string path_curr);

    public:
        bool featureExtractionAndMapping();
        bool ridigMotionCalculation();
        bool pointcloudMerging();
        bool pointcloudStorage(string);

    public:
        void SimpleSLAM_INFO(string str);
        void SimpleSLAM_ERROR(string str);

    private:   
        bool pcdReadingFromDisk(string path_base, string path_curr);

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_curr;

        pcl::PointCloud<pcl::PointXYZ>::Ptr final_base;
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_curr;

        pcl::PointCloud<pcl::PointXYZ> cloud;

        vector<Feature> feature_base;
        vector<Feature> feature_curr;

        Point base_p1;
        Point base_p2;

        Point curr_p1;
        Point curr_p2;

        Vector translation_from_curr_to_base;
        double rotation_angle;

        double error_x;
        double error_y;
};

#endif // _SIMPLESLAM_H_