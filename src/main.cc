#include <iostream>
#include <cmath>
#include <vector>
#include <set>

using namespace std;

#include "SimpleSLAM.h"


int main(int argc, char ** argv)
{
    if(argc != 3)
    {
        cout << "Usage Error!" << endl
             << "Usage: program_name basePcd currPcd" << endl;
    }

    string basePath(argv[1]);
    string currPath(argv[2]);

    SimpleSLAM slamer(basePath, currPath);

    if(!slamer.featureExtractionAndMapping())
    {
        slamer.SimpleSLAM_INFO("Feature Extraction or Mapping Failed.");
        exit(-1);
    }
    slamer.SimpleSLAM_INFO("Feature Extraction or Mapping Done.");

    if(!slamer.ridigMotionCalculation())
    {
        slamer.SimpleSLAM_INFO("Motion Estimation Failed.");
        exit(-1);
    }
    slamer.SimpleSLAM_INFO("Motion Estimation Done.");

    if(!slamer.pointcloudMerging())
    {
        slamer.SimpleSLAM_INFO("Pointcloud Merging Failed.");
        exit(-1);
    }
    slamer.SimpleSLAM_INFO("Pointcloud Merging Done.");

    if(!slamer.pointcloudStorage("result.pcd"))
    {
        slamer.SimpleSLAM_INFO("Result Output Failed.");
        exit(-1);
    }
    slamer.SimpleSLAM_INFO("Result Output Done.");

}
