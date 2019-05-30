#include "SimpleSLAM.h"

SimpleSLAM::SimpleSLAM(string path_base, string path_curr):cloud_base(new pcl::PointCloud<pcl::PointXYZ>), 
                                                           cloud_curr(new pcl::PointCloud<pcl::PointXYZ>),
                                                           final_base(new pcl::PointCloud<pcl::PointXYZ>),
                                                           final_curr(new pcl::PointCloud<pcl::PointXYZ>)
{
    if(!pcdReadingFromDisk(path_base, path_curr))
    {
        exit(-1);
    }

#ifdef DOWN_SAMPLE_ENABLED
    pcl::PCLPointCloud2::Ptr pcloud_base(new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr pcloud_curr(new pcl::PCLPointCloud2 ());

    pcl::toPCLPointCloud2(*cloud_base, *pcloud_base);
    pcl::toPCLPointCloud2(*cloud_curr, *pcloud_curr);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    
    sor.setInputCloud(pcloud_base);
    sor.setLeafSize(0.05f, 0.05f, 0.01f);
    sor.filter(*pcloud_base);

    sor.setInputCloud(pcloud_curr);
    sor.setLeafSize(0.05f, 0.05f, 0.01f);
    sor.filter(*pcloud_curr);

    pcl::fromPCLPointCloud2(*pcloud_base, *cloud_base);
    pcl::fromPCLPointCloud2(*pcloud_curr, *cloud_curr);

    // SimpleSLAM_INFO("Point Cloud Downsampling: Done.")
#endif
}

bool SimpleSLAM::pcdReadingFromDisk(string path_base, string path_curr)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_base, *cloud_base) == -1) //* load the base map
    {
        SimpleSLAM_ERROR("Cannot read file " + path_base);
        return false;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_curr, *cloud_curr) == -1) //* load the current map
    {
        SimpleSLAM_ERROR("Cannot read file " + path_curr);
        return false;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_base, *final_base) == -1) //* load the base map
    {
        SimpleSLAM_ERROR("Cannot read file " + path_base);
        return false;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_curr, *final_curr) == -1) //* load the current map
    {
        SimpleSLAM_ERROR("Cannot read file " + path_curr);
        return false;
    }

    return true;
}

bool SimpleSLAM::featureExtractionAndMapping()
{
    unsigned int psize_base = cloud_base->width * cloud_base->height;
    unsigned int psize_curr = cloud_curr->width * cloud_curr->height;

    for(unsigned int i = 1; i < psize_base - 1; i++)
    {
        double tempx;
        double tempy;

        tempx = cloud_base->points[i - 1].x - cloud_base->points[i].x;
        tempy = cloud_base->points[i - 1].y - cloud_base->points[i].y;
        Vector left_vec(tempx, tempy);

        tempx = cloud_base->points[i + 1].x - cloud_base->points[i].x;
        tempy = cloud_base->points[i + 1].y - cloud_base->points[i].y;
        Vector right_vec(tempx, tempy);

        double angle = left_vec.calcTheta(right_vec);

        if(MIN_FEATURE_THRES <= angle && angle <= MAX_FEATURE_THRES)
        {
            feature_base.push_back(Feature(angle, i));
        }
    }

    for(unsigned int i = 1; i < psize_curr - 1; i++)
    {
        double tempx;
        double tempy;

        tempx = cloud_curr->points[i - 1].x - cloud_curr->points[i].x;
        tempy = cloud_curr->points[i - 1].y - cloud_curr->points[i].y;
        Vector left_vec(tempx, tempy);

        tempx = cloud_curr->points[i + 1].x - cloud_curr->points[i].x;
        tempy = cloud_curr->points[i + 1].y - cloud_curr->points[i].y;
        Vector right_vec(tempx, tempy);

        double angle = fabs(left_vec.calcTheta(right_vec));

        if(MIN_FEATURE_THRES <= angle && angle <= MAX_FEATURE_THRES)
        {
            feature_curr.push_back(Feature(angle, i));
        }
    }

    vector<Info> feature_match;
    set<int> matched_index;

    int fsize_base = feature_base.size();  // cout << "fsize_base    " << fsize_base << endl;
    int fsize_curr = feature_curr.size();  // cout << "fsize_curr    " << fsize_curr << endl;

    

    if(fsize_base > 2 && fsize_curr > 2)
    {
        for(int i = 2; i < fsize_base - 2; i++)
        {
            double min_score = 1000;
            double min_index_base = 2;
            double min_index_curr = 2;

            for(int j = 2; j < fsize_curr; j++)
            {
                double score = 0;
                score += 0.3 * fabs(feature_base[i - 2].feature - feature_curr[j - 2].feature);
                score += 0.6 * fabs(feature_base[i - 1].feature - feature_curr[j - 1].feature);
                score += 1.0 * fabs(feature_base[  i  ].feature - feature_curr[  j  ].feature);
                score += 0.6 * fabs(feature_base[i + 1].feature - feature_curr[j + 1].feature);
                score += 0.3 * fabs(feature_base[i + 2].feature - feature_curr[j + 2].feature);

                if(score < min_score)
                {
                    set<int>::iterator iter = matched_index.find(feature_curr[j].index);
                    if(iter == matched_index.end())
                    {
                        min_score = score;
                        min_index_base = feature_base[i].index;
                        min_index_curr = feature_curr[j].index;
                        matched_index.insert(min_index_curr);
                    }  
                }
            }
            feature_match.push_back(Info(min_score, min_index_base, min_index_curr));
        }

        unsigned int matched_size = feature_match.size();

        for(unsigned int i = 0; i < 2; i++)  //  The most 2 similar pairs of points
        {
            for(unsigned int j = i + 1; j < matched_size; j++)
            {
                if(feature_match[i].score > feature_match[j].score)
                {
                    swap(feature_match[i], feature_match[j]);
                }
            }
        }

        base_p1 = {cloud_base->points[feature_match[0].ind_base].x, cloud_base->points[feature_match[0].ind_base].y};
        base_p2 = {cloud_base->points[feature_match[1].ind_base].x, cloud_base->points[feature_match[1].ind_base].y};

        curr_p1 = {cloud_curr->points[feature_match[0].ind_curr].x, cloud_curr->points[feature_match[0].ind_curr].y};
        curr_p2 = {cloud_curr->points[feature_match[1].ind_curr].x, cloud_curr->points[feature_match[1].ind_curr].y};
    }
    else
    {
        base_p1 = {cloud_base->points[feature_base[0].index].x, cloud_base->points[feature_base[0].index].y};
        base_p2 = {cloud_base->points[feature_base[1].index].x, cloud_base->points[feature_base[1].index].y};

        curr_p1 = {cloud_curr->points[feature_curr[0].index].x, cloud_curr->points[feature_curr[0].index].y};
        curr_p2 = {cloud_curr->points[feature_curr[1].index].x, cloud_curr->points[feature_curr[1].index].y};
    }

    return true;
}

bool SimpleSLAM::ridigMotionCalculation()
{
    Vector base_vec(base_p1.x-base_p2.x, base_p1.y-base_p2.y);
    Vector curr_vec(curr_p1.x-curr_p2.x, curr_p1.y-curr_p2.y);

    rotation_angle = curr_vec.calcAngle(base_vec); // from curr to base

    double rotated_curr_p1_x = curr_p1.x * cos(rotation_angle) - curr_p1.y * sin(rotation_angle);
    double rotated_curr_p1_y = curr_p1.x * sin(rotation_angle) + curr_p1.y * cos(rotation_angle);
    double rotated_curr_p2_x = curr_p2.x * cos(rotation_angle) - curr_p2.y * sin(rotation_angle);
    double rotated_curr_p2_y = curr_p2.x * sin(rotation_angle) + curr_p2.y * cos(rotation_angle);

    
    translation_from_curr_to_base.setValueX(((base_p1.x-rotated_curr_p1_x)+(base_p2.x-rotated_curr_p2_x))/2);
    translation_from_curr_to_base.setValueY(((base_p1.y-rotated_curr_p1_y)+(base_p2.y-rotated_curr_p2_y))/2);                                         
    
    error_x = fabs((base_p1.x-rotated_curr_p1_x) - (base_p2.x-rotated_curr_p2_x)); 
    error_y = fabs((base_p1.y-rotated_curr_p1_y) - (base_p2.y-rotated_curr_p2_y)); 

    return true;
}

bool SimpleSLAM::pointcloudMerging()
{
    cloud.width = final_base->width + final_curr->width;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    for(int i = 0; i < final_base->width; i++)
    {
        cloud.points[i].x = cloud_base->points[i].x;
        cloud.points[i].y = cloud_base->points[i].y;
        cloud.points[i].z = 0;
    }

    for(int i = 0; i < final_curr->width; i++)
    {
        double temp_x = final_curr->points[i].x;
        double temp_y = final_curr->points[i].y;
        cloud.points[i + final_base->width].x = temp_x * cos(rotation_angle) - temp_y * sin(rotation_angle) + translation_from_curr_to_base.x;
        cloud.points[i + final_base->width].y = temp_x * sin(rotation_angle) + temp_y * cos(rotation_angle) + translation_from_curr_to_base.y;
        cloud.points[i + final_base->width].z = 0;
    }

#ifdef DOWN_SAMPLE_ENABLED
    pcl::PCLPointCloud2::Ptr cloud_optimal(new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(cloud, *cloud_optimal);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_optimal);
    sor.setLeafSize(error_x, error_y, 0.01f);
    sor.filter(*cloud_optimal);
    pcl::fromPCLPointCloud2(*cloud_optimal, cloud);
#endif

    return true;
}

bool SimpleSLAM::pointcloudStorage(string output_name)
{
    pcl::io::savePCDFileASCII (output_name, cloud);
    return true;
}

void SimpleSLAM::SimpleSLAM_INFO(string str)
{
    cout << "[SimpleSLAM-INFO]    " << str << endl;
}

void SimpleSLAM::SimpleSLAM_ERROR(string str)
{
    cout << "[SimpleSLAM-ERROR]    " << str << endl;
}
