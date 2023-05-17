#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : lidar_type(tofRGBD), blind(0.01), point_filter_num(1) {
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num) {
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}


void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         PointCloudXYZRGB::Ptr &pcl_out) {
  switch (lidar_type) {
  case tofRGBD:
    tofRGBD_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::tofRGBD_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pl_surf.clear();
  pcl::PointCloud<PointTypeRGB> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_surf.reserve(plsize);
  for (int i = 0; i < pl_orig.size(); i++) {
      if(isnan(pl_orig.points[i].z)||i % point_filter_num != 0){
          continue;
      }
    PointTypeRGB added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.r = pl_orig.points[i].r;
    added_pt.g = pl_orig.points[i].g;
    added_pt.b = pl_orig.points[i].b;

    pl_surf.push_back(added_pt);
  }
}

