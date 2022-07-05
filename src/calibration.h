#ifndef FEATURE_CALIBRATION_H_
#define FEATURE_CALIBRATION_H_

#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "logger.h"

namespace Calibration {
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr CloudPtr;
typedef Eigen::Matrix4f RTmatrix;

struct Ballot {
  Ballot() : x(0), y(0), z(0), angle(0){};
  float x;
  float y;
  float z;
  float angle;
};

class Operator {
 public:
  Operator() {
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
    kdtree_.reset((new pcl::KdTreeFLANN<pcl::PointXYZI>));
    normal_estimator_.setSearchMethod(tree_);
    normal_estimator_.setKSearch(50);
  }
  ~Operator() {}

  void findAround(CloudPtr source, CloudPtr &around, CloudPtr &attention,
                  const float &intensity);

  void guide_filter(CloudPtr source, CloudPtr &result);
  void dbscan(CloudPtr source, CloudPtr &result);
  void euclidean_cluster(CloudPtr source, CloudPtr &result);
  void region_growing_cluster(CloudPtr source, CloudPtr &result);

  void voteNormal(CloudPtr source, CloudPtr &result, float &angle);
  void voteDistance(CloudPtr reference, CloudPtr target, CloudPtr &result);

  void fitPlane(CloudPtr source, CloudPtr &result, float &a, float &b, float &c,
                float &d);
  void fitCircle(CloudPtr source, float &center_x, float &center_y,
                 float &center_z, float &radius);
  void findPlate(CloudPtr source, CloudPtr &result,
                 const pcl::Normal &reference);
  void mergePointCloud(CloudPtr source, CloudPtr &result);
  void nearMarkable(CloudPtr source, CloudPtr &result);

  Eigen::Vector3f find_feature_point(CloudPtr source);

  bool compute(Eigen::Vector3f &feature_point);

  // Settings
  inline void setFrameName(const std::string& name) { frame_name_ = name; }
  inline void setDebugFolder(const std::string& name) { debug_folder_ = name; }
  inline void setInputCloud(const CloudPtr &input) { input_cloud_ = input; }
  inline void setSearchRadius(const float &input) {
    if (input > 0) {
      search_radius_ = input;
    }
  }
  inline void setPickPoint(const float &x, const float &y, const float &z) {
    pick_x_ = x;
    pick_y_ = y;
    pick_z_ = z;
  }
  inline void setPickPoint(const Eigen::Vector3f& point) {
    pick_x_ = point.x();
    pick_y_ = point.y();
    pick_z_ = point.z();
  }
  inline void setSaveFlag(const bool &flag) { save_flag_ = flag; }

 private:
  float search_radius_;
  float dilatation_coeff_;
  float pick_x_;
  float pick_y_;
  float pick_z_;
  float a_;
  float b_;
  float c_;
  float d_;
  int index_;
  bool save_flag_;
  std::string frame_name_;
  std::string debug_folder_;
  CloudPtr input_cloud_;  // 完整的输入点云
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_;
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator_;
};
Eigen::Matrix3f transforme_normal_normal_3d(const Eigen::Vector3f &start,
                                            const Eigen::Vector3f &final);
}  // namespace Calibration

#endif  // FEATURE_CALIBRATION_H_