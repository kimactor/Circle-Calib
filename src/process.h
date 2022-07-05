#ifndef FEATURE_PROCESS_H_
#define FEATURE_PROCESS_H_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <unordered_map>
#include <sys/stat.h>
#include "logger.h"
#include "calibration.h"

namespace Calibration {

class Process {
 public:
  Process(std::string folder):folder_(folder) {
    select_yaml_file_ = folder_ + "selected.yaml";
    feature_yaml_file_ = folder_ + "feature.yaml";
    debug_folder_ = folder_ + "debug/";
    select_map_ = {};
    feature_map_ = {};
    mkdir(debug_folder_.c_str(), S_IRWXU);
  }
  ~Process() {}
  void read_ref_from_yaml();
  void extract();
  void save();
  void compute();

 private:
  std::string folder_;
  std::string select_yaml_file_;
  std::string feature_yaml_file_;
  std::string debug_folder_;
  std::unordered_map<std::string, Eigen::Vector3f> select_map_;
  std::unordered_map<std::string, Eigen::Vector3f> feature_map_;
};

}  // namespace Calibration
#endif  // FEATURE_PROCESS_H_