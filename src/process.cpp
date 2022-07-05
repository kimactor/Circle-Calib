#include "process.h"

namespace Calibration {

void Process::read_ref_from_yaml() {
  YAML::Node tf = YAML::LoadFile(select_yaml_file_);
  for (YAML::const_iterator pcd_iter = tf.begin(); pcd_iter != tf.end();
       ++pcd_iter) {
    std::string pcd_name = pcd_iter->first.as<std::string>();
    YAML::Node point_node = pcd_iter->second;
    select_map_.insert(
        {pcd_name, Eigen::Vector3f(point_node["x"].as<float>(),
                                   point_node["y"].as<float>(),
                                   point_node["z"].as<float>())});
  }
  inno_log_info("Load %ld selected points from yaml file", select_map_.size());
}

void Process::extract() {
  std::shared_ptr<Operator> extractor = std::make_shared<Operator>();
  CloudPtr input = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  extractor->setDebugFolder(debug_folder_);
  extractor->setSearchRadius(1.0);
  for (auto pair : select_map_) {
    extractor->setFrameName(pair.first);
    pcl::io::loadPCDFile<pcl::PointXYZI>(folder_ + pair.first, *input);
    extractor->setInputCloud(input);
    extractor->setPickPoint(pair.second);
    Eigen::Vector3f feature;
    if (extractor->compute(feature)) {
      feature_map_.insert({pair.first, feature});
    }
  }
}

void Process::save() {
  YAML::Emitter out;
  out << YAML::BeginMap;
  for (auto pair : feature_map_) {
    out << YAML::Key << pair.first;
    out << YAML::Value << YAML::BeginMap << YAML::Key << "x" << YAML::Value
        << pair.second.x() << YAML::Key << "y" << YAML::Value << pair.second.y()
        << YAML::Key << "z" << YAML::Value << pair.second.z() << YAML::EndMap;
  }

  out << YAML::EndMap;
  std::ofstream fout;
  fout.open(feature_yaml_file_.c_str());
  fout << out.c_str();
  fout.close();
}

void Process::compute() {
  read_ref_from_yaml();
  extract();
  save();
}
}  // namespace Calibration