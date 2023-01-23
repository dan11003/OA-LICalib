/*
 * OA-LICalib:
 * Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems
 *
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Xingxing Zuo
 * Copyright (C) 2022 Kewei Hu
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <calib/calib_helper.h>
#include <pangolin/pangolin.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <trajectory/trajectory_viewer.h>
#include <string>
#include <boost/program_options.hpp>

namespace po = boost::program_options;


using namespace liso;

class CalibUI : public LICalibrHelper {
public:
  CalibUI(const YAML::Node& config_node, StructorPars& pars)
    : LICalibrHelper(config_node,pars) ,
      iteration_num_(1),
      pan_opt_time_offset_("ui.opt_time_offset", false, false, true),
      pan_opt_lidar_intrinsic_("ui.opt_lidar_intrinsic", false, false, true),
      pan_opt_imu_intrinsic_("ui.opt_imu_intrinsic", false, false, true),
      pan_apply_lidar_intrinstic_("ui.apply_lidar_intrinstic", false, false,true) {
    if (config_node["iteration_num"])
      iteration_num_ = config_node["iteration_num"].as<int>();
  }

  void InitGui() {
    pangolin::CreateWindowAndBind("Main", UI_WIDTH, 485);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    pangolin::Var<std::function<void(void)>> initialization(
                                               "ui.Initialization", std::bind(&CalibUI::Initialization, this));

    pangolin::Var<std::function<void(void)>> data_association_in_odom(
                                               "ui.DataAssociationInOdom",
                                               std::bind(&CalibUI::DataAssociationInOdom, this));

    pangolin::Var<std::function<void(void)>> data_association_in_locator(
                                               "ui.DataAssociationInLocator",
                                               std::bind(&CalibUI::DataAssociationInLocator, this));

    pangolin::Var<std::function<void(void)>> batch_optimization(
                                               "ui.BatchOptimization", std::bind(&CalibUI::BatchOptimization, this));

    pangolin::Var<std::function<void(void)>> refinement(
                                               "ui.Refinement", std::bind(&CalibUI::Refinement, this));

    std::cout << "\nInitUI Done. \n";
  }

  void RenderingLoop() {
    while (!pangolin::ShouldQuit() && ros::ok()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      this->calib_param_manager_->calib_option.opt_time_offset =
          (pan_opt_time_offset_ == true) ? true : false;
      this->calib_param_manager_->calib_option.opt_lidar_intrinsic =
          (pan_opt_lidar_intrinsic_ == true) ? true : false;
      this->calib_param_manager_->calib_option.opt_IMU_intrinsic =
          (pan_opt_imu_intrinsic_ == true) ? true : false;

      this->calib_param_manager_->calib_option.apply_lidar_intrinstic_to_scan =
          (pan_apply_lidar_intrinstic_ == true) ? true : false;

      pangolin::FinishFrame();
      usleep(10);
    }
  }

  void RunSimulation() {
    TicToc timer;

    timer.tic();
    this->Initialization();
    std::cout << "[Paper] Initialization costs " << std::fixed << std::setprecision(2) << timer.toc() << " ms\n";

    // if has map, then use locator
    if (this->calib_param_manager_->locator_segment_param.empty()) {
      this->DataAssociationInOdom();
    } else {
      this->DataAssociationInLocator();
    }

    timer.tic();
    this->BatchOptimization();
    std::cout << "[Paper] First opt costs " << std::fixed
              << std::setprecision(2) << timer.toc() << " ms\n";

    SaveCalibResult(0);

    for (size_t iter = 1; iter < iteration_num_; iter++) {
      if (iter > 1) {
        this->calib_param_manager_->calib_option.opt_lidar_intrinsic = false;
        this->calib_param_manager_->calib_option.opt_IMU_intrinsic = false;
        this->calib_param_manager_->calib_option.opt_time_offset = false;
      }
      timer.tic();
      this->Refinement();
      std::cout << "[Paper] Refinement costs " << std::fixed
                << std::setprecision(2) << timer.toc() << " ms\n";

      if (iter > 10) {
        this->calib_param_manager_->calib_option
            .apply_lidar_intrinstic_to_scan = true;
      }
      SaveCalibResult(iter);
    }
  }

  void Run() {
    this->Initialization();

    // 没有地图就用里程计
    if (this->calib_param_manager_->locator_segment_param.empty()) {
      this->DataAssociationInOdom();
    } else {
      this->DataAssociationInLocator();
    }

    this->BatchOptimization();

    for (size_t iter = 0; iter < iteration_num_; iter++) {
      this->calib_param_manager_->calib_option.opt_time_offset = true;
      this->Refinement();
    }

    std::cout << "Calibration finished." << std::endl;
  }

  void SaveCalibResult(size_t iteration) {
    std::ofstream outfile;
    std::string calib_result_file =
        this->cache_path_parent_ + "/simu_calib_result.txt";
    outfile.open(calib_result_file, std::ios::app);

    Eigen::Vector3d p_LinI = this->calib_param_manager_->p_LinI;
    Eigen::Quaterniond q_LtoI = this->calib_param_manager_->q_LtoI;
    Eigen::Matrix<double, 6, 1> Mw =
        this->calib_param_manager_->imu_intrinsic.Mw_vec_;
    Eigen::Matrix<double, 6, 1> Ma =
        this->calib_param_manager_->imu_intrinsic.Ma_vec_;
    Eigen::Matrix<double, 9, 1> Aw =
        this->calib_param_manager_->imu_intrinsic.Aw_vec_;
    Eigen::Quaterniond q_WtoA =
        this->calib_param_manager_->imu_intrinsic.q_WtoA_;

    double time_offset =
        this->calib_param_manager_->segment_param[0].time_offset;
    outfile << this->bag_name_ << "," << iteration << "," << time_offset << ","
            << p_LinI(0) << "," << p_LinI(1) << "," << p_LinI(2) << ","
            << q_LtoI.x() << "," << q_LtoI.y() << "," << q_LtoI.z() << ","
            << q_LtoI.w();
    for (int i = 0; i < 6; i++) {
      outfile << "," << Mw(i);
    }
    for (int i = 0; i < 6; i++) {
      outfile << "," << Ma(i);
    }
    outfile << "," << q_WtoA.x() << "," << q_WtoA.y() << "," << q_WtoA.z()
            << "," << q_WtoA.w();

    //    for (int i = 0; i < 9; i++) {
    //      outfile << "," << Aw(i);
    //    }
    outfile << "\n";
    outfile.close();

    /// Save lidar intrinsic
    std::ofstream lidar_file;
    std::string lidar_file_name =
        this->cache_path_ + "/lidar-" + std::to_string(iteration) + ".txt";
    lidar_file.open(lidar_file_name);
    std::cout << "Save lidar intrinsic to " << lidar_file_name << std::endl;

    lidar_file << "dist_scale,dist_offset_mm,vert_offset_mm,horiz_offset_mm,"
               << "vert_degree,delta_horiz_degree\n";

    const auto laser_param_vec =
        this->calib_param_manager_->lidar_intrinsic.GetLaserParamVec();
    int order[16] = {15, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 0};
    for (size_t dsr = 0; dsr < 16; dsr++) {
      std::vector<double> v = laser_param_vec.at(order[dsr]);
      lidar_file << v[0] << ","               //
                         << v[1] * 1000 << ","        //
                         << v[2] * 1000 << ","        //
                         << v[3] * 1000 << ","        //
                         << v[4] * 180 / M_PI << ","  //
                         << v[5] * 180 / M_PI << std::endl;
    }
    lidar_file.close();
  }

private:
  int iteration_num_;

  static constexpr int UI_WIDTH = 300;

  pangolin::Var<bool> pan_opt_time_offset_;
  pangolin::Var<bool> pan_opt_lidar_intrinsic_;
  pangolin::Var<bool> pan_opt_imu_intrinsic_;
  pangolin::Var<bool> pan_apply_lidar_intrinstic_;
  StructorPars pars_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "li_calib_node");
  ros::NodeHandle nh("~");


  po::options_description desc{"Options"};
  desc.add_options()
      ("help,h", "Help screen")
      ("output_path", po::value<std::string>()->default_value(""), "res")
      ("config_file", po::value<std::string>()->default_value(""), "res")
      ("start_time", po::value<double>()->default_value(-1.0), "res")
      ("end_time", po::value<double>()->default_value(-1.0), "range resolution");


  po::variables_map vm;
  store(parse_command_line(argc, argv, desc), vm);
  notify(vm);

  StructorPars sPar;
  liso::publisher::SetPublisher(nh);


  std::string config_file_path;

  if (vm.count("config_file")){
    cout << "config file provided" << endl;
    //double start_time = vm["start_time"].as<int>();
    config_file_path = vm["config_file"].as<std::string>();
    std::cout << "yaml path boost: " << config_file_path <<std::endl;;

  }else{
    cout << "No config file provided" << endl;
    std::string config_path;
    nh.param<std::string>("config_path", config_path, "/config/li-calib.yaml");
    std::string package_name = "oa_licalib";
    std::string PACKAGE_PATH = ros::package::getPath(package_name);
    config_file_path = PACKAGE_PATH + config_path;
  }


  std::cout << "Load config from: " << config_file_path << std::endl;
  YAML::Node config_node = YAML::LoadFile(config_file_path);
  bool use_gui = config_node["use_gui"].as<bool>();
  std::cout << "gui: " << use_gui << std::endl;


  std::cout  <<"Default: " << config_node["selected_segment"][0]["start_time"] << ", " << config_node["selected_segment"][0]["end_time"] << endl; //<< ", " << config_node["selected_segment"][0]  << std::endl;
  if (vm.count("start_time")){
    sPar.start_time = vm["start_time"].as<double>();
    config_node["selected_segment"][0]["start_time"] = sPar.start_time;
  }
  if (vm.count("end_time")){
    sPar.end_time = vm["end_time"].as<double>();
    config_node["selected_segment"][0]["end_time"]  = sPar.end_time;
  }
  if(vm.count("output_path")){
    config_node["output_path"] = vm["output_path"].as<std::string>();
  }
  std::cout  <<"after: " << config_node["selected_segment"][0]["start_time"]  << ", " << config_node["selected_segment"][0]["end_time"] << ", output: " << config_node["output_path"] << std::endl;




  CalibUI calib_ui(config_node, sPar);



  if (use_gui) {
    calib_ui.InitGui();
    calib_ui.RenderingLoop();
  } else {
    // calib_ui.Run();
    calib_ui.RunSimulation();
  }
}
