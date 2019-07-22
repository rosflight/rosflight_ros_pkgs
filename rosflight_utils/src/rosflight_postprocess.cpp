#include <iostream>
#include <cstdio>
#include <vector>
#include <unistd.h>
#include <sys/stat.h>
#include <string>
#include <fstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <yaml-cpp/yaml.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <rosflight_msgs/Command.h>
#include <boost/foreach.hpp>
#include <experimental/filesystem>
#pragma GCC diagnostic pop

#include "rosflight_utils/input_parser.h"
#include "rosflight_utils/progress_bar.h"
#include "rosflight.h"
#include "test_board.h"
#include "mavlink/mavlink.h"

using namespace std;
namespace fs = std::experimental::filesystem;

bool loadParameters(const string filename, rosflight_firmware::ROSflight& RF)
{
  (void) RF;
  if (!fs::exists(filename))
  {
    cout << "unable to find parameter file " << filename << endl;
    return false;
  }

  try
  {
    YAML::Node node = YAML::LoadFile(filename);
    for (auto it = node.begin(); it != node.end(); it++)
    {
      if ((*it)["type"].as<int>() == 6)
        RF.params_.set_param_by_name_int((*it)["name"].as<string>().c_str(), (*it)["value"].as<int>());
      else if ((*it)["type"].as<int>() == 9)
        RF.params_.set_param_by_name_float((*it)["name"].as<string>().c_str(), (*it)["value"].as<float>());
      else
        throw std::runtime_error("unrecognized parameter type");
    }
    int debug = 1;
    (void)debug;
    (void)node;
  }
  catch (...)
  {
    std::cout << "Failed to Read yaml file.  Could you have a syntax error?" << filename << std::endl;
    return false;
  }
  cout << "loaded parameters from " << filename << endl;
  return true;
}


void displayHelp()
{
  cout << "USAGE: rosbag_parser [options]" << "\n\n";
  cout << "Options:\n";
  cout << "\t -h, --help\tShow this help message and exit\n";
  cout << "\t -f FILENAME\tBagfile to parse\n";
  cout << "\t -p FILENAME\tParameter file to load\n";
  cout << "\t -s START_TIME\tstart time of bag (seconds)\n";
  cout << "\t -u DURATION\tduration to run bag (seconds)\n";
  cout << "\t -v Show Verbose Output\n";
  cout << endl;
}



int main(int argc, char * argv[])
{
  string bag_filename = "";
  bool verbose = false;
  double start_time = 0;
  double duration = INFINITY;
  string param_filename = "";
  InputParser argparse(argc, argv);
  if (argparse.cmdOptionExists("-h"))
    displayHelp();
  if (!argparse.getCmdOption("-f", bag_filename))
    displayHelp();
  if (!argparse.getCmdOption("-p", param_filename))
    displayHelp();
  argparse.getCmdOption("-s", start_time);
  argparse.getCmdOption("-d", duration);
  verbose = argparse.cmdOptionExists("-v");

  rosbag::Bag bag;
  try
  {
    bag.open(bag_filename.c_str(), rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
    fprintf(stderr, "unable to load rosbag %s, %s", bag_filename.c_str(), e.what());
    return -1;
  }
  rosbag::View view(bag);

  // Get list of topics and print to screen - https://answers.ros.org/question/39345/rosbag-info-in-c/
  if (verbose)
  {
    vector<const rosbag::ConnectionInfo *> connections = view.getConnections();
    vector<string> topics;
    vector<string> types;
    cout << "\nloaded bagfile: " << bag_filename << "\n===================================\n";
    cout << "Topics\t\tTypes\n----------------------------\n\n" << endl;
    for(const rosbag::ConnectionInfo *info : connections) {
      topics.push_back(info->topic);
      types.push_back(info->datatype);
      cout << info->topic << "\t\t" << info->datatype << endl;
    }
  }

  // Figure out the end time of the bag
  double end_time = start_time + duration;
  end_time = (end_time < view.getEndTime().toSec() - view.getBeginTime().toSec()) ? end_time : view.getEndTime().toSec() - view.getBeginTime().toSec();
  if (verbose)
    cout << "Playing bag from: = " << start_time << "s to: " << end_time << "s" << endl;

  // Create the ROSflight object
  rosflight_firmware::testBoard board;
  rosflight_firmware::Mavlink mavlink(board);
  rosflight_firmware::ROSflight RF(board, mavlink);
  RF.init();

  if (!param_filename.empty())
  {
    if (!loadParameters(param_filename, RF))
      return -1;
  }

  // Get some time variables
  ros::Time bag_start = view.getBeginTime() + ros::Duration(start_time);
  ros::Time bag_end = view.getBeginTime() + ros::Duration(end_time);


  // Prepare the output file
  fstream est_log, truth_log, imu_log, filtered_imu_log, cmd_log;
  fs::create_directories("/tmp/rosflight_post_process/");
  est_log.open("/tmp/rosflight_post_process/estimate.bin", std::ofstream::out | std::ofstream::trunc);
  truth_log.open("/tmp/rosflight_post_process/truth.bin", std::ofstream::out | std::ofstream::trunc);
  imu_log.open("/tmp/rosflight_post_process/imu.bin", std::ofstream::out | std::ofstream::trunc);
  filtered_imu_log.open("/tmp/rosflight_post_process/imu_filt.bin", std::ofstream::out | std::ofstream::trunc);
  cmd_log.open("/tmp/rosflight_post_process/cmd.bin", std::ofstream::out | std::ofstream::trunc);


  ProgressBar prog(view.size(), 80);
  int i = 0;
  bool time_initialized = false;
  bool pose_initialized = false;
  for(rosbag::MessageInstance const m : view)
  {
    // skip messages before start time
    if (m.getTime() < bag_start) continue;

    // End bag after duration has passed
    if (m.getTime() > bag_end) break;

    prog.print(++i);

    /// Call all the callbacks

    // Cast datatype into proper format and call the appropriate callback
    string datatype = m.getDataType();

    if (datatype.compare("sensor_msgs/Imu") == 0)
    {
      const sensor_msgs::ImuConstPtr imu(m.instantiate<sensor_msgs::Imu>());


      if (!time_initialized)
      {
        bag_start = imu->header.stamp;
        board.set_time(0);
        time_initialized = true;
      }

      // Move the board time forward
      float acc[3] = {(float)imu->linear_acceleration.x,
                      (float)imu->linear_acceleration.y,
                      (float)imu->linear_acceleration.z};
      float gyro[3] = {(float)imu->angular_velocity.x,
                       (float)imu->angular_velocity.y,
                       (float)imu->angular_velocity.z};
      int64_t t_us = (imu->header.stamp - bag_start).toNSec()/1000;


      board.set_imu(acc, gyro, t_us);
      RF.run();
      double est[8] = {(double) t_us/1e6,
                       (double) RF.estimator_.state().attitude.w,
                       (double) RF.estimator_.state().attitude.x,
                       (double) RF.estimator_.state().attitude.y,
                       (double) RF.estimator_.state().attitude.z,
                       (double) RF.estimator_.bias().x,
                       (double) RF.estimator_.bias().y,
                       (double) RF.estimator_.bias().z};
      est_log.write((char*) est, sizeof(est));

      double imud[7] = {(double) t_us/1e6,
                        (double)acc[0], (double)acc[1], (double)acc[2],
                        (double)gyro[0], (double)gyro[1], (double)gyro[2]};
      imu_log.write((char*) imud, sizeof(imud));

      double imuf[7] = {(double)t_us/1e6,
                        (double)RF.estimator_.accLPF().x,
                        (double)RF.estimator_.accLPF().y,
                        (double)RF.estimator_.accLPF().z,
                        (double)RF.estimator_.gyroLPF().x,
                        (double)RF.estimator_.gyroLPF().y,
                        (double)RF.estimator_.gyroLPF().z};

      filtered_imu_log.write((char*) imuf, sizeof(imuf));
    }

    else if (datatype.compare("geometry_msgs/PoseStamped") == 0)
    {
      const geometry_msgs::PoseStampedConstPtr pose(m.instantiate<geometry_msgs::PoseStamped>());
      double t = (pose->header.stamp - bag_start).toSec();
      double truth[5] = {t,
                         pose->pose.orientation.w,
                         pose->pose.orientation.x,
                         pose->pose.orientation.y,
                         pose->pose.orientation.z};
      truth_log.write((char*) truth, sizeof(truth));
    }

    else if (datatype.compare("geometry_msgs/TransformStamped") == 0)
    {
      const geometry_msgs::TransformStampedConstPtr trans(m.instantiate<geometry_msgs::TransformStamped>());
      double t = (trans->header.stamp - bag_start).toSec();
      double truth[5] = {t,
                         trans->transform.rotation.w,
                         trans->transform.rotation.x,
                         trans->transform.rotation.y,
                         trans->transform.rotation.z};
      truth_log.write((char*) truth, sizeof(truth));
    }

    else if (datatype.compare("rosflight_msgs/Command") == 0)
    {
      const rosflight_msgs::CommandConstPtr cmd(m.instantiate<rosflight_msgs::Command>());
      double t = (cmd->header.stamp - bag_start).toSec();
      double cmdarr[5] = {t,
                          cmd->x,
                          cmd->y,
                          cmd->z,
                          cmd->F};
      cmd_log.write((char*)cmdarr, sizeof(cmdarr));
    }
  }
  prog.finished();
  std::cout << std::endl;
}
#pragma GCC diagnostic pop


