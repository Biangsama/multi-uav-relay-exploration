#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <cstdlib>
#include <string>

#include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

using std::string;

std::string mtsp_dir1_;
std::string mtsp_dir2_;
std::string mtsp_dir3_;
std::string service_name_;
int drone_id_, problem_id_;

namespace {
const char* probLabel(const int prob) {
  switch (prob) {
    case 1:
      return "amtsp / ATSP";
    case 2:
      return "amtsp2 / ATSP";
    case 3:
      return "amtsp3 / ACVRP";
    default:
      return "unknown";
  }
}

int solverFamilyForProb(const int prob) {
  switch (prob) {
    case 1:
    case 2:
      return 1;
    case 3:
      return 2;
    default:
      return 0;
  }
}

const char* solverFamilyLabel(const int family_id) {
  switch (family_id) {
    case 1:
      return "ATSP family";
    case 2:
      return "ACVRP family";
    default:
      return "unknown";
  }
}

const char* allowedProbList(const int problem_id) {
  switch (problem_id) {
    case 1:
      return "[1(amtsp / ATSP), 2(amtsp2 / ATSP)]";
    case 2:
      return "[3(amtsp3 / ACVRP)]";
    default:
      return "[]";
  }
}

const char* backendLabel(const int prob) {
  return prob == 3 ? "/usr/local/bin/LKH" : "solveMTSPWithLKH3";
}

const char* serviceNamePrefix(const int problem_id) {
  switch (problem_id) {
    case 1:
      return "/solve_tsp_";
    case 2:
      return "/solve_acvrp_";
    default:
      return nullptr;
  }
}

bool serviceAllowsProb(const int problem_id, const int prob) {
  const int solver_family = solverFamilyForProb(prob);
  return solver_family != 0 && solver_family == problem_id;
}

const std::string* parFileForProb(const int prob) {
  switch (prob) {
    case 1:
      return &mtsp_dir1_;
    case 2:
      return &mtsp_dir2_;
    case 3:
      return &mtsp_dir3_;
    default:
      return nullptr;
  }
}
}  // namespace

bool mtspCallback(
    lkh_mtsp_solver::SolveMTSP::Request& req, lkh_mtsp_solver::SolveMTSP::Response& res) {
  const std::string* par_file = parFileForProb(req.prob);
  if (par_file == nullptr) {
    ROS_ERROR_STREAM("[MTSP_NODE]: unsupported req.prob=" << req.prob << ", service="
                     << service_name_ << ", drone_id=" << drone_id_);
    return false;
  }
  if (!serviceAllowsProb(problem_id_, req.prob)) {
    ROS_ERROR_STREAM("[MTSP_NODE]: dispatch mismatch service=" << service_name_
                     << ", drone_id=" << drone_id_ << ", problem_id=" << problem_id_ << " ("
                     << solverFamilyLabel(problem_id_) << "), req.prob=" << req.prob << " ("
                     << probLabel(req.prob) << "), allowed_probs=" << allowedProbList(problem_id_)
                     << ", rejecting request");
    return false;
  }

  ROS_INFO_STREAM("[MTSP_NODE]: dispatch service=" << service_name_ << ", drone_id="
                  << drone_id_ << ", problem_id=" << problem_id_ << " ("
                  << solverFamilyLabel(problem_id_) << "), req.prob=" << req.prob << " ("
                  << probLabel(req.prob) << "), par_file=" << *par_file << ", backend="
                  << backendLabel(req.prob));

  if (req.prob == 3) {
    string cmd = "/usr/local/bin/LKH " + *par_file;
    const int exit_code = system(cmd.c_str());
    if (exit_code != 0) {
      ROS_ERROR_STREAM("[MTSP_NODE]: backend failed req.prob=" << req.prob
                       << ", par_file=" << *par_file << ", exit_code=" << exit_code);
      return false;
    }
  } else {
    solveMTSPWithLKH3(par_file->c_str());
  }

  ROS_INFO_STREAM("[MTSP_NODE]: completed req.prob=" << req.prob << ", service="
                  << service_name_ << ", par_file=" << *par_file);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mtsp_node");
  ros::NodeHandle nh("~");

  std::string mtsp_dir;
  nh.param("exploration/mtsp_dir", mtsp_dir, std::string("null"));
  nh.param("exploration/drone_id", drone_id_, 1);
  nh.param("exploration/problem_id", problem_id_, 1);

  mtsp_dir1_ = mtsp_dir + "/amtsp_" + std::to_string(drone_id_) + ".par";
  mtsp_dir2_ = mtsp_dir + "/amtsp2_" + std::to_string(drone_id_) + ".par";
  mtsp_dir3_ = mtsp_dir + "/amtsp3_" + std::to_string(drone_id_) + ".par";

  const char* service_prefix = serviceNamePrefix(problem_id_);
  if (service_prefix == nullptr) {
    ROS_FATAL_STREAM("[MTSP_NODE]: unsupported exploration/problem_id=" << problem_id_
                     << ", drone_id=" << drone_id_);
    return 1;
  }
  service_name_ = std::string(service_prefix) + std::to_string(drone_id_);
  ros::ServiceServer mtsp_server = nh.advertiseService(service_name_, mtspCallback);

  ROS_WARN_STREAM("[MTSP_NODE]: ready drone_id=" << drone_id_ << ", problem_id="
                  << problem_id_ << " (" << solverFamilyLabel(problem_id_) << "), service="
                  << service_name_ << ", allowed_probs=" << allowedProbList(problem_id_)
                  << ", backend=" << (problem_id_ == 2 ? "/usr/local/bin/LKH" : "solveMTSPWithLKH3")
                  << ", amtsp=" << mtsp_dir1_
                  << ", amtsp2=" << mtsp_dir2_ << ", amtsp3=" << mtsp_dir3_);
  ros::spin();

  return 1;
}
