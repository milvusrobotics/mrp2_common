#ifndef MRP2_ANALYZER_H
#define MRP2_ANALYZER_H

#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>
#include <string>

namespace diagnostic_aggregator {

class MRP2Analyzer : public Analyzer
{
public:
  MRP2Analyzer();

  ~MRP2Analyzer();

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name);

  bool analyze(const boost::shared_ptr<StatusItem> item);

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return motors_path_; }

  std::string getName() const { return "MRP2Analyzer"; }

private:

  boost::shared_ptr<StatusItem> motor_l_item_;
  boost::shared_ptr<StatusItem> motor_r_item_;
  boost::shared_ptr<StatusItem> battery_item_;
  boost::shared_ptr<StatusItem> lights_item_;

  std::string motors_path_, l_motor_path_, r_motor_path_, battery_path_, lights_path_, power_board_name_;

  bool runstop_hit_, stall_l_, stall_r_, motors_controller_halt_, batt_high_, batt_low_, lights_controller_halt_, has_initialized_, has_motor_l_data_, has_motor_r_data_, has_battery_data_, has_lights_data_;
};

}
#endif //MRP2_ANALYZER_H