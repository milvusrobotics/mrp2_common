#include "mrp2_analyzer/mrp2_analyzer.h"

using namespace diagnostic_aggregator;
using namespace std;

PLUGINLIB_REGISTER_CLASS(MRP2Analyzer,  
                         diagnostic_aggregator::MRP2Analyzer, 
                         diagnostic_aggregator::Analyzer)

MRP2Analyzer::MRP2Analyzer() :
  l_motor_path_(""), r_motor_path_(""), battery_path_(""), lights_path_(""), power_board_name_(""), 
  runstop_hit_(false), has_initialized_(false), has_motor_l_data_(false), has_motor_r_data_(false),  
  has_battery_data_(false), has_lights_data_(false)
{ }

MRP2Analyzer::~MRP2Analyzer() { }

bool MRP2Analyzer::init(const string base_name, const ros::NodeHandle &n)
{ 
  // path_ = BASE_NAME/Motors
  motors_path_ = base_name + "Motors";
  l_motor_path_ = motors_path_ + "/Motor Left";
  r_motor_path_ = motors_path_ + "/Motor Right";
  battery_path_ = base_name + "Battery";
  lights_path_ = base_name + "Lights";

  if (!n.getParam("power_board_name", power_board_name_))
  {
     ROS_ERROR("No power board name was specified in MRP2Analyzer! Power board must be \"MRP Powerboard 10XX\". Namespace: %s", n.getNamespace().c_str());
     return false;
  }

  boost::shared_ptr<StatusItem> item_l_motor(new StatusItem("Motor Left"));
  boost::shared_ptr<StatusItem> item_r_motor(new StatusItem("Motor Right"));
  boost::shared_ptr<StatusItem> item_battery(new StatusItem("Battery"));
  boost::shared_ptr<StatusItem> item_lights(new StatusItem("Lights"));

  motor_l_item_ = item_l_motor;
  motor_r_item_ = item_r_motor;
  battery_item_ = item_battery;
  lights_item_ = item_lights;

  has_initialized_ = true;
  
  return true;
}

bool MRP2Analyzer::match(const std::string name)
{
  if (name == "Motor Left" || name == "Motor Right" || name == "Battery" || name == "Lights" )
    return true;

  return name == power_board_name_;
}

bool MRP2Analyzer::analyze(const boost::shared_ptr<StatusItem> item)
{

  if(item->getName() == "Motor Left")
  {
  	has_motor_l_data_ = true;
  	stall_l_ = item->getValue("Stall") == "True";
  	motors_controller_halt_ = item->getValue("Controller Halt") == "True";

  	has_motor_l_data_ = true;

  	motor_l_item_ = item;
  	return true;
  }

  if(item->getName() == "Motor Right")
  {
  	has_motor_r_data_ = true;
  	stall_r_ = item->getValue("Stall") == "True";
  	motors_controller_halt_ = item->getValue("Controller Halt") == "True";

  	has_motor_r_data_ = true;

  	motor_r_item_ = item;
  	return true;
  }

  if(item->getName() == "Battery")
  {
  	has_battery_data_ = true;
  	batt_high_ = item->getValue("Over Voltage") == "True";
  	batt_low_ = item->getValue("Under Voltage") == "True";
   	battery_item_ = item;
  	return true;	
  }

  if(item->getName() == "Lights")
  {
  	has_lights_data_ = true;
  	lights_controller_halt_ = item->getValue("Controller Halt") == "True";
  	lights_item_ = item;
  	return true;
  }

  return false;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > MRP2Analyzer::report()
{
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> motor_l_stat = motor_l_item_->toStatusMsg(l_motor_path_);
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> motor_r_stat = motor_r_item_->toStatusMsg(r_motor_path_);
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> battery_stat = battery_item_->toStatusMsg(battery_path_);
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> lights_stat = lights_item_->toStatusMsg(lights_path_);

  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> motors_stat(new diagnostic_msgs::DiagnosticStatus());

  motors_stat->name = motors_path_;
  motor_l_stat->name = l_motor_path_;
  motor_r_stat->name = r_motor_path_;
  battery_stat->name = battery_path_;
  lights_stat->name = lights_path_;

  if (has_motor_l_data_ && !stall_l_ && !motors_controller_halt_)
  {
    motor_l_stat->level = diagnostic_msgs::DiagnosticStatus::OK;
  }else{
    motor_l_stat->level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  if (has_motor_r_data_ && !stall_r_ && !motors_controller_halt_)
  {
    motor_r_stat->level = diagnostic_msgs::DiagnosticStatus::OK;
  }else{
    motor_r_stat->level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  if (motor_l_stat->level == diagnostic_msgs::DiagnosticStatus::ERROR || motor_r_stat->level == diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    motors_stat->level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }else{
    motors_stat->level = diagnostic_msgs::DiagnosticStatus::OK;
  }

  if(has_lights_data_ && !lights_controller_halt_)
  {
  	lights_stat->level = diagnostic_msgs::DiagnosticStatus::OK;
  }else{
  	lights_stat->level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  if(has_battery_data_ && !batt_low_ && !batt_high_)
  {
  	battery_stat->level = diagnostic_msgs::DiagnosticStatus::OK;
  }else if(batt_low_ || batt_high_){
  	battery_stat->level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }


  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;
  output.push_back(motors_stat);
  output.push_back(motor_l_stat);
  output.push_back(motor_r_stat);
  output.push_back(battery_stat);
  output.push_back(lights_stat);

  return output;
}
