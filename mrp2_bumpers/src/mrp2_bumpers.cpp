#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

void bumper_recovery(void);
void recovery_action(void);

bool bumpers[4] = { false, false, false, false};
bool bumpers_init = false;
bool e_stop_btn = false;
bool e_stop_btn_init = false;

double x_vel = 0;
double z_vel = 0;
bool recovery_st = false;

ros::Subscriber bumper_sub;
ros::Subscriber estop_btn_sub;

ros::Publisher vel_pub;
ros::Publisher estop_clear_pub;

typedef enum {
  recover_front,
  recover_rear,
  stuck,
  estop_pressed,
  normal
}status_t;

status_t last_status = normal;
status_t now_status = normal;

void BumpersCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  bumpers[0] = msg->data[0];
  bumpers[1] = msg->data[1];
  bumpers[2] = msg->data[2];
  bumpers[3] = msg->data[3];

  if(bumpers_init == false)
    bumpers_init = true;

    bumper_recovery();
}

void EStopBtnCallback(const std_msgs::Bool::ConstPtr& msg)
{
  e_stop_btn = msg->data;

  if(e_stop_btn_init == false)
    e_stop_btn_init = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrp2_bumpers");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  bumper_sub = n.subscribe("/bumpers", 1, BumpersCallback);
  estop_btn_sub = n.subscribe("/estop_btn", 1, EStopBtnCallback);

  vel_pub = n.advertise<geometry_msgs::Twist>("/recovery_vel", 10);
  estop_clear_pub = n.advertise<std_msgs::Empty>("/estop_clear", 1);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    recovery_action();
  }
}

void recovery_action(void)
{
  geometry_msgs::Twist vel;
  std_msgs::Empty msg;

  if(now_status != last_status) //If there is a status change, inform user.
  {
    switch(now_status)
    {
        case recover_front:
          estop_clear_pub.publish(msg); // Clear the e-stop
          ROS_WARN("Front bumper is collided. Trying to recover.");
      break;
        case recover_rear:
          estop_clear_pub.publish(msg); // Clear the e-stop
          ROS_WARN("Rear bumper is collided. Trying to recover.");
      break;
        case stuck:
          ROS_ERROR("Can't recover because front and rear bumpers are collided.");
      break;
        case normal:
          vel.linear.x =0;
          vel.angular.z = 0;
          vel_pub.publish(vel);
          ROS_WARN("Recovery success.");
      break;
        case estop_pressed:
          vel.linear.x =0;
          vel.angular.z = 0;
          vel_pub.publish(vel);
          ROS_ERROR("Won't try to recover because E-STOP Button is pressed.");
        break;
        default:
      break;
    }
    last_status = now_status;
  }

  if(recovery_st)
  {
    if(e_stop_btn == true)
    {
      return;
    }
    vel.linear.x = x_vel;
    vel.angular.z = z_vel;
    vel_pub.publish(vel);
  }
}

void bumper_recovery(void)
{
  if((bumpers[2] || bumpers[3]) && (bumpers[0] == 0 && bumpers[1] == 0)) // front bumper collision
  {
    if(e_stop_btn_init) // Make sure we have updated messages
    {
      now_status = recover_front;
      recovery_st = true;
      x_vel = -0.1;
      z_vel = 0.0;
      e_stop_btn_init = false;
    }
  }

  if((bumpers[0] || bumpers[1]) && (bumpers[2] == 0 && bumpers[3] == 0)) // rear bumper collision
  {
    if(e_stop_btn_init) // Make sure we have updated messages
    {
      now_status = recover_rear;
      recovery_st = true;
      x_vel = 0.1;
      z_vel = 0.0;
      e_stop_btn_init = false;
    }
  }

  if((bumpers[0] || bumpers[1]) && (bumpers[2] || bumpers[3])) // both bumpers collision
  {
    now_status = stuck;
    recovery_st = false;
    x_vel = 0;
    z_vel = 0;
  }

  if(e_stop_btn == true)
  {
    now_status = estop_pressed;
  }

  if(bumpers[0] == 0 && bumpers[1] == 0 && bumpers[2] == 0 && bumpers[3] == 0)
  {
    now_status = normal;
    recovery_st = false;
    return;
  }

}
