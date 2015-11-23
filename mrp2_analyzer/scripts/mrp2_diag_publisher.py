#!/usr/bin/env python

import roslib; roslib.load_manifest('mrp2_analyzer')

import rospy
import std_msgs
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool

stall_l = False
stall_r = False
batt_low = False
batt_high = False
controller = False
aux_lights = False

def motor_stall_l_callback(data):
  global stall_l
  stall_l = data.data

def motor_stall_r_callback(data):
  global stall_r
  stall_r = data.data

def batt_low_callback(data):
  global batt_low
  batt_low = data.data

def batt_high_callback(data):
  global batt_high
  batt_high = data.data

def controller_callback(data):
  global controller
  controller = data.data

def aux_lights_callback(data):
  global aux_lights
  aux_lights = data.data

if __name__ == '__main__':
    rospy.init_node('mrp2_diagnostics_publisher')

    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size = 10)

    rospy.Subscriber("hw_monitor/diagnostics/motor_stall_l", Bool, motor_stall_l_callback)
    rospy.Subscriber("hw_monitor/diagnostics/motor_stall_r", Bool, motor_stall_r_callback)
    rospy.Subscriber("hw_monitor/diagnostics/batt_low", Bool, batt_low_callback)
    rospy.Subscriber("hw_monitor/diagnostics/batt_high", Bool, batt_high_callback)
    rospy.Subscriber("hw_monitor/diagnostics/controller", Bool, controller_callback)
    rospy.Subscriber("hw_monitor/diagnostics/aux_lights", Bool, aux_lights_callback)

    array = DiagnosticArray()

    my_rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():

        motor_l_stat = DiagnosticStatus(name = 'Motor Left', level = 0, message = 'Running')
        motor_l_stat.values = [ KeyValue(key = 'Stall', value = 'False'), KeyValue(key = 'Controller Halt', value = 'False')]

        motor_r_stat = DiagnosticStatus(name = 'Motor Right', level = 0, message = 'Running')
        motor_r_stat.values = [ KeyValue(key = 'Stall', value = 'False'), KeyValue(key = 'Controller Halt', value = 'False')]
      
        if controller == True:
          motor_l_stat = DiagnosticStatus(name = 'Motor Left', level = 0, message = 'Halted')
          motor_l_stat.values = [ KeyValue(key = 'Stall', value = 'False'),KeyValue(key = 'Controller Halt', value = 'True')]
          motor_r_stat = DiagnosticStatus(name = 'Motor Right', level = 0, message = 'Halted')
          motor_r_stat.values = [ KeyValue(key = 'Stall', value = 'False'),KeyValue(key = 'Controller Halt', value = 'True')]
     
        if stall_l == True: 
          motor_l_stat = DiagnosticStatus(name = 'Motor Left', level = 0, message = 'Stalled')
          motor_l_stat.values = [ KeyValue(key = 'Stall', value = 'True'), KeyValue(key = 'Controller Halt', value = 'False')]

        if stall_l == True & controller == True:
          motor_l_stat = DiagnosticStatus(name = 'Motor Left', level = 0, message = 'Stopped')
          motor_l_stat.values = [ KeyValue(key = 'Stall', value = 'True'), KeyValue(key = 'Controller Halt', value = 'True')]
      
        if controller == True:
          motor_r_stat = DiagnosticStatus(name = 'Motor Right', level = 0, message = 'Halted')
          motor_r_stat.values = [ KeyValue(key = 'Stall', value = 'False'), KeyValue(key = 'Controller Halt', value = 'True')]
     
        if stall_r == True: 
          motor_r_stat = DiagnosticStatus(name = 'Motor Right', level = 0, message = 'Stalled')
          motor_r_stat.values = [ KeyValue(key = 'Stall', value = 'True'), KeyValue(key = 'Controller Halt', value = 'False')]

        if stall_r == True & controller == True:
          motor_r_stat = DiagnosticStatus(name = 'Motor Right', level = 0, message = 'Stopped')
          motor_r_stat.values = [ KeyValue(key = 'Stall', value = 'True'), KeyValue(key = 'Controller Halt', value = 'True')]

        battery_stat = DiagnosticStatus(name = 'Battery', level = 0, message = 'OK')
        battery_stat.values = [ KeyValue(key = 'Over Voltage', value = 'False'), KeyValue(key = 'Under Voltage', value = 'False')]

        if batt_high == True:
          battery_stat = DiagnosticStatus(name = 'Battery', level = 0, message = 'Voltage error')
          battery_stat.values = [ KeyValue(key = 'Over Voltage', value = 'True'), KeyValue(key = 'Under Voltage', value = 'False')]

        if batt_low == True:
          battery_stat = DiagnosticStatus(name = 'Battery', level = 0, message = 'Voltage error')
          battery_stat.values = [ KeyValue(key = 'Over Voltage', value = 'False'), KeyValue(key = 'Under Voltage', value = 'True')]

        lights_stat = DiagnosticStatus(name = 'Lights', level = 0, message = 'OK')
        lights_stat.values = [ KeyValue(key = 'Controller Halt', value = 'False')]

        if aux_lights == True:
          lights_stat = DiagnosticStatus(name = 'Lights', level = 0, message = 'Controller Error')
          lights_stat.values = [ KeyValue(key = 'Controller Halt', value = 'True')]

        array.status = [ motor_l_stat, motor_r_stat, battery_stat, lights_stat ]
        pub.publish(array)
        my_rate.sleep()