#!/usr/bin/env python

import roslib; roslib.load_manifest('mrp2_analyzer')

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

if __name__ == '__main__':
    rospy.init_node('mrp2_power_sim')

    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size = 10)
    
    array = DiagnosticArray()
    # Fake power board status, estop is on
    motor_l_stat = DiagnosticStatus(name = 'Motor Left', level = 0, 
                                  message = 'Running')
    motor_l_stat.values = [ KeyValue(key = 'Stall', value = 'True'),
                          KeyValue(key = 'Controller Halt', value = 'False')]
    motor_r_stat = DiagnosticStatus(name = 'Motor Right', level = 0, 
                                  message = 'Running')
    motor_r_stat.values = [ KeyValue(key = 'Stall', value = 'False'),
                          KeyValue(key = 'Controller Halt', value = 'False')]
    battery_stat = DiagnosticStatus(name = 'Battery', level = 0, 
                                  message = 'OK')
    battery_stat.values = [ KeyValue(key = 'Over Voltage', value = 'False'),
                          KeyValue(key = 'Under Voltage', value = 'False')]
    lights_stat = DiagnosticStatus(name = 'Lights', level = 0, 
                                  message = 'OK')
    lights_stat.values = [ KeyValue(key = 'Controller Halt', value = 'False')]

    array.status = [  motor_l_stat, motor_r_stat, battery_stat, lights_stat ]

    my_rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        pub.publish(array)
        my_rate.sleep()