#!/usr/bin/env python

import roslib; roslib.load_manifest('mrp2_analyzer')

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

if __name__ == '__main__':
    rospy.init_node('mrp2_power_sim')

    pub = rospy.Publisher('/diagnostics', DiagnosticArray)
    
    array = DiagnosticArray()
    # Fake power board status, estop is hit
    power_stat = DiagnosticStatus(name = 'Power board 1000', level = 0, 
                                  message = 'Running')
    power_stat.values = [ KeyValue(key = 'Runstop hit', value = 'False'),
                          KeyValue(key = 'Estop hit', value = 'True')]
    # Fake EtherCAT Master status, level is Error
    eth_stat = DiagnosticStatus(name='EtherCAT Master', level = 2,
                                message = 'Motors Halted')

    array.status = [ power_stat, eth_stat ]

    my_rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        pub.publish(array)
        my_rate.sleep()