#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(data):
    rospy.loginfo("{:.04}".format(data.ranges[0]))

def main():
    rospy.init_node("measure_velocity_node", log_level=rospy.INFO)
    scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
