#!/usr/bin/env python
import rospy
import tf2_ros
import pandas as pd
import numpy as np

def radius2angle():
    rospy.init_node('radius2angle_node')

    input_filename = rospy.get_param('~input_measurement_series')
    output_filename = rospy.get_param('~output_measurement_series')
    base_frame = rospy.get_param('~base_frame')
    laser_frame = rospy.get_param('~laser_frame')
    wheel_base = rospy.get_param('~wheel_base')

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(1.0)
    start_time = rospy.Time.now()
    timeout = rospy.Duration(10.0)

    while not rospy.is_shutdown():

        passed_time = rospy.Time.now() - start_time
        if passed_time >= timeout:
            rospy.logerr('Timeout - Could not receive transform from base frame to laser frame!')
            return

        try:
            trans = buffer.lookup_transform(base_frame, laser_frame, rospy.Time())
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass
        
        rate.sleep()

    df = pd.read_csv(input_filename)
    actuator_value = df['actuator_value'].values.astype(np.float64)
    turning_radius = df['measurement'].values.astype(np.float64)

    x = abs(trans.transform.translation.x)
    y = trans.transform.translation.y

    if (np.any(turning_radius < x)):
        rospy.logerr('turning radius must be greater than x-displacement between base and laser frame')
        return
    
    turning_radius = np.sqrt(np.square(turning_radius) - np.square(x))
    # Considering y displacement depending on the direction of the turn.
    # Positive actuator values indicate left turns while negative actuator values indicate right turns.
    turning_radius = turning_radius - np.sign(actuator_value) * y
    steering_angle = np.arctan(wheel_base / turning_radius)

    df = pd.DataFrame.from_dict({'actuator_value': actuator_value, 'measurement': steering_angle})
    df.to_csv(output_filename, sep=',', index=False)

if __name__ == '__main__':
    try:
        radius2angle()
    except rospy.ROSInterruptException:
        pass