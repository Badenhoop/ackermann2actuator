#!/usr/bin/env python
import rospy

def radius2angle():
    rospy.init_node('radius2angle_node')

    input_filename = rospy.get_param('~input_measurement_series')
    output_filename = rospy.get_param('~output_measurement_series')

    data_df = pd.read_csv(input_filename)
    X = data_df['actuator_value'].values.astype(np.float64)
    X = np.expand_dims(X, axis=1)
    y = data_df['measurement'].values.astype(np.float64)

    # TODO: implemenation

if __name__ == '__main__':
    try:
        radius2angle()
    except rospy.ROSInterruptException:
        pass