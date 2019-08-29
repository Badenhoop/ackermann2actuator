#!/usr/bin/env python
import rospy
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression

def measurements2params():
    rospy.init_node('measurements2params_node')

    input_filename = rospy.get_param('~measurement_series')
    output_filename = rospy.get_param('~params')

    df = pd.read_csv(input_filename)
    X = df['actuator_value'].values.astype(np.float64)
    X = np.expand_dims(X, axis=1)
    y = df['measurement'].values.astype(np.float64)

    reg = LinearRegression().fit(X, y)
    
    with open(output_filename, 'w') as f:
        f.write('coefficient: {}\n'.format(reg.coef_[0]))
        f.write('intercept: {}\n'.format(reg.intercept_))

if __name__ == '__main__':
    try:
        measurements2params()
    except rospy.ROSInterruptException:
        pass