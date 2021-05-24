#!/usr/bin/python

"""
Print quarternion value of RPY rotation [0, 0, pi/2]
"""

from tf_conversions import transformations as tf
from math import pi

if __name__ == '__main__':

    q = tf.quaternion_from_euler(0, 0, pi / 2)

    print(q)