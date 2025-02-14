#!/usr/bin/env python

'''
Fake node to pub state msg (should be published by state classification)

[0] good: follow predefined trajectory
[1] too left ~ move to right: y-
[2] too right ~ move to left: y+
[3] too high ~ move to lower: z-
[4] too low ~ move to higher: z+
[5] too far ~ move back: x-
'''

import rospy
from std_msgs.msg import Int32
import random
import time

class FakeClassifierNode:
    def __init__(self):
        rospy.init_node('fake_classifier_node', anonymous=True)
        self.class_pub = rospy.Publisher('/classification', Int32, queue_size=10)

    def run(self):
        rate = rospy.Rate(0.5)  
        while not rospy.is_shutdown():
            # Generate a random classification value between 0 and 5
            classification_value = random.randint(0, 5)

            # Publish the classification message
            self.class_pub.publish(classification_value)
            print("fake state class: ", classification_value)

            rate.sleep()

if __name__ == '__main__':
    try:
        fake_classifier = FakeClassifierNode()
        fake_classifier.run()
    except rospy.ROSInterruptException:
        pass
