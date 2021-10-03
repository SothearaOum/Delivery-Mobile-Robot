#! /usr/bin/env python
# pip install inputs
from __future__ import print_function
from inputs import devices, get_gamepad
import rospy
from geometry_msgs.msg import Twist

# print connected device
print("Detected controller:\n")
	for device in devices:
		print(device)

# create ros node and publisher
rospy.init_node("cmd_v_xbox")
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
twt = Twist()

while not rospy.is_shutdown():
	events = get_gamepad()
		for event in events:
			print(event.ev_type, event.code, event.state)
	twt.linear.x = events[]
	twt.angular.z = events[]
	pub.publish(twt)
	rospy.sleep(0.05)