#!/usr/bin/env python

import rospy
import math
from amigo_msgs.msg import RGBLightCommand
def talker():
	pub = rospy.Publisher('user_set_rgb_lights', RGBLightCommand)
	rospy.init_node('police')
	while not rospy.is_shutdown():
		rgb_msg = RGBLightCommand()
		if rospy.get_time()%1<=0.5:
			rgb_msg.color.r = 1.0
			rgb_msg.color.b = 0.0
		else:
			rgb_msg.color.r = 0.0
			rgb_msg.color.b = 1.0
		#amp = floor(amp+0.5);
		rgb_msg.show_color.data = True
		pub.publish(rgb_msg)
		rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
