#!/usr/bin/env python
# license removed for brevity
import rospy
import rbha


def talker():
    rbha.open_serial()
    while not rospy.is_shutdown():
        rbha.reader()
        rbha.do_update()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
