#!/usr/bin/env python
# The node for the action server that executes the task plan

import rospy

from rail_sound_interface import SoundServer


def main():
    rospy.init_node(SoundServer.SOUND_SERVER_NAME)
    server = SoundServer()
    server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
