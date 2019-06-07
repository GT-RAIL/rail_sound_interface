#!/usr/bin/env python
# Command line utility to using the sound client

from __future__ import print_function, division

import sys
import argparse

import rospy

from rail_sound_interface import SoundClient


# Speak
def _on_speak(client, args):
    text = args.text
    affect = args.affect
    client.say(text, affect, blocking=True)


# Beep
def _on_beep(client, args):
    client.beep(args.key, blocking=True)


# Get the arg parser
def _get_arg_parser(client=None):
    if client is None:
        client = SoundClient()

    parser = argparse.ArgumentParser()
    parser.add_argument("--server", help="Name of the action server for sounds")
    subparsers = parser.add_subparsers()

    speak_parser = subparsers.add_parser("speak", help="get the robot to speak")
    speak_parser.add_argument("text", help="text to speak")
    speak_parser.add_argument("--affect", choices=client.get_affect_names(),
                              help="say things in an affected voice")
    speak_parser.set_defaults(func=_on_speak)

    beep_parser = subparsers.add_parser("beep", help="get the robot to beep")
    beep_parser.add_argument("key", help="type of beep to produce",
                             choices=client.get_beep_names())
    beep_parser.set_defaults(func=_on_beep)

    return parser


# The main script
def main():
    rospy.init_node('speaker_test')
    parser = _get_arg_parser()
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    client = SoundClient(server_name=args.server)
    client.connect()

    args.func(client, args)
    rospy.sleep(0.5)


if __name__ == '__main__':
    main()
