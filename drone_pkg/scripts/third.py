#!/usr/bin/env python


from __future__ import print_function

import argparse

import rospy
import mavros
from mavros.utils import *
from mavros import command


def _check_ret(args, ret):
    if not ret.success:
        fault("Request failed. Check mavros logs. ACK:", ret.result)

    print_if(args.verbose, "Command ACK:", ret.result)


def do_long(args):
    try:
        ret = command.long(
            broadcast=args.broadcast,
            command=args.command, confirmation=int(args.confirmation),
            param1=args.param1,
            param2=args.param2,
            param3=args.param3,
            param4=args.param4,
            param5=args.param5,
            param6=args.param6,
            param7=args.param7)
    except rospy.ServiceException as ex:
        fault(ex)

    _check_ret(args, ret)







def main():
    parser = argparse.ArgumentParser(description="Command line tool for sending commands to MAVLink device.")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace", default=mavros.DEFAULT_NAMESPACE)
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose output")
    parser.add_argument('--wait', action='store_true', help="Wait for establishing FCU connection")
    subarg = parser.add_subparsers()

    long_args = subarg.add_parser('long', help="Send any command (COMMAND_LONG)")
    long_args.set_defaults(func=do_long)
    long_args.add_argument('-c', '--confirmation', action='store_true', help="Require confirmation")
    long_args.add_argument('-b', '--broadcast', action='store_true', help="Broadcast command")
    long_args.add_argument('command', type=int, help="Command Code")
    long_args.add_argument('param1', type=float, help="param1")
    long_args.add_argument('param2', type=float, help="param2")
    long_args.add_argument('param3', type=float, help="param3")
    long_args.add_argument('param4', type=float, help="param4")
    long_args.add_argument('param5', type=float, help="param5 / x_lat")
    long_args.add_argument('param6', type=float, help="param6 / y_long")
    long_args.add_argument('param7', type=float, help="param7 / z_alt")

    
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("mavcmd", anonymous=True)
    mavros.set_namespace(args.mavros_ns)
    rospy.loginfo(args)
    if args.wait:
        wait_fcu_connection()

    args.func(args)


if __name__ == '__main__':
    main()
