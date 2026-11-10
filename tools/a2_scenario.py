#!/usr/bin/env python3
"""
a2_scenario.py
==============
Plays the TEACHER during our own test flights: calls only the sensor simulator's services
(/localization/set_gps_enabled, /localization/set_vo_enabled, /localization/inject_gps_outlier)
at chosen mission legs, exactly like the commands from the assignment.  It never talks to the
estimator or the controller; it only listens to /mission/leg to know when a leg starts.

Legs are the waypoint indices of /mission/leg: 0..3 = lap 1 corners, 4..7 = lap 2 corners
(leg 2 = lap 1, (2.5,-4) -> (2.5,0), a middle leg).

Usage (sim time, so the 15 s are simulation seconds):
    python3 tools/a2_scenario.py                       # GPS 15 s at leg 2, outlier at leg 5, VO 3 s at leg 6
    python3 tools/a2_scenario.py --outlier-leg -1      # -1 disables an event
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt32
from std_srvs.srv import SetBool, Trigger


class Scenario(Node):
    def __init__(self, args):
        super().__init__('a2_scenario', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UInt32, '/mission/leg', self.on_leg, latched)
        self.create_subscription(String, '/mission/state', self.on_state, latched)
        self.gps = self.create_client(SetBool, '/localization/set_gps_enabled')
        self.vo = self.create_client(SetBool, '/localization/set_vo_enabled')
        self.outlier = self.create_client(Trigger, '/localization/inject_gps_outlier')
        self.done_legs = set()
        self.finished = False
        self.timers_ = []
        self.get_logger().info(
            f'Scenario: GPS off {args.gps_outage_s:.0f} s at leg {args.gps_leg} (+{args.delay_s:.1f} s), '
            f'outlier at leg {args.outlier_leg}, VO off {args.vo_outage_s:.0f} s at leg {args.vo_leg}')

    def later(self, seconds, fn):
        """One-shot timer on the node clock (simulation time)."""
        holder = []

        def fire():
            holder[0].cancel()
            fn()
        holder.append(self.create_timer(max(seconds, 0.01), fire))
        self.timers_.append(holder[0])

    def call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'{what}: service {client.srv_name} not available')
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().warn(f'[sim t={t:.2f}] {what}')
        client.call_async(request)

    def set_gps(self, on):
        self.call(self.gps, SetBool.Request(data=on), 'GPS ENABLED' if on else 'GPS DISABLED')

    def set_vo(self, on):
        self.call(self.vo, SetBool.Request(data=on), 'VO ENABLED' if on else 'VO DISABLED')

    def on_leg(self, msg):
        leg = msg.data
        if leg in self.done_legs:
            return
        self.done_legs.add(leg)
        a = self.args
        if leg == a.gps_leg:
            self.later(a.delay_s, lambda: self.set_gps(False))
            self.later(a.delay_s + a.gps_outage_s, lambda: self.set_gps(True))
        if leg == a.outlier_leg:
            self.later(a.delay_s, lambda: self.call(self.outlier, Trigger.Request(), 'GPS OUTLIER injected'))
        if leg == a.vo_leg:
            self.later(a.delay_s, lambda: self.set_vo(False))
            self.later(a.delay_s + a.vo_outage_s, lambda: self.set_vo(True))

    def on_state(self, msg):
        if msg.data in ('DONE', 'ABORTED', 'STOPPED', 'FAULT'):
            self.get_logger().info(f'Mission ended ({msg.data}); restoring both sensors and exiting')
            self.set_gps(True)
            self.set_vo(True)
            self.finished = True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--gps-leg', type=int, default=2)
    ap.add_argument('--gps-outage-s', type=float, default=15.0)
    ap.add_argument('--outlier-leg', type=int, default=5)
    ap.add_argument('--vo-leg', type=int, default=6)
    ap.add_argument('--vo-outage-s', type=float, default=3.0)
    ap.add_argument('--delay-s', type=float, default=1.0, help='seconds after the leg starts')
    args = ap.parse_args()
    rclpy.init()
    node = Scenario(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(20):                      # let the last service calls go out
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
