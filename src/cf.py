#!/usr/bin/python3.8

import rospy
import time
from std_msgs.msg import Float32MultiArray
import cflib.crtp
from cflib.utils import uri_helper
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from threading import Thread
uri = 'radio://0/90/2M/E7E7E7E703'
uri = uri_helper.uri_from_env(default=uri)

class CrazylieInterface:
    def __init__(self):
        rospy.init_node('cf', anonymous=True)
        self.cf = Crazyflie()
        self.rate = rospy.Rate(60) # 60hz

        self.oripub = rospy.Publisher('/crazyflie/oriention', Float32MultiArray, queue_size=1)
        self.command_sub = rospy.Subscriber('/crazyflie/command', Float32MultiArray, self.command_callback)
        self.command = [0, 0, 0, 0]
        self.step = 0

        self.cf.open_link(uri)
        cflib.crtp.init_drivers()
        self.cf.connected.add_callback(self._connected)

        self.lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self.lg_stab.add_variable('stabilizer.roll', 'float')
        self.lg_stab.add_variable('stabilizer.pitch', 'float')
        self.lg_stab.add_variable('stabilizer.yaw', 'float')

    def main(self):

        with SyncCrazyflie(uri, cf=self.cf) as scf:
            self.simple_log_async(scf, self.lg_stab)
        self.cf.commander.send_setpoint(0, 0, 0, 0)

    def log_stab_callback(self,timestamp, data, logconf):
        data = Float32MultiArray(data=[data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw']])
        self.oripub.publish(data)

    def simple_log_async(self,scf, logconf):
        cf = scf.cf
        cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(self.log_stab_callback)
        logconf.start()
        while not rospy.is_shutdown():
            time.sleep(1)
        logconf.stop()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start()
    def command_callback(self, msg):
        print("received command")
        roll = msg.data[0]
        pitch = msg.data[1]
        yawrate = msg.data[2]
        thrust = int(msg.data[3]/1.05)
        self.command = [roll, pitch, yawrate, thrust]

    def _ramp_motors(self):
        # Unlock startup thrust protection
        self.cf.commander.send_setpoint(0, 0, 0, 0)

        while not rospy.is_shutdown():
            roll = int(self.command[0])
            pitch = -int(self.command[1])
            yawrate = int(self.command[2])
            thrust = int(self.command[3])
            # print("time:",time.time(),thrust)
            self.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            self.rate.sleep()
        self.cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self.cf.close_link()
cf = CrazylieInterface()
cf.main()