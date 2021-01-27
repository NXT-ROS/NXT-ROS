#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



import roslib; roslib.load_manifest('nxt_ros')
import nxt.locator
import rospy
import math
from nxt.motor import PORT_A, PORT_B, PORT_C
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
from nxt.sensor import Type
import nxt.sensor
import nxt.motor
import thread
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool
from nxt_msgs.msg import Range, Contact, JointCommand, Color, Light, Gyro, Accelerometer
from PyKDL import Rotation

POWER_TO_NM = 0.01
POWER_MAX = 125

global my_lock
my_lock = thread.allocate_lock()

def check_params(ns, params):
    for p in params:
        if not rospy.get_param(ns+'/'+p):
            return False
    return True


# base class for sensors
class Device:
    def __init__(self, params):
        self.desired_period = 1.0 / params['desired_frequency']
        self.period = self.desired_period
        self.initialized = False
        self.name = params['name']

    def needs_trigger(self):
        # initialize
        if not self.initialized:
            self.initialized = True
            self.last_run = rospy.Time.now()
            rospy.logdebug('Initializing %s'%self.name)
            return False
        # compute frequency
        now = rospy.Time.now()
        period = 0.9 * self.period + 0.1 * (now - self.last_run).to_sec()

        # check period
        if period > self.desired_period * 1.2:
            rospy.logwarn("%s not reaching desired frequency: actual %f, desired %f"%(self.name, 1.0/period, 1.0/self.desired_period))
        elif period > self.desired_period * 1.5:
            rospy.logerr("%s not reaching desired frequency: actual %f, desired %f"%(self.name, 1.0/period, 1.0/self.desired_period))

        return period > self.desired_period


    def do_trigger(self):
        try:
          rospy.logdebug('Trigger %s with current frequency %f'%(self.name, 1.0/self.period))
          now = rospy.Time.now()
          self.period = 0.9 * self.period + 0.1 * (now - self.last_run).to_sec()
          self.last_run = now
          self.trigger()
          rospy.logdebug('Trigger %s took %f mili-seconds'%(self.name, (rospy.Time.now() - now).to_sec()*1000))
        except nxt.error.DirProtError:
          rospy.logwarn("caught an exception nxt.error.DirProtError")
          pass
        except nxt.error.I2CError:
          rospy.logwarn("caught an exception nxt.error.I2CError")
          pass


class Motor(Device):
    def __init__(self, params, comm):
        Device.__init__(self, params)
        # create motor
        self.name = params['name']
        self.motor = nxt.motor.Motor(comm, eval(params['port']))
        self.cmd = 0 # the commanded power value

        # Use absolute position
        # True  = Return the position as an absolute value between
        #         0 to 2*pi radians (one full rotation).
        # False = The position ranges from -32,768 to +32,767
        #         degrees, relative to the starting position.
        #         After this the 16-bit signed integer will overflow.
        self.use_absolute_position = False
        # Number of degrees per revolution (integer),
        # this can be tuned for better acuracy
        self.counts_per_rev = 358

        if self.use_absolute_position:
            # Upon initialization, reset the motor encoder to zero.
            # Note: Setting this to True seems to do nothing.
            self.motor.reset_position(False)

        # create publisher
        self.pub = rospy.Publisher('joint_state', JointState, queue_size=10)
        self.last_js = None

        # create subscriber
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

        # Register the shutdown method
        rospy.on_shutdown(self.shutdown)

    def cmd_cb(self, msg):
        if msg.name == self.name:
            # Store the commanded power value,
            # limited to the range +/-125
            cmd = msg.effort / POWER_TO_NM
            if cmd > POWER_MAX:
                cmd = POWER_MAX
            elif cmd < -POWER_MAX:
                cmd = -POWER_MAX
            self.cmd = cmd

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append(self.name)

        # Get the rotational position of the motor
        rotation_count = self.motor.get_tacho().rotation_count
        position_in_radians = rotation_count * math.pi / 180.0

        if self.use_absolute_position:
            # Check if we have gone backwards
            # past the starting position
            if position_in_radians < 0.0:
                position_in_radians = 2.0*math.pi + position_in_radians

        js.position.append(position_in_radians)
        js.effort.append(self.cmd * POWER_TO_NM) # this is just the commanded effort
        vel = 0
        if self.last_js:
            vel = (js.position[0]-self.last_js.position[0])/(js.header.stamp-self.last_js.header.stamp).to_sec()
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.pub.publish(js)
        self.last_js = js

        if self.use_absolute_position:
            # If motor has done a full rotation,
            # reset the encoder to zero
            if (rotation_count >= self.counts_per_rev) \
            or (rotation_count <= -1.0*self.counts_per_rev):
                self.motor.reset_position(False)

        # send command
        self.motor.run(int(self.cmd), 0)

    def shutdown(self):
        ''' Used to relese the NXT nicely, kill motor, etc ... '''
        print("Killing the motor " + self.name)
        self.motor.run(0, 0)

class TouchSensor(Device):
    def __init__(self, params, comm):
        Device.__init__(self, params)
        # create touch sensor
        self.touch = nxt.sensor.Touch(comm, eval(params['port']))
        self.frame_id = params['frame_id']

        # create publisher
        self.pub = rospy.Publisher(params['name'], Contact)

    def trigger(self):
        ct = Contact()
        ct.contact = self.touch.get_sample()
        ct.header.frame_id = self.frame_id
        ct.header.stamp = rospy.Time.now()
        self.pub.publish(ct)


class UltraSonicSensor(Device):
    def __init__(self, params, comm):
        Device.__init__(self, params)
        # Create ultrasonic sensor
        self.ultrasonic = nxt.sensor.Ultrasonic(comm, eval(params['port']))
        self.frame_id = params['frame_id']
        self.spread = params['spread_angle']
        self.min_range = params['min_range']
        self.max_range = params['max_range']

        # Enable the sensor
        mode = nxt.sensor.Ultrasonic.Commands.CONTINUOUS_MEASUREMENT
        self.ultrasonic.command(mode)

        # Create publisher
        self.pub = rospy.Publisher(params['name'], Range, queue_size=10)

    def trigger(self):
        ds = Range()
        ds.header.frame_id = self.frame_id
        ds.header.stamp = rospy.Time.now()

        # Read the distance and convert to meters
        ds.range = self.ultrasonic.get_sample()/100.0

        # Publish the spread angle and min/max
        # range values as defined by the user
        ds.spread_angle = self.spread
        ds.range_min = self.min_range
        ds.range_max = self.max_range
        self.pub.publish(ds)


class GyroSensor(Device):
    """
    This uses the HiTechnic gyro sensor to
    measure angular velocity around the x,y,z axes
    and calculate an estimate of the orientation.
    """
    def __init__(self, params, comm):
        Device.__init__(self, params)
        #create gyro sensor
        self.gyro = nxt.sensor.HTGyro(comm, eval(params['port']))
        self.frame_id = params['frame_id']
        self.orientation = 0.0
        self.offset = params['offset']
        self.prev_time = rospy.Time.now()

        # calibrate
        rospy.loginfo("Calibrating Gyro. Don't move the robot now")
        start_time = rospy.Time.now()
        cal_duration = rospy.Duration(2.0)
        offset = 0
        tmp_time = rospy.Time.now()
        while rospy.Time.now() < start_time + cal_duration:
            rospy.sleep(0.01)
            sample = self.gyro.get_sample()
            now = rospy.Time.now()
            offset += (sample * (now - tmp_time).to_sec())
            tmp_time = now
        self.offset = offset / (tmp_time - start_time).to_sec()
        rospy.loginfo("Gyro calibrated with offset %f"%self.offset)

        # create publisher
        self.pub = rospy.Publisher(params['name'], Gyro, queue_size=10)

        # create publisher
        self.pub2 = rospy.Publisher(params['name']+"_imu", Imu, queue_size=10)

    def trigger(self):
        sample = self.gyro.get_sample()
        gs = Gyro()
        gs.header.frame_id = self.frame_id
        gs.header.stamp = rospy.Time.now()
        gs.calibration_offset.x = 0.0
        gs.calibration_offset.y = 0.0
        gs.calibration_offset.z = self.offset
        gs.angular_velocity.x = 0.0
        gs.angular_velocity.y = 0.0
        gs.angular_velocity.z = (sample-self.offset)*math.pi/180.0
        gs.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
        self.pub.publish(gs)

        imu = Imu()
        imu.header.frame_id = self.frame_id
        imu.header.stamp = rospy.Time.now()
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = (sample-self.offset)*math.pi/180.0
        imu.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
        imu.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.1]
        self.orientation += imu.angular_velocity.z * (imu.header.stamp - self.prev_time).to_sec()
        self.prev_time = imu.header.stamp
        (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) = Rotation.RotZ(self.orientation).GetQuaternion()
        self.pub2.publish(imu)


class AccelerometerSensor(Device):
    """
    This uses the HiTechnic accelerometer sensor
    to measure acceleration on the x,y,z axes.
    """
    def __init__(self, params, comm):
        Device.__init__(self, params)
        # create accelerometer sensor
        self.accel = nxt.sensor.HTAccelerometer(comm, eval(params['port']))
        self.frame_id = params['frame_id']

        # create publisher
        self.pub = rospy.Publisher(params['name'], Accelerometer)

    def trigger(self):
        gs = Accelerometer()
        gs.header.frame_id = self.frame_id
        gs.header.stamp = rospy.Time.now()
        x,y,z = self.accel.get_acceleration()

        gs.linear_acceleration.x = x*9.8
        gs.linear_acceleration.y = y*9.8
        gs.linear_acceleration.z = z*9.8
        gs.linear_acceleration_covariance = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.pub.publish(gs)


class ColorSensor(Device):
    """
    This uses the NXT 2.0 RGB color sensor
    to detect 1 of 6 preset color values.
    """
    def __init__(self, params, comm):
        Device.__init__(self, params)
        # create color sensor
        self.color = nxt.sensor.Color20(comm, eval(params['port']))
        self.frame_id = params['frame_id']

        # Turn on the LED light
        self.color.set_light_color(Type.COLORFULL) # white

        # create publisher
        self.pub = rospy.Publisher(params['name'], Color)

    def trigger(self):
        co = Color()
        co.header.frame_id = self.frame_id
        co.header.stamp = rospy.Time.now()
        # Get the color value (an integer from 1 to 6)
        color = self.color.get_color()
        if color == 1:  # black
            co.r = 0.0
            co.g = 0.0
            co.b = 0.0
        elif color == 2: # blue
            co.r = 0.0
            co.g = 0.0
            co.b = 1.0
        elif color == 3: # green
            co.r = 0.0
            co.g = 1.0
            co.b = 0.0
        elif color == 4: # yellow
            co.r = 1.0
            co.g = 1.0
            co.b = 0.0
        elif color == 5: # red
            co.r = 1.0
            co.g = 0.0
            co.b = 0.0
        elif color == 6: # white
            co.r = 1.0
            co.g = 1.0
            co.b = 1.0
        else:
            rospy.logerr('Undefined color of color sensor')
        co.intensity = color
        self.pub.publish(co)


class IntensitySensor(Device):
    """
    This uses the NXT 2.0 RGB color sensor to
    measure the intensity of reflected light,
    with the option to enable the red, green,
    or blue LEDs individually.
    """
    def __init__(self, params, comm):
        Device.__init__(self, params)
        # create intensity sensor
        self.intensity = nxt.sensor.Color20(comm, eval(params['port']))
        self.frame_id = params['frame_id']
        self.color_r = params['color_r']
        self.color_g = params['color_g']
        self.color_b = params['color_b']

        if self.color_r == 1.0 and self.color_g == 0.0 and self.color_b == 0.0:
            self.color = Type.COLORRED
        elif self.color_r == 0.0 and self.color_g == 1.0 and self.color_b == 0.0:
            self.color = Type.COLORGREEN
        elif self.color_r == 0.0 and self.color_g == 0.0 and self.color_b == 1.0:
            self.color = Type.COLORBLUE
        elif self.color_r == 0.0 and self.color_g == 0.0 and self.color_b == 0.0:
            self.color = Type.COLORNONE
        else:
            rospy.logerr('Invalid RGB values specifies for intensity color sensor')

        # create publisher
        self.pub = rospy.Publisher(params['name'], Color)

    def trigger(self):
        co = Color()
        co.header.frame_id = self.frame_id
        co.header.stamp = rospy.Time.now()
        co.r = self.color_r
        co.g = self.color_g
        co.b = self.color_b
        # Get the reflected light reading.
        # This turns on the LED if color = red/green/blue
        co.intensity = self.intensity.get_reflected_light(self.color)
        self.pub.publish(co)


class LightSensor(Device):
    """
    This uses the NXT light sensor to
    measure the intensity of ambient light,
    with the option to enable the LED to
    measure reflected light.
    """
    def __init__(self, params, comm):
        Device.__init__(self, params)
        # Create light sensor
        self.light = nxt.sensor.Light(comm, eval(params['port']))
        self.frame_id = params['frame_id']
        self.illuminated = params['illuminated']

        # Enable the sensor
        if self.illuminated:
            self.light.set_illuminated(active=True)
        else:
            self.light.set_illuminated(active=False)

        # Create publisher
        self.pub = rospy.Publisher(params['name'], Light)

    def trigger(self):
        co = Light()
        co.header.frame_id = self.frame_id
        co.header.stamp = rospy.Time.now()
        co.intensity = self.light.get_lightness()
        self.pub.publish(co)


def main():
    rospy.init_node('nxt_ros')
    ns = 'nxt_robot'
    host = rospy.get_param("~host", None)
    b = nxt.locator.find_one_brick(host)

    config = []

    # Define exit handler
    def cleanup_node():
        print("Shutting down node")
        for c in config:
            if c['type'] == 'color':
                # If there's a color sensor, turn off the LED light
                cs = nxt.sensor.Color20(b, eval(c['port']))
                cs.set_light_color(Type.COLORNONE)
            elif c['type'] == 'intensity':
                # If there's an intensity sensor, turn off the LED light
                s = nxt.sensor.Color20(b, eval(c['port']))
                s.set_light_color(Type.COLORNONE)
            elif c['type'] == 'light':
                # If there's a light sensor, turn off the LED light
                ls = nxt.sensor.Light(b, eval(c['port']))
                ls.set_illuminated(active=False)
            elif c['type'] == 'ultrasonic':
                # If there's an ultrasonic sensor, turn it off
                us = nxt.sensor.Ultrasonic(b, eval(c['port']))
                mode = nxt.sensor.Ultrasonic.Commands.CONTINUOUS_MEASUREMENT
                us.command(mode)
    rospy.on_shutdown(cleanup_node)

    config = rospy.get_param("~"+ns)
    components = []
    for c in config:
        rospy.loginfo("Creating %s with name %s on %s",c['type'],c['name'],c['port'])
        if c['type'] == 'motor':
            components.append(Motor(c, b))
        elif c['type'] == 'touch':
            components.append(TouchSensor(c, b))
        elif c['type'] == 'ultrasonic':
            components.append(UltraSonicSensor(c, b))
        elif c['type'] == 'color':
            components.append(ColorSensor(c, b))
        elif c['type'] == 'intensity':
            components.append(IntensitySensor(c, b))
        elif c['type'] == 'light':
            components.append(LightSensor(c, b))
        elif c['type'] == 'gyro':
            components.append(GyroSensor(c, b))
        elif c['type'] == 'accelerometer':
            components.append(AccelerometerSensor(c, b))
        else:
            rospy.logerr('Invalid sensor/actuator type %s'%c['type'])

    callback_handle_frequency = 100.0
    last_callback_handle = rospy.Time.now()
    while not rospy.is_shutdown():
        my_lock.acquire()
        triggered = False
        for c in components:
            if c.needs_trigger() and not triggered:
                c.do_trigger()
                triggered = True
        my_lock.release()
        now = rospy.Time.now()
        if (now - last_callback_handle).to_sec() > 1.0/callback_handle_frequency:
            last_callback_handle = now
            rospy.sleep(0.01)


if __name__ == '__main__':
    main()

