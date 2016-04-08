#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Fontys
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
#
# Revision $Id$

PACKAGE = 'rosbee_node' # this package name
NAME = 'robot_node' # this node name

"""
Robot node for ROS.
  This node is based on turtlebot_node by Willow Garage, 
  which is based on otl_roomba by OTL (otl-ros-pkg)
  and Damon Kohler's pyrobot.py.
"""

# ROS imports
# import roslib; roslib.load_manifest(PACKAGE)
import rospy
import tf.transformations
from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, \
    Quaternion, Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry
from covariances import \
    ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2

# Other imports
import serial
import threading
from math import sin, cos


SERIAL_TIMEOUT = 0.1  # Serial read timeout (seconds)

class SerialError(Exception):
  pass
  
class SerialInterface(object):

  """A wrapper around pySerial for use with robot platform"""

  def __init__(self, tty, baudrate):
    self.ser = None
    try:
      self.ser = serial.Serial(tty, baudrate, 8, 'N', 1, SERIAL_TIMEOUT)
    except Exception, err:
    #if self.ser == None:
      raise SerialError('ERROR trying to open serial port')
    self.lock = threading.RLock()

  def __del__(self):
    if not self.ser == None:
      self.ser.close()
    
  def send(self, bytes):
    """Send a string to the robot."""
    sent = self.ser.write(bytes)
    if not sent==len(bytes):
      raise SerialError('ERROR writing to serial port.')

  def read(self):
    """Read a string from the robot."""
    raw_data = self.ser.readline()
    data = raw_data
    # strip \r and other whitespace
    data = raw_data.strip() 
    if not data:
      raise SerialError('ERROR reading from serial port. No data.')
    return data

  def flush_input(self):
    """Flush input buffer."""
    self.ser.flushInput()

class Robot(object):

  """Represents the robot."""

  def __init__(self, tty, baudrate, connection_timeout, fake_connection=False, fake_respons=False):
    self.fake_connection = fake_connection
    self.fake_respons = fake_respons
    self.connection_timeout = connection_timeout
    self.tty = tty
    self.baudrate = baudrate
    self.sci = None

  def send_command(self, command):
    """Send command to the robot.
    @param command: command to send to robot (tuple)   
    returns: respons from robot (tuple)
    """
    #msg = ','.join(map(str, command))
    msg = ','.join(str(x) for x in command) + '\r'
    #rospy.loginfo("msg=%s", str(msg))
    
    if not self.fake_connection:
      done = False
      start_time = rospy.get_rostime()
      while not done:
        try:
          if self.sci is None:
            self.sci = SerialInterface(self.tty, self.baudrate)
          with self.sci.lock:
            self.sci.flush_input()
            self.sci.send(msg)
            rsp = self.sci.read()
          #rsp = msg
          #rospy.loginfo("rsp=%s", str(rsp))
          done = True
        except Exception, err:
          rospy.loginfo(str(err))
          if rospy.get_rostime() - start_time > self.connection_timeout:
            raise rospy.ROSInterruptException("connection to robot seems broken")
          self.sci = None # assume serial interface is broken
          rospy.loginfo('waiting for connection')
          rospy.sleep(1.) # wait 1 seconds before retrying
              
    if self.fake_connection or self.fake_respons:
      rsp = msg

    return tuple(rsp.split(','))
  
  def start(self):
	  """Start the robot."""
	  command = ('$1',)
	  respons = self.send_command(command)
	  rospy.loginfo('robot started')
    
  def stop(self):
    """Stop the robot."""
    command = ('$0',)
    respons = self.send_command(command)
    rospy.loginfo('robot stopped')
    
  def drive(self, cmd_vel):
    """Drive the robot.
    @param cmd_vel: velocity setpoint = (vx, vy, vth)
          vx:  linear velocity along the x-axis (m/s)
          vy:  linear velocity along the y-axis (m/s)
          vth: angular velocity about the z-axis (rad/s), also called yaw 

    returns current state, including velocity, as measured by robot using its encoders     
    """
    vx, vy, vth = cmd_vel

    cmd = ('$2', int(round(vx*1000)), int(round(vy*1000)), int(round(vth*1000)))
    rsp = self.send_command(cmd)
    
    if rsp[0] == '$2':
    	state = tuple(float(x)/1000 for x in rsp[1:])
    else:
      state = 'INV_DATA'
      rospy.logerr('Invalid data received from from Propellor')   
    
    return state

    
class RobotNode(object):

  """The ROS robot node."""

  def __init__(self, name):

    rospy.init_node(name)
        
    self._init_params()
    self._init_pubsub()
        
    self.robot = Robot(self.port, self.baudrate, self.connection_timeout, self.fake_connection, self.fake_respons)

    self.req_cmd_vel = None
    self._pos2d = Pose2D()
    
    self.start()
    rospy.loginfo('spinning')
    self.spin()
    self.stop()
   
  def _init_params(self):

    # node general
    self.update_rate = rospy.get_param('~update_rate', 50.0)
    self.verbose = rospy.get_param('~verbose', True)
    
    # fake serial connection to a robot
    self.fake_connection = rospy.get_param('~fake_connection', True)

    # fake ideal status respons from the robot
    self.fake_respons = rospy.get_param('~fake_respons', False)

    # serial connection parameters
    self.port = rospy.get_param('~port', '/dev/ttyUSB0')
    self.baudrate = rospy.get_param('~baudrate', 115200)
    self.connection_timeout = rospy.Duration(rospy.get_param('~connection_timeout', 60))

    # cmd_vel
    self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
    self.min_abs_yaw_vel = rospy.get_param('~min_abs_yaw_vel', None)
    self.max_abs_yaw_vel = rospy.get_param('~max_abs_yaw_vel', None)
    
    # odom: correction factors from calibration
    self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
    self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
    
    # tf
    self.publish_tf = rospy.get_param('~publish_tf', True)
    self.odom_frame = rospy.get_param('~odom_frame', 'odom')
    self.base_frame = rospy.get_param('~base_frame', 'base_footprint')

    # print
    if self.fake_connection:
      rospy.loginfo("fake robot connection")
    else:
      rospy.loginfo("connect to real robot")
      rospy.loginfo("serial port: %s"%(self.port))
      rospy.loginfo("baudrate: %s"%(self.baudrate))
    
    if self.fake_respons:
      rospy.loginfo("fake robot respons")

    rospy.loginfo("update_rate: %s"%(self.update_rate))
    
    if self.publish_tf:
       rospy.loginfo("publish tf")
    else:
       rospy.loginfo("no tf published")

  def _init_pubsub(self):
    self.odom_pub = rospy.Publisher('odom', Odometry, queue_size = 10)
    self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
    self.transform_broadcaster = None
    if self.publish_tf:
      self.transform_broadcaster = tf.TransformBroadcaster()

  def cmd_vel(self, msg):
    # Clamp to min abs yaw velocity, to avoid trying to rotate at low
    # speeds, which doesn't work well.
    if self.min_abs_yaw_vel is not None and \
          msg.angular.z != 0.0 and \
            abs(msg.angular.z) < self.min_abs_yaw_vel:
      msg.angular.z = self.min_abs_yaw_vel if msg.angular.z > 0.0 else -self.min_abs_yaw_vel
    # Limit maximum yaw to avoid saturating the gyro
    if self.max_abs_yaw_vel is not None and \
            self.max_abs_yaw_vel > 0.0 and \
              msg.angular.z != 0.0 and \
                abs(msg.angular.z) > self.max_abs_yaw_vel: 
      msg.angular.z = self.max_abs_yaw_vel if msg.angular.z > 0.0 else -self.max_abs_yaw_vel 

    # Convert twist to velocity setpoint
    # Assume robot drives in 2-dimensional world
    self.req_cmd_vel = msg.linear.x, msg.linear.y, msg.angular.z

  def compute_odom(self, velocities, last_time, current_time, odom):
    """
    Compute current odometry.  Updates odom instance and returns tf
    transform. compute_odom() does not set frame ids or covariances in
    Odometry instance.  It will only set stamp, pose, and twist.

    @param velocities: linear and angular velocities
      @type  velocities: (vx, vy, vth)
    @param last_time: time of last calculation
      @type  last_time: rospy.Time
    @param current_time: current time
      @type  current_time: rospy.Time
    @param odom: Odometry instance to update.
      @type  odom: nav_msgs.msg.Odometry

    @return: transform
    @rtype: ( (float, float, float), (float, float, float, float) )
    """
    dt = (current_time - last_time).to_sec()
    vx, vy, vth = velocities
    
    # Calculating delta distance (d) and delta_angle (angle) from velocities
    d  = (vx * dt) * self.odom_linear_scale_correction #correction factor from calibration
    angle = (vth * dt) * self.odom_angular_scale_correction #correction factor from calibration

    x = cos(angle) * d
    y = -sin(angle) * d

    last_angle = self._pos2d.theta
    self._pos2d.x += cos(last_angle)*x - sin(last_angle)*y
    self._pos2d.y += sin(last_angle)*x + cos(last_angle)*y
    self._pos2d.theta += angle

    # Rosbee quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
    odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))

    # construct the transform
    transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

    # update the odometry state
    odom.header.stamp = current_time
    odom.pose.pose   = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
    odom.twist.twist = Twist(Vector3(d/dt, 0, 0), Vector3(0, 0, angle/dt))
    """
    # old code did not apply correction to velocities
    # odom calculation from velocities
    th = self._pos2d.theta
    self._pos2d.x += (vx * cos(th) - vy * sin(th)) * self.odom_linear_scale_correction * dt;
    self._pos2d.y += (vx * sin(th) + vy * cos(th)) * self.odom_linear_scale_correction* dt;
    self._pos2d.theta += vth * self.odom_angular_scale_correction * dt;
        
    # Quaternion from yaw
    odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))

    # construct the transform
    transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

    # update the odometry state
    odom.header.stamp = current_time
    odom.pose.pose   = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    """


    if vx == 0.0 and \
          vy == 0.0 and \
            vth == 0.0:
      odom.pose.covariance = ODOM_POSE_COVARIANCE2
      odom.twist.covariance = ODOM_TWIST_COVARIANCE2
    else:
      odom.pose.covariance = ODOM_POSE_COVARIANCE
      odom.twist.covariance = ODOM_TWIST_COVARIANCE

    # return the transform
    return transform   
        
  def publish_odometry_transform(self, odometry):
    self.transform_broadcaster.sendTransform(
        (odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z),
        (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
         odometry.pose.pose.orientation.w),
         odometry.header.stamp, odometry.child_frame_id, odometry.header.frame_id)
   
  def spin(self):

    odom = Odometry(header=rospy.Header(frame_id=self.odom_frame), child_frame_id=self.base_frame)
    transform = None    
    last_state_time = rospy.get_rostime()
    last_vel_state = (0.0, 0.0, 0.0)
    last_other_state = ()
    req_cmd_vel = (0.0, 0.0, 0.0)
    last_cmd_vel_time = rospy.get_rostime()
    
    r = rospy.Rate(self.update_rate)
    while not rospy.is_shutdown():
      current_time = rospy.get_rostime()
                 
      # ACT & SENSE
      if self.req_cmd_vel is not None:
        req_cmd_vel = self.req_cmd_vel
        # set to None to signal that command is handled
        self.req_cmd_vel = None
        # reset time for timeout
        last_cmd_vel_time = current_time
      else:
        #stop on timeout
        if current_time - last_cmd_vel_time > self.cmd_vel_timeout:
          req_cmd_vel = (0.0, 0.0, 0.0)
          if self.verbose:
            rospy.loginfo('timeout')
      
      # send velocity command & receive state
      old_state_time = last_state_time
      old_vel_state = last_vel_state     
      last_state = self.robot.drive(req_cmd_vel)
      last_state_time = current_time
      
      if last_state != 'INV_DATA':
       	last_vel_state = last_state[:3]
      	last_other_state = last_state[3:]

      	# COMPUTE ODOMETRY
      	# use average velocity, i.e. assume constant acceleration
      	avg_vel_state = tuple((float(x) + float(y))/2 for x, y in zip(old_vel_state, last_vel_state))
      	transform = self.compute_odom(avg_vel_state, old_state_time, last_state_time, odom)
      	# PUBLISH ODOMETRY
      	self.odom_pub.publish(odom)
    
        if self.publish_tf:
          self.publish_odometry_transform(odom)              

      if self.verbose:
        rospy.loginfo("velocity setpoint: %s", str(req_cmd_vel))
        rospy.loginfo("velocity measured: %s", str(last_vel_state))
        rospy.loginfo("pose: %s", str(transform))
        rospy.loginfo("debug: %s", str(last_other_state))
        rospy.loginfo("last_state: %s", str(last_state))

      r.sleep()
            
  def start(self):
    self.robot.start()
    rospy.loginfo('robot node started')
    
  def stop(self):
    self.robot.stop()
    rospy.loginfo('robot node stopped')


if __name__ == '__main__':
  rospy.loginfo("%s started...", NAME)
  try:
    RobotNode(NAME)
  except rospy.ROSInterruptException, err:
    rospy.loginfo(str(err))
    rospy.loginfo("%s stopped working...", NAME)
 
