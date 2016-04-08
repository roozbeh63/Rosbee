# !/usr/bin/env python
from nav_msgs.msg import Odometry
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf
import rbha
from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, \
    Quaternion, Twist, TwistWithCovariance, Vector3
from math import sin, cos
from covariances import \
    ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2
PACKAGE = 'rosbee_node'  # this package name
NAME = 'robot_node'  # this node name

class Robot(object):
    # Disable of robot
    def __init__(self):
        self.open_robot_connection()
        

    def turn_off(self):
        rbha.disable_robot()

    # Enable of robot
    def turn_on(self):
        rbha.do_enable()

    # request robot enable status. Returns true if enabled and false if disabled
    def request_status(self):
        return rbha.request_enable_status()

    # request alarm bit status of robot. Returns true if any alarm set
    def request_alarm(self):
        return rbha.request_alarm()

    # Get actual move speed and rotational speed of robot in SI units
    # Reports calculated speed x from motor encoders and robot rotation based on either encoders of gyro depending on gyrobased being true
    def get_movesteer(self, gyrobased):
        return rbha.get_movesteer(gyrobased)

    # Move command. Input in SI units. speed in m/s dir in radians per sec
    def drive(self, vx, vth):
        """Drive the robot.
@param cmd_vel: velocity setpoint = (vx, vy, vth)
      vx:  linear velocity along the x-axis (m/s)
      vy:  linear velocity along the y-axis (m/s)
      vth: angular velocity about the z-axis (rad/s), also called yaw
returns current state, including velocity, as measured by robot using its encoders
"""
        rbha.do_movesteer_int(vx, vth)

    # Stop robot communication
    def close_robot_connection(self):
        rbha.close_serial()

    # Start robot communication
    def open_robot_connection(self):
        try:
            rbha.open_serial()
        except:
            print ("error occurred in opening the port")
    def get_gyro(self):
        rbha.handle_report_gyro()
        return rbha.rb1.gyroZrad

    # Call this routine from the ROS spin loop to uodate the data from Rosbee to ROS
    def get_update_from_rosbee(self):
        if rbha.isportopen():  # request data from embedded controller at regular intervals
            rbha.send(rbha.cmd_get_adc)  # get adc values
            rbha.send(rbha.cmd_get_status)  # get status and errors
            rbha.send(rbha.cmd_get_counters)  # get process counters
            rbha.send(rbha.cmd_get_times)  # get process times
            rbha.send(rbha.cmd_get_position)  # get wheel encoder positions
            rbha.send(rbha.cmd_get_gyro)  # get gyro data
            rbha.sendnewsetpoints()  # send new setpoints to wheels if port open


class RobotNode(object):
    def __init__(self, name, robot):
        self.robot = robot
        rospy.init_node(name, anonymous=True)
        self.req_cmd_vel = None
        self.robot.turn_on()
        self._pos2d = Pose2D()
        self._init_params()
        self._init_pubsub()
        self.spin()


    def _init_params(self):

        # node general
        self.update_rate = rospy.get_param('~update_rate', 50.0)
        self.verbose = rospy.get_param('~verbose', True)

        # fake serial connection to a robot
        self.fake_connection = rospy.get_param('~fake_connection', False)

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
            rospy.loginfo("serial port: %s" % (self.port))
            rospy.loginfo("baudrate: %s" % (self.baudrate))

        if self.fake_respons:
            rospy.loginfo("fake robot respons")

        rospy.loginfo("update_rate: %s" % (self.update_rate))

        if self.publish_tf:
            rospy.loginfo("publish tf")
        else:
            rospy.loginfo("no tf published")

    def _init_pubsub(self):
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
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
        #self.robot.drive(msg.linear.x, msg.angular.z)

    def compute_imu(self):
        orientation_covariance = [
        0.0025 , 0 , 0,
        0, 0.0025, 0,
        0, 0, 0.0025
        ]
        # Angular velocity covariance estimation:
        # Observed gyro noise: 4 counts => 0.28 degrees/sec
        # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
        # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
        angular_velocity_covariance = [
        0.02, 0 , 0,
        0 , 0.02, 0,
        0 , 0 , 0.02
        ]
        # linear acceleration covariance estimation:
        # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
        # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
        # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
        linear_acceleration_covariance = [
        0.04 , 0 , 0,
        0 , 0.04, 0,
        0 , 0 , 0.04
        ]
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
        d = (vx * dt) * self.odom_linear_scale_correction  # correction factor from calibration
        angle = (vth * dt) * self.odom_angular_scale_correction  # correction factor from calibration

        x = cos(angle) * d
        y = -sin(angle) * d

        last_angle = self._pos2d.theta
        self._pos2d.x += cos(last_angle) * x - sin(last_angle) * y
        self._pos2d.y += sin(last_angle) * x + cos(last_angle) * y
        self._pos2d.theta += angle

        # Rosbee quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        odom_quat = (0., 0., sin(self._pos2d.theta / 2.), cos(self._pos2d.theta / 2.))

        # construct the transform
        transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

        # update the odometry state
        odom.header.stamp = current_time
        odom.pose.pose = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(d / dt, 0, 0), Vector3(0, 0, angle / dt))
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
        imu = Imu()
        transform = None
        last_state_time = rospy.get_rostime()
        last_vel_state = (0.0, 0.0, 0.0)
        dictToList = []
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
            self.robot.drive(req_cmd_vel[0], req_cmd_vel[2])
            last_state = robot.get_movesteer(robot.get_gyro())
            for key, value in last_state.iteritems():
                temp = [value]
                dictToList.append(temp[0])
            last_vel_state = (dictToList[1], 0, dictToList[0])
            last_state_time = current_time

            # COMPUTE ODOMETRY
            # use average velocity, i.e. assume constant acceleration
            avg_vel_state = tuple((float(x) + float(y))/2 for x, y in zip(old_vel_state, last_vel_state))
            transform = self.compute_odom(avg_vel_state, old_state_time, last_state_time, odom)
            # PUBLISH ODOMETRY
            self.odom_pub.publish(odom)
            self.imu_pub.publish(imu)
            if self.publish_tf:
              self.publish_odometry_transform(odom)

            if self.verbose:
                rospy.loginfo("velocity setpoint: %s", str(req_cmd_vel))
                rospy.loginfo("velocity measured: %s", str(last_vel_state))
                rospy.loginfo("pose: %s", str(transform))


            r.sleep()

if __name__ == '__main__':
    robot = Robot()
    rospy.loginfo("%s started...", NAME)
    try:
        RobotNode(NAME, robot)
    except rospy.ROSInterruptException, err:
        rospy.loginfo(str(err))
        rospy.loginfo("%s stopped working...", NAME)
