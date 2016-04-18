import rbha



# Opens serial port, initializes hardware abstraction and executes run once commands
def init_robot():
    rbha.open_serial()
    if rbha.isportopen():
        rbha.send(rbha.cmd_version)
        rbha.receive()
        rbha.send(rbha.cmd_get_adc_labels)
        rbha.receive()
        rbha.send(rbha.cmd_conversionfactors)
        rbha.receive()
        rbha.send(rbha.cmd_reset_minmax)
        rbha.receive()
# Get actual move speed and rotational speed of robot in SI units
# Reports calculated speed x from motor encoders and robot rotation based on either encoders of gyro depending on gyrobased being true

def get_movesteer(gyro):
    if gyro:
        vel = rbha.get_movesteer(gyro)
    else:
        vel = rbha.get_movesteer(None)
    dictToList = []
    for key, value in vel.iteritems():
        temp = [value]
        dictToList.append(temp[0])
    if dictToList[0] <= 0:
        dictToList[0] = 0.0

    vel = (dictToList[1], 0, dictToList[0])
    return vel

# Move command. Input in SI units. speed in m/s dir in radians per sec
def do_movesteer(vx, vth):
    """Drive the robot.
@param cmd_vel: velocity setpoint = (vx, vy, vth)
  vx:  linear velocity along the x-axis (m/s)
  vy:  linear velocity along the y-axis (m/s)
  vth: angular velocity about the z-axis (rad/s), also called yaw
returns current state, including velocity, as measured by robot using its encoders
"""
    rbha.do_movesteer_int(vx, vth)


def get_gyro():
    return rbha.rb1.gyroZrad

# Call this routine from the ROS spin loop to uodate the data from Rosbee to ROS
def get_update_from_rosbee():
    if rbha.isportopen():  # request data from embedded controller
        rbha.send(rbha.cmd_get_adc)  # get adc values
        rbha.receive()
        rbha.send(rbha.cmd_get_status)  # get status and errors
        rbha.receive()
        rbha.send(rbha.cmd_get_counters)  # get process counters
        rbha.receive()
        rbha.send(rbha.cmd_get_times)  # get process times
        rbha.receive()
        rbha.send(rbha.cmd_get_position)  # get wheel encoder positions
        rbha.receive()
        rbha.send(rbha.cmd_get_gyro)  # get gyro data
        rbha.receive()
        rbha.sendnewsetpoints()  # send new setpoints to wheels if port open
        rbha.receive()




# Disable of robot
def disable_robot():
    rbha.disable_robot()

# Enable of robot
def enable_robot():
    rbha.enable_robot()

# request robot enable status. Returns true if enabled and false if disabled
def request_enable_status():
    return rbha.request_enable_status()

# request alarm bit status of robot. Returns true if any alarm set
def request_alarm():
    return rbha.request_alarm()




# Stop robot communication
def close_robot():
    rbha.close_serial()
