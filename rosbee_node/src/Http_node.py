import json
from flask import Flask, url_for, request, Response
import rospy
from nav_msgs.msg import Odometry
import threading
from geometry_msgs.msg import Twist
app = Flask(__name__)

odom = Odometry()
vx = 0
vth = 0
@app.route('/')
def api_root():
    return 'Welcome'

@app.route('/speed')
def get_speed():
    global vx, vth
    vx = request.args['vx']
    vth = request.args['vth']
    data = {
        'vx'  : vx,
        'vth' : vth
    }
    js = json.dumps(data)
    resp = Response(js, status=200, mimetype='application/json')
    print (data)
    return resp

@app.route('/odom')
def get_odom():
    data = {
        'vx'  : odom.pose.pose.position.x,
        'vth' : odom.twist.twist.angular.z
    }
    js = json.dumps(data)
    resp = Response(js, status=200, mimetype='application/json')
    print (data)
    return resp
    return resp


def get_robot_odom(msg):
    odom = msg

def set_speed():
    global vx, vth
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    odom_sub = rospy.Subscriber('odom', Odometry, get_robot_odom)
    rospy.init_node('http_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = int(vx)
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y= 0
        msg.angular.z = int(vth)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    t = threading.Thread(target=app.run)  # create update thread to read data from wheel module
    t.setDaemon(1)
    t.start()
    try:
        set_speed()
    except rospy.ROSInterruptException:
        pass

