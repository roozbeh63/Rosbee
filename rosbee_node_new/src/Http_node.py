import json
from flask import Flask, url_for, request, Response
import rospy
from geometry_msgs.msg import Twist
app = Flask(__name__)


@app.route('/')
def api_root():
    return 'Welcome'

@app.route('/speed')
def get_speed():
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
    return 'odom'

def talker():
    pub = rospy.Publisher('chatter', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 2
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y= 0
        msg.angular.z = 0
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    app.run()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

