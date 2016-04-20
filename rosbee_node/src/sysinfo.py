import rospy
#from rosbee_node.msg import BatteryStatus, NetworkStatus, SysInfo, SystemStatus
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
import rosinterface
class SystemInfo(object):
    def __init__(self):
        self.robot = rosinterface
        self.robot.init_robot()
        self.robot.enable_robot()
        rospy.init_node("sysinfo")
        pub_diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        diag_msg = DiagnosticArray()
        self.update_rate = rospy.get_param('~update_rate', 50)
        r = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.robot.get_update_from_rosbee()
            diag_msg.header.stamp = rospy.Time.now()
            diag_msg.status.append(self.battery_status())
            diag_msg.status.append(self.info_status())
            diag_msg.status.append(self.network_status())
            diag_msg.status.append(self.speed_status())
            pub_diagnostics.publish(diag_msg)
            print (self.robot.request_enable_status())
            r.sleep()

    def battery_status(self):
        stat = DiagnosticStatus(name="Battery", level=DiagnosticStatus.OK, message="OK")
        stat.values = [
            KeyValue("Voltage (V)", str(self.robot.)),
            KeyValue("Current (A)", str(self.robot)),
            KeyValue("Charge (Ah)", str(self.robot)),
            KeyValue("Capacity (Ah)", str(self.robot))]

        return stat

    def network_status(self):
        stat = DiagnosticStatus(name="Network", level=DiagnosticStatus.OK, message="OK")

        return stat

    def speed_status(self):
        return stat

    def info_status(self):
        return stat


if __name__ == '__main__':
    try:
        obj = SystemInfo()
    except rospy.ROSInterruptException:
        pass