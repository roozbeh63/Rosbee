import rospy
import roslib
from std_msgs.msg import Float32
from std_msgs.msg import String
from rosbee_node.msg import BatteryStatus, NetworkStatus
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

class SystemInfo(object):
    def __init__(self):
        self.stat_bat_pc = []
        self.stat_network = []
        self.stat_system = []
        rospy.init_node("sysinfo")
        pub_diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray)
        diag_msg = DiagnosticArray()
        self.update_rate = rospy.get_param('~update_rate', 50)
        r = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            diag_msg.header.stamp = rospy.Time.now()
            diag_msg.status.append(self.stat_bat_pc)
            diag_msg.status.append(self.stat_network)
            diag_msg.status.append(self.stat_system)
            pub_diagnostics.publish(diag_msg)

            r.sleep()

    def battery_base_status(self):
        msg = BatteryStatus()

        msg.percent = self.base_bat_percent
        msg.plugged_in = self.base_bat_plugged_in
        msg.voltage = self.base_bat_voltage
        msg.watt = self.base_bat_watt
        msg.temp = self.base_bat_temp

        #self.stat_bat_base = DiagnosticStatus(name="Base Battery",level=DiagnosticStatus.OK,message="OK")
        #self.stat_bat_base.values = [ KeyValue("Voltage (V)",str(msg.voltage)),
        #                            KeyValue("Percentage",str(msg.percent)),
        #                            KeyValue("Charging",str(msg.plugged_in))]
        #
        #if msg.voltage < SystemInfo.BAT_VOLT_ERROR:
        #    self.stat_bat_base.level = DiagnosticStatus.ERROR
        #    self.stat_bat_base.message = "Battery almost empty"
        #elif msg.voltage < SystemInfo.BAT_VOLT_WARN:
        #    self.stat_bat_base.level = DiagnosticStatus.WARN
        #    self.stat_bat_base.message = "Battery almost empty"

        return msg






if __name__ == '__main__':
    try:
        obj = SystemInfo()
    except rospy.ROSInterruptException:
        pass