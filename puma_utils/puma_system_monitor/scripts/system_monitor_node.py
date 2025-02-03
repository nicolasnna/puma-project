#!/usr/bin/env python3
import rospy
import time
import os
from std_msgs.msg import Float32

class CPUMonitor:
    def __init__(self):
        self.prev_total = 0.0
        self.prev_idle = 0.0
        self.num_cpus = os.cpu_count() or 1  # Solo para referencia

    def get_cpu_usage(self):
        try:
            with open('/proc/stat', 'r') as f:
                lines = f.readlines()
            
            # Leemos solo la línea global (primera línea)
            cpu_line = next(line for line in lines if line.startswith('cpu '))
            fields = [float(col) for col in cpu_line.split()[1:]]
            
            idle = fields[3] + fields[4]  # idle + iowait
            total = sum(fields)
            
            if self.prev_total == 0:
                self.prev_total = total
                self.prev_idle = idle
                return 0.0  # Primera lectura
            
            # Diferencias
            total_delta = total - self.prev_total
            idle_delta = idle - self.prev_idle
            
            self.prev_total = total
            self.prev_idle = idle
            
            if total_delta == 0:
                return 0.0
            
            # Cálculo CORREGIDO (sin dividir por núcleos)
            usage_percent = (total_delta - idle_delta) / total_delta * 100.0
            return usage_percent
            
        except Exception as e:
            rospy.logerr(f"Error en CPU: {e}")
            return 0.0


def get_memory_usage():
    try:
        with open('/proc/meminfo', 'r') as f:
            meminfo = f.read()

        total_memory = int([x for x in meminfo.splitlines() if "MemTotal" in x][0].split()[1])
        available_memory = int([x for x in meminfo.splitlines() if "MemAvailable" in x][0].split()[1])
        
        return (total_memory - available_memory) / total_memory * 100.0
    except Exception as e:
        rospy.logerr(f"Error reading memory stats: {e}")
        return 0.0

def get_cpu_temperature(number_zone):
    try:
        with open(f'/sys/class/thermal/thermal_zone{number_zone}/temp', 'r') as f:
            return int(f.read().strip()) / 1000.0
    except Exception as e:
        rospy.logerr(f"Error reading temperature: {e}")
        return 0.0

def system_monitor():
    rospy.init_node('system_monitor', anonymous=True)
    pub_cpu_temp = rospy.Publisher('/cpu_temperature', Float32, queue_size=10)
    pub_cpu_usage = rospy.Publisher('/cpu_usage', Float32, queue_size=10)
    pub_mem_usage = rospy.Publisher('/memory_usage', Float32, queue_size=10)
    
    thermal_zone = rospy.get_param("~thermal_zone", "0")
    cpu_monitor = CPUMonitor()

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        cpu_temp = get_cpu_temperature(thermal_zone)
        cpu_usage = cpu_monitor.get_cpu_usage()
        mem_usage = get_memory_usage()

        pub_cpu_temp.publish(cpu_temp)
        pub_cpu_usage.publish(cpu_usage)
        pub_mem_usage.publish(mem_usage)

        rate.sleep()

if __name__ == '__main__':
    try:
        system_monitor()
    except rospy.ROSInterruptException:
        pass