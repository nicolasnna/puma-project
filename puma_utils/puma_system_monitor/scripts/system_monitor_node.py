#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

number_zone = rospy.get_param("thermal_zone", "0")

def get_cpu_usage():
    with open('/proc/stat', 'r') as f:
        line = f.readline()
        fields = [float(column) for column in line.strip().split()[1:]]

    idle_time = fields[3] + fields[4]  # idle + iowait
    total_time = sum(fields)

    return (total_time - idle_time) / total_time * 100.0  # Porcentaje de uso del CPU

def get_memory_usage():
    with open('/proc/meminfo', 'r') as f:
        meminfo = f.read()

    total_memory = int([x for x in meminfo.splitlines() if "MemTotal" in x][0].split()[1])
    available_memory = int([x for x in meminfo.splitlines() if "MemAvailable" in x][0].split()[1])
    
    used_memory = total_memory - available_memory
    memory_usage_percent = (used_memory / total_memory) * 100.0
    
    return memory_usage_percent

def get_cpu_temperature():
    with open('/sys/class/thermal/thermal_zone'+number_zone+'/temp', 'r') as f:
        temp = int(f.readline().strip()) / 1000.0  # Convertimos de miligrados a grados Celsius
    return temp

def system_monitor():
    rospy.init_node('system_monitor', anonymous=True)
    pub_cpu_temp = rospy.Publisher('/cpu_temperature', Float32, queue_size=10)
    pub_cpu_usage = rospy.Publisher('/cpu_usage', Float32, queue_size=10)
    pub_mem_usage = rospy.Publisher('/memory_usage', Float32, queue_size=10)

    rate = rospy.Rate(1)  # Publicar cada 1 segundo

    while not rospy.is_shutdown():
        cpu_temp = get_cpu_temperature()
        cpu_usage = get_cpu_usage()
        mem_usage = get_memory_usage()

        #rospy.loginfo(f"CPU Temp: {cpu_temp}°C, CPU Usage: {cpu_usage}%, Memory Usage: {mem_usage}%")

        pub_cpu_temp.publish(cpu_temp)
        pub_cpu_usage.publish(cpu_usage)
        pub_mem_usage.publish(mem_usage)

        rate.sleep()

if __name__ == '__main__':
    try:
        system_monitor()
    except rospy.ROSInterruptException:
        pass
