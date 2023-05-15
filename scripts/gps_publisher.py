#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Header
import serial
from gps.msg import gps
from sensor_msgs import NavSatFix
import utm
import os

# pip3 install pyserial 
# pip3 install utm
# rosrun gps gps_publisher.py

def talker():
    pub = rospy.Publisher('gps_data', gps, queue_size=10)
    rospy.init_node('gps_publisher', anonymous=True)


    serial_port = rospy.get_param('~port',os.getenv('GPS_DEVICE')) 
    serial_baud = rospy.get_param('~baudrate',os.getenv('GPS_BAUD_RATE')) 

    port = serial.Serial(serial_port, serial_baud, timeout=3.) 
    
    # port.open()
    rospy.logdebug(f"is_open: {port.is_open}")

    # enable gps tacking
    port.write("AT+CGPS=1\r".encode())

    # port.write("AT+CGPSINFO")

    gps_msg = NavSatFix()

    r = rospy.Rate(10) # 10hz
    GPS_POSITION_PREFIX="+CGPSINFO: "
    while not rospy.is_shutdown():
        port.write("AT+CGPSINFO\r".encode())
        # ['+CGPSINFO: 4926.366667', 'N', '03204.246624', 'E', '120523', '121009.0', '140.4', '0.0', '\r\n']
        # I am not able to find the format. Analyzing data I can state that latitude and longitude are formatted like in GPGLL
        # https://docs.novatel.com/OEM7/Content/Logs/GPGLL.htm
        line = port.readline().decode()
        rospy.logdebug(line) 
 
        if line.startswith(GPS_POSITION_PREFIX): 
            line = line.replace(GPS_POSITION_PREFIX, '');
            rospy.loginfo(line)
                
            gps_msg.raw_data = line
            data = line.split(',')
            print(data)
            # Data Format:
            # Latitude (DDmm.mm),
            # Latitude direction (N = North, S = South),
            # Longitude (DDDmm.mm)
            # Longitude direction (E = East, W = West)
            # UTC time status of position (hours/minutes/seconds/decimal seconds)
            # Atitude in meters? not sure....


            lat = data[0]
            lat_dir = data[1]
            lon = data[2]
            lon_dir = data[3]
            # utcTime = data[4]
            # alt = data[6]

            # if lat != '' and lat_dir != '' and lon != '' and lon_dir != '' and utcTime != '' and alt != '':
            if lat != '' and lat_dir != '' and lon != '' and lon_dir != '':   
            
                # Latitude (DDmm.mm) (N = North, S = South)
                if lat_dir == 'N':
                    gps_msg.latitude = float(lat[:2]) + float(lat[2:])/60
                    print(gps_msg.lat)
                elif lat_dir == 'S':
                    gps_msg.latitude = -1 * (float(lat[:2]) + float(lat[2:])/60)

                # Longitude Longitude (DDDmm.mm) (E = East, W = West)
                if lon_dir == 'E':
                    gps_msg.longitude = float(lon[:3]) + float(lon[3:])/60
                elif lon_dir == 'W':
                    gps_msg.longitude = -1 * (float(lon[:3]) + float(lon[3:])/60)

                # Header
                header_msg = Header()
                # timestamp from UTC time status of position (hours/minutes/seconds/decimal seconds)
                # hh = float(utcTime[:2])
                # mm = float(utcTime[2:4])
                # ss = float(utcTime[4:])
                # secs = hh * 3600 + mm * 60 + ss
                # header_msg.stamp.secs = int(secs)
                # header_msg.stamp.nsecs = int(secs * 1000000)

                header_msg.stamp = rospy.get_rostime()
                header_msg.seq += 1
                gps_msg.header = header_msg
              

                rospy.loginfo(f"header timestamp: {header_msg.stamp.secs}")

                # Altitude
                # gps_msg.alt = float(alt)

                # (gps_msg.utm_easting, gps_msg.utm_northing, gps_msg.zone, gps_msg.letter) = utm.from_latlon(gps_msg.lat, gps_msg.lon)

                pub.publish(gps_msg)        
        str = "ros time %s"%rospy.get_time()
        rospy.loginfo(str)        
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass


