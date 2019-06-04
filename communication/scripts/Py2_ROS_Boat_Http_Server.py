#!/usr/bin/env python


from flask import Flask
from std_msgs.msg import String
import rospy
import subprocess
import os


rospy.init_node('station_rec', anonymous=True)
#pub = rospy.Publisher('/station', String, queue_size=10)
#rospy.Subscriber("status", Float64, self.dspeed_callback)

app = Flask("Py2_Server")


@app.route("/")
def index():
    return 'Index Page'

@app.route("/AutonomousNavigation")
def receive_autonomous_navigation():
    #pub.publish("AutonomousNavigation")
    subprocess.Popen("rosrun sensors gps_navigation.py", shell = True)
    return "Launching Rostopic AutonomousNavigation"

@app.route("/FindThePath")
def receive_find_the_path():
    subprocess.Popen("rosnode kill /GPS_navigation", shell = True)
    return "Launching Rostopic FindThePath"

@app.route("/SpeedChallenge")
def receive_speed_challenge():
    s = 'rostopic pub /waypoints std_msgs/Float32MultiArray "layout: dim: - label: '' size: 0 stride: 0 data_offset: 6 data: [1.0,1.0,2.0,2.0,3.0,3.0]"'
    s = s.split(' ')
    print(s)
    subprocess.Popen(s, shell = True)
    return "Launching Rostopic SpeedChallenge"

@app.route("/RaiseTheFlag")
def receive_raise_the_flag():
    return "Launching Rostopic RaiseTheFlag"

@app.route("/AutomatedDocking")
def receive_automated_docking():
    return "Launching Rostopic AutomatedDocking" 
    
@app.route("/GPSNavigation")
def receive_GPS_navigation():
    return "Launching Rostopic GPSNavigation"

@app.route("/Teleop")
def receive_teleop():
    return "Launching Rostopic teleop"



if __name__ == '__main__':
    app.run(debug=True)






