#!/usr/bin/env python


from flask import Flask
from std_msgs.msg import String
from std_msgs.msg import Int32


import rospy
import subprocess
import os


def status_callback(msg):
    global status
    status = msg.data
    print(status)

rospy.init_node('status', anonymous=True)
pub = rospy.Publisher('/course', String, queue_size=10)
rospy.Subscriber("status", Int32, status_callback)
#subprocess.Popen("roslaunch zed_wrapper zed.launch", shell = True)

app = Flask("Py2_Server")





@app.route("/")
def index():
    return 'Index Page'

@app.route("/A")
def receive_A():
    global status
    pub.publish("A")
    subprocess.Popen("rosrun boat auto_nav.py", shell = True)
    while status == 0:
        print("auto_nav working")
    subprocess.Popen("rosnode kill /auto_nav", shell = True)
    print("auto_nav_finish")
    return "Launching Rostopic A"

@app.route("/FindThePath")
def receive_find_the_path():
    subprocess.Popen("rosnode kill /GPS_navigation", shell = True)
    return "Launching Rostopic FindThePath"

@app.route("/SpeedChallenge")
def receive_speed_challenge():
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
    global status
    status = 0
    subprocess.Popen("roslaunch boat general.launch", shell = True)
    #subprocess.Popen("roslaunch zed_wrapper zed.launch", shell = True)
    app.run(debug=True)






