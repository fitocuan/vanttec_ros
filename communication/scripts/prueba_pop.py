import subprocess


cmd = '''
rostopic pub /waypoints std_msgs/Float32MultiArray "layout:
  dim:                                                     
  - label: ''
    size: 0
    stride: 0
  data_offset: 6.0                                  
data: [1.0,1.0,2.0,2.0,3.0,3.0]" 
'''
#subprocess.Popen("roscore", shell = True)
subprocess.Popen(cmd, shell = True)