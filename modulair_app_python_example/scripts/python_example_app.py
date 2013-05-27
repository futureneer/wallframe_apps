#!/usr/bin/env python
import roslib; roslib.load_manifest('modulair_app_python_example')
import rospy

class PythonExampleApp():
  def __init__(self):

    rospy.init_node('modulair_app_python_example',anonymous=True)
    rospy.logwarn('PythonExampleApp: Running')
    rospy.spin()
    rospy.logwarn('PythonExampleApp: Finished')

# MAIN
if __name__ == '__main__':
  app = PythonExampleApp()