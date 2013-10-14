#!/usr/bin/env python
import roslib; roslib.load_manifest('wallframe_app_python_example')
import rospy, sys
# Wallframe Core Functionality
import wallframe_core
from wallframe_core import WallframeAppWidget
# PySide Imports
import PySide
from PySide.QtGui import QApplication

class PythonAppWidget(WallframeAppWidget):
  def __init__(self,name,app):
    super(PythonAppWidget,self).__init__(name,app)
    # Put stuff in here and make sure to make any widgets children of 'self'
    pass

class PythonExampleApp():
  def __init__(self):
    rospy.init_node('python_example_app',anonymous=True)
    app = QApplication(sys.argv)
    app_widget = PythonAppWidget("PythonExampleApp",app)
    # Running
    rospy.logwarn("PythonExampleApp: Started")  
    app.exec_()
    # Done
    rospy.logwarn('PythonExampleApp: Finished')

# MAIN
if __name__ == '__main__':
  app = PythonExampleApp()