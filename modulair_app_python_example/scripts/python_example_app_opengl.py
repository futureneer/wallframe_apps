#!/usr/bin/env python
################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Johns Hopkins University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of the Johns Hopkins University nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
################################################################################

#
# Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
#

#!/usr/bin/env python
import roslib; roslib.load_manifest('modulair_app_python_example')
import rospy, sys
# Modulair Core Functionality
import modulair_core
from modulair_core import ModulairAppWidget
### PySide Imports ###
import PySide
from PySide import QtCore
from PySide import QtGui
from PySide.QtOpenGL import QGLWidget
from PySide.QtGui import QApplication
from modulair_core import ModulairAppWidget
import math

try:
  from OpenGL import GL
except ImportError:
  app = QtGui.QApplication(sys.argv)
  QtGui.QMessageBox.critical(None, "OpenGL hellogl",
                          "PyOpenGL must be installed to run this example.",
                          QtGui.QMessageBox.Ok | QtGui.QMessageBox.Default,
                          QtGui.QMessageBox.NoButton)
  sys.exit(1)

class PythonAppWidget(ModulairAppWidget):
  signal_update_rotation = QtCore.Signal()
  def __init__(self,name,app):
    super(PythonAppWidget,self).__init__(name,app)
    self.glWidget = GLWidget()
    mainLayout = QtGui.QGridLayout()
    mainLayout.addWidget(self.glWidget)
    self.setLayout(mainLayout)

    self.signal_update_rotation.connect(self.update_rotation)
    pass

  def update_rotation(self):
    self.glWidget.setXRotation(self.rot_x_)
    self.glWidget.setYRotation(self.rot_y_)

  def user_state_cb(self,msg):
    self.current_users_ = msg.users
    # print len(self.current_users_)

    if len(self.current_users_) > 0:
      self.current_users_ = msg.users
      self.rot_y_ = self.current_users_[0].translations_mm[8].x / 1.0
      self.rot_x_ = ((self.current_users_[0].translations_mm[8].y- 200) / 1.0 )
      self.signal_update_rotation.emit() 
    # Signal functions here that you want to run on an event callback
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

class GLWidget(QGLWidget):

  def __init__(self, parent=None):

      QGLWidget.__init__(self, parent)
      self.object = 0
      self.xRot = 0
      self.yRot = 0
      self.zRot = 0
      self.lastPos = QtCore.QPoint()
      self.trolltechGreen = QtGui.QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
      self.trolltechPurple = QtGui.QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)

  def xRotation(self):

      return self.xRot

  def yRotation(self):

      return self.yRot

  def zRotation(self):

      return self.zRot

  def minimumSizeHint(self):

      return QtCore.QSize(50, 50)

  def sizeHint(self):

      return QtCore.QSize(400, 400)

  def setXRotation(self, angle):

      angle = self.normalizeAngle(angle)

      if angle != self.xRot:

          self.xRot = angle

          self.emit(QtCore.SIGNAL("xRotationChanged(int)"), angle)

          self.updateGL()

  def setYRotation(self, angle):

      angle = self.normalizeAngle(angle)

      if angle != self.yRot:

          self.yRot = angle

          self.emit(QtCore.SIGNAL("yRotationChanged(int)"), angle)

          self.updateGL()

  def setZRotation(self, angle):

      angle = self.normalizeAngle(angle)

      if angle != self.zRot:

          self.zRot = angle

          self.emit(QtCore.SIGNAL("zRotationChanged(int)"), angle)

          self.updateGL()

  def initializeGL(self):

      self.qglClearColor(self.trolltechPurple.darker())

      self.object = self.makeObject()

      GL.glShadeModel(GL.GL_FLAT)

      GL.glEnable(GL.GL_DEPTH_TEST)

      GL.glEnable(GL.GL_CULL_FACE)

  def paintGL(self):

      GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

      GL.glLoadIdentity()

      GL.glTranslated(0.0, 0.0, -10.0)

      GL.glRotated(self.xRot / 16.0, 1.0, 0.0, 0.0)

      GL.glRotated(self.yRot / 16.0, 0.0, 1.0, 0.0)

      GL.glRotated(self.zRot / 16.0, 0.0, 0.0, 1.0)

      GL.glCallList(self.object)

  def resizeGL(self, width, height):

      side = min(width, height)

      GL.glViewport((width - side) / 2, (height - side) / 2, side, side)

      GL.glMatrixMode(GL.GL_PROJECTION)

      GL.glLoadIdentity()

      GL.glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0)

      GL.glMatrixMode(GL.GL_MODELVIEW)

  def mousePressEvent(self, event):

      self.lastPos = QtCore.QPoint(event.pos())

  def mouseMoveEvent(self, event):

      dx = event.x() - self.lastPos.x()

      dy = event.y() - self.lastPos.y()

      if event.buttons() & QtCore.Qt.LeftButton:

          self.setXRotation(self.xRot + 8 * dy)

          self.setYRotation(self.yRot + 8 * dx)

      elif event.buttons() & QtCore.Qt.RightButton:

          self.setXRotation(self.xRot + 8 * dy)

          self.setZRotation(self.zRot + 8 * dx)

      self.lastPos = QtCore.QPoint(event.pos())

  def makeObject(self):

      genList = GL.glGenLists(1)

      GL.glNewList(genList, GL.GL_COMPILE)

      GL.glBegin(GL.GL_QUADS)

      x1 = +0.06

      y1 = -0.14

      x2 = +0.14

      y2 = -0.06

      x3 = +0.08

      y3 = +0.00

      x4 = +0.30

      y4 = +0.22

      self.quad(x1, y1, x2, y2, y2, x2, y1, x1)

      self.quad(x3, y3, x4, y4, y4, x4, y3, x3)

      self.extrude(x1, y1, x2, y2)

      self.extrude(x2, y2, y2, x2)

      self.extrude(y2, x2, y1, x1)

      self.extrude(y1, x1, x1, y1)

      self.extrude(x3, y3, x4, y4)

      self.extrude(x4, y4, y4, x4)

      self.extrude(y4, x4, y3, x3)

      Pi = 3.14159265358979323846

      NumSectors = 200

      for i in range(NumSectors):

          angle1 = (i * 2 * Pi) / NumSectors

          x5 = 0.30 * math.sin(angle1)

          y5 = 0.30 * math.cos(angle1)

          x6 = 0.20 * math.sin(angle1)

          y6 = 0.20 * math.cos(angle1)

          angle2 = ((i + 1) * 2 * Pi) / NumSectors

          x7 = 0.20 * math.sin(angle2)

          y7 = 0.20 * math.cos(angle2)

          x8 = 0.30 * math.sin(angle2)

          y8 = 0.30 * math.cos(angle2)

          self.quad(x5, y5, x6, y6, x7, y7, x8, y8)

          self.extrude(x6, y6, x7, y7)

          self.extrude(x8, y8, x5, y5)

      GL.glEnd()

      GL.glEndList()

      return genList

  def quad(self, x1, y1, x2, y2, x3, y3, x4, y4):

      self.qglColor(self.trolltechGreen)

      GL.glVertex3d(x1, y1, -0.05)

      GL.glVertex3d(x2, y2, -0.05)

      GL.glVertex3d(x3, y3, -0.05)

      GL.glVertex3d(x4, y4, -0.05)

      GL.glVertex3d(x4, y4, +0.05)

      GL.glVertex3d(x3, y3, +0.05)

      GL.glVertex3d(x2, y2, +0.05)

      GL.glVertex3d(x1, y1, +0.05)

  def extrude(self, x1, y1, x2, y2):

      self.qglColor(self.trolltechGreen.darker(250 + int(100 * x1)))

      GL.glVertex3d(x1, y1, +0.05)

      GL.glVertex3d(x2, y2, +0.05)

      GL.glVertex3d(x2, y2, -0.05)

      GL.glVertex3d(x1, y1, -0.05)

  def normalizeAngle(self, angle):

      while angle < 0:

          angle += 360 * 16

      while angle > 360 * 16:

          angle -= 360 * 16

      return angle


# MAIN
if __name__ == '__main__':
  app = PythonExampleApp()