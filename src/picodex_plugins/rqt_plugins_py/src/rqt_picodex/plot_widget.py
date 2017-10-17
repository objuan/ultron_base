#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import rospkg
import roslib
import sys, math
from enum import Enum

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot,QPointF,QRectF
from python_qt_binding.QtGui import QIcon,QPen,QColor,QBrush,QPolygonF,QPainter,QVector3D
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget
import numpy as np
from ellipsoid_fit import ellipsoid_fit as ellipsoid_fit, data_regularize

#from PyQt5 import QtCore, QtGui, QtWidgets

import rospy

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

from . rosplot import ROSData, RosPlotException


####################################
"""
    def createPoly(self, n, r, s):
        polygon = QPolygonF()
        w = 360 / n  # angle per step
        for i in range(n):  # add the points of polygon
            t = w * i + s
            x = r * math.cos(math.radians(t))
            y = r * math.sin(math.radians(t))
            polygon.append(QPointF(self.width() / 2 + x, self.height() / 2 + y))

        return polygon
"""

###########################################################
class Plane(Enum):
    XY = 0
    XZ = 1
    YZ = 2
    XYZ = 3

class ProjectPlane(QWidget):

    def __init__(self, plotWidget,plane, parent=None):
        QWidget.__init__(self, parent)
        self.setObjectName('ProjectPlane')

        self.pen = QPen(QColor(0, 0, 0))  # set lineColor
        self.pen.setWidth(1)  # set lineWidth
        self.brush = QBrush(QColor(255, 255, 255, 255))  # set fillColor

        self.plotWidget=plotWidget
        self.smartVector = plotWidget.smartVector
        self.plane=plane
        #self.plot=plotWidget.plot
        self.p_center = QPointF()
        self.p_min = QPointF()
        self.p_max = QPointF()


        self.r_scale  = QPointF(1,1)
        self.r_offset = QPointF(0,0)


        self.colors = [QColor(255,0,0),QColor(0,255,0),QColor(0,0,255)]

    def init_draw(self):
        """
        if (self.plane == Plane.XZ):
            self.p_min = QPointF(self.plot.value_min.x(), self.plot.value_min.z())
            self.p_max = QPointF(self.plot.value_max.x(), self.plot.value_max.z())
            self.p_center = QPointF(self.plot.value_center.x(), self.plot.value_center.z())

        if (self.plane == Plane.YZ):
            self.p_min = QPointF(self.plot.value_min.y(), self.plot.value_min.z())
            self.p_max = QPointF(self.plot.value_max.y(), self.plot.value_max.z())
            self.p_center = QPointF(self.plot.value_center.y(), self.plot.value_center.z())

        if (self.plane == Plane.XY):
            self.p_min = QPointF(self.plot.value_min.x(), self.plot.value_min.y())
            self.p_max = QPointF(self.plot.value_max.x(), self.plot.value_max.y())
            self.p_center =QPointF(self.plot.value_center.x(), self.plot.value_center.y())
        """
        scale = self.plotWidget.spinBoxRange.value()
        dx = scale*2
        dy = scale*2
        w = self.back_rect.width() -10
        h = self.back_rect.height() -10 # bordo

        self.r_offset =   self.center# + self.p_center
        self.r_scale = QPointF( w / dx , h / dy )



    def paintEvent(self, event):
        #back
        self.back_rect = QRectF(self.rect())
        self.center = self.back_rect.center()

        painter = QPainter(self)
        painter.setPen(self.pen)
        painter.setBrush(self.brush)
        painter.drawRect(self.back_rect)

        self.init_draw()
        # axis
        painter.setBrush(self.brush)

        self.drawAxis(painter)

        # values
        painter.setPen( self.colors[self.plane.value] )
        #painter.setPen(QColor(0, 0, 0))

        for p in self.smartVector.values:
            self.drawPoint(painter,p)

        # calibrated

        painter.setPen(QColor(20,100,100))

        for p in self.smartVector.values:
            self.drawPointCalibrated(painter, p)

        #
        #ellipse
        c = QPointF()
        r = QPointF()


        try:
            if (self.plane == Plane.XZ):
                c = QPointF(self.smartVector.center[0],self.smartVector.center[2])
                r = QPointF(self.smartVector.radii[0], self.smartVector.radii[2])
            if (self.plane == Plane.YZ):
                c = QPointF(self.smartVector.center[1], self.smartVector.center[2])
                r = QPointF(self.smartVector.radii[1], self.smartVector.radii[2])
            if (self.plane == Plane.XY):
                c = QPointF(self.smartVector.center[0], self.smartVector.center[1])
                r = QPointF(self.smartVector.radii[0], self.smartVector.radii[1])
            c.setX(c.x() * self.r_scale.x() + self.r_offset.x())
            c.setY(c.y() * self.r_scale.y() + self.r_offset.y())

            painter.setPen(QColor(255,128,70))
            painter.setBrush(QBrush(QColor(255, 255, 255, 0)))
            #painter.fillRect(rect, QBrush(QColor(128, 128, 255, 128)));
            painter.drawEllipse(QPointF(c.x(),self.back_rect.height()-  c.y()),r.x() / 3, r.y() / 3)

            # info
            painter.setBrush(self.brush)
            info ="Total: "+ str(len(self.smartVector.values))+" Range: " + vetToString(self.smartVector.value_min)+" - "+vetToString(self.smartVector.value_max)
            painter.setPen(QColor(0, 0, 0))
            painter.drawText(10,15,info)

        except Exception as e:
            pass#print  str(e)


    def redraw(self):
        self.repaint()

    def getPoint(self,point):
        pos2D = QPointF()
        if (self.plane == Plane.XZ):
            pos2D = QPointF(point.x(), point.z())
        if (self.plane == Plane.YZ):
            pos2D = QPointF(point.y(), point.z())
        if (self.plane == Plane.XY):
            pos2D = QPointF(point.x(), point.y())
        return pos2D

    def drawPoint(self,painter,point):
        pos2D = self.getPoint(point)

        pos2D.setX(pos2D.x() * self.r_scale.x() + self.r_offset.x())
        pos2D.setY(pos2D.y() * self.r_scale.y() + self.r_offset.y())

        #print pos2D
        #/painter.drawPoint(QPointF(10,10))

        painter.drawRect(QRectF(pos2D.x(),self.back_rect.height()-  pos2D.y(),2,2))

    def drawPointCalibrated(self, painter,point):
        pos2D = self.getPoint(point)
        calib_offset = self.getPoint(self.smartVector.calib_offset)
        calib_scale = self.getPoint(self.smartVector.calib_scale)

        pos2D.setX(pos2D.x() * calib_scale.x() - calib_offset.x())
        pos2D.setY(pos2D.y() * calib_scale.y() - calib_offset.y())

        pos2D.setX(pos2D.x() * self.r_scale.x() + self.r_offset.x())
        pos2D.setY(pos2D.y() * self.r_scale.y() + self.r_offset.y())

        # print pos2D
        # /painter.drawPoint(QPointF(10,10))

        painter.drawRect(QRectF(pos2D.x(), self.back_rect.height() - pos2D.y(), 2, 2))

    def drawAxis(self,painter):
        painter.setPen(QColor(0, 0, 0))

        painter.drawLine(QPointF(self.back_rect.left(), self.center.y()),
                         QPointF(self.back_rect.right(), self.center.y()))
        painter.drawLine(QPointF(self.center.x(), self.back_rect.top()),
                         QPointF(self.center.x(), self.back_rect.bottom()))


###########################################################

def vetToString(vec):
    return "["+str('%.1f' % vec.x())+","+str('%.1f' % vec.y())+","+str('%.1f' % vec.z())+"]"

class SmartVector(QWidget):
    _max_points = 1000

    def __init__(self,plotWidget, parent=None):
        QWidget.__init__(self, parent)
        self.setObjectName('SmartVector')

        self.plotWidget=plotWidget

        self.pen = QPen(QColor(0, 0, 0))  # set lineColor
        self.pen.setWidth(1)  # set lineWidth
        self.brush = QBrush(QColor(255, 255, 255, 255))  # set fillColor

        # init
        self.center = QPointF()

        self.value_min = QVector3D();
        self.value_max = QVector3D();
        self.value_center = QVector3D();
        #self.values = [QVector3D(-10,-10,10),QVector3D(100,100,5),QVector3D(0,5,40),QVector3D(-30,50,-40)]
        #self.values = [QVector3D(10,10,0),QVector3D(20,0,20),QVector3D(0,30,30)]
        self.values = []

        self.calib_offset = QVector3D(50.61970901489258, 141.49842834472656, -106.39176177978516)
        self.calib_scale = QVector3D(1,1,1)

        #planes
        #self.planes = [ProjectPlane(self,Plane.XZ),ProjectPlane(self,Plane.YZ),ProjectPlane(self,Plane.XY)]


    #def init_draw(self):

    #    for p in self.planes:
    #       p.init_draw()


    def calibrate(self):
        print "calibrate"
        if (len(self.values)==0):
            return

        ## ,om - max
        value_max = QVector3D(-9999, -9999, -9999)
        value_min = QVector3D(9999, 9999, 9999)

        for val in self.values:
            value_min.x = min(value_min.x, val.x)
            value_min.y = min(value_min.y, val.y)
            value_min.z = min(value_min.z, val.z)

            value_max.x = max(value_max.x, val.x)
            value_max.y = max(value_max.y, val.y)
            value_max.z = max(value_max.z, val.z)

        # Get hard iron correction

        value_center = value_min + (value_max - value_min) / 2

        # mono update
        self.value_min = value_min
        self.value_max = value_max
        self.value_center = value_center

        ###########
        print 'process'

        _data = []
        for v in self.values:
            vv =  [v.x(),v.y(),v.z()]
            _data.append(vv)

        data = np.array(_data, float)

        #print data

        try:
            data2 = data_regularize(data)

            self.center, self.radii, self.evecs, self.v = ellipsoid_fit(data)

            a, b, c = self.radii
            self.r = (a * b * c) ** (1. / 3.)
            D = np.array([[self.r / a, 0., 0.], [0., self.r / b, 0.], [0., 0., self.r / c]])
            TR = self.evecs.dot(D).dot(self.evecs.T)

            #print
            #print 'center: ', self.center ,' radii' , self.radii
            #print 'transformation:'
            #print TR

            #self.calib_offset = QVector3D(self.center[0],self.center[1],self.center[2])

            # Get hard iron correction
            bias = (value_max + value_min) / 2
            scale = (value_max - value_min) / 2

            avg_rad = scale.x() +scale.y()+scale.z()
            avg_rad /= 3.0

            self.calib_offset =  bias#QVector3D(bias[0],bias[1],bias[2])
            self.calib_scale =  avg_rad / scale #QVector3D(avg_rad / scale[0],bias[1],bias[2])

            #calib_scale = QVector3D(self.center[0], self.center[1], self.center[2])

            print "calib: "+str(self.calib_offset)
            print "scale: " + str(self.calib_scale)

        except Exception as e:
            print e

        ################

    ## from work thread
    def update_values(self,times,values):

        #print values
        for v in values:
            self.values.append(QVector3D(v.x,v.y,v.z))

        while len(self.values) > self._max_points:
            del  self.values[0]

    def clear(self):
        self.calibrate()
        self.values = []


    def load(self, plugin_settings, instance_settings):

        #print "load"
        try:
            values = instance_settings.value('values', float)

            #print "values "+ str(len(values))

            for i in range(0, len(values), 3):
               v = QVector3D(float(values[i]), float(values[i + 1]), float(values[i + 2]))
               #print v
               self.values.append(v)

            #print self.values

            self.calibrate()

        except Exception as e:
           print e

    def save(self,  plugin_settings, instance_settings):
        s = []
        for v in self.values:
            s.append(v.x())
            s.append(v.y())
            s.append(v.z())

        #print s
        instance_settings.set_value("values", s)


###########################################################

class PlotWidget(QWidget):
    _redraw_interval = 40

    def __init__(self, initial_topics="", start_paused=False):
        super(PlotWidget, self).__init__()
        self.setObjectName('PlotWidget')

        self._initial_topics = initial_topics

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_picodex'), 'resource', 'plot.ui')
        loadUi(ui_file, self)

        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))

        self.smartVector = SmartVector(self,self)

        self.data_plot_0 = self.add_plot_widget(0)
        self.data_plot_1 = self.add_plot_widget(1)
        self.data_plot_2 = self.add_plot_widget(2)
        #self.data_plot_3 = self.add_plot_widget(3)

        if start_paused:
            self.pause_button.setChecked(True)

        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics()
        self.topic_edit.setCompleter(self._topic_completer)

        self._start_time = rospy.get_time()
        self._rosdata = {}
        self._remove_topic_menu = QMenu()

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        #self._update_plot_timer.start(self._redraw_interval);

        self.spinBoxMaxPoints.setValue(1000)

    def init(self):
        for topic_name, rosdata in self._rosdata.items():
            self.topic_edit.setText(topic_name)

    def add_plot_widget(self,  idx):
        layout = self.data_plot_layout_1
        plot = None
        if (idx == 0):
            layout = self.data_plot_layout_1
            plot = ProjectPlane(self, Plane.XY)
        if (idx == 1):
            layout = self.data_plot_layout_2
            plot = ProjectPlane(self, Plane.XZ)
        if (idx == 2):
            layout = self.data_plot_layout_3
            plot = ProjectPlane(self, Plane.YZ)
        if (idx == 3):
            layout = self.data_plot_layout_4
            plot = ProjectPlane(self, Plane.XYZ)

        layout.addWidget(plot)

        return plot

    @Slot(int)
    def on_spinBoxMaxPoints_valueChanged(self, value):
        #print "->"+str(value)
        self.smartVector._max_points = value

    @Slot(int)
    def on_spinBoxRange_valueChanged(self, value):
        # print "->"+str(value)
        self._value_range = value


    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completer.update_topics()

        plottable, message = True,""#is_plottable(topic_name)

    @Slot()
    def on_topic_edit_returnPressed(self):
        ##if self.subscribe_topic_button.isEnabled():
        self.add_topic(str(self.topic_edit.text()))


    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot()
    def on_clear_button_clicked(self):
        self.clear_plot()


    ## work thread

    def update_plot(self):

        if self.data_plot_0 is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_t, data_v = rosdata.next()
                    if data_t or data_v:
                        #print data_v
                        self.smartVector.update_values( data_t, data_v)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot_0.redraw()
                self.data_plot_1.redraw()
                self.data_plot_2.redraw()
                #self.data_plot_3.redraw()

    def _subscribed_topics_changed(self):
        self._update_remove_topic_menu()
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._rosdata)

        self.data_plot_0.redraw()
        self.data_plot_1.redraw()
        self.data_plot_2.redraw()
        #self.data_plot_3.redraw()

        if self.data_plot_0 is not None:
            self.data_plot_0.redraw()
            self.data_plot_1.redraw()
            self.data_plot_2.redraw()
            # self.data_plot_3.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        if len(self._rosdata) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)


    def add_topic(self, topic_name):
        print "add topic : " + topic_name
        if self.data_plot_0 is not None:
            topics_changed = False
            #for topic_name in get_plot_fields(topic_name)[0]:
            print topic_name
            if topic_name in self._rosdata:
                qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
                return
            self._rosdata[topic_name] = ROSData(topic_name, self._start_time)
            if self._rosdata[topic_name].error is not None:
                qWarning(str(self._rosdata[topic_name].error))
                del self._rosdata[topic_name]
            else:
                topics_changed = True

            if topics_changed:
                self._subscribed_topics_changed()

    def remove_topic(self, topic_name):
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]

        self._subscribed_topics_changed()

    def clear_plot(self):
        #for topic_name, _ in self._rosdata.items():
        #    self.data_plot_src.clear_values(topic_name)

        if self.data_plot_0 is not None:
            self.data_plot_0.redraw()
            self.data_plot_1.redraw()
            self.data_plot_2.redraw()
            #self.data_plot_3.redraw()


    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self. smartVector.calibrate()
            self._update_plot_timer.stop()
