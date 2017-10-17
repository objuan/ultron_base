import os
import rospkg
import rospy
import argparse

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug
from rqt_gui_py.plugin import Plugin
from rqt_py_common.ini_helper import pack, unpack

from .plot_widget import PlotWidget
from .data_plot import DataPlot

class MagCalibratorPlugin(Plugin):

    def __init__(self, context):
        super(MagCalibratorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MagCalibratorPlugin')
        self._context = context
        self._args = self._parse_args(context.argv())


        self._widget = PlotWidget(initial_topics=self._args.topics, start_paused=self._args.start_paused)

         # disable autoscaling of X, and set a sane default range

        # self._widget.switch_data_plot_widget(self._data_plot_src,0)
        # self._widget.switch_data_plot_widget(self._data_plot_ret,1)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_plot', add_help=False)
        args = parser.parse_args(argv)

        args.topics = []
        args.start_paused = True;
        args.start_empty = False
        return args

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_plot plugin')
        group.add_argument('-P', '--pause', action='store_true', dest='start_paused',
                           help='Start in paused state')
        group.add_argument('-e', '--empty', action='store_true', dest='start_empty',
                           help='Start without restoring previous topics')
        group.add_argument('topics', nargs='*', default=[], help='Topics to plot')

    def _update_title(self):
        self._widget.setWindowTitle("Mag Calibrate")

    def save_settings(self, plugin_settings, instance_settings):

        #self._data_plot_src.save_settings(plugin_settings, instance_settings)
        #self._data_plot_ret.save_settings(plugin_settings, instance_settings)

        #instance_settings.set_value('autoscroll', self._widget.autoscroll_checkbox.isChecked())
        print "save" + str(self._widget._rosdata.keys())
        instance_settings.set_value('topics', pack(self._widget._rosdata.keys()))
        instance_settings.set_value('maxPoints',self._widget.spinBoxMaxPoints.value())
        instance_settings.set_value('valueRange', self._widget.spinBoxRange.value())
        self._widget.smartVector.save(plugin_settings,instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        #autoscroll = instance_settings.value('autoscroll', True) in [True, 'true']
        #self._widget.autoscroll_checkbox.setChecked(autoscroll)
        #self._data_plot.autoscroll(autoscroll)

        self._update_title()

        self._widget.spinBoxMaxPoints.setValue(int(instance_settings.value('maxPoints', "10000")))
        self._widget.spinBoxRange.setValue(int(instance_settings.value('valueRange', "400")))
        self._widget.smartVector.load(plugin_settings, instance_settings)

        if len(self._widget._rosdata.keys()) == 0 and not self._args.start_empty:
            topics = unpack(instance_settings.value('topics', []))
            print "load: " + str(topics)
            if topics:
                for topic in topics:
                    self._widget.add_topic(topic)


        self._widget.init()
        #self._data_plot_src.restore_settings(plugin_settings, instance_settings)
        #self._data_plot_ret.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        #self._data_plot_src.doSettingsDialog()
        #self._data_plot_ret.doSettingsDialog()
        self._update_title()

    def shutdown_plugin(self):
        self._widget.clean_up_subscribers()
