"""
This demo demonstrates how to draw a dynamic mpl (matplotlib) 
plot in a wxPython application.

It allows "live" plotting as well as manual zooming to specific
regions.

Both X and Y axes allow "auto" or "manual" settings. For Y, auto
mode sets the scaling of the graph to see all the data points.
For X, auto mode makes the graph "follow" the data. Set it X min
to manual 0 to always see the whole data from the beginning.

Note: press Enter in the 'manual' text box to make a new value 
affect the plot.

Eli Bendersky (eliben@gmail.com)
License: this code is in the public domain
Last modified: 31.07.2008
"""
import os
import pprint
import random
import sys
import wx

# The recommended way to use wx with mpl is with the WXAgg
# backend. 
#
import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
	FigureCanvasWxAgg as FigCanvas, \
	NavigationToolbar2WxAgg as NavigationToolbar
import numpy as np
import pylab

import PIDLoop
import math


class DataGen(object):
	""" A silly class that generates pseudo-random data for
		display in the plot.
	"""
	def __init__(self, initx=0, inity=0):
		self.datax = self.initx = initx
		self.datay = self.inity = inity

		self.PIDx = PIDLoop.PID()
		self.dt = 0.5
		self.threshold = 0.1
		self.curPos = np.array([0,0])
		self.desPos = [100,100]
		self.speed = np.array([0,0])
		self.lasterror = np.array([2,2])
		
	def next(self):
		if((math.fabs(self.desPos[0] - self.curPos[0]) > self.threshold) and
			(math.fabs(self.desPos[1] - self.curPos[1]) > self.threshold)):
			self._recalc_data()
			return [self.datax, self.datay]
		else:
			return 0
	
	def _recalc_data(self):
		self.speed, self.lasterror = self.PIDx.getNewSpeed(self.curPos, self.desPos, self.lasterror)
		self.curPos = self.PIDx.getNewPos(self.curPos, self.speed, self.dt)
		# print self.lasterror, self.speed, self.curPos
		self.datax = self.curPos[0]
		self.datay = self.curPos[1]


class SetNewDestBox(wx.Panel):
	""" A static box with a couple of radio buttons and a text
		box. Allows to switch between an automatic mode and a 
		manual mode with an associated value.
	"""
	def __init__(self, parent, ID, label, initval):
		wx.Panel.__init__(self, parent, ID)
		self.parent = parent
		self.value = initval
		
		box = wx.StaticBox(self, -1, label)
		sizer = wx.StaticBoxSizer(box, wx.VERTICAL)
		
		self.X_lbl = wx.StaticText(self, -1,
									  label="X-Pos")
		self.X_text = wx.TextCtrl(self, -1, 
			size=(35,-1),
			value=str(initval[0]),
			style=wx.TE_PROCESS_ENTER)
		self.Y_lbl = wx.StaticText(self, -1,
									  label="Y-Pos")
		self.Y_text = wx.TextCtrl(self, -1, 
			size=(35,-1),
			value=str(initval[1]),
			style=wx.TE_PROCESS_ENTER)
		
		self.Bind(wx.EVT_TEXT_ENTER, self.on_text_enter, self.X_text)
		self.Bind(wx.EVT_TEXT_ENTER, self.on_text_enter, self.Y_text)

		self.set_button = wx.Button(self, -1, "Set")
		self.Bind(wx.EVT_BUTTON, self.on_set_button, self.set_button)
		
		X_Box = wx.BoxSizer(wx.HORIZONTAL)
		X_Box.Add(self.X_lbl, flag=wx.ALIGN_CENTER_VERTICAL)
		X_Box.Add(self.X_text, flag=wx.ALIGN_CENTER_VERTICAL)
		
		Y_Box = wx.BoxSizer(wx.HORIZONTAL)
		Y_Box.Add(self.Y_lbl, flag=wx.ALIGN_CENTER_VERTICAL)
		Y_Box.Add(self.Y_text, flag=wx.ALIGN_CENTER_VERTICAL)

		sizer.Add(X_Box, 0, wx.ALL, 10)
		sizer.Add(Y_Box, 0, wx.ALL, 10)
		sizer.Add(self.set_button, 0, wx.ALL, 10)
		
		self.SetSizer(sizer)
		sizer.Fit(self)
	
	def on_text_enter(self, event):
		self.value[0] = int(self.X_text.GetValue())
		self.value[1] = int(self.Y_text.GetValue())

	def on_set_button(self, event):
		self.value[0] = int(self.X_text.GetValue())
		self.value[1] = int(self.Y_text.GetValue())
		self.parent.GetParent().datagen.desPos = self.value
		print self.parent.GetParent().datagen.desPos
		
	def getValue(self):
		return self.value


class GraphFrame(wx.Frame):
	""" The main frame of the application
	"""
	title = 'Demo: dynamic matplotlib graph'
	
	def __init__(self):
		wx.Frame.__init__(self, None, -1, self.title)
		
		self.datagen = DataGen()
		# bla = self.datagen.next()
		self.datax = [self.datagen.datax]
		self.datay = [self.datagen.datay]
		self.paused = False
		
		self.create_menu()
		self.create_status_bar()
		self.create_main_panel()
		
		self.redraw_timer = wx.Timer(self)
		self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)        
		self.redraw_timer.Start(100)

	def create_menu(self):
		self.menubar = wx.MenuBar()
		
		menu_file = wx.Menu()
		m_expt = menu_file.Append(-1, "&Save plot\tCtrl-S", "Save plot to file")
		self.Bind(wx.EVT_MENU, self.on_save_plot, m_expt)
		menu_file.AppendSeparator()
		m_exit = menu_file.Append(-1, "E&xit\tCtrl-X", "Exit")
		self.Bind(wx.EVT_MENU, self.on_exit, m_exit)
				
		self.menubar.Append(menu_file, "&File")
		self.SetMenuBar(self.menubar)

	def create_main_panel(self):
		self.panel = wx.Panel(self)

		self.init_plot()
		self.canvas = FigCanvas(self.panel, -1, self.fig)

		self.des_control = SetNewDestBox(self.panel, -1, "Destination", [50, 50])
		
		self.pause_button = wx.Button(self.panel, -1, "Pause")
		self.Bind(wx.EVT_BUTTON, self.on_pause_button, self.pause_button)
		self.Bind(wx.EVT_UPDATE_UI, self.on_update_pause_button, self.pause_button)
		
		self.cb_grid = wx.CheckBox(self.panel, -1, 
			"Show Grid",
			style=wx.ALIGN_RIGHT)
		self.Bind(wx.EVT_CHECKBOX, self.on_cb_grid, self.cb_grid)
		self.cb_grid.SetValue(True)
		
		self.cb_xlab = wx.CheckBox(self.panel, -1, 
			"Show X labels",
			style=wx.ALIGN_RIGHT)
		self.Bind(wx.EVT_CHECKBOX, self.on_cb_xlab, self.cb_xlab)        
		self.cb_xlab.SetValue(True)
		
		self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
		self.hbox1.Add(self.pause_button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
		self.hbox1.AddSpacer(20)
		self.hbox1.Add(self.cb_grid, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
		self.hbox1.AddSpacer(10)
		self.hbox1.Add(self.cb_xlab, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
		
		self.hbox2 = wx.BoxSizer(wx.HORIZONTAL)
		self.hbox2.AddSpacer(24)
		self.hbox2.Add(self.des_control, border=5, flag=wx.ALL)
		
		self.vbox = wx.BoxSizer(wx.VERTICAL)
		self.vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)        
		self.vbox.Add(self.hbox1, 0, flag=wx.ALIGN_LEFT | wx.TOP)
		self.vbox.Add(self.hbox2, 0, flag=wx.ALIGN_LEFT | wx.TOP)
		
		self.panel.SetSizer(self.vbox)
		self.vbox.Fit(self)
	
	def create_status_bar(self):
		self.statusbar = self.CreateStatusBar()

	def init_plot(self):
		self.dpi = 100
		self.fig = Figure((3.0, 3.0), dpi=self.dpi)

		self.axes = self.fig.add_subplot(111)
		self.axes.set_axis_bgcolor('black')
		self.axes.set_title('Very important random data', size=12)
		
		pylab.setp(self.axes.get_xticklabels(), fontsize=8)
		pylab.setp(self.axes.get_yticklabels(), fontsize=8)

		# plot the data as a line series, and save the reference 
		# to the plotted line series
		#
		self.plot_data = self.axes.plot(
			self.datax, self.datay,
			linewidth=1,
			color=(1, 1, 0),
			)[0]

	def draw_plot(self):
		""" Redraws the plot
		"""            
		xmin = -100
		xmax = 100

		ymin = -100
		ymax = 100

		self.axes.set_xbound(lower=xmin, upper=xmax)
		self.axes.set_ybound(lower=ymin, upper=ymax)
		
		# anecdote: axes.grid assumes b=True if any other flag is
		# given even if b is set to False.
		# so just passing the flag into the first statement won't
		# work.
		#
		if self.cb_grid.IsChecked():
			self.axes.grid(True, color='gray')
		else:
			self.axes.grid(False)

		# Using setp here is convenient, because get_xticklabels
		# returns a list over which one needs to explicitly 
		# iterate, and setp already handles this.
		#  
		pylab.setp(self.axes.get_xticklabels(), 
			visible=self.cb_xlab.IsChecked())
		
		self.plot_data.set_xdata(np.array(self.datax))
		self.plot_data.set_ydata(np.array(self.datay))
		
		self.canvas.draw()
	
	def on_pause_button(self, event):
		self.paused = not self.paused
	
	def on_update_pause_button(self, event):
		label = "Resume" if self.paused else "Pause"
		self.pause_button.SetLabel(label)
	
	def on_cb_grid(self, event):
		self.draw_plot()
	
	def on_cb_xlab(self, event):
		self.draw_plot()
	
	def on_save_plot(self, event):
		file_choices = "PNG (*.png)|*.png"
		
		dlg = wx.FileDialog(
			self, 
			message="Save plot as...",
			defaultDir=os.getcwd(),
			defaultFile="plot.png",
			wildcard=file_choices,
			style=wx.SAVE)
		
		if dlg.ShowModal() == wx.ID_OK:
			path = dlg.GetPath()
			self.canvas.print_figure(path, dpi=self.dpi)
			self.flash_status_message("Saved to %s" % path)
	
	def on_redraw_timer(self, event):
		# if paused do not add data, but still redraw the plot
		# (to respond to scale modifications, grid change, etc.)
		#
		if not self.paused:
			bla = self.datagen.next()
			if(bla):
				self.datax.append(bla[0])
				self.datay.append(bla[1])
		
		self.draw_plot()
	
	def on_exit(self, event):
		self.Destroy()
	
	def flash_status_message(self, msg, flash_len_ms=1500):
		self.statusbar.SetStatusText(msg)
		self.timeroff = wx.Timer(self)
		self.Bind(
			wx.EVT_TIMER, 
			self.on_flash_status_off, 
			self.timeroff)
		self.timeroff.Start(flash_len_ms, oneShot=True)
	
	def on_flash_status_off(self, event):
		self.statusbar.SetStatusText('')


if __name__ == '__main__':
	app = wx.PySimpleApp()
	app.frame = GraphFrame()
	app.frame.Show()
	app.MainLoop()
