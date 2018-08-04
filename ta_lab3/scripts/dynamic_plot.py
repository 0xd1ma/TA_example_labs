#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np

# this is a convenient class for visualizing data while developing in Python
# once your code is done, you should instead use RViz to visualize important
# intermediate products as you see fit
class DynamicPlot():
    def initialize(self):
        plt.ion()
        #Set up plot
        self.fig = plt.figure(figsize=plt.figaspect(2.))
        
        self.ax0 = self.fig.add_subplot(2,1,1)

        self.laser_angular, = self.ax0.plot([],[], 'r.')
        self.laser_filtered, = self.ax0.plot([],[], 'b-')

        self.ax0.set_ylim(-1, 15)
        self.ax0.set_xlim(-np.pi, +np.pi)
        self.ax0.invert_xaxis()
        self.ax0.grid()

        self.ax1 = self.fig.add_subplot(2,1,2) 
        self.ax1.invert_xaxis()
        self.ax1.grid()
        self.laser_euclid, = self.ax1.plot([],[], '.')
        self.laser_regressed, = self.ax1.plot([],[], 'g')
        
        self.redraw()
        
    def redraw(self):
        #Need both of these in order to rescale
        self.ax0.relim()
        self.ax0.autoscale_view()
        
        self.ax1.relim()
        self.ax1.autoscale_view()
        
        #We need to draw *and* flush
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()