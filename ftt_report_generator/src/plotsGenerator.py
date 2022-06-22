#!/usr/bin/env python3
"""plotsGenerator.py: class to compute statistics and generate plots for a complete test event."""

__author__ = "Johannes Pellenz, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import matplotlib
matplotlib.use('Agg')
from matplotlib.path import Path
from matplotlib.pyplot import imread
import matplotlib.patches as patches
import numpy as np
import os.path
import pylab as plt
from matplotlib.font_manager import FontProperties
from matplotlib.offsetbox import AnnotationBbox, OffsetImage
from matplotlib.ticker import MaxNLocator


class PlotsGenerator:

    #---------------------------------------------------------------------------
    # Class constructor.
    #---------------------------------------------------------------------------
    def __init__(self, db_adapter, test_event_id, shifts, local, built_dir, image_dir, logos_dir, image_ext):
        
        # Initialize variables.
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.db_adapter = db_adapter
        self.test_event_id = test_event_id
        self.shifts = shifts
        self.local = local
        self.built_dir = built_dir
        self.image_dir = image_dir
        self.logos_dir = logos_dir
        self.image_ext = image_ext

        self.ito_reason_color_dict = self.db_adapter.read_color_dict("ito_reason")
        self.segment_type_color_dict = self.db_adapter.read_color_dict("segment_type")

        # Compute distances for the required statistics
        self.auto_dists = [None]*len(self.shifts)
        self.total_auto_dist = [None]*len(self.shifts)
        self.mean_auto_dist = [None]*len(self.shifts)
        self.std_auto_dist = [None]*len(self.shifts)
        self.max_auto_dist = [None]*len(self.shifts)
        self.total_manual_dist = [None]*len(self.shifts)
        # Iterate through the shifts
        for i, shift in enumerate(self.shifts):
            # Variables to hold distances for each segment.
            dist_manual = []
            dist_auto = []
            # Extract main info from master segments of selected shift.
            segment_values = self.db_adapter.get_master_segments(shift['id'])
            # Iterate through the shifts
            for row in segment_values:
                # Get the segment distance
                distance = self.db_adapter.get_segment_distance(row['segment_id'], self.local)
                if row['segment_type_short_description'] == 'ITO':
                    dist_manual.append(distance)
                elif row['segment_type_short_description'] == 'AUTO':
                    dist_auto.append(distance)
            # Compute auto distance statistics
            self.auto_dists[i] = dist_auto
            self.total_auto_dist[i] = sum(dist_auto)
            self.mean_auto_dist[i] = 0.0 if len(dist_auto) == 0 else self.total_auto_dist[i]/len(dist_auto)
            self.std_auto_dist[i] = 0.0
            for d in dist_auto:
                self.std_auto_dist[i] += (d-self.mean_auto_dist[i])**2
            self.std_auto_dist[i] = self.std_auto_dist[i]**0.5
            self.max_auto_dist[i] = 0.0 if len(dist_auto) == 0 else max(dist_auto)
            # Compute manual distance statistics
            self.total_manual_dist[i] = sum(dist_manual)

    #---------------------------------------------------------------------------
    # Convert (R,G,B) color string to (r,g,b) color tuple
    #---------------------------------------------------------------------------
    def plotColor(self, colorString):
        (R,G,B) = colorString.split(',')
        return (float(R)/255.0, float(G)/255.0, float(B)/255.0)

    #---------------------------------------------------------------------------
    # Print weather icon(s) for a shift
    #---------------------------------------------------------------------------
    def print_weather_icon(self, shift_id, x_pos, y_pos, num_shifts):
        # Look up the weather info for this shift in the leg data
        weather_row = 0
        last_weather_icon_filename = None
        weather_icons = self.db_adapter.get_shift_weather_icons(shift_id)
        for row in weather_icons:
            weather_icon_filename = row['weather_icon_filename']
            if (last_weather_icon_filename is None) or (last_weather_icon_filename != weather_icon_filename):
                weather_icon_path = self.logos_dir + '/%s.png' % (weather_icon_filename,)
                arr_weather = imread(weather_icon_path)
                icon_zoom = 0.15
                if (num_shifts > 6):
                    icon_zoom = 0.10;           
                imagebox = OffsetImage(arr_weather, zoom=icon_zoom)
                xy = [x_pos, y_pos] # coordinates to position this image
                ab = AnnotationBbox(imagebox, 
                    xy,
                    xybox=(weather_row * 15, weather_row * 15),
                    xycoords='data',
                    boxcoords='offset points',
                    frameon = True)
                self.ax.add_artist(ab)      
                weather_row = weather_row + 1
                last_weather_icon_filename = weather_icon_filename

    #---------------------------------------------------------------------------
    # Generate the x labels with performer, date, time
    #---------------------------------------------------------------------------
    def get_x_labels(self):
        # Set the x labels
        xLabels = []
        for shift in self.shifts:
            shift_id = shift['id']
            performer = shift['performer_institution']
            # start_datetime = shift['shift_start_datetime_raw']
            # xLabels.append ('%s\n%s\n%s' % (start_datetime.strftime('%m-%d'), start_datetime.strftime('%H:%M'), performer)) # short date
            # xLabels.append ('%s\n%s\n%s' % (start_datetime.strftime('%Y-%m-%d'), start_datetime.strftime('%H:%M'), performer)) # long date
            xLabels.append ('S.%s\n%s' % (shift_id, performer[0:8])) # shift and performer (only first 8 characters)
        return xLabels;    

    #---------------------------------------------------------------------------
    # Total auto distance plot.
    #---------------------------------------------------------------------------
    def total_auto_dist_plot(self):
        # Clear figure.
        self.fig.delaxes(self.ax)
        self.fig.clear()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        # Define ticks and labels for the x-axis.
        xTicks = list(range(1,len(self.shifts)+1))
        xLabels = self.get_x_labels ()
        # Iterate through the shifts' auto distances
        for i, auto_dist in enumerate(self.auto_dists):
            cum_dist = 0.0
            # Iterate through the segments' auto distances
            for distance in auto_dist:
                # plot the graph
                verts = [(xTicks[i]-0.25, cum_dist),            # left, bottom
                         (xTicks[i]-0.25, cum_dist+distance),   # left, top
                         (xTicks[i]+0.25, cum_dist+distance),   # right,top
                         (xTicks[i]+0.25, cum_dist),            # right, bottom
                         (0.0, 0.0)]                            # ignored
                cum_dist += distance
                codes = [Path.MOVETO,
                         Path.LINETO,
                         Path.LINETO,
                         Path.LINETO,
                         Path.CLOSEPOLY]
                path = Path(verts, codes)
                patch = patches.PathPatch(path, facecolor=self.plotColor(self.segment_type_color_dict['auto']), lw=2)
                self.ax.add_patch(patch)
        self.ax.relim()
        self.ax.autoscale_view()
        ylim = max(self.total_auto_dist)
        offset = (ylim)/18.0
        for i, shift in enumerate(self.shifts):
            # print weather icon on top of bar
            self.print_weather_icon (shift['id'], xTicks[i], ylim+3*offset, len(self.shifts))
            # print total distance driven
            plt.annotate(int(self.total_auto_dist[i]),
                xy=(xTicks[i], self.total_auto_dist[i]+offset),
                xycoords='data', 
                xytext=(xTicks[i], self.total_auto_dist[i]+offset),
                textcoords='data',
                size=10, 
                va="center",
                ha="center",
                rotation=0
            )
        plt.xticks(xTicks, xLabels, rotation=0, fontsize=10)
        plt.ylabel('Autonomous Distance [m]')
        plt.xlim(0.5,len(self.shifts)+0.5)
        plt.ylim(0,ylim+5*offset)

        # Save the plot.
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        plt.savefig('%s/TotalAutoDist.%s' % (self.image_dir, self.image_ext), transparent=False)

    #---------------------------------------------------------------------------
    # Mean auto distance plot.
    #---------------------------------------------------------------------------
    def mean_auto_dist_plot(self):
        # Clear figure.
        self.fig.delaxes(self.ax)
        self.fig.clear()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        # Define ticks and labels for the x-axis.
        xTicks = list(range(1,len(self.shifts)+1))
        xLabels = self.get_x_labels()
        # Compute the error bars.
        err = [[0]*len(self.shifts) for x in range(2)]
        for i in range(len(self.shifts)):
            err[0][i] = 0.0
            err[1][i] = self.max_auto_dist[i]-self.mean_auto_dist[i]
        # Plot the error bars.
        plt.errorbar(xTicks,
            self.mean_auto_dist,
            yerr=err,
            color='k',
            elinewidth=2,
            ecolor=self.plotColor(self.segment_type_color_dict['auto']),
            marker='o',
            markersize=10,
            markerfacecolor=self.plotColor(self.segment_type_color_dict['auto']),
            capsize=5,
            markeredgewidth=2,
            linestyle='None',
            label="MeanAutoDist"
        )
        # Compute values to annotate the plot.
        min_dist_disp = float('inf')
        for dist in self.mean_auto_dist:
            dist_disp = self.ax.transData.transform(np.array([(0.0,dist)]))
            if dist_disp[0][1] is np.ma.masked: continue
            if dist_disp[0][1] < min_dist_disp: min_dist_disp = dist_disp[0][1]

        max_dist_disp = 0.0
        for dist in self.max_auto_dist:
            dist_disp = self.ax.transData.transform(np.array([(0.0,dist)]))
            if dist_disp[0][1] is np.ma.masked: continue
            if dist_disp[0][1] > max_dist_disp: max_dist_disp = dist_disp[0][1]

        inv = self.ax.transData.inverted()
        offset_disp = (max_dist_disp-min_dist_disp)/10.0
        min_dist = inv.transform(np.array([(0.0,min_dist_disp-3*offset_disp)]))[0][1]
        max_dist = inv.transform(np.array([(0.0,max_dist_disp+3*offset_disp)]))[0][1]
        offset = max_dist / 20
    
        for i, shift in enumerate(self.shifts):
            # Print weather icon on top of the bar.
            self.print_weather_icon(shift['id'], xTicks[i], max_dist+3*offset, len(self.shifts))
            # Annotate average and max values on the plot
            dist_disp = self.ax.transData.transform(np.array([(0.0,self.mean_auto_dist[i])]))
            dist_disp[0][1] -= offset_disp
            y_label = inv.transform(dist_disp)[0][1]
            plt.annotate(int(self.mean_auto_dist[i]), xy=(xTicks[i], y_label), 
                         xycoords='data', xytext=(xTicks[i], y_label), 
                         textcoords='data', size=10, va="center", ha="center", rotation=0)
            dist_disp = self.ax.transData.transform(np.array([(0.0,self.max_auto_dist[i])]))
            dist_disp[0][1] += offset_disp
            y_label = inv.transform(dist_disp)[0][1]
            plt.annotate(int(self.max_auto_dist[i]), xy=(xTicks[i], y_label), 
                         xycoords='data', xytext=(xTicks[i], y_label), 
                         textcoords='data', size=10, va="center", ha="center", rotation=0)

        self.ax.relim()
        self.ax.autoscale_view()
        plt.ylabel('Mean Autonomous Distance [m]')
        plt.xticks(xTicks, xLabels, rotation=0, fontsize=10)
        plt.xlim(0,len(self.shifts)+1)
        plt.ylim(min_dist, max_dist + 5 * offset)

        # Save the plot.
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        plt.savefig('%s/MeanAutoDist.%s' % (self.image_dir, self.image_ext), transparent=False)

    #---------------------------------------------------------------------------
    # Override count plot.
    #---------------------------------------------------------------------------
    def override_count_distributed_plot(self):
        # Get all keys for the test event.
        ito_stats = self.db_adapter.get_test_event_ito_statistics(self.test_event_id)
        keys = list([x['ito_key'] for x in ito_stats])
        ito_reasons = list([x['ito_reason'] for x in ito_stats])
        nkeys = len (keys)
        nshifts = len(self.shifts)
        # Set up the xTicks and labels.
        xSpace = np.array(list(range(nshifts)))
        xLines = [x for x in (2+nkeys)*xSpace if x > 1]
        xTicks = [x for x in (2+nkeys)*xSpace+(2+nkeys)/2]
        xLabels = [None]*nshifts
        heights = [None]*nshifts
        # Clear figure
        self.fig.delaxes(self.ax)
        self.fig.clear()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        # Retrieve the counts.
        countsY = {}
        countsX = {}
        legend_text = []        
        p = []
        xLabels = self.get_x_labels ()
        for i, key in enumerate(keys):
            countsX [key] = [x for x in (2+nkeys)*xSpace+i+1]
            countsY [key] = [0]*len(self.shifts)
            # Count entries for this key
            for j, shift in enumerate(self.shifts):
                countsY [key][j] = self.db_adapter.get_shift_ito_count(shift['id'], key)
                if countsY [key][j] > heights[j]:
                    heights[j] = countsY [key][j]
            # Draw the bar plot for this key.
            p.append (plt.bar(countsX [key], countsY [key], 1.0, color=self.plotColor(self.ito_reason_color_dict[key])))
            legend_text.append (ito_reasons[i])
        ylim = max(heights)
        offset = (ylim)/18.0
        for i, shift in enumerate(self.shifts):
            # Print weather icon on top of bar
            self.print_weather_icon (shift['id'], xTicks[i], ylim+3*offset, len(self.shifts))
            # Print total distance driven
            plt.annotate(("%d m" % int(self.total_manual_dist[i])), xy=(xTicks[i], heights[i]+offset), 
                         xycoords='data', xytext=(xTicks[i], heights[i]+offset), textcoords='data',
                         size=10, va="center", ha="center", rotation=0)
            if xLines != []:
                plt.plot([xLines]*2, [0,ylim+10*offset], color='b', linestyle='--')
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        # Move the legend out of the plot area
        fontP = FontProperties()
        fontP.set_size('small')
        box = self.ax.get_position()
        self.ax.set_position([box.x0, box.y0 + box.height * 0.1, box.width, box.height * 0.9])
        self.ax.legend(p, legend_text, prop = fontP, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=5, fancybox=False, shadow=False)

        plt.ylabel('Count')
        plt.xticks(xTicks, xLabels, rotation=0, fontsize=10)
        plt.xlim(0, (2+nkeys)*len(self.shifts))
        # org: plt.ylim(0,ylim+10*offset)
        plt.ylim(0,ylim + 5 * offset)

        # Save the plot.
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        plt.savefig('%s/OverrideCounts.%s' % (self.image_dir, self.image_ext), transparent=False)

    def override_count_stacked_plot(self):
        # Clear figure
        self.fig.delaxes(self.ax)
        self.fig.clear()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        # Define ticks and labels for the x-axis.
        xSpace = np.array(list(range(len(self.shifts))))
        xTicks = (2*xSpace+1.5).tolist()
        xLabels = self.get_x_labels ()
        # Get all keys for the test event.
        ito_stats = self.db_adapter.get_test_event_ito_statistics(self.test_event_id)
        keys = list([x['ito_key'] for x in ito_stats])
        ito_reasons = list([x['ito_reason'] for x in ito_stats])
        nkeys = len (keys)
        # Retrieve the counts.
        total = np.array([0]*len(self.shifts))
        legend_text = []        
        p = []
        for i, key in enumerate(keys):
            counts = [0]*len(self.shifts)
            # Count entries for this key
            for j, shift in enumerate(self.shifts):
                counts[j] = self.db_adapter.get_shift_ito_count(shift['id'], key)
            # Plot the bar for the key on top of the previous.
            p.append (plt.bar(xTicks, counts, 1.0, bottom=total, color=self.plotColor(self.ito_reason_color_dict[key])))
            legend_text.append (ito_reasons[i])
            total += np.array(counts)
        ylim = max(total)
        offset = (ylim)/18.0
        for i, shift in enumerate(self.shifts):
            # Print weather icon on top of bar
            self.print_weather_icon (shift['id'], xTicks[i], ylim+3*offset, len(self.shifts))
            # Print total manual distance driven
            plt.annotate(("%d m" % int(self.total_manual_dist[i])), xy=(xTicks[i], total[i]+offset), 
                         xycoords='data', xytext=(xTicks[i], total[i]+offset), textcoords='data',
                         size=10, va="center", ha="center", rotation=0)
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        # Move the legend out of the plot area
        fontP = FontProperties()
        fontP.set_size('small')
        box = self.ax.get_position()
        self.ax.set_position([box.x0, box.y0 + box.height * 0.1, box.width, box.height * 0.9])
        self.ax.legend(p, legend_text, prop = fontP, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=5, fancybox=False, shadow=False)

        plt.ylabel('Number of Stops')
        plt.xticks(xTicks, xLabels, rotation=0, fontsize=10)
        plt.xlim(0, 2*len(self.shifts)+1)
        plt.ylim(0, ylim + 5 * offset)

        # Save the plot.
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        plt.savefig('%s/OverrideCounts.%s' % (self.image_dir, self.image_ext), transparent=False)
        


    #---------------------------------------------------------------------------
    # Manual operation time plot.
    #---------------------------------------------------------------------------
    def ito_time_plot(self):
        # Clear figure.
        self.fig.delaxes(self.ax)
        self.fig.clear()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        # Axis config.
        xSpace = np.array(list(range(len(self.shifts))))
        xTicks = (2*xSpace+1.5).tolist()
        MntTimeY = [0]*len(self.shifts)
        xLabels = self.get_x_labels()
        # Get manual operation time in minutes.
        for i, shift in enumerate(self.shifts):
            MntTimeY[i] = self.db_adapter.get_shift_ito_time(shift['id'])
            MntTimeY[i] = 0.0 if MntTimeY[i] is None else MntTimeY[i]/60.0

        # Draw the plot.
        plt.bar(xTicks, MntTimeY, 1.0, color=self.plotColor(self.segment_type_color_dict['ito']))
        self.ax.relim()
        self.ax.autoscale_view()
        # Annotate the time on top of the bar.
        ylim = max(MntTimeY)
        offset = (ylim)/20.0
        for i in range(len(self.shifts)):
            plt.annotate("{:.2f}".format(MntTimeY[i]), xy=(xTicks[i], MntTimeY[i]+offset), xycoords='data', 
                         xytext=(xTicks[i], MntTimeY[i]+offset), textcoords='data', size=10, 
                         va="center", ha="center", rotation=0)
        plt.ylabel('Manual Operation Time [mins]')
        plt.xticks(xTicks, xLabels, rotation=0, fontsize=10)
        plt.xlim(0,2*len(self.shifts)+1)
        plt.ylim(0,ylim+2*offset)

        # Save the plot.
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        plt.savefig('%s/ManualTimes.%s' % (self.image_dir, self.image_ext), transparent=False)

    #---------------------------------------------------------------------------
    # Generate all plots.
    #---------------------------------------------------------------------------
    def generate_all_plots(self):
        self.total_auto_dist_plot()
        self.mean_auto_dist_plot()
        # self.override_count_distributed_plot()
        self.override_count_stacked_plot()
        self.ito_time_plot()
