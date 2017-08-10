#!/usr/bin/python

"""
Created on Jul 10, 2017

@author: flg-ma
@attention: Jerk Metric
@contact: marcel.albus@ipa.fraunhofer.de (Marcel Albus)
@version: 1.7.0
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
import sys
import listener
import time


# AD stands for ArrayData
class AD(enumerate):
    TIME = 0  # time = '%time'
    HS = 1  # hs = 'field.header.seq'
    FHS = 2  # fhs = 'field.header.stamp'  # stamp for calculating differentiation
    VEL_X = 3  # velocity x-direction
    VEL_Y = 4  # velocity y-direction
    OME_Z = 5  # omega around z-axis
    POS_X = 6  # position x-axis
    POS_Y = 7  # position y-axis


# colours in terminal prints
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class JerkEvaluation:
    def __init__(self):
        # number counter for figures
        self.n = 1
        # smoothing parameter value [30 is good value]
        self.smo_para = 30

        # save header names for further use
        self.time = '%time'
        self.hs = 'field.header.seq'
        self.fhs = 'field.header.stamp'  # stamp for calculating differentiation
        self.vel_x = 'field.twist.twist.linear.x'  # velocity x-direction
        self.vel_y = 'field.twist.twist.linear.y'  # velocity y-direction
        self.ome_z = 'field.twist.twist.angular.z'  # omega around z-axis
        self.pos_x = 'field.pose.pose.position.x'  # position x-axis
        self.pos_y = 'field.pose.pose.position.y'  # position y-axis
        # list for header-names from csv
        self.data = [self.time, self.hs, self.fhs, self.vel_x, self.vel_y, self.ome_z, self.pos_x, self.pos_y]

        # create array
        self.A = np.ones([0, 8], dtype=np.float64)

        self.A_grad_vel = np.ones([0, 8], dtype=np.float64)
        self.A_grad_vel_x = np.ones([0, 8], dtype=np.float64)
        self.A_grad_vel_y = np.ones([0, 8], dtype=np.float64)
        self.A_grad_vel_smo = np.ones([0, 8], dtype=np.float64)

        self.A_grad_acc = np.ones([0, 8], dtype=np.float64)
        self.A_grad_acc_x = np.ones([0, 8], dtype=np.float64)
        self.A_grad_acc_y = np.ones([0, 8], dtype=np.float64)
        self.A_grad_acc_smo = np.ones([0, 8], dtype=np.float64)
        self.A_grad_smo_acc = np.ones([0, 8], dtype=np.float64)

        self.A_grad_jerk = np.ones([0, 8], dtype=np.float64)
        self.A_grad_jerk_x = np.ones([0, 8], dtype=np.float64)
        self.A_grad_jerk_y = np.ones([0, 8], dtype=np.float64)
        self.A_grad_jerk_smo = np.ones([0, 8], dtype=np.float64)
        self.A_grad_smo_jerk = np.ones([0, 8], dtype=np.float64)

        self.A_diff = np.ones([0, 8], dtype=np.double)

    # plot data in one figure
    def plot1figure(self, xAxis, yAxis, legendLabel='legend label', xLabel='x-axis label', yLabel='y-axis label',
                    title='plot', axSize='auto', show=0):
        """
        :param xAxis: time axis data
        :param yAxis: data for y axis
        :param legendLabel: label name for first y-axis data (e.g. '$v_x$' for velocity in x-direction)
        :param xLabel: label for time axis (mostly 'Time [s]')
        :param yLabel: label for first y-axis (e.g. '$v [m/s]$', for given example above)
        :param title: title of the plot (obviously)
        :param axSize: 'auto' means min and max is chosen automatically, otherwise: [x_min, x_max, y_min, y_max]
        :param show: shall plot be shown? 1: yes / 2: no
        """
        if show == 1:
            plt.figure(self.n, figsize=(16.0, 10.0))
            plt.plot(xAxis, yAxis, 'r', label=legendLabel)
            plt.title(title, fontsize=20)
            plt.xlabel(xLabel, fontsize=20)
            plt.ylabel(yLabel, fontsize=20)
            plt.grid(True)

            if axSize != 'auto':
                plt.axis(axSize)

            plt.legend(fontsize=15)
            plt.savefig('Plots/' + title.lower() + '_' + time.strftime("%d.%m.%Y---%H:%M") + '.pdf',
                        bbox_inches='tight')

            # increment figure number counter
            self.n += 1
        else:
            pass

    # plot 2 subplots in one figure
    def plot2Subplots(self, xAxis, yAxis1, yAxis2, legendLabel1='first legend label',
                      legendLabel2='second legend label',
                      xLabel='x-axis label', yLabel1='y-axis label 1', yLabel2='y-axis label 2',
                      title='plot', axSize='auto', show=0):
        """
        @param xAxis: time axis array
        @param yAxis1: data for first y-axis as array
        @param yAxis2: data for second y-axis as array
        @param legendLabel1: label name for first y-axis data (e.g. '$v_x$' for velocity in x-direction)
        @param legendLabel2: label name for second y-axis data (e.g. '$v_y$' for velocity in y-direction)
        @param xLabel: label for time axis (mostly 'Time [s]')
        @param yLabel1: label for first y-axis (e.g. '$v [m/s]$', for given example above)
        @param yLabel2: label for second y-axis (e.g. '$v [m/s]$', for given example above)
        @param title: title of the plot (obviously)
        @param axSize: 'auto' means min and max is chosen automatically, otherwise: [x_min, x_max, y_min, y_max]
        @param show: shall plot be shown? 1: yes / 2: no
        @return: nothing
        """

        if show == 1:
            plt.figure(self.n, figsize=(16.0, 10.0))
            plt.subplot(211)
            plt.plot(xAxis, yAxis1, 'r', label=legendLabel1)
            plt.title(title, fontsize=20)
            plt.ylabel(yLabel1, fontsize=20)
            plt.grid(True)
            if axSize != 'auto':
                plt.axis(axSize)
            # legend: loc='best' sets legend to best location
            plt.legend()
            plt.subplot(212)
            plt.plot(xAxis, yAxis2, 'g', label=legendLabel2)
            plt.xlabel(xLabel, fontsize=20)
            plt.ylabel(yLabel2, fontsize=20)
            plt.grid(True)
            if axSize != 'auto':
                plt.axis(axSize)
            # legend: loc='best' sets legend to best location
            plt.legend()
            plt.savefig('Plots/' + title.lower() + '_' + time.strftime("%d.%m.%Y---%H:%M") + '.pdf',
                        bbox_inches='tight')

            # increment figure number counter
            self.n += 1
        else:
            pass

    # plot the specified figures
    def show_figures(self):
        # plot position
        self.plot2Subplots(self.A[:, AD.FHS], self.A[:, AD.POS_X], self.A[:, AD.POS_Y],
                           '$\mathrm{Pos_x}$', '$\mathrm{Pos_y}$', 'Time [s]', '$\mathrm{x\;[m]}$',
                           '$\mathrm{x\;[m]}$', 'Position', axSize='auto', show=0)

        # plot velocity odometry controller
        self.plot2Subplots(self.A[:, AD.FHS], self.A[:, AD.VEL_X], self.A[:, AD.VEL_Y],
                           '$\mathrm{v_x}$', '$\mathrm{v_y}$', 'Time [s]', '$\mathrm{v\;[m/s]}$', '$\mathrm{v\;[m/s]}$',
                           title='Velocity', show=0)

        # plot velocity (x^2+y^2)^0.5 diff
        self.plot2Subplots(self.A[:-1, AD.FHS], np.sqrt(self.A_diff[:, AD.POS_X] ** 2 + self.A_diff[:, AD.POS_Y] ** 2),
                           np.sqrt(self.A[:-1, AD.VEL_X] ** 2 + self.A[:-1, AD.VEL_Y] ** 2),
                           '$\mathrm{v_{x,diff,root}}$', '$\mathrm{v_{x,odo,root}}$', 'Time [s]', '$\mathrm{v\;[m/s]}$',
                           '$\mathrm{v\;[m/s]}$', title='Velocity calculated using \'diff\'', show=0)

        # plot velocity (x^2+y^2)^0.5 gradient
        self.plot2Subplots(self.A[:, AD.FHS], self.A_grad_vel[:, ],
                           np.sqrt(self.A[:, AD.VEL_X] ** 2 + self.A[:, AD.VEL_Y] ** 2),
                           '$\mathrm{v_{x,grad,root}}$', '$\mathrm{v_x{x,odo,root}}$', 'Time [s]',
                           '$\mathrm{v\;[m/s]}$', '$\mathrm{v\;[m/s]}$', title='Velocity calculated using \'gradient\'',
                           axSize=[0, 73, -.05, .3], show=0)

        # plot acceleration diff: x,y
        self.plot2Subplots(self.A[:-1, AD.FHS], self.A_diff[:, AD.VEL_X], self.A_diff[:, AD.VEL_Y],
                           '$a_x$', '$a_y$', 'Time [s]', '$\mathrm{a\quad[m/s^2]}$', '$\mathrm{a\quad[m/s^2]}$',
                           'Acceleration', axSize='auto', show=0)

        # plot diff and gradient method comparison for acceleration
        self.plot2Subplots(self.A[:-1, AD.FHS], self.A_grad_acc[:-1, ],
                           np.sqrt(self.A_diff[:, AD.VEL_X] ** 2 + self.A_diff[:, AD.VEL_Y] ** 2),
                           '$\mathrm{a_{grad}}$', '$\mathrm{a_{diff}}$', 'Time [s]', '$\mathrm{a\;[m/s^2]}$',
                           '$\mathrm{a\;[m/s^2]}$', 'Diff_Grad', axSize='auto', show=0)

        # plot acceleration smoothed and noisy signal
        self.plot2Subplots(self.A[:, AD.FHS], self.A_grad_acc_smo[:, ],
                           self.A_grad_acc[:, ], '$\mathrm{a_{grad,smoothed}}$', '$\mathrm{a_{grad,noisy}}$',
                           'Time [s]', '$\mathrm{a\;[m/s^2]}$', '$\mathrm{a\;[m/s^2]}$',
                           'Acceleration', axSize=[0, 80, -.1, 1.0], show=0)

        # plot acceleration x,y separately
        self.plot2Subplots(self.A[:, AD.FHS], self.A_grad_acc_x, self.A_grad_acc_y, '$a_{grad,x}$', '$a_{grad,y}$',
                           'Time [s]', '$\mathrm{a\;[m/s^2]}$', '$\mathrm{a\;[m/s^2]}$',
                           title='Acceleration: x,y direction', show=0)

        # plot jerk smoothed and noisy: 30 is good value for smoothing
        self.plot2Subplots(self.A[:, AD.FHS], self.A_grad_smo_jerk[:, ],
                           self.A_grad_jerk[:, ], '$\mathrm{j_{grad,smooth}}$', '$\mathrm{j_{grad,noisy}}$',
                           'Time [s]', '$\mathrm{j\;[m/s^3]}$', '$\mathrm{j\;[m/s^3]}$',
                           'Jerk', axSize=[0, 80, -.5, 15], show=0)

        # plot complete jerk smoothed
        self.plot1figure(self.A[:, AD.FHS], self.A_grad_smo_jerk,
                         '$\mathrm{j_{smooth,30}}$', 'Time [s]', '$\mathrm{j\;[m/s^3]}$', 'Jerk Smoothed',
                         axSize='auto', show=1)

        # plot velocity and jerk
        self.plot2Subplots(self.A[:, AD.FHS], np.sqrt(self.A[:, AD.VEL_X] ** 2 + self.A[:, AD.VEL_Y] ** 2),
                           self.A_grad_smo_jerk, '$\mathrm{v_{A}}$', '$\mathrm{j_{smooth,30}}$', 'Time [s]',
                           '$\mathrm{v\;[m/s]}$', '$\mathrm{j\;[m/s^3]}$', 'Velocity and Jerk', show=1)

        plt.show()

    # plot smoothing comparison between 1x and 2x smoothing
    def smoothing_times_plot(self):
        plt.figure(self.n, figsize=(16.0, 10.0))
        plt.plot(self.A[:, AD.TIME], self.A[:, AD.VEL_X], 'r',
                 label='$v_{normal}$')
        plt.plot(self.A[:, AD.TIME], self.smooth(self.A[:, AD.VEL_X], 30, window='hanning'),
                 label='$v_{smooth,1\,times}$')
        plt.plot(self.A[:, AD.TIME], self.smooth(
            self.smooth(self.A[:, AD.VEL_X], 10, window='hanning'),
            50, window='hamming'), label='$v_{smooth,2\,times}$')
        plt.grid(True)
        plt.xlabel('Time [s]', fontsize=20)
        plt.ylabel('v [m/s]', fontsize=20)
        plt.title('Smoothing Comparison', fontsize=20)
        plt.legend(fontsize=15)
        plt.savefig('smoothing_plot.pdf', bbox_inches='tight')

        # increment figure counter
        self.n += 1

    # plot jerk comparison between smoothed and noisy signal
    def jerk_comparison(self):
        plt.figure(self.n, figsize=(16.0, 10.0))
        for i in [10, 20, 30, 40, 50]:
            plt.plot(self.A[:, AD.TIME], self.smooth(self.A_grad_jerk[:, ], i, window='hanning'),
                     label='$j_{grad,smooth,' + str(i) + '}$')
            plt.xlabel('Time [s]', fontsize=20)
            plt.ylabel('j $[m/s^3]$', fontsize=20)
            plt.grid(True)

        plt.plot(self.A[:, AD.TIME], self.bandwidth(4.5), 'k--', label='$Bandwidth$')
        plt.title('Jerk comparison different smoothing', fontsize=20)
        plt.legend(fontsize=15)
        plt.axis([18, 23, -.5, 7])
        plt.draw()
        plt.savefig('jerk_comparison.pdf', bbox_inches='tight')

        # increment figure counter
        self.n += 1

    def smooth(self, x, window_len=11, window='hanning'):
        """smooth the data using a window with requested size.

        This method is based on the convolution of a scaled window with the signal.
        The signal is prepared by introducing reflected copies of the signal
        (with the window size) in both ends so that transient parts are minimized
        in the begining and end part of the output signal.

        input:
            x: the input signal
            window_len: the dimension of the smoothing window; should be an odd integer
            window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
                flat window will produce a moving average smoothing.

        output:
            the smoothed signal

        example:

        t=linspace(-2,2,0.1)
        x=sin(t)+randn(len(t))*0.1
        y=smooth(x)

        see also:

        numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
        scipy.signal.lfilter

        TODO: the window parameter could be the window itself if an array instead of a string
        NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
        """

        if x.ndim != 1:
            raise ValueError, "smooth only accepts 1 dimension arrays."

        if x.size < window_len:
            raise ValueError, "Input vector needs to be bigger than window size."

        if window_len < 3:
            return x

        if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
            raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"

        s = np.r_[x[window_len - 1:0:-1], x, x[-2:-window_len - 1:-1]]
        # print(len(s))
        if window == 'flat':  # moving average
            w = np.ones(window_len, 'd')
        else:
            w = eval('np.' + window + '(window_len)')

        y = np.convolve(w / w.sum(), s, mode='valid')

        return y[(window_len / 2 - 1):-(window_len / 2)]
        # return y

    # read data from .csv-file
    def read_data_csv(self):
        # global A
        global m_A
        global n_A

        with open('Ingolstadt_Test3.csv', 'rb') as csvfile:
            odometry_reader = csv.DictReader(csvfile, delimiter=',')
            # column_names_csv is of type 'list'
            column_names_csv = odometry_reader.fieldnames
            # get number of rows in csv-file
            row_number = sum(1 for line in odometry_reader)
            A = np.zeros([row_number, self.data.__len__()], dtype=np.float64)
            # set pointer to first row
            csvfile.seek(0)
            # jump over first now with names
            odometry_reader.next()

            i = 0
            for row in odometry_reader:
                # jump over header row with names
                if row[self.data[0]] == self.time:
                    continue
                j = 0
                for name in self.data:
                    if name == self.time or name == self.fhs:
                        # scale time and field.header.stamp with factor 1e-9
                        scale = 10 ** -9
                    else:
                        # otherwise no scaling is needed
                        scale = 1
                    a = row[name]
                    A[i, j] = float(a) * scale
                    j += 1
                i += 1

        # set time to start at 0s
        A[:, AD.TIME] = A[:, AD.TIME] - A[0, AD.TIME]
        A[:, AD.FHS] = A[:, AD.FHS] - A[0, AD.FHS]
        # save dimensions of A
        m_A, n_A = A.shape

        print 'Time of Interval: {:.3f} [s]'.format(A[-1, AD.TIME] - A[0, AD.TIME])
        print 'Time of Interval: {:.3f} [s]'.format(A[-1, AD.FHS] - A[0, AD.FHS])
        self.A = A

    def read_data_subscriber(self):
        # global A
        global m_A
        global n_A

        # instantiate class NodeListener
        nl = listener.NodeListener()
        # subscribe to odometry
        nl.listener()
        self.A = np.array(nl.return_array())
        print bcolors.OKBLUE + 'Got this array: ', self.A.shape, bcolors.ENDC

        # set time to start at 0s
        self.A[:, AD.FHS] = self.A[:, AD.FHS] - self.A[0, AD.FHS]
        # save dimensions of A
        m_A, n_A = self.A.shape

        print 'Time of Interval: {:.4f} [s]'.format(self.A[-1, AD.FHS] - self.A[0, AD.FHS])

    # get differentiation from given data
    def differentiation(self):
        # # global A_grad_vel
        # global A_grad_vel_smo
        # # global A_grad_vel_x
        # # global A_grad_vel_y
        # global A_grad_acc
        # global A_grad_acc_smo
        # # global A_grad_acc_x
        # # global A_grad_acc_y
        # global A_grad_jerk
        # global A_grad_jerk_smo
        # # global A_grad_jerk_x
        # # global A_grad_jerk_y
        #
        # # global A_diff
        #
        # global A_grad_smo_acc
        # global A_grad_smo_jerk

        # differentiation
        self.A_grad_vel_x = np.gradient(self.A[:, AD.POS_X], self.A[1, AD.FHS] - self.A[0, AD.FHS])
        self.A_grad_vel_y = np.gradient(self.A[:, AD.POS_Y], self.A[1, AD.FHS] - self.A[0, AD.FHS])
        # (x^2+y^2)^0.5 to get absolut velocity
        self.A_grad_vel = np.sqrt(self.A_grad_vel_x[:, ] ** 2 + self.A_grad_vel_y[:, ] ** 2)
        self.A_grad_vel_smo = self.smooth(self.A_grad_vel[:, ], self.smo_para, window='hanning')

        # differentiation
        # compute acceleration from velocity by differentiation
        self.A_grad_acc_x = np.gradient(self.A[:, AD.VEL_X], self.A[1, AD.FHS] - self.A[0, AD.FHS])
        self.A_grad_acc_y = np.gradient(self.A[:, AD.VEL_Y], self.A[1, AD.FHS] - self.A[0, AD.FHS])
        # (x^2+y^2)^0.5 to get absolute acceleration
        self.A_grad_acc = np.sqrt(self.A_grad_acc_x[:, ] ** 2 + self.A_grad_acc_y[:, ] ** 2)
        # smoothed after differentiation
        self.A_grad_acc_smo = self.smooth(self.A_grad_acc[:, ], self.smo_para, window='hanning')
        # smoothed acc used for (x^2+y^2)^0.5 to get absolute acceleration
        self.A_grad_smo_acc = np.sqrt(self.smooth(self.A_grad_acc_x[:, ], 30, window='hanning') ** 2 +
                                      self.smooth(self.A_grad_acc_y[:, ], 30, window='hanning') ** 2)

        # differentiation
        # compute jerk from acceleration by differentiation
        self.A_grad_jerk_x = np.gradient(self.A_grad_acc_x[:, ], self.A[1, AD.FHS] - self.A[0, AD.FHS])
        self.A_grad_smo_jerk_x = np.gradient(self.smooth(self.A_grad_acc_x[:, ], 30, window='hanning'),
                                             self.A[1, AD.FHS] - self.A[0, AD.FHS])
        # noisy acc used for differentiation
        self.A_grad_jerk_y = np.gradient(self.A_grad_acc_y[:, ], self.A[1, AD.FHS] - self.A[0, AD.FHS])
        # smoothed acc used for differentiation
        self.A_grad_smo_jerk_y = np.gradient(self.smooth(self.A_grad_acc_y[:, ], 30, window='hanning'),
                                             self.A[1, AD.FHS] - self.A[0, AD.FHS])
        # (x^2+y^2)^0.5 to get absolut jerk
        self.A_grad_jerk = np.sqrt(self.A_grad_jerk_x[:, ] ** 2 + self.A_grad_jerk_y[:, ] ** 2)
        # smoothed after differentiation
        self.A_grad_jerk_smo = self.smooth(self.A_grad_jerk[:, ], 30, window='hanning')
        # smoothed acc used for differentiation
        self.A_grad_smo_jerk = np.sqrt(self.A_grad_smo_jerk_x[:, ] ** 2 + self.A_grad_smo_jerk_y[:, ] ** 2)

        # differentiation using diff
        self.A_diff = np.diff(np.transpose(self.A))
        self.A_diff = np.transpose(self.A_diff)

    def save_csv(self):
        print 'Date: ' + time.strftime("%d.%m.%Y-%H:%M")
        data_matrix = np.array([[self.data[i] for i in xrange(0, self.data.__len__())]])
        B = np.concatenate((data_matrix, self.A), axis=0)
        # fmt='%.18e' for float
        np.savetxt('csv/' + time.strftime("%d.%m.%Y---%H:%M") + '.csv', B, fmt='%s', delimiter=',')

    # creating bandwidth matrix
    def bandwidth(self, max):
        B = np.zeros([m_A, 1])
        for i in xrange(0, m_A):
            B[i, 0] = max
        return B

    # compare jerk with given max bandwidth, if jerk is to big function returns false
    def jerk_metrics(self, max_jerk):
        '''
        jerk metrics to see if max jerk is in desired range
        :param max_jerk: max allowed jerk for comparison
        :return: false --
        '''
        for i in xrange(0, m_A):
            if self.A_grad_smo_jerk[i,] >= max_jerk:
                output = bcolors.FAIL + 'Jerk: {:.3f} [m/s^3] at time: {:.6f} s is bigger than max ' \
                                        'allowed jerk: {:.3f} [m/s^3]' + bcolors.ENDC
                print output.format(self.A_grad_smo_jerk[i,], self.A[i, AD.FHS], max_jerk)
                print 'Jerk below: {:.3f} [m/s^3] at time: {:.6f} s is in ' \
                      'range'.format(self.A_grad_smo_jerk[i - 1,], self.A[i - 1, AD.FHS])
                print 'Max Jerk: {:.4f} [m/s^3]'.format(self.A_grad_smo_jerk.max())
                return False
        print bcolors.OKGREEN + 'Jerk is in desired range!' + bcolors.ENDC
        print 'Max Jerk: {:.4f} [m/s^3]'.format(self.A_grad_smo_jerk.max())
        return True

    # smoothing in workflow comparison
    def smoothing_workflow_comparison(self):
        plt.figure(self.n, figsize=(16.0, 10.0))
        plt.subplot(211)
        plt.plot(self.A[:, AD.TIME], self.A_grad_acc, 'b', label='unsmoothed')
        plt.plot(self.A[:, AD.TIME], self.A_grad_acc_smo, 'k', label='smoothed after differentiation')
        plt.plot(self.A[:, AD.TIME], self.A_grad_smo_acc, 'r', label='smoothed acc x and y used')
        plt.ylabel('a $[m/s^2]$', fontsize=20)
        plt.legend()
        plt.grid(True)

        plt.subplot(212)
        plt.plot(self.A[:, AD.TIME], self.A_grad_jerk, 'b', label='unsmoothed')
        plt.plot(self.A[:, AD.TIME], self.A_grad_jerk_smo, 'k', label='smoothed after differentiation')
        plt.plot(self.A[:, AD.TIME], self.A_grad_smo_jerk, 'r', label='smoothed acc used for differentiation')
        plt.ylabel('j $[m/s^3]$', fontsize=20)
        plt.grid(True)

        plt.xlabel('Time [s]', fontsize=20)
        plt.legend()
        plt.draw()
        plt.savefig('smoothing_in_workflow_comparison.pdf', bbox_inches='tight')
        self.n += 1

    # calling the other functions
    def main(self):
        # close all existing figures
        plt.close('all')
        self.read_data_csv()
        # self.read_data_subscriber()
        self.differentiation()
        self.save_csv()
        # smoothing_times_plot()
        # jerk_comparison()
        # smoothing_workflow_comparison()
        self.show_figures()


# commandline input: -jerk=*max_jerk*
# if no commandline input is given, max_jerk=4.0 is set
if __name__ == '__main__':
    je = JerkEvaluation()
    je.main()
    if len(sys.argv) > 1:
        jerk = sys.argv[1]
        max_jerk = float(jerk[6:])
        je.jerk_metrics(max_jerk)
    else:
        je.jerk_metrics(4.0)
pass

# TODO:
# REVIEW:
