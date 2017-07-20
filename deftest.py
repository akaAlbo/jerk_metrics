'''
@author: flg-ma
@version: 1.0
'''

import matplotlib.pyplot as plt
import numpy as np

n = 1


def plot1figure(xAxis, yAxis, legendLabel='legend label', xLabel='x-axis label',
                yLabel='y-axis label', title='plot', axSize='auto', show=0):
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
        global n
        plt.figure(n, figsize=(16.0, 10.0))
        plt.plot(xAxis, yAxis, label=legendLabel)
        plt.title(title, fontsize=20)
        plt.xlabel(xLabel, fontsize=20)
        plt.ylabel(yLabel, fontsize=20)
        plt.grid(True)

        if axSize != 'auto':
            plt.axis(axSize)

        plt.legend()
        plt.savefig(title + '.pdf', bbox_inches='tight')

        # increment figure counter
        n += 1
        plt.show()
        print 'number of figures: %i' % n
    else:
        pass


plot1figure([1,2,3,4], [1,2,3,4],'data1', 'time [s]', 'test data', 'plot test', show=0)

x = 25
print 'print root of {}: {}'.format(x, np.sqrt(x))