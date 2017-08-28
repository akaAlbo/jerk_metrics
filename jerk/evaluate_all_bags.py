#!/usr/bin/python

"""
Created on Aug 28, 2017

@author: flg-ma
@attention: Evaluate all bagfiles in one directory
@contact: marcel.albus@ipa.fraunhofer.de (Marcel Albus)
@version: 1.0.0
"""

from bcolors import TerminalColors as tc
import os
import glob

if __name__ == '__main__':
    path = '/home/flg-ma/bagfiles/ipa-apartment/bags'
    # files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]

    files = glob.glob(path + '/' + 'ipa-apartment*.bag')
    main_file = '/home/flg-ma/PycharmProjects/jerk_metrics/jerk/main.py'

    # sort alphabetically
    files.sort()
    for f in files:
        print tc.OKBLUE + '=' * (67 + f.__len__())
        print main_file + ' -rb -bag ' + f + ' -s'
        # evaluate all the bagfiles using the main.py programm
        os.system(main_file + ' -rb -bag ' + f + ' -s')
