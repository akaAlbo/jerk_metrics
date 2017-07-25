# Jerk Metrics
Writing a python programm to get the "/base/odometry_controller"-speed and use it for data analytics. Concerning max allowed jerk in velocity-profile.

## Usage
### Plots
All generated plots are saved as pdf-file for further usage. The archive file includes all plots in subfolder "Plots", named according to plotted data. See example:
![Jerk comparison example plot](https://github.com/ipa-flg-ma/SciPy_Test/blob/master/jerk_comparison.png)

### Terminal
Command-line support for max allowed jerk value given to metrics. Compare all jerk-data to maximum and give either passed or failed feedback (added terminal colour support: failed -- red | passed -- green)
```
-jerk=4.5
```
is setting
```
max_jerk=4.5
```
in the 
```
jerk_metrics(max_jerk)
```
function. Max jerk value has to be determined empirically.

## History
**V 1.5.3:**
- added full terminal support. Command-line arguments now supported, such as
```
-jerk=4.5
```
- Improved terminal output for 'jerk_metrics'-function (now in fancy colours)

**V 1.5.2:**
- code cleanup
- added bandwith support
- added 'jerk_metrics'-function
- made 'main.py' executable

**V 1.5.0:**
- added one figure support
- added smoothing for data
