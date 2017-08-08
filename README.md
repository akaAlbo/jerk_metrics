# Jerk Metrics
Writing a python programm to get the "/base/odometry_controller"-speed and use it for data analytics. Concerning max allowed jerk in velocity-profile.

## Usage
### Plots
All generated plots are saved as pdf-file for further usage. The archive file includes all plots in subfolder "Plots", named according to plotted data. See example:
![Jerk comparison example plot](https://github.com/ipa-flg-ma/SciPy_Test/blob/master/jerk_comparison.png)

### ROS Subscriber Support
Included subscriber to ROS-topic 
```
/base/odometry_controller/
```
topic, of type 'nav_msgs.msg' as 'Odometry.msg'. 
Access data via:
```
data.header.seq
data.header.stamp
data.twist.twist.linear.x
data.twist.twist.linear.y
data.twist.twist.angular.z
data.pose.pose.position.x
data.pose.pose.position.y
```
"stamp" data is given in seconds and nanoseconds. Listener is wizard and calculates one time value.
```
data.header.stamp.secs
data.header.stamp.nsecs
```

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

### Bandwidth
Max allowed jerk is given as bandwidth above which jerk should not go.
![Jerk_with_bandwith](https://github.com/ipa-flg-ma/jerk_metrics/blob/ipa/Jerk_with_bandwith.png)

## History
**V 1.6.2:**
- added minor improvements

**V 1.6.1:**
- bug fixing (time calculating bug because of nanoseconds in timestamp)

**V 1.6.0:**
- added full rostopic subscriber support

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

# HOW-TO New Metric
The following steps are needed to implement a new metrics in ATF:
- Create new python-file for the metrics, using the following nameconvention:
```
calculate_*name*.py
```
- copy existing structure from one of the implemented metrics, looking like:
```python
class CalculatePublishRateParamHandler
    def parse_parameter(self, testblock_name, params):
class CalculatePublishRate:
    def __init__(self, groundtruth, groundtruth_epsilon):
    def start(self, timestamp):
    def stop(self, timestamp):  
    def pause(self, timestamp):
    def purge(self, timestamp):   
    def get_result(self):
```
  using the "publish\_rate"-metrics as an example. Replace "PublishRate" with the name of your newly generated metrics.
- In file ```atf\_metrics\src\atf\_metrics\__init__.py``` add:
```python
from atf_metrics.calculate_*name* import Calculate*Name*, Calculate*Name*ParamHandler
```
  e.g.
```python
from atf_metrics.calculate_jerk import CalculateJerk, CalculateJerkParamHandler
```
  here *name* stands for the name of your new metric (obviously).
  
- In file ```atf\_metrics\config\metrics.yaml``` add:
```
*name*:
   handler: Calculate*Name*ParamHandler
```
  e.g.
```
jerk:
  handler: CalculateJerkParamHandler
```
- In file ```atf\_presenter\html\js\atf\_tools\test\_list.js``` add (using "jerk" as an example):
```javascript
var plot_options = {
      jerk: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Jerk'
        },
        yAxis: {
          title: {
            text: 'Jerk [m/s^3]'
          }
        },
        xAxis: {
          labels: {
            enabled: false
          }
        },
        plotOptions: {},
        tooltip: plot_tooltip
      },
};
```
  search the following if-statement:
```javascript
if ((metric_name == 'time') || (metric_name == 'path_length') || (metric_name == 'publish_rate') || (metric_name == 'interface') || (metric_name == 'jerk'))
```
  and add the new metrics as ```|| (metric_name == '*name*')```. In the following lines...
```javascript
if (metric_name == 'path_length') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['root_frame'] + " to " + metric_data['details']['measured_frame'] + ")"
if (metric_name == 'publish_rate') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")"
if (metric_name == 'interface') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
if (metric_name == 'jerk') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")"
if (metric_name == '*name*') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
```
  add...
```javascript
if (metric_name == '*name*') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
```
  to get add information under the metrics-name in the presenter. The "details" you store in the "metrics\_data" will be shown under the metrics-name in brackets.
