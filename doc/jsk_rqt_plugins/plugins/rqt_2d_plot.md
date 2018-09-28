rqt\_2d\_plot
=============

![](images/rqt_2d_plot.png)

Plot data of specified topic as scatter plot.


Topic Type
----------

* `jsk_recognition_msgs/PlotData`


Optional Arguments
------------------

* `--line`: Plot with lines instead of scatter.

* `--fit-line`: Plot line with least-square fitting.

* `--fit-line-ransac`: Plot line with RANSAC.

* `--fit-line-ransac-outlier`: Plot line with RANSAC.


Sample
------

```
$ roslaunch jsk_rqt_plugins sample_2dplot.launch
```
