rqt\_drc\_mini\_maxwell
=======================

![](images/rqt_drc_mini_maxwell.gif)

Subscribe specified topic and show status in facial expression.


Subscribing Topic
-----------------

* `/drc_2015_environment/is_disabled` (`std_msgs/Bool`)

* `/drc_2015_environment/is_blackout` (`std_msgs/Bool`)

* `/drc_2015_environment/next_whiteout_time` (`std_msgs/Time`)

If `is_disabled` is True, then it frowns and the background color becomes gray.

If `is_disabled` is False and `is_blackout` is True, then it frowns and the background color becomes red.
`next_whiteout_time` is enabled only in this condition.

If `is_disabled` is False and `is_blackout` is False, then it smiles and the background color becomes green.


Sample
------

```
$ roslaunch jsk_rqt_plugins sample_drc_mini_maxwell.launch
```
