BoundingBoxArray
================

What is this?
-------------

Visualize ``jsk_recognition_msgs/BoundingBoxArray.msg``.

.. image:: images/bounding_box_array.png

Properties
**********

.. image:: images/bounding_box_array_rviz.gif


- `Topic`

  Name of topic of `jsk_recognition_msgs/BoundingBoxArray`

- `coloring`

  `Flat Color` applies the same color to all boxes. `Label` uses the
  label value of each box for coloring, and `Value` applies the alpha
  value from the box message.

- `alpha method`

  `flat` applies the same alpha value to all boxes. You can set this
  value in the `alpha` field. `value` uses the box message data, and the
  alpha can be scaled by setting the `alpha min` and `alpha max` fields.

- `only edge`

  If it is true, only the edges of the boxes will be displayed.

- `show coords`

  If it is true, the coordinates of the boxes will be displayed.

- `value threshold`

  Display only boxes with alpha value avobe this threshold.
