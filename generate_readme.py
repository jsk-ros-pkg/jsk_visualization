#!/usr/bin/env python
"""Generate README.md of jsk_visualization.

    ./generate_readme.py

writes README.md next to this script (it reads the deb status table of the
current README.md when jsk_tools is not available, so do not redirect the
output to README.md).

The gallery links each sample to its section in the README of its package
(jsk_rviz_plugins/README.md, ...), which are written by hand.
"""

import datetime
import io
import os.path as osp
import subprocess
import sys


here = osp.dirname(osp.realpath(__file__))

# the README of each package, relative to this directory
PACKAGE_READMES = [
    ('jsk_rviz_plugins', 'jsk_rviz_plugins/README.md'),
    ('jsk_rqt_plugins', 'jsk_rqt_plugins/README.md'),
    ('jsk_interactive_marker',
     'jsk_interactive_markers/jsk_interactive_marker/README.md'),
]

# (package, sample, plugins, thumbnail); the sample is the heading of its
# section in the README of the package, the thumbnail is in .readme/gallery/
GALLERY = [
    ('jsk_rviz_plugins', 'overlay_sample',
     'OverlayText, String, Plotter2D, PieChart, OverlayImage, OverlayMenu, '
     'OverlayDiagnostic', 'rviz_overlay_ros2.gif'),
    ('jsk_rviz_plugins', 'pictogram_sample',
     'Pictogram, PictogramArray', 'rviz_pictogram_ros2.png'),
    ('jsk_rviz_plugins', 'linear_gauge_sample',
     'LinearGauge', 'rviz_linear_gauge_ros2.png'),
    ('jsk_rviz_plugins', 'piechart_sample',
     'PieChart', 'rviz_piechart_ros2.png'),
    ('jsk_rviz_plugins', 'robot_command_interface_sample',
     'RobotCommandInterfaceAction', 'rviz_robot_command_interface_ros2.png'),
    ('jsk_rviz_plugins', 'service_call_panel',
     'EmptyServiceCallInterfaceAction', 'rviz_service_call_panel_ros2.png'),
    ('jsk_rviz_plugins', 'boundingbox_sample',
     'BoundingBoxArray', 'rviz_bounding_box_array_ros2.png'),
    ('jsk_rviz_plugins', 'polygon_array_sample',
     'PolygonArray', 'rviz_polygon_array_ros2.png'),
    ('jsk_rviz_plugins', 'segment_array_sample',
     'SegmentArray', 'rviz_segment_array_ros2.png'),
    ('jsk_rqt_plugins', 'rqt_string_label',
     'StringLabel', 'rqt_string_label_ros2.png'),
    ('jsk_rqt_plugins', 'rqt_status_light',
     'StatusLight', 'rqt_status_light_ros2.png'),
    ('jsk_rqt_plugins', 'rqt_yn_btn',
     'YesNoButton', 'rqt_yn_btn_ros2.png'),
    ('jsk_rqt_plugins', 'sample_image_view2',
     'ImageView2Plugin', 'rqt_image_view2_ros2.gif'),
    ('jsk_rqt_plugins', 'sample_3dplot',
     'Plot3D', 'rqt_3d_plot_ros2.png'),
    ('jsk_rqt_plugins', 'sample_service_buttons',
     'ServiceButtons', 'rqt_service_buttons_ros2.png'),
    ('jsk_rqt_plugins', 'sample_service_radio_buttons',
     'ServiceRadioButtons', 'rqt_service_radio_buttons_ros2.png'),
    ('jsk_rqt_plugins', 'sample_tabbed_buttons',
     'ServiceTabbedButtons', 'rqt_tabbed_buttons_ros2.png'),
    ('jsk_rqt_plugins', 'sample_drc_mini_maxwell',
     'DRCEnvironmentViewer', 'rqt_drc_mini_maxwell_ros2.png'),
    ('jsk_rqt_plugins', 'sample_histogram',
     'HistogramPlot', 'rqt_histogram_plot_ros2.png'),
    ('jsk_rqt_plugins', 'sample_2dplot',
     'Plot2D', 'rqt_2d_plot_ros2.png'),
    ('jsk_interactive_marker', 'marker_6dof_sample',
     'marker_6dof', 'im_marker_6dof_ros2.gif'),
    ('jsk_interactive_marker', 'transformable_server_sample',
     'transformable_server_sample', 'im_transformable_server_ros2.gif'),
]

COLUMNS = 4
THUMBNAIL_WIDTH = 200


def github_slug(heading):
    """Anchor id of a heading on GitHub (for the names used here)."""
    slug = ''.join(c for c in heading.strip().lower()
                   if c.isalnum() or c in ' -_')
    return slug.replace(' ', '-')


def get_gallery():
    content = []
    for pkg, readme in PACKAGE_READMES:
        entries = [e for e in GALLERY if e[0] == pkg]
        content.append('### [%s](%s)' % (pkg, readme))
        content.append('')
        content.append('<table>')
        for i in range(0, len(entries), COLUMNS):
            content.append('<tr>')
            for _, sample, plugins, thumbnail in entries[i:i + COLUMNS]:
                href = '%s#%s' % (readme, github_slug(sample))
                img = '.readme/gallery/%s' % thumbnail
                if not osp.exists(osp.join(here, img)):
                    raise IOError('%s not found' % img)
                content.append(
                    '<td align="center" valign="top" width="25%%">'
                    '<a href="%s"><img src="%s" width="%d"></a><br>'
                    '<a href="%s"><code>%s</code><br><sub>%s</sub></a>'
                    '</td>' % (href, img, THUMBNAIL_WIDTH,
                               href, sample, plugins))
            content.append('</tr>')
        content.append('</table>')
        content.append('')
    return '\n'.join(content).rstrip()


def get_deb_status_table():
    cmd = 'rosrun jsk_tools generate_deb_status_table.py jsk_visualization'
    try:
        table = subprocess.check_output(
            cmd, shell=True, stderr=subprocess.PIPE)
        return table.decode('utf-8').strip()
    except (OSError, subprocess.CalledProcessError):
        # jsk_tools is not available: keep the table of the current README
        sys.stderr.write('%s failed, reusing the deb status table '
                         'of README.md\n' % cmd)
        with io.open(osp.join(here, 'README.md'), encoding='utf-8') as f:
            readme = f.read()
        heading = 'Deb build status\n----------------\n'
        if heading not in readme:
            raise RuntimeError('no deb status table in README.md')
        return readme.split(heading, 1)[1].strip()


template = u'''\
<!--
    DO NOT EDIT THIS FILE BY HAND.

    This file is automatically generated by {SCRIPT} at {TIMESTAMP}.
-->

jsk\\_visualization
=================

 [![Build Status](https://travis-ci.com/jsk-ros-pkg/jsk_visualization.svg?branch=master)](https://travis-ci.com/jsk-ros-pkg/jsk_visualization)
[![Read the Docs](https://readthedocs.org/projects/pip/badge/?version=latest)](https://jsk-visualization.readthedocs.org)

jsk visualization ros package.

See [read the docs](http://jsk-visualization.readthedocs.org/en/latest/).

The same source tree builds for ROS 1 (catkin; rviz, rqt) and ROS 2 (ament;
rviz2, rqt), selected by `$ROS_VERSION`. The samples are launch files
(`roslaunch <pkg> <sample>.launch` on ROS 1,
`ros2 launch <pkg> <sample>.launch.py` on ROS 2) and scripts
(`rosrun` / `ros2 run`); the README of each package shows each sample on
ROS 1 and on ROS 2, with the commands to run it.


Gallery
-------

Click a sample to see how to run it.

{GALLERY}


Deb build status
----------------

{DEB_STATUS_TABLE}
'''


def main():
    readme = template.format(
        SCRIPT=osp.basename(__file__),
        TIMESTAMP=datetime.datetime.now().isoformat(),
        GALLERY=get_gallery(),
        DEB_STATUS_TABLE=get_deb_status_table(),
    )
    with io.open(osp.join(here, 'README.md'), 'w', encoding='utf-8') as f:
        f.write(readme)


if __name__ == '__main__':
    main()
