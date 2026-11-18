#!/bin/bash
# checkout_unreleased.sh: for the ROS 1 CI, check out jsk_ros1_ros2_compat of jsk_common into ./jsk_common if
# ros-$ROS_DISTRO-jsk-ros1-ros2-compat is not released for the distribution. jsk_ros1_ros2_compat is only in
# jsk_common 3.x; only its directory is checked out (a sparse checkout, which git 1.9 of Ubuntu 14.04 also does),
# as the other packages of jsk_common are released.
set -e
package=ros-${ROS_DISTRO}-jsk-ros1-ros2-compat
if apt-cache policy "$package" 2>/dev/null | grep -q 'Candidate: [0-9]'; then
  echo "$package is released"
  exit 0
fi
echo "$package is not released: check out jsk_ros1_ros2_compat of jsk_common"
mkdir -p jsk_common
cd jsk_common
git init -q .
git remote add origin https://github.com/jsk-ros-pkg/jsk_common.git
git config core.sparseCheckout true
echo jsk_ros1_ros2_compat/ > .git/info/sparse-checkout
git fetch -q --depth=1 origin master
git checkout -q FETCH_HEAD
ls
