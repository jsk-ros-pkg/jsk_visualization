# docker for jsk_visualization

## Usage

```
# melodic
docker build --build-arg ROS_DISTRO=melodic --build-arg UBUNTU_VERSION=bionic -t jskrobotics/jsk_visualization:melodic-latest .
# kinetic
docker build --build-arg ROS_DISTRO=kinetic --build-arg UBUNTU_VERSION=xenial -t jskrobotics/jsk_visualization:kinetic-latest .
```
