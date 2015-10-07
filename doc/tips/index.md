Tips about visualization
========================

Record rviz
-----------

### kazam
You can use kazam to record desktop movie easily.

```
sudo apt-get install kazam
```

### glc
You can use [glc](https://github.com/nullkey/glc/wiki) to record OpenGL rendering.
glc is the best way to record OpenGL application because it can record movie efficiently.
glc record OpenGL rendering to a special file called `.glc`
and you can convert the `.glc` into several movie format.
It is also good for irtviewer.

#### Install
You can install glc via ppa package. ([see this tutorial](https://github.com/nullkey/glc/wiki/Install))

```
sudo add-apt-repository ppa:arand/ppa
sudo apt-get update
sudo apt-get install glc
```

#### Capture
You can use glc as wrap command like:

```
glc rviz
```

If you want to use it in launch file, use `launch-prefix` attribute:

```xml
<node pkg="rviz" type="rviz" name="rviz" launch-prefix="glc" />
```

You need to type `Shift + F8` to start and stop capturing. ([see this tutorial for detail](https://github.com/nullkey/glc/wiki/Capture))

#### Convert to movie
You can use `glc_encode.sh` under `jsk_tools`.

```
rosrun jsk_tools glc_encode.sh foo.glc
```
