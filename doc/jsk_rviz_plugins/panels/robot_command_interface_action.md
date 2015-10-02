# RobotCommandInterfaceAction
![RobotCommandInterfaceAction](images/robot_command_interface_action.png)

This will call service to /eus_command with jsk_rviz_plugins/EusCommand srv.
All the buttons are configured via `~robot_command_buttons` parameters.
See `robot_command_interface_sample.launch` file to know how to use it.

Parameter format is:
```yaml
robot_command_buttons:
  - name: <name, required>
    icon: <path to icon file, optional>
    type: <"euscommand" or "emptysrv", required>
    command: <S expression to send to eusclient, required if type is euscommand>
    srv: <service name, required if type is "emptysrv">
  - name: ...
```
