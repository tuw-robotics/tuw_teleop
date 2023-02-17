# tuw_gamepad_python

This package provides a node to read messages from a gamepad (messages of the type `sensor_msgs.Joy`) and convert them to any other type.
Since the messages from a gamepad have different form depending on the hardware this package also allows to unify the messages passed to the converters.

This package is an alternative to the `tuw_gamepad` package.
In order to add a new devices a `.yaml` file has to be added to the `/cfg/device` directory.
In order to add a new converter the converter has to be written and added to the list of converters in the main file of the node (which is `/src/gamepad_node`).

## Adding a gamepad

Adding a new device to the list of supported devices is simple.
Create a new `.yaml` file in the folder `/cfg/device` which has a unique name (in this folder).
The file should contain the following:

`<device_name>`: the name of the device, same as file name.
`<mode>`: some devices have a switch for different modes. This should be the mode which was active while creating the mapping

```yaml
# Mapping for the <device name> Controller (<mode>)

axes:
    x_left: x         # left axis stick left/right
    y_left: x         # left axis stick up/down
    x_right: x        # right axis stick left/right
    y_right: x        # right axis stick up/down
    trigger_left: x   # left trigger
    trigger_right: x  # right trigger
    x_cross: x        # cross key left/right
    y_cross: x        # cross key up/down

buttons:
    a_button: x       # a button (or cross button)
    b_button: x       # b button (or circle button)
    x_button: x       # x button (or square button)
    y_button: x       # y button (or triangle button)
    shoulder_left: x  # left shoulder button
    shoulder_right: x # right shoulder button
    joy_left: x       # left joystick button
    joy_right: x      # right joystick button
    select: x         # select / back button
    start: x          # start button
```

### Create a mapping

In order to create a mapping run a joy node (note that the joy package has to be installed).
To start the joy node run `rosrun joy joy_node`.
Press the buttons from the list above and check which field (index of the field) changes when the button is pressed.
Replace the `x` of the button pressed with the index in the corresponding list (`axes` or `buttons`).
Other  axes and buttons than the axes and buttons listed above are not supported.
If any of the buttons above is not present on your device replace the `x` with `None`.
This field will always be `0.0`.

## Adding a converter

In order to write a new converter create a file in `/src/converter/implementation`.
The new converter should inherit from the `AbstractConverter` class where `T` is replaced by the output message type.
It is important that the class constant `NAME` has to be overwritten.
The converter can have a dynamic reconfigure with parameters specific to that converter.
Note that the dynamic reconfigure serve requires a namespace in this case.

The new converter has to be added to the list of converters in the `/src/gamepad_node` below the comment `# add new converters here`.

## Launch file

This package a generic launch file which can be started with the following arguments:
- `converter`: same as class constant `NAME`
- `device`: same as device file name without `.yaml`
