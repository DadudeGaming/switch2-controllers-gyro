# switch2 controllers
An app to use switch 2 joycons on pc as gamepad and mouse

### Setup

Download python 3.7+ and install, Setup pip as path so you can run pip --version and pip install <file>

Then run
pip install bleak
pip install pywin32
pip install pyyaml
pip install vgamepad
Download and run the exe from https://github.com/nefarius/ViGEmBus/releases/tag/v1.22.0
Run gui.py (Or the exe if it is bundled)

### Usage

No need to pair the controller in the bluetooth settings.

Simply launch the app, and do what it says.

If you already paired the joycons in windows bluetooth settings, remove it before attempting to use it with this app.

### Using as a mouse

By default the app switches a joycon to mouse mode when it detects it's being used a mouse (side of of the joycon against a flat surface)

When in mouse mode, the following buttons are used as mouse buttons and no longer useable as gamepad buttons :
L/R : left click
ZL/ZR : right click
joystick : mouse wheel and middle button (click)

If you do not wish to use mouse mode, you can disable it in the config

Mouse mode now detects if it is in a grip. If there is no mouse movement for a short amount of time, it will assume it is in a grip and disable the mouse until it is lifted.
When the mouse sensor is not on a surface, the mouse mode detection resets and waits to check again.

### Using joycons sideways

By default, the app will always try to combine a right and left joycons together to make a single virtual controller.

If you wish to use both joycons sideway, you can hold SL\SR while turning them on
An other option is to set `combine_joycons` in the config to false so that the app will never try to combine joycons

### Gyro
Gyro is not great, it currently uses a udp server as output, and since steam detects it as an xbox controller, it will not take gyro input.
