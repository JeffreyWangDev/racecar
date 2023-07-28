### Hardwear 
## Servo wheels gets pulled to one side:
Unplug battery and wiggle wheels


### Softwear
## Camera stale images
Restart python program
## Remote unresponsive
Reboot system


### ROS2 dead nodes
## PWM
Run: ```ls /dev/tty*```
Make sure ```/dev/ttyACM0``` is shown
If not:
    sudo shutdown now
    unplug PWM board
    start it 
    plug in board 
Else:
    Run : ```sudo chmod 666 /dev/ttyACM0``` this will give perms to the user to look at that port
Else:
    Reboot and try again, your not trying hard enough
## Camera/IMU
Run ```teleop topic list```
Make sure camera_node is there
If not:
    Stop teleop
    unplug/replug camera
    Rerun teleop