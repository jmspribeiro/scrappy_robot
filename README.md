#  Jorge Pais Ribeiro - Scrappy robot 
 :warning: WIP - Not functional


## Install dependencies
`rosdep install --from-paths src --ignore-src -r -y`
`sudo apt install bluetooth`

## Setup your joystick
run the following command to check vendor and product ids:
`udevadm info --attribute-walk  --name /dev/input/js*`
change your udev rule accordingly and test it, example:
`udevadm test $(udevadm info --query=path --name=/dev/input/js0)`