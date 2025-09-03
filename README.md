# balancing_robot
Simple balancing robot using 2 DC motors. Built using ESP8266, RTOS and websockets to constantly show states/measurements.

built in ubuntu 24.04

local venv environment exists that contains all ESP8266 RTOS SDK related python modules (found in ~/env)

Made alias for initialising the $PATH environment variable to include these modules (get_lx106).

To build and monitor, run the command `make flash monitor`. For it to be successful, the terminal must have run the above alias, and be running within the venv environment inside of the root of this repo.

Must keep the AP/router info updated in the sdkconfig. `make menuconfig` command to do that.

Consult tutorials of installing the RTOS SDK for setup guide.
