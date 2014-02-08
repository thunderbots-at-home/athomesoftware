#!/bin/bash
cp -r $PWD ~/sketchbook
rm ~/sketchbook/libraries/add_libraries_to_arduino.sh

cp -r $PWD/* ~/catkin_ws/src/athomesoftware/talos_mechanism_controllers/talos_base_controller/extra/ArduinoCode/Base_Controller_Arduino_Code/util
rm ~/catkin_ws/src/athomesoftware/talos_mechanism_controllers/talos_base_controller/extra/ArduinoCode/Base_Controller_Arduino_Code/util/add_libraries_to_arduino.sh
