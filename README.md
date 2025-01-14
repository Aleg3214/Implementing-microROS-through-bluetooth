The code implements microROS communication through Bluetooth RFCOMM simulated serial port (replacing the serial USB cable).
The implementation has been done using a NUCLEO STM32F401RE and a bluetooth module HC-05.
Please check the documentation file "ROS2_Bluetooth.pdf" for specific informations on how to:
-Setup STM Cube IDE for the current application;
-Setup STM Cube MX for the current application;
-Setup hardware peripherals;
-Install ROS2 and create an agent;
After ensuring all the previously written steps are executed and Bluetooth communication is correctly working is needed to:
-Pair (through settings) and bind the HC-05 module using the command:
  sudo rfcomm bind /dev/rfcomm0 98:D3:31:F5:AB:E2
-Access the "agent_ws" folder and launch the following commands:
  colcon build
  source install/setup.bash
  ros2 run micro ros agent micro ros agent serial -b 9600 --dev /dev/rfcomm0
-After running these commands, ROS2 functions initialization starts. It might be needed to press the reset button on the board to make the iniziliation start correctly.
