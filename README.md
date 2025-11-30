### Folder composition
10_DOF_ROS_IMU_(A)_Code  # the main folder of the project
│  README.md
│  
├─3d_model # 3d model of the imu
│      10 DOF ROS IMU SHELL.step
│      9DOF ROS IMU PCB.step
│      
├─ble_example_code # ble communication example code
│  ├─Linux
│  │      imu_ble.py
│  │      README.md
│  │      
│  └─Windows
│          imu_ble.py
│          README.md
│          
├─init_mag_code # magnetic field calibration code
│      imu_init.py
│      README.md
│         
├─init_acc_code # accelerometer calibration code
│      imu_init.py
│      README.md
│      
├─ros_example_code # ros communication example code
│  ├─ros1 # ros1 noetic communication example code
│  │  └─imu
│  │      │  CMakeLists.txt
│  │      │  package.xml
│  │      │  README.md
│  │      │  
│  │      ├─include
│  │      │  └─imu
│  │      │          imu_cmd.h
│  │      │          
│  │      ├─launch
│  │      │      imu_view.launch
│  │      │      
│  │      ├─rviz
│  │      │      imu_view.rviz
│  │      │      
│  │      └─src
│  │              imu_cmd.cpp
│  │              imu_node.cpp
│  │              
│  ├─ros2 #ros2 humble communication example code
│  │  └─imu
│  │      │  CMakeLists.txt
│  │      │  package.xml
│  │      │  README.md
│  │      │  
│  │      ├─include
│  │      │  └─imu
│  │      │          imu_cmd.h
│  │      │          
│  │      ├─launch
│  │      │      imu_view.launch.py
│  │      │      
│  │      ├─rviz
│  │      │      imu_view.rviz
│  │      │      
│  │      └─src
│  │              imu_cmd.cpp
│  │              imu_node.cpp
│  │              
│  └─serial #ros2 humble serial port library
│      │  CMakeLists.txt
│      │  Makefile
│      │  package.xml
│      │  README.md
│      │  
│      ├─images
│      │      install_Serial_ubuntu.png
│      │      
│      ├─include
│      │  └─serial
│      │      │  serial.h
│      │      │  v8stdint.h
│      │      │  
│      │      └─impl
│      │              unix.h
│      │              win.h
│      │              
│      └─src
│          │  serial.cc
│          │  
│          └─impl
│              │  unix.cc
│              │  win.cc
│              │  
│              └─list_ports
│                      list_ports_linux.cc
│                      list_ports_osx.cc
│                      list_ports_win.cc
│                  
└─uart_example_code # uart communication example code
        imu_uart.py 
        imu_uart_gui.py
        README.md