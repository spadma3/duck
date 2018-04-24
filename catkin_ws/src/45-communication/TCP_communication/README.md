# Package `tcp_communication` {#tcp_communication}




This package contains the necessary functions to communicate between different Duckiebots via a variable server.

## How to use this package

Start a TCP server on any computer / Duckiebot with ROS. We will use the standard IP 192.168.1.222 with the PORT 5678. You are able to change those settings in the coresponding yaml files. Do not forget to bind this IP to the computer on your router. 

  make tcp-server

  where ![robot_name] is the name of your Duckiebot.
