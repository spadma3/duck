# Package `tcp_communication` {#tcp_communication}




This package contains the necessary functions to communicate between different Duckiebots via a variable server. Delay approx. 50ms

## How to use this package

Start a TCP server on any computer / Duckiebot with ROS. We will use the standard IP 192.168.1.222 with the PORT 5678. You are able to change those settings in the coresponding yaml files. Do not forget to bind this IP to the computer on your router.

    make tcp-server

To use the client (the communication on every Duckiebot), you need to add the tcp_communication_client_node to the launch file of your demo (see megacity.launch as example).

Your code needs to import tcp_communication from duckietown_utils

    from duckietown_utils import tcp_communication

tcp_communication contains the following functions:

| purpose      | function definition                                 |
|--------------|-----------------------------------------------------|
| get Variable | object getVariable(string value_name)               |
| set Variable | bool setVariable(string value_name, object value)   |


## Example

The following example uses the communication link:


```python
# Import the TCP communication from duckietown_utils
# Note that the tcp_communication_client_node needs to be running
from duckietown_utils import tcp_communication

def someFunction():
  # Set someVariable1 to someValue. The second argument could have any type
  success = tcp_communication.setVariable("someVariable1", "someValue")

  if success:
      print("someVariable1 successfully set.")
  elif success == "ERROR":
      print("Value of someVariable1 is too long to fit insize BUFFER_SIZE.")

  # Get value of someVariable2
  val = tcp_communication.getVariable("someVariable2")

  if val == None:
      print("Value is not set yet.")
  else:
      print("someVariable2 = " + str(val))

```
