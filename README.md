# ROS2 time synchronization protocol.

This a ROS2 node that helps provide time to embedded devices which dont't have a RTC or network connection to set own clocks. This is useful for microcontrollers using Pico-ROS connected through serial links.

The protocol itself is patterned after NTP.

## Messages

The protocol is structured as follows:

1. The client sends a syncronization request.
    This is a message of type `std_msgs.msg.String` containing a token value. The default topic is `sync_time_request`.

1. The server responds with a message of type `sensor_msgs.msg.TimeReference`.
    The `time_ref` value contains the servers time stamp at arrival time of the request. The `source` field contains the token of the orignal request. The default topic is `sync_time_response`.

With that response, the client has the four timestamps needed to compute the RTT and time offset as defined by NTP.

## Client code

Example client code for a microcontroller can be found in [picorosso-espidf](https://github.com/xopxe/picorosso-espidf/blob/main/src/sync_time.cpp).

## Deployment

The reach of this protocol can be limited to the links of interest using Zenoh ACLs.

## Running

You can use native ROS 2 installation or the included Docker. Start with:

```sh
ros2 run sync_time sync_time_node
```

## Authors and acknowledgment

<jvisca@fing.edu.uy>

[Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2025

## License

Apache 2.0
