# ROS 2 time synchronization protocol

This is a ROS 2 node that helps provide time to embedded devices that don't have an RTC or network connection to set their clocks. This is useful for microcontrollers using Pico-ROS connected through serial links.

The protocol itself is patterned after NTP.

## Messages

The protocol is structured as follows:

1. The client sends a synchronization request.
    This is a message of type `std_msgs.msg.String` containing a token value. The default topic is `/sync_time_request`.

1. The server responds with a message of type `sensor_msgs.msg.TimeReference`.
    The `time_ref` value contains the server's timestamp at the time of request arrival. The `source` field contains the token of the original request. The default topic is `/sync_time_response`.

With that response, the client has the four timestamps needed to compute the RTT and time offset as defined by NTP.

## Instalation

This worskpace pulls the service package as a submodule. Clone this workspace as:

```sh
git clone --recurse-submodules https://github.com/xopxe/oruga_ws.git
```

Alternatively, you can update the submodule from inside the cloned repo:

```sh
git submodule init
got submodule update
```

## Client code

Example client code for a microcontroller can be found in [picorosso-espidf](https://github.com/xopxe/picorosso-espidf/blob/main/src/sync_time.cpp).

## Deployment

The reach of this protocol can be limited to the links of interest using Zenoh ACLs.

## Running

If you have a native ROS 2 installation. Start with:

```sh
ros2 run sync_time sync_time_node
```

You can also use the provided Docker image. You can do it directly from VSCode, or manually:

### Build Docker image

From this project directory call:

```sh
docker image build --rm -t ros2_sync_time:jazzy .devcontainer/
docker run -it --user ubuntu -v $PWD:/sync_time_ws \
  ros2_sync_time:jazzy .devcontainer/postCreateCommand.sh
```

### Start Docker image

```sh
docker run --init --rm --user ubuntu \
  --network=host -v $PWD:/sync_time_ws \
  -e ROS_DOMAIN_ID=0 \
  ros2_sync_time:jazzy
```

## Install as `systemd` service

Edit `ros2_sync_time.service` file, set path to this directory in the ExecStart line (just after the `-v`). The file is set up to  start the Docker image. If you are running ROS 2 natively, change the `ExecStart` line. You can also change the `ROS_DOMAIN_ID` to match your deployment.  

Then:

```sh
sudo cp -v ros2_sync_time.service /etc/systemd/system
sudo systemctl enable ros2_sync_time.service
sudo systemctl start ros2_sync_time.service
```

Verify it is working:

```sh
sudo systemctl status ros2_sync_time.service
sudo journalctl -f -u ros2_sync_time.service
```

To uninstall:

```sh
sudo systemctl stop ros2_sync_time.service
sudo systemctl disable ros2_sync_time.service
sudo systemctl daemon-reload
```

## Authors and acknowledgment

<jvisca@fing.edu.uy>

[Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2025

## License

Apache 2.0
