![alt tag](https://img.shields.io/badge/python-2.7-blue.svg)
# cometa-dronekit
Cloud API for drones and unmanned vehicles

## Overview
Low-latency, secure cloud communication for connected UAVs and ROVs using the Cometa server. [Cometa](http://www.cometa.io) is a two-way message broker for device-to-cloud and cloud-to-device secure communication. It runs on a server in the cloud and it uses HTTPS and secure WebSockets for efficient remote interaction of applications and vehicles.

`cometa-dronekit` is a [DroneKit](http://python.dronekit.io/) script running on a UV that manages the connection to the Cometa server and exposes methods for [JSON-RPC](http://www.jsonrpc.org/specification) remote procedure calls to the vehicle. Applications use the JSON-RPC API documented below to interact with one or many connected UVs using HTTPS and to receive real-time telemetry on secure WebSockets.

This software relies on DroneKit and it runs natively on 3DR Solo drones and on any UV’s [Companion Computer](http://python.dronekit.io/develop/companion-computers.html#supported-companion-computers) such as RPI, Odroid, Edison, and Volta's [4Gmetry](http://4gmetry.voltarobots.com/).

**If you are interested in receiving beta tester credentials and access to a Cometa cloud server for testing the `cometa-dronekit` API send an email to marco@cometarobotics.com**

## Dependencies
```
pip install dronekit
pip install dronekit-sitl
pip install http-parser
pip install pymavlink
```
## Usage
`cometa-dronekit` is a stand-alone application. Configuration is in the `config.json` file.
```
./application.py
```
To run in the background, detach it from the terminal and re-direct output to a log file:
```
nohup python ./application.py > /tmp/dronekit.log 2>&1 &
```

## Connecting to autopilot
DroneKit relies on a ArduPilot APM compatible stack and controller, such as the [Pixhawk](https://pixhawk.org/modules/pixhawk).
The autopilot connection string for the `cometa-dronekit` script is specified in the `config.json` file. To use `cometa-dronekit` with DroneKit's embedded SITL, set the `sitl` parameter to `true` and use the appropriate connection string.

>DroneKit connection strings options are documented in  [Connecting to a Vehicle](http://python.dronekit.io/guide/connecting_vehicle.html)


Cloud API
----------
Once the `cometa-dronekit` application is started on the vehicle, it will connect to the Cometa server and the Application Key specified in the `config.json` file. This connection will be kept open permanently and used for full-duplex communication through the Cometa message broker. Since the connection is initiated from within a NAT or a firewall on stadard port 443, which is typically open for outgoing traffic, the vechicle becomes accessible from an application using the `cometa-dronekit` API without exposing or even knowing its public IP address.

An application sends JSON-RPC requests to a connected vehicle through the Cometa cloud server `send` method.

>To use the Cometa API and the `send` method, an `APPLICATION_KEY` and a `COMETA_SECRET` are needed as credentials for authentication. Users of the Cometa Robotics cloud service can create applications and manage their vehicles in the cloud as well as develop applications using the API provided by `cometa-dronekit`. 

>Public availability of Cometa Robotics cloud service is planned for 1Q 2017. Send an email to marco@cometarobotics.com to request early access.

#### Cometa Authentication
An application is authenticated by including an Authorization HTTPS request header in every request.

| HTTP HEADER                    | DESCRIPTION                 | VALUE                         |
 -------------------------------|-------------------------|-------------
|  `Authorization`              | authorization token      | OAuth {`COMETA_SECRET`}         

Example:

`Authorization: OAuth b4f51e1e6125dca873e5`

### Send RPC Message 

Send a JSON-RPC message to a vehicle.

```
POST /v1/applications/{APPLICATION_KEY}/devices/{DEVICE_ID}/send
``` 
The message containing the JSON-RPC request is in the POST `body`.

| URL PARAMETER        | DESCRIPTION                      | TYPE                            |
 ------------------|----------------------------------|-------------------------------------
| `APPLICATION_KEY` | Cometa Application Key           | String |
| `DEVICE_ID` | Device Cometa Id           | String |

The `DEVICE_ID` is the vehicle ID provided by `cometa-dronekit` when a vehicle is connected to a Cometa server. **The default value is the vehicle's MAC address.**

The `cometa-dronekit` vehicle agent always reply to a JSON-RPC message with a JSON response.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0", "method":"takeoff","params":{"alt":20.0},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{"jsonrpc": "2.0", "result": {"success": true}, "id": 7}
```


### WebSockets Endpoint

The Cometa server exposes a WebSockets endpoint for each vehicle connected to it. A vehicle WebSocket can be opened only once. To obtain an endpoint, an application must request a new WebSockets `DEVICE_KEY` every time is needed using the following HTTPS method:

```
GET /applications/{APPLICATION_KEY}/devices/{DEVICE_ID}/websocket
```

 URL PARAMETER        | DESCRIPTION                      | TYPE                            |
 ------------------|----------------------------------|-------------------------------------
| `APPLICATION_KEY` | Cometa Application Key           | String |
| `DEVICE_ID` | Device Cometa Id           | String |

The method returns a `DEVICE_KEY` that is used to obtain the WebSocket vehicle's endpoint as follows:

`wss://{COMETA_HOST}:{PORT}/websockets/{DEVICE_ID}/{DEVICE_KEY}`


Example:
```
$ curl -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/websocket

{
    "device_id":"e984060007",
    "device_key":"dc670dae876ee4f919de9e777c9bd98a5e182cd8"
}
```
WebSocket endpoint (one-time use only):

    wss://dronekit.cometa.io/v1/websockets/e984060007/dc670dae876ee4f919de9e777c9bd98a5e182cd8

Opening a device WebSocket would fail if the vehicle is not connected. Upon vehicle disconnection, the WebSocket is closed by the server after the inactivity timeout period. **Immediately after opening a WebSocket, and without any other request, an application starts receiving telemetry messages at expiration of every period of the duration indicated in the `config.json` file.**

>On an open WebSocket an application receives both telemetry messages without requesting them, and responses to JSON-RPC requests.

WebSockets are asynchronous, full-duplex channels to exchange messages directly between an application and a remote vehicle running `dronekit-cometa`. A WebSocket `send()` is relaying the message to the vehicle the same way as an HTTPS `send`. From the vehicle's standpoint messages are received the same way regardless the method used by an application, that is using WebSockets `send()` or a Cometa HTTPS `send`. **On an open WebSocket an application receives both telemetry messages without requesting them, and responses to JSON-RPC requests.**

> Before sending a message to a WebSocket an application should always check its `readyState` attribute to check the WebSocket connection is open and ready to communicate (`readyState === WS_OPEN`).

### Connected Vehicles

Get a list of vehicle connected to the Cometa server. 
```
GET /v1/applications/{APPLICATION_KEY}/devices
``` 

The message containing the JSON-RPC request is in the POST `body`.

| URL PARAMETER        | DESCRIPTION                      | TYPE                            |
 ------------------|----------------------------------|-------------------------------------
| `APPLICATION_KEY` | Cometa Application Key           | String |

Example:
```
$ curl -H 'Authorization: OAuth b4f51e1e6125dcc873e9' -H 'Content-type: application/json' \
    http://dronekit.cometa.io/v1/applications/a0353b75b8fa61889d19/devices

{
    "num_devices": 11, 
    "devices": [
        "cc79cf45f1b4", 
        "cc79cf45f1d4", 
        "cc79cf45f421", 
        "cc79cf45f1ba", 
        "cc79cf45f400", 
        "cc79cf45f401", 
        "cc79cf45f2ab", 
        "cc79cf45f307", 
        "cc79cf45f221", 
        "cc79cf45f314", 
        "cc79cf45f2d8"
    ]
}
```

### Cometa Vehicle Presence

Get vehicle connection state and statistics information from the Cometa server. 
```
GET /v1/applications/{APPLICATION_KEY}/devices/{DEVICE_ID}
``` 

The message containing the JSON-RPC request is in the POST `body`.

| URL PARAMETER        | DESCRIPTION                      | TYPE                            |
 ------------------|----------------------------------|-------------------------------------
| `APPLICATION_KEY` | Cometa Application Key           | String |
| `DEVICE_ID` | Device Cometa Id           | String |

Example:
```
$ curl -H 'Authorization: OAuth b4f51e1e6125dcc873e9' -H 'Content-type: application/json' \
    http://dronekit.cometa.io/v1/applications/a0353b75b8fa61889d19/devices/e984060007

{
    "device_id": "e984060007", 
    "ip_address": "73.202.12.128:64471", 
    "heartbeat": "1478378638", 
    "info": "APM:Copter-3.3.0", 
    "connected_at": "1478373655", 
    "latency": "45", 
    "websockets": "1", 
    "net": {
        "received": {
            "bytes": "4237", 
            "messages": "12"
        }, 
        "sent": {
            "bytes": "34789", 
            "messages": "781"
        }
    }
}
```
**Latency is in milliseconds and indicates the average time of a round-trip from the server to the vehicle (message/response). Latencies of less than 100 msec for a round-trip are typical for vehicles connected in the US.**
>Vehicle connections are maintained by a regular heartbeat message sent by `cometa-dronekit` to the server (60 seconds period). The Cometa server times out and disconnects a vehicle 90 seconds after receiving the last heartbeat message. A disconnected vehicle may appear to be connected for up to an additional 90 seconds, if it disconnect abruptly without cleanly close its socket connection. 

## Vehicle Attributes and Parameters

DroneKit exposes most state information (position, speed, etc.) through vehicle Attributes, while vehicle settings are in vehicle Parameters. 

Vehicle parameters provide the information used to configure the autopilot for the vehicle-specific hardware/capabilities. The available parameters for each platform are documented in the ArduPilot wiki: [Copter Parameters](http://ardupilot.org/copter/docs/parameters.html), [Plane Parameters](http://plane.ardupilot.com/wiki/arduplane-parameters/), [Rover Parameters](http://ardupilot.org/rover/docs/parameters.html).

The JSON-RPC methods below allow applications to access and when allowed, set the Attributes and Parameters values.

>[List of vehicle Attributes and Parameters.](http://python.dronekit.io/guide/vehicle_state_and_parameters.html)

### Get Veichle Attributes
 `vehicle_attributes`

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"vehicle_attributes","params":{},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "airspeed": 0, 
        "gimbal": {
            "yaw": null, 
            "roll": null, 
            "pitch": null
        }, 
        "battery": {
            "current": 0, 
            "voltage": 12.587, 
            "level": 100
        }, 
        "ekf_ok": true, 
        "state": "STANDBY", 
        "armable": true, 
        "attitude": {
            "yaw": -0.41401541233062744, 
            "roll": -0.0011191728990525007, 
            "pitch": -0.0010953218443319201
        }, 
        "rangefinder": {
            "distance": null, 
            "voltage": null
        }, 
        "location": {
            "relative": {
                "lat": 37.423455, 
                "alt": 0.01, 
                "lon": -122.1763939
            }, 
            "global": {
                "lat": 37.423455, 
                "alt": 40, 
                "lon": -122.1763939
            }, 
            "local": {
                "down": null, 
                "east": null, 
                "north": null
            }
        }, 
        "groundspeed": 0, 
        "velocity": [
            0.01, 
            -0.02, 
            0
        ], 
        "mode": "STABILIZE", 
        "last_heartbeat": 0.3592782229970908, 
        "armed": false, 
        "heading": 336, 
        "gps": {
            "epv": 65535, 
            "fix_type": 3, 
            "eph": 121, 
            "satellites_visible": 10
        }
    }, 
    "id": 7
}
```

### Set Veichle Attributes
`set_attributes`

Writable attributes are: `armed`, `airspeed`, `groundspeed`, and `mode`. The method allows for one or more name/value pairs to be set at a time.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"set_attributes","params":{"mode":"GUIDED","groundspeed":3.2},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}    
```

### Get Autopilot Attributes
`autopilot_attributes`

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"autopilot_attributes","params":{},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send
{
    "jsonrpc": "2.0", 
    "result": {
        "release": {
            "version": 0, 
            "type": "rc", 
            "stable": true
        }, 
        "firmware": {
            "major": 3, 
            "version": "APM:Copter-3.3.0", 
            "minor": 3, 
            "patch": 0
        }, 
        "capabilities": {
            "mission_float": true, 
            "ftp": false, 
            "command_int": false, 
            "param_float": true, 
            "set_actuator_target": false, 
            "param_union": false, 
            "terrain": true, 
            "set_attitude_target_local_ned": true, 
            "mission_int": false, 
            "set_attitude_target": false, 
            "set_altitude_target_global_int": true, 
            "compass_calibration": false, 
            "flight_termination": true
        }
    }, 
    "id": 7
}
```

### Get Vehicle Parameters
`vehicle_parameters`

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"vehicle_parameters","params":{},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "RC7_REV": 1, 
        "GPS_INJECT_TO": 127, 
        ... 
            omitted total of 525 vehicle parameters
        ...
        "SIM_FLOW_DELAY": 0, 
        "BATT_CURR_PIN": 12
    }, 
    "id": 7
}    
```

### Set Vehicle Parameters
`set_parameters`

Set one vehicle parameter, using an object with a `key` and a `value` attribute.
Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"set_parameters","params":{"key":"THR_MIN","value":100},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send
    
{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}    
```

## Telemetry Settings
Telemetry messages on a WebSockets start as soon as the vehicle WebSocket is open. The initial period in seconds is specified in the `app_params` object in the `config.json` file.
```
"app_params":{
    "debug":false,
    "telemetry_period":1
}
```
Attributes that are initially in the message by default are:
* attitude
* location
* velocity
* groundspeed
* airspeed
* battery
* mode
* armed
* state

**Example of telemetry message:**

```
{
    'type': 1,
    'id': 'e984060007',    
    'time': 1478386938, 
    'battery': {
        'current': 0.0, 
        'voltage': 12.587, 
        'level': 100
    }, 
    'groundspeed': 0.0, 
    'state': 'STANDBY', 
    'airspeed': 0.0, 
    'attitude': {
        'yaw': -0.20035716891288757, 
        'roll': -0.006740430369973183, 
        'pitch': -0.0021510079968720675
    }, 
    'location': {
        'relative': {
            'lat': 37.423455, 
            'alt': 0.0, 
            'lon': -122.1763939
        }, 
        'global': {
            'lat': 37.423455,  
            'alt': 39.99, 
            'lon': -122.1763939
        }, 
        'local': {
            'down': None, 
            'east': None, 
            'north': None
        }
    },  
    'velocity': [0.02, -0.13, -0.01], 
    'armed': False, 
    'mode': 'STABILIZE'
}
```

### Set Telemetry Attributes
`set_telemetry_attributes`

Set the attributes in the array passed as parameter to be included in the telemetry message. Attributes can be one of the following: `'attitude','location','velocity','groundspeed','airspeed','battery','mode','armed','state'`

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"set_telemetry_attributes","params":["attitude","location"],"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send
    
{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}    
```
After receiving the method above, the telemetry message from the vehicle becomes as follows:
```
{
    'type': 1,
    'id': 'e984060007', 
    'time': 1478389239,
    'attitude': {
        'yaw': -0.4880271553993225, 
        'roll': 0.006115585565567017, 
        'pitch': 0.003535753581672907
    }, 
    'location': {
        'relative': {
            'lat': 37.423455, 
            'alt': 0.07, 
            'lon': -122.1763939
        }, 
        'global': {
            'lat': 37.423455, 
            'alt': 40.06, 
            'lon': -122.1763939
        }, 
        'local': {
            'down': None, 
            'east': None, 
            'north': None
        }
    }
}
```

### Set Telemetry Period
`set_telemetry_period`

Set the period of telemetry messages in seconds.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"set_telemetry_period","params":{"period":5},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send
    
{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}    
```

## Home Location
The Home location is set when a vehicle first gets a good location fix from the GPS. The location is used as the target when the vehicle does a “return to launch”.
### Get Home Location
`home_location`

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"home_location","params":{},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send
    
{
    "jsonrpc": "2.0", 
    "result": {
        "lat": 37.42345428466797, 
        "local_frame": null, 
        "alt": 39.9900016784668, 
        "global_frame": null, 
        "lon": -122.17639923095703
    }, 
    "id": 7
}
```
### Set Home Location
`set_home_location`

Set the vehicle's home location to the specified global location object.
The latitude and longitude are relative to the WGS84 coordinate system. The altitude in meters is relative to mean sea-level (MSL).

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"set_home_location","params":{"lat":-34.3,"lon":149.23,"alt":33},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}    
```
## Guiding and Controlling Vehicle
Methods to control vehicle movement and to fly/drive vehicles autonomously without a predefined a mission. They allow an application, for instance a Ground Control Station (GCS), to control the vehicle “on the fly” and react to new events or situations as they occur.

### Takeoff
`takeoff`

Vehicle takes off to the specified altitude in meters.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"takeoff","params":{"alt":12.0},"id":7},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}    
```

### Goto Position
`goto`

Move the vehicle to the specified position. The new position can be either in the absolute or relative  to the home position, depending on the value of the 'relative' attribute.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"goto","params":{"lat":-32.2,"lon":149.2 "alt":3.0,"relative":false},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

### Goto Destination
`goto_destination`

Move the vehicle to the specified destination North and East of the current position. The North and East distances are in meter.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"goto_destination","params":{"dNorth":1.5,"dEast":29.3},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

### Goto Global Destination
`goto_position_global_int`

Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system. This method sends  a `SET_POSITION_TARGET_GLOBAL_INT` MAVLink message to the autopilot, which is used to directly specify the target location of the vehicle. As used in `cometa-dronekit`, this method is effectively the same as `goto`.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"goto_position_global_int","params":{"lat":-32.2,"lon":149.2,"alt":3.0},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

### Goto Local Destination
`goto_position_local_ned`

Goto the specified location in NED co-ordinates relative to the home location or the vehicle itself.
This method sends a `SET_POSITION_TARGET_LOCAL_NED` MAVLink message  to the autopilot, which is used to directly specify the target location in the North, East, Down frame. In the NED frame positive altitudes are entered as negative “Down” values. So if down is “10”, this will be 10 metres below the home altitude.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"goto_position_local_ned","params":{"north":1.5,"east":29.3,"down":-2},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

### Set Velocity Local Frame
`send_ned_velocity`

Controlling vehicle movement using velocity. This method sends a `SET_POSITION_TARGET_LOCAL_NED` MAVLink message to the autopilot, which is used to directly specify the speed components of the vehicle in the `MAV_FRAME_LOCAL_NED` frame (relative to home location). 

>From Copter 3.3 the vehicle will stop moving if a new message is not received in approximately 3 seconds. Prior to Copter 3.3 the message only needs to be sent once, and the velocity remains active until the next movement command is received.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"send_ned_velocity","params":{"velocity_x":1.5,"velocity_y":2.3,"velocity_z":0.2},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

### Set Velocity Global Frame
`send_global_velocity`

Controlling vehicle movement using velocity. This method sends a `SET_POSITION_TARGET_GLOBAL_INT` MAVLink message to the autopilot, which is used to directly specify the speed components of the vehicle in the NED frame.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"send_global_velocity","params":{"velocity_x":1.5,"velocity_y":2.3,"velocity_z":0.2},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

## Mission Control
DroneKit provides basic methods to download and clear the current mission commands from the vehicle, to add and upload new mission commands, and to start a mission. These methods allow applications to remotely create high-level mission planning functionality.

### New Mission
`new_mission`

Reset the existing waypoints and start a new mission. This method must be called before setting any waypoint for a mission.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"new_mission","params":{},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

### Add Waypoint to Mission
`add_mission_item`

Add a mission item, such as a waypoint object to a mission. This method encodes an item command with 14 parameters in an array.

The mission item can be either in x, y, z meters (type: LOCAL), or x:lat, y:lon, z:alt. Local frame is Z-axis down, right handed (NED). Global frame is Z-axis up, right handed (ENU). See [http://qgroundcontrol.org/mavlink/waypoint_protocol](http://qgroundcontrol.org/mavlink/waypoint_protocol)

Parameters:
* `target_system` - This can be set to any value (DroneKit changes the value to the MAVLink ID of the connected vehicle before the command is sent).

* `target_component` - The component id if the message is intended for a particular component within the target system (for example, the camera). Set to zero (broadcast) in most cases. 

* `seq` - The sequence number within the mission (the autopilot will reject messages sent out of sequence). This should be set to zero as the API will automatically set the correct value when uploading a mission. 

* `frame` - The frame of reference used for the location parameters (x, y, z). In most cases this will be `mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT`, which uses the WGS84 global coordinate system for latitude and longitude, but sets altitude as relative to the home position in metres (home altitude = 0). For more information see http://ardupilot.org/planner/docs/common-mavlink-mission-command-messages-mav_cmd.html#frames-of-reference.

* `command`- The specific mission command (e.g. ``mavutil.mavlink.MAV_CMD_NAV_WAYPOINT``). The supported commands (and command parameters are listed on http://ardupilot.org/planner/docs/common-mavlink-mission-command-messages-mav_cmd.html.

* `current` - Set to zero (not supported). 

* `autocontinue` - Set to zero (not supported). 

* `param1` - Command specific parameter (depends on specific `Mission Command (MAV_CMD) http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd.

* `param2` - Command specific parameter.

* `param3` - Command specific parameter.
* 
* `param4` - Command specific parameter.

* `x` - (param5) Command specific parameter used for latitude (if relevant to command). 

* `y` - (param6) Command specific parameter used for longitude (if relevant to command).

* `z` (param7) Command specific parameter used for altitude (if relevant). The reference frame for altitude depends on the "frame". 

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"add_mission_item","params":[0, 0, 0, 3, 22, 0, 0, 0, 0, 0, 0, 0, 0, 10],"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```
### Start Mission
`start_mission`

Start current flight plan. Uploads all the commmands added with `add_mission_items` and set the vehicle mode to `AUTO` to start the mission.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"start_mission","params":{},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

## Streaming Video
The Cometa Robotics cloud service includes a video management platform to ingest, record and playback live and recorded video. It uses the open source [ffmpeg](http://www.ffmpeg.org/) streamer and the RTMP protocol to ingest video from vehicles. The `ffmpeg` streaming software must be available on the vehicle's Companion Computer.
>Cloud video management is at the moment in private beta and some methods are not implemented in the available version of `cometa-dronekit`. However, the methods documented below are available for immediate use.

**If you are interested in testing the video streaming API send an email to marco@cometarobotics.com**

### Set Vehicle Yaw
`condition_yaw`

Point the vehicle at a specific heading (in degrees) relative to the direction or as asbsolute value.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"condition_yaw","params":{"heading":353.0, "relative":false},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

### Point Camera to ROI
`point_camera`

Point the camera at the specified global position.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"point_camera","params":{"lat":34.1, "lon": -129.2, "alt": 3.0},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "success": true
    }, 
    "id": 7
}
```

### Get Video Devices
`video_devices`

List connected V4L video devices.

Example:
```
$ curl -X POST -H 'Authorization: OAuth a724dc4811d507688' -H 'Content-type: application/json' \
    -d '{"jsonrpc":"2.0","method":"video_devices","params":{},"id":7}' \
    https://dronekit.cometa.io/v1/applications/a94660d971eca2879/devices/e984060007/send

{
    "jsonrpc": "2.0", 
    "result": {
        "names": [
            "Logitech, Inc. HD Pro Webcam C920", 
        ], 
        "devices": [
            "/dev/video0"
        ]
    }, 
    "id": 7
}
```
