#!/usr/bin/env python
"""
Author: Marco Graziano
"""
__license__ = """
Copyright 2016 Visible Energy Inc. All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import time
import sys, getopt
import string
import json
import subprocess
from uuid import getnode as get_mac
from time import gmtime, strftime

from runtime import Runtime
from cometalib import CometaClient

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import utils

# JSON-RPC errors
JSON_RPC_PARSE_ERROR = '{"jsonrpc": "2.0","error":{"code":-32700,"message":"Parse error"},"id": null}'
JSON_RPC_INVALID_REQUEST = '{"jsonrpc": "2.0","error":{"code":-32600,"message":"Invalid Request"},"id":null}'

JSON_RPC_METHOD_NOTFOUND_FMT_STR = '{"jsonrpc":"2.0","error":{"code": -32601,"message":"Method not found"},"id": %s}'
JSON_RPC_METHOD_NOTFOUND_FMT_NUM = '{"jsonrpc":"2.0","error":{"code": -32601,"message":"Method not found"},"id": %d}'
JSON_RPC_INVALID_PARAMS_FMT_STR = '{"jsonrpc":"2.0","error":{"code": -32602,"message":"Method not found"},"id": %s}'
JSON_RPC_INVALID_PARAMS_FMT_NUM = '{"jsonrpc":"2.0","error":{"code": -32602,"message":"Method not found"},"id": %d}'
JSON_RPC_INTERNAL_ERROR_FMT_STR = '{"jsonrpc":"2.0","error":{"code": -32603,"message":"Method not found"},"id": %s}'
JSON_RPC_INTERNAL_ERROR_FMT_NUM = '{"jsonrpc":"2.0","error":{"code": -32602,"message":"Method not found"},"id": %d}'

# shortcut to refer to the system log in Runtime
Runtime.init_runtime()
syslog = Runtime.syslog

# --------------------
# 
# RPC Methods

# @info Start a subprocess shell to execute the specified command and return its output.
#
# {"jsonrpc":"2.0","method":"shell","params":["/bin/cat /etc/hosts"],"id":1}
#
def _shell(params):
    # check that params is a list
	if not isinstance(params, list) or len(params) == 0:
		return "Parameter must be a not empty list"    
	command = params[0]
	try:
		subprocess.check_call(command,shell=True)
		out = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE).stdout.read()
		return '\n' + out.decode()
	except Exception, e:
		print e
		return "{\"msg\":\"Invalid command.\"}"

def _video_devices(params):
	vdevices = Runtime.list_camera_devices()
	ret = {}
	ret['devices'] = vdevices[0]
	ret['names'] = vdevices[1]
	return ret

def _get_autopilot_attributes(params):
	ret = {}
	ret['firmware'] = {'version':"%s" % vehicle.version, 'major':vehicle.version.major, 'minor':vehicle.version.minor, 'patch':vehicle.version.patch}
	ret['release'] = {'type': vehicle.version.release_type(), 'version': vehicle.version.release_version(), 'stable': vehicle.version.is_stable()}
	ret['capabilities'] = vehicle.capabilities.__dict__
	return ret

global attribute_names
attribute_names = ('attitude','location','velocity','gps','gimbal','battery','ekf_ok','last_heartbeat','rangefinder','heading','armable','state','groundspeed','airspeed','mode','armed')

def _get_vehicle_attributes(params):
	ret = {}
	ret['attitude'] = vehicle.attitude.__dict__
	f = vehicle.location.global_frame
	r = vehicle.location.global_relative_frame
	l = vehicle.location.local_frame
	ret['location'] = {'global':{'lat':f.lat,'lon':f.lon,'alt':f.alt}, 'relative':{'lat':r.lat,'lon':r.lon,'alt':r.alt},'local':{'north':l.north,'east':l.east,'down':l.down}}
	ret['velocity'] = vehicle.velocity
	ret['gps'] = vehicle.gps_0.__dict__
	ret['gimbal'] = {'pitch' : vehicle.gimbal._pitch, 'yaw': vehicle.gimbal._yaw, 'roll': vehicle.gimbal._roll}
	ret['battery'] = vehicle.battery.__dict__
	ret['ekf_ok'] = vehicle.ekf_ok
	ret['last_heartbeat'] = vehicle.last_heartbeat
	ret['rangefinder'] =  vehicle.rangefinder.__dict__
	ret['heading'] = vehicle.heading
	ret['armable'] = vehicle.is_armable
	ret['state'] =vehicle.system_status.state
	ret['groundspeed'] = vehicle.groundspeed
	ret['airspeed'] = vehicle.airspeed
	ret['mode'] = vehicle.mode.name
	ret['armed'] = vehicle.armed
	return ret

"""
Expects an object such as {'armed': True} or {'airspeed':3.2} or {'mode':'GUIDED'}
"""
def _set_vehicle_attributes(params):
    if type(params) is not dict:
        return {"success": False}
    allowed = ('armed','airspeed','groundspeed','mode')
    for x in params:
        if x not in allowed:
            return {"success": False}        
	if 'armed' in params.keys():
		vehicle.armed = params['armed']
	if 'airspeed' in params.keys():
		vehicle.airspeed = params['airspeed']
	if 'groundspeed' in params.keys():
		vehicle.groundspeed = params['groundspeed']
	if 'mode' in params.keys():
		vehicle.mode = VehicleMode(params['mode'])
	return {"success": True}

def _get_vehicle_parameters(params):
	return vehicle.parameters.__dict__['_attribute_cache']

"""
Expects params as  {'key':'THR_MIN', 'value': 100}
"""
def _set_vehicle_parameters(params):
	if ('key' and 'value') not in params.keys():
		return {"success": False}
	vehicle.parameters[params['key']] = params['value']
	return {"success": True}

def _get_home_location(params):
	# Get Vehicle Home location - will be `None` until first set by autopilot
	while not vehicle.home_location:
	    cmds = vehicle.commands
	    cmds.download()
	    cmds.wait_ready()
	    if not vehicle.home_location:
	        pass
	# TODO set a timeout
	# We have a home location.
	return vehicle.home_location.__dict__

"""
Expects params as {'lat':-35.3, 'alt':584, 'lon':149.1}
"""
def _set_home_location(params):
	if ('lat' and 'lon' and 'alt') not in params.keys():
		return {"success": False} 
	vehicle.home_location = LocationGlobal(lat=params['lat'],lon=params['lon'],alt=params['alt'])
	return {"success": True}

"""
Expects an object like {'period':5} 
"""
def _set_telemetry_period(params):
	if type(params) is not dict or 'period' not in params.keys():
		return {"success": False}
	if params['period'] <= 0:
		return {"success": False}
	config['app_params']['event_loop'] = params['period']
	return {"success": True}

"""
Set the telemetry params to a new tuple. Pass an array of attribute names to be compatible with JSON/RPC.
"""
def _set_telemetry_attributes(params):
    if type(params) is not list:
        return {"success": False}
    tparams = tuple(params)
    allowed = ('attitude','location','velocity','battery','state','groundspeed','airspeed','mode','armed')
    for x in tparams:
        if x not in allowed:
            return {"success": False}
    telemetry_attributes_names = tparams
    return {"success": True}

"""
Vehicle takeoff to the specified altitude.
{"alt": 4.3}
"""
def _takeoff(params):
	if type(params) is not dict:
		return {"success": False}
	if ('alt') not in params.keys():
		return {"success": False}
	vehicle.simple_takeoff(params['alt'])

"""
Moves the vehicle to the absolute or relative position specified.
{"lat": -34.2, "lon": 149.2, "alt" = 3.0, "relative": true}
"""
def _goto(params):
	if type(params) is not dict:
		return {"success": False}
	if ('lat' and 'lon' and 'alt' and 'relative') not in params.keys():
		return {"success": False}
	if relative:
		dest = LocationGlobalRelative(lat, lon, alt)
	else:
		dest = LocationGlobal(params['lat'], params['lon'], params['alt'])
	vehicle.simple_goto(dest)
	return {"success": True}

"""
Moves the vehicle to the destination North and East meters of the current position.
{"dNorth": 1.5, "dEast": 10.2}
"""
def _goto_destination(params):
	if type(params) is not dict:
		return {"success": False}
	if ('dNorth' and 'dEast') not in params.keys():
		return {"success": False}
	curLocation = vehicle.location.global_relative_frame
	dest = utils.get_location_meters(curLocation, dNorth, dEast)
	vehicle.simple_goto(dest)
	return {"success": True}

"""
Moves the vehicle to the global position specified.
{"lat": -34.2, "lon": 149.2, "alt" = 3.0}
"""
def _goto_position_target_global_int(params):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    if type(params) is not dict:
		return {"success": False}
    if ('lat' and 'lon' and 'alt') not in params.keys():
		return {"success": False}    
    lat = params['lat']
    lon = params['lon']
    alt = params['alt']
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        lat * 1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        lon * 1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    return {"success": True}

"""
Moves the vehicle to the global position specified.
{"north": -34.2, "east": 149.2, "down" = 3.0}
"""
def _goto_position_target_local_ned(params):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    if type(params) is not dict:
		return {"success": False}
    if ('north' and 'east' and 'down') not in params.keys():
		return {"success": False}  
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    return {"success": True}

"""
 Point vehicle at a specified heading (in degrees) relative to the direction or absolut according to the specified value.
 {"heading": 45.0, relative": false}
"""
def _condition_yaw(params):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if type(params) is not dict:
		return {"success": False}
    if ('heading' and 'relative') not in params.keys():
		return {"success": False} 
    if params['relative']:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        params['heading'],    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    return {"success": True}

"""
Point the camera at the global position specified.
{"lat": -34.2, "lon": 149.2, "alt" = 3.0}
"""
def _point_camera(params):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see: 
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    if type(params) is not dict:
		return {"success": False}
    if ('lat' and 'lon' and 'alt') not in params.keys():
		return {"success": False}    
    lat = params['lat']
    lon = params['lon']
    alt = params['alt']
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        lat,
        lon,
        alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    return {"success": True}

"""
Move vehicle in direction based on specified velocity vectors in the local frame.
{"velocity_x": 2.3, "velocity_y": 5.0, "velocity_z":0.2}
"""
def _send_ned_velocity(params):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    if type(params) is not dict:
        return {"success": False}
    if ('velocity_x' and 'velocity_y' and 'velocity_z') not in params.keys():
        return {"success": False}     
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        params['velocity_x'], params['velocity_y'], params['velocity_z'], # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    return {"success": True}

"""
Move vehicle in direction based on specified velocity vectors in the global frame.
{"velocity_x": 2.3, "velocity_y": 5.0, "velocity_z":0.2}
"""
def _send_global_velocity(params):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    if type(params) is not dict:
        return {"success": False}
    if ('velocity_x' and 'velocity_y' and 'velocity_z') not in params.keys():
        return {"success": False}    
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    return {"success": True}

global rpc_methods
rpc_methods = ({'name':'shell','function':_shell}, 
#               {'name':'status','function':show_status}, 
               {'name':'video_devices','function':_video_devices}, 
               {'name':'vehicle_attributes','function':_get_vehicle_attributes},
			   {'name':'set_attributes','function':_set_vehicle_attributes},
               {'name':'autopilot_attributes','function':_get_autopilot_attributes},
               {'name':'vehicle_parameters','function':_get_vehicle_parameters},
			   {'name':'set_parameters','function':_set_vehicle_parameters},
			   {'name':'set_telemetry_period','function':_set_telemetry_period},
               {'name':'set_telemetry_attributes','function':_set_telemetry_attributes},
               {'name':'home_location','function':_get_home_location},
               {'name':'set_home_location','function':_set_home_location},
               {'name':'takeoff','function':_takeoff},
               {'name':'goto','function':_goto},
               {'name':'goto_destination','function':_goto_destination},
               {'name':'goto_position_global_int','function':_goto_position_target_global_int},
               {'name':'goto_position_local_ned','function':_goto_position_target_local_ned},
               {'name':'condition_yaw','function':_condition_yaw},
               {'name':'point_camera','function':_point_camera},
               {'name':'send_ned_velocity','function':_send_ned_velocity},
               {'name':'send_global_velocity','function':_send_global_velocity},
)

def message_handler(msg, msg_len):
    """
    The generic message handler for Cometa receive callback.
    Invoked every time the Cometa object receives a JSON-RPC message for this device.
    It returns the JSON-RPC result object to send back to the application that sent the request.
    The rpc_methods tuple contains the mapping of names into functions.
    """
    #pdb.set_trace()
    try:
        req = json.loads(msg)
    except:
		# the message is not a json object
		syslog("Received JSON-RPC invalid message (parse error): %s" % msg, escape=True)
		return JSON_RPC_PARSE_ERROR

    # check the message is a proper JSON-RPC message
    ret,id = utils.check_rpc_msg(req)
    if not ret:
        if id and utils.isanumber(id):
            return JSON_RPC_INVALID_PARAMS_FMT_NUM % id
        if id and isinstance(id, str):
            return JSON_RPC_INVALID_PARAMS_FMT_STR % id
        else:
            return JSON_RPC_PARSE_ERROR

    syslog("JSON-RPC: %s" % msg, escape=True)

    method = req['method']
    func = None
    # check if the method is in the registered list
    for m in rpc_methods:
        if m['name'] == method:
            func = m['function']
            break

    if func == None:
        return JSON_RPC_INVALID_REQUEST

    # call the method
    try:
        result = func(req['params'])
    except Exception as e:
        print e
        return JSON_RPC_INTERNAL_ERROR_FMT_STR % str(id)

    # build the response object
    reply = {}
    reply['jsonrpc'] = "2.0"
    reply['result'] = result
    reply['id'] = req['id']

    return json.dumps(reply)

global telemetry_attributes_names
telemetry_attributes_names = ('attitude','location','velocity','battery','state','groundspeed','airspeed','mode','armed')

def get_telemetry():
	attr = _get_vehicle_attributes(None)
	ret = {}
	for k in telemetry_attributes_names:
		ret[k] = attr[k]
	return ret

# --------------------
# 
# Entry point

def main(argv):
	global config
	config = Runtime.read_config()

	if config['sitl']:
		import dronekit_sitl
		global vsitl 
		vsitl = dronekit_sitl.start_default()

	print "\nConnecting to vehicle at: %s" % (config['connection_string'])

	global vehicle 
	vehicle = connect(config['connection_string'], wait_ready=True)
	vehicle.wait_ready('autopilot_version')

	# Get some vehicle attributes (state)
	print " GPS: %s" % vehicle.gps_0
	print " Battery: %s" % vehicle.battery
	print " Is Armable?: %s" % vehicle.is_armable
	print " System status: %s" % vehicle.system_status.state
	print " Mode: %s" % vehicle.mode.name

	cometa_server = config['cometa']['server']
	cometa_port = config['cometa']['port']
	application_id = config['cometa']['app_key']
	# use the machine's MAC address as Cometa device ID
	device_id = Runtime.get_serial()

	# ------------------------------------------------ #
	print "Cometa client started.\r\ncometa_server:", cometa_server, "\r\ncometa_port:", cometa_port, "\r\napplication_id:", application_id, "\r\ndevice_id:", device_id

	# Instantiate a Cometa object
	com = CometaClient(cometa_server, cometa_port, application_id, config['cometa']['ssl'])
	# Set debug flag
	com.debug = config['app_params']['debug']

	# Bind the message_handler() callback. The callback is doing the function of respoding
	# to remote requests and handling the core part of the work of the application.
	com.bind_cb(message_handler)

	# Attach the device to Cometa.
	ret = com.attach(device_id, "%s" % vehicle.version)
	if com.error != 0:
		print "(FATAL) Error in attaching to Cometa.", com.perror()
		sys.exit(2)

	# When attach is successful the server returns an object of the format:
	# {"msg":"200 OK","heartbeat":60,"timestamp":1441405206}
	try:
		ret_obj = json.loads(ret)
	except Exception, e:
		print "(FATAL) Error in parsing the message returned after attaching to Cometa. Message:", ret
		sys.exit(2)

	print "Device [%s] attached to Cometa. Server timestamp: %d" % (device_id, ret_obj['timestamp'])
	if com.debug:
		print "Server returned:", ret

	# Application main loop.
	while True:
		"""
		Send a telemetry data event upstream. 
		"""
		time.sleep(config['app_params']['event_loop'])
		msg = get_telemetry()
		msg['id'] = device_id
		msg['time'] = int(time.time())
		msg['type'] = 1

		#now = strftime("%Y-%m-%d %H:%M:%S", gmtime())
		#msg = "{\"id\":\"%s\",\"time\":\"%s\"}" % (device_id, now)
		continue #DEBUG
		if com.send_data(str(msg)) < 0:
			print "Error in sending data."
		else:
			if com.debug:
				print "sending data event.", msg

	print "***** should never get here"

if __name__ == "__main__":
    main(sys.argv[1:])