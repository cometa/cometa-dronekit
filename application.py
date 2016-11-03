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

from dronekit import connect, VehicleMode, LocationGlobal

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
               {'name':'home_location','function':_get_home_location},
               {'name':'set_home_location','function':_set_home_location}
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
    ret,id = check_rpc_msg(req)
    if not ret:
        if id and isanumber(id):
            return JSON_RPC_INVALID_PARAMS_FMT_NUM % id
        if id and isinstance(id, str):
            return JSON_RPC_INVALID_PARAMS_FMT_STR % id
        else:
            return JSON_RPC_PARSE_ERROR

    syslog("Received JSON-RPC message: %s" % msg, escape=True)

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
# Utility functions
def check_rpc_msg(req):
    ret = False
    id = None
    k = req.keys()
    # check presence of required id attribute
    if 'id' in k:
        id = req['id']
    else:
        return ret, id
    # check object length
    if (len(k) != 4):
        return ret, id
    # check presence of required attributes
    if (not 'jsonrpc' in k) or (not 'method' in k) or (not 'params' in k):
        return ret, id
    # check for version
    if req['jsonrpc'] != "2.0":
        return ret, id
    # valid request
    return True,id

def isanumber(x):
    try:
        int(x)
    except ValueError:
        try:
            float(x)
        except ValueError:
            return False
    return True

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
	ret = com.attach(device_id, "DroneKit")
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