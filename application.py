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
    out = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE).stdout.read()
    return '\n' + out.decode()

def _video_devices(params):
	vdevices = Runtime.list_camera_devices()
	ret = {}
	ret['devices'] = vdevices[0]
	ret['names'] = vdevices[1]
	return ret
# 	return str(vdevices).decode()

def _get_vehicle_attributes(params):
	ret = {}
	ret['attitude'] = vehicle.attitude.__dict__
	ret['location'] = vehicle.location.__dict__
	return ret

def _get_vehicle_parameters(params):
	return vehicle.parameters.__dict__['_attribute_cache']

rpc_methods = ({'name':'shell','function':_shell}, 
#               {'name':'status','function':show_status}, 
               {'name':'video_devices','function':_video_devices}, 
               {'name':'vehicle_attributes','function':_get_vehicle_attributes},
               {'name':'vehicle_parameters','function':_get_vehicle_parameters},
#               {'name':'set_schedule','function':set_schedule}
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
        int(a)
    except ValueError:
        try:
            float(a)
        except ValueError:
            return False
    return True

# --------------------
# 
# Entry point

from dronekit import connect, VehicleMode

config = Runtime.read_config()

if config['sitl']:
	import dronekit_sitl
	vsitl = dronekit_sitl.start_default()

vehicle = connect(config['connection_string'], wait_ready=True)

def main(argv):

	# Get some vehicle attributes (state)
	print "Get some vehicle attribute values:"
	print " GPS: %s" % vehicle.gps_0
	print " Battery: %s" % vehicle.battery
	print " Last Heartbeat: %s" % vehicle.last_heartbeat
	print " Is Armable?: %s" % vehicle.is_armable
	print " System status: %s" % vehicle.system_status.state
	print " Mode: %s" % vehicle.mode.name    # settable

	cometa_server = config['cometa_server']
	cometa_port = config['cometa_port']
	application_id = config['cometa_app']
	# use the machine's MAC address as Cometa device ID
	device_id = Runtime.get_serial()

	# ------------------------------------------------ #
	print "Cometa client started.\r\ncometa_server:", cometa_server, "\r\ncometa_port:", cometa_port, "\r\napplication_id:", application_id, "\r\ndevice_id:", device_id

	# Instantiate a Cometa object
	com = CometaClient(cometa_server, cometa_port, application_id)
	# Set debug flag
	# com.debug = True

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
		time.sleep(60)
		now = strftime("%Y-%m-%d %H:%M:%S", gmtime())
		msg = "{\"id\":\"%s\",\"time\":\"%s\"}" % (device_id, now)

		if com.send_data(msg) < 0:
			print "Error in sending data."
		else:
			if com.debug:
				print "sending data event.", msg

	print "***** should never get here"

if __name__ == "__main__":
    main(sys.argv[1:])