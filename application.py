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

from cometalib import CometaClient
import time
import sys, getopt
import string
import json
import subprocess

from uuid import getnode as get_mac
from time import gmtime, strftime

from runtime import Runtime

def message_handler(msg, msg_len):
	"""
		The message handler is the Cometa receive callback. 
		Every time the Cometa library receives a message for this device the message_handler is invoked.
	"""
	try:
		c = json.loads(msg)
	except Exception, e:
		print "Error in parsing the message: ", msg
		return "{\"msg\":\"Invalid JSON object.\"}"

	if 'cmd' in c:
		print "Command received: ", c['cmd']	#DEBUG
		command = c['cmd']
		# execute the command in a shell on the device
		out = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE).stdout.read()
		return out
	else:
		print "Invalid command."
		return "{\"msg\":\"Invalid command.\"}"
	
def main(argv):

	Runtime.init_runtime()
	syslog = Runtime.syslog
	config = Runtime.read_config()

	cometa_server = config['cometa_server']
	cometa_port = config['cometa_port']
	application_id = config['cometa_app']
	# if not specified use the machine's MAC address as Cometa device ID
	device_id = Runtime.get_serial()

	# ------------------------------------------------ #
	print "Cometa client started.\r\nParams: cometa_server:", cometa_server, "cometa_port:", cometa_port, "application_id:", application_id, "device_id:", device_id

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
			Send a data event with the device ID and the current time upstream to the Cometa
			server every minute. This is to demonstrate use of the asynchronous data event messages.
			Once a data event message is received by Cometa, the message is propagated to all opened 
			device Websockets. If a Webhook for the application is specified in the Cometa configuration
			file /etc/cometa.conf the message is also posted to the configured Webhook.
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