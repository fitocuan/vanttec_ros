# Copyright 2017, Digi International Inc.
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from digi.xbee.devices import XBeeDevice
import json
import time

#****************************************************************************************#
# TODO: Replace with the serial port where your local module is connected to.
PORT = "/dev/ttyUSB0" #La estacion
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 9600
REMOTE_NODE_ID = "vtecboat" #El remoto es el bote, Esta es la estacion
#****************************************************************************************#


def main():
    print(" +--------------------------------------+")
    print(" |           Sender (Station)           |")
    print(" +--------------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)
    try:
        device.open()
        # Obtain the remote XBee device from the XBee network.
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)

        commActive = True
        if remote_device is None:
            print("Could not find the remote device")
            exit(1)


        while commActive :
            print(remote_device)
            action = input("action: ")
            device.send_data_async(remote_device, action)
            time.sleep(1)
            xbee_message = device.read_data()

            if xbee_message is not None:
                #Print the message and 
                message = xbee_message.data.decode()
                print("Received Message: " , message)
            #print("Success")
            if action == "exit":
                commActive = False

    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    main()