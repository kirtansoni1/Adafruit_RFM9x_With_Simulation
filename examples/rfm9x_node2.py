# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Example to send a packet periodically between addressed nodes
# Author: Jerry Needell
#
import time
import board
import busio
import digitalio
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--simulate", action="store_true", help="Run in simulated mode")
args = parser.parse_args()

# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

if args.simulate:
    import time
    import sys
    import os
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from simulated_rfm9x import SimulatedRFM9x
    rfm9x = SimulatedRFM9x(frequency=RADIO_FREQ_MHZ)
else:
    import adafruit_rfm9x
    # Define pins connected to the chip.
    CS = digitalio.DigitalInOut(board.CE1)
    RESET = digitalio.DigitalInOut(board.D25)

    # Initialize SPI bus.
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

    # Initialze RFM radio
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# set node addresses
rfm9x.node = 2
rfm9x.destination = 1

if args.simulate:
    rfm9x.initialize() # Must be called to set the attributes while simulation!

# initialize counter
counter = 0
# send a broadcast message from my_node with ID = counter
rfm9x.send(bytes("startup message from node {} ".format(rfm9x.node), "UTF-8"))

# Wait to receive packets.
print("Waiting for packets...")
# initialize flag and timer
time_now = time.monotonic()
while True:
    # Look for a new packet: only accept if addresses to my_node
    packet = rfm9x.receive(with_header=True)
    # If no packet was received during the timeout then None is returned.
    if packet is not None:
        # Received a packet!
        # Print out the raw bytes of the packet:
        print("Received (raw header):", [hex(x) for x in packet[0:4]])
        print("Received (raw payload): {0}".format(packet[4:]))
        print("Received RSSI: {0}".format(rfm9x.last_rssi))
        # send reading after any packet received
        counter = counter + 1
        # after 10 messages send a response to destination_node from my_node with ID = counter&0xff
        if counter % 10 == 0:
            time.sleep(0.5)  # brief delay before responding
            rfm9x.identifier = counter & 0xFF
            rfm9x.send(
                bytes(
                    "message number {} from node {} ".format(counter, rfm9x.node),
                    "UTF-8",
                ),
                keep_listening=True,
            )
