#!/usr/bin/env python3
import socket
import struct
import time

# Constant to convert between Unix and NTP epochs.
NTP_DELTA = 2208988800

def system_to_ntp_time(timestamp):
    """
    Convert a system time (Unix epoch) to NTP time.
    Returns a tuple (seconds, fraction).
    """
    ntp_time = timestamp + NTP_DELTA
    seconds = int(ntp_time)
    fraction = int((ntp_time - seconds) * (2**32))
    return seconds, fraction

def ntp_to_system_time(seconds, fraction):
    """
    Convert NTP time to system time.
    """
    return seconds - NTP_DELTA + float(fraction) / (2**32)

# Replace <server-ip> with the IP address of your NTP server (the other RPi).
server_address = ("<server-ip>", 12345)

# Create a UDP socket.
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client.settimeout(2)

print("Starting NTP client. Sending requests to {}:{}".format(server_address[0], server_address[1]))

while True:
    try:
        # Record client transmit time (T1) and convert to NTP format.
        t1 = time.time()
        t1_sec, t1_frac = system_to_ntp_time(t1)

        # Build a 48-byte NTP request packet.
        # For a client request, LI=0, VN=3, Mode=3. This makes the first byte 0x1b.
        # The Originate Timestamp field is set with T1. All other timestamp fields are zero.
        packet = struct.pack("!B B b b 11I",
                             0x1b, 0, 0, 0,
                             0, 0, 0,
                             0, 0,
                             t1_sec, t1_frac,   # Originate Timestamp (T1)
                             0, 0,              # Receive Timestamp (to be filled by server)
                             0, 0)              # Transmit Timestamp (to be filled by server)

        client.sendto(packet, server_address)
        data, addr = client.recvfrom(1024)
        t4 = time.time()  # Client receive time (T4)

        if len(data) < 48:
            print("Received an invalid packet")
            continue

        # Unpack the reply packet (same structure as on the server).
        unpacked = struct.unpack("!B B b b 11I", data)
        # Extract timestamps:
        # T1 (Originate) is at indices 9 and 10.
        # T2 (Receive) is at indices 11 and 12.
        # T3 (Transmit) is at indices 13 and 14.
        r_t1_sec, r_t1_frac = unpacked[9], unpacked[10]
        r_t2_sec, r_t2_frac = unpacked[11], unpacked[12]
        r_t3_sec, r_t3_frac = unpacked[13], unpacked[14]

        # Convert timestamps back to system time.
        T1 = ntp_to_system_time(r_t1_sec, r_t1_frac)
        T2 = ntp_to_system_time(r_t2_sec, r_t2_frac)
        T3 = ntp_to_system_time(r_t3_sec, r_t3_frac)
        T4 = t4

        # Compute round-trip delay and offset.
        # Round-trip delay d = (T4 - T1) - (T3 - T2)
        # Offset = ((T2 - T1) + (T3 - T4)) / 2
        round_trip_delay = (T4 - T1) - (T3 - T2)
        offset = ((T2 - T1) + (T3 - T4)) / 2

        print("RTT: {:.3f} ms, Offset: {:.3f} ms".format(round_trip_delay * 1000, offset * 1000))
        time.sleep(1)  # Adjust the frequency of requests as needed.
    except socket.timeout:
        print("Request timed out.")
        time.sleep(1)
