#!/usr/bin/env python3
import socket
import struct
import time

# Constant for converting Unix time (1970) to NTP time (1900)
NTP_DELTA = 2208988800

def system_to_ntp_time(timestamp):
    """
    Convert a system time (Unix epoch) to NTP time.
    Returns a tuple of (seconds, fraction) as integers.
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

# Create and bind the UDP socket on all interfaces using port 12345.
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("0.0.0.0", 12345))
print("NTP server listening on UDP port 12345")

while True:
    data, addr = server.recvfrom(1024)
    if len(data) < 48:
        continue  # Ignore packets that are too short

    # Record the server's receive timestamp (T2)
    recv_time = time.time()
    
    # Unpack the received 48-byte packet.
    # The packet structure (per NTP standard) is:
    #  Bytes  0: LI_VN_Mode
    #         1: Stratum
    #         2: Poll
    #         3: Precision
    #  Bytes  4-7: Root Delay
    #  Bytes  8-11: Root Dispersion
    #  Bytes 12-15: Reference Identifier
    #  Bytes 16-19: Reference Timestamp (seconds)
    #  Bytes 20-23: Reference Timestamp (fraction)
    #  Bytes 24-27: Originate Timestamp (seconds) -- T1 from client
    #  Bytes 28-31: Originate Timestamp (fraction)
    #  Bytes 32-35: Receive Timestamp (seconds)  -- To be filled in (T2)
    #  Bytes 36-39: Receive Timestamp (fraction)
    #  Bytes 40-43: Transmit Timestamp (seconds) -- To be filled in (T3)
    #  Bytes 44-47: Transmit Timestamp (fraction)
    unpacked = struct.unpack("!B B b b 11I", data)
    # Extract the client's T1 (Originate Timestamp) from indices 9 and 10:
    t1_sec = unpacked[9]
    t1_frac = unpacked[10]

    # Compute T2 (server receive time) and T3 (server transmit time) in NTP format.
    t2_sec, t2_frac = system_to_ntp_time(recv_time)
    t3_time = time.time()  # Record just before sending reply
    t3_sec, t3_frac = system_to_ntp_time(t3_time)

    # Build the reply packet.
    # For a server response, set LI=0, VN=3, Mode=4. This makes the first byte 0x1c.
    li_vn_mode = 0x1c
    stratum = 1
    poll = 0
    precision = 0
    root_delay = 0
    root_dispersion = 0
    ref_id = 0
    ref_timestamp_sec = 0
    ref_timestamp_frac = 0

    reply_packet = struct.pack("!B B b b 11I",
                               li_vn_mode, stratum, poll, precision,
                               root_delay, root_dispersion, ref_id,
                               ref_timestamp_sec, ref_timestamp_frac,
                               t1_sec, t1_frac,      # Originate Timestamp (from client)
                               t2_sec, t2_frac,      # Receive Timestamp (T2)
                               t3_sec, t3_frac)      # Transmit Timestamp (T3)
    server.sendto(reply_packet, addr)
