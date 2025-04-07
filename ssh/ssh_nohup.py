#!/usr/bin/env python3
import paramiko
import os

# ----------- Configuration for Raspberry Pis -----------

RPIS = [
    {
        "label": "PI1",
        "ip": "172.20.10.7",
        "username": "pi",
        "password": "dart",
        "exec_path": "/home/pi/udp_test"
    },
    {
        "label": "PI2",
        "ip": "172.20.10.8",
        "username": "pi",
        "password": "dart",
        "exec_path": "/home/pi/udp_test"
    }
]

# -------------------------------------------------------

def run_in_background(rpi):
    label = rpi["label"]
    ip = rpi["ip"]
    username = rpi["username"]
    password = rpi["password"]
    exec_path = rpi["exec_path"]

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        print(f"[{label}] Connecting to {ip}...")
        client.connect(ip, username=username, password=password)
        print(f"[{label}] Connected.")

        exec_dir = os.path.dirname(exec_path)
        exec_name = os.path.basename(exec_path)

        command = f'cd "{exec_dir}" && nohup sudo ./{exec_name} > output.log 2>&1 &'
        print(f"[{label}] Launching background process...")
        client.exec_command(command)
        print(f"[{label}] Command sent. Disconnecting.")

    except Exception as e:
        print(f"[{label} ERROR] {e}")

    finally:
        client.close()
        print(f"[{label}] SSH connection closed.")

def main():
    for rpi in RPIS:
        run_in_background(rpi)

if __name__ == "__main__":
    main()
