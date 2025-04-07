#!/usr/bin/env python3
import paramiko
import os
import threading

# ----------- Configuration for Raspberry Pis -----------

RPIS = {

    "PI1":{
        "id": 1,
        "threshold": 60,
        "ip": "172.20.10.7",
        "username": "pi",
        "password": "dart",
        "exec_path": "/home/pi/udp_test"
    },
    "PI2":{
        "id": 2,
        "threshold": 60,
        "ip": "172.20.10.8",
        "username": "pi",
        "password": "dart",
        "exec_path": "/home/pi/udp_test"
    }
    
}

# -------------------------------------------------------

def run_in_background(label):
    rpi = RPIS[label]
    pi_id = rpi["id"]
    threshold = rpi["threshold"]
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

        command = f'cd "{exec_dir}" && nohup sudo ./{exec_name} {threshold} {pi_id} > output_{pi_id}.log 2>&1 &'
        print(f"[{label}] Running: {command}")
        client.exec_command(command)
        print(f"[{label}] Command sent. Disconnecting.")

    except Exception as e:
        print(f"[{label} ERROR] {e}")

    finally:
        client.close()
        print(f"[{label}] SSH connection closed.")

def main():
    threads = []
    for label in RPIS:
        thread = threading.Thread(target=run_in_background, args=(label,))
        thread.start()
        threads.append(thread)

    for t in threads:
        t.join()

if __name__ == "__main__":
    main()
