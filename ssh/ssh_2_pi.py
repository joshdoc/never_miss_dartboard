#!/usr/bin/env python3
import paramiko
import os
import sys
import time
import threading

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

def run_remote_command(rpi):
    label = rpi["label"]
    ip = rpi["ip"]
    username = rpi["username"]
    password = rpi["password"]
    exec_path = rpi["exec_path"]

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        print(f"[{label}] Connecting to {ip} as {username}...")
        client.connect(ip, username=username, password=password)
        print(f"[{label}] Connected successfully!")

        exec_dir = os.path.dirname(exec_path)
        exec_name = os.path.basename(exec_path)

        command = f'cd "{exec_dir}" && sudo ./{exec_name}'
        print(f"[{label}] Running command: {command}")

        stdin, stdout, stderr = client.exec_command(command, get_pty=True)
        channel = stdout.channel
        channel.settimeout(0.0)

        while True:
            if channel.recv_ready():
                out = channel.recv(1024).decode("utf-8")
                if out:
                    for line in out.splitlines():
                        print(f"[{label}] {line}")

            if channel.recv_stderr_ready():
                err = channel.recv_stderr(1024).decode("utf-8")
                if err:
                    for line in err.splitlines():
                        print(f"[{label} ERROR] {line}", file=sys.stderr)

            if channel.exit_status_ready():
                while channel.recv_ready():
                    out = channel.recv(1024).decode("utf-8")
                    if out:
                        for line in out.splitlines():
                            print(f"[{label}] {line}")
                while channel.recv_stderr_ready():
                    err = channel.recv_stderr(1024).decode("utf-8")
                    if err:
                        for line in err.splitlines():
                            print(f"[{label} ERROR] {line}", file=sys.stderr)
                break

            time.sleep(0.1)

        exit_status = channel.recv_exit_status()
        print(f"[{label}] Command exited with status: {exit_status}")

    except Exception as e:
        print(f"[{label} ERROR] {e}", file=sys.stderr)

    finally:
        client.close()
        print(f"[{label}] SSH connection closed.")

def main():
    threads = []
    for rpi in RPIS:
        thread = threading.Thread(target=run_remote_command, args=(rpi,))
        thread.start()
        threads.append(thread)

    for t in threads:
        t.join()

if __name__ == "__main__":
    main()
