#!/usr/bin/env python3
import paramiko
import sys
import threading

# ----------- Configuration for Raspberry Pis -----------

RPIS = {
    "PI1": {
        "ip": "172.20.10.7",
        "username": "pi",
        "password": "dart",
    },
    "PI2": {
        "ip": "172.20.10.8",
        "username": "pi",
        "password": "dart",
    }
}

# -------------------------------------------------------

def kill_remote_process(label, process_name):
    rpi = RPIS[label]
    ip = rpi["ip"]
    username = rpi["username"]
    password = rpi["password"]

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"[{label}] Connecting to {ip}...")
        # Increase banner_timeout, disable agent and key lookups
        client.connect(ip,
                       username=username,
                       password=password,
                       banner_timeout=200,
                       allow_agent=False,
                       look_for_keys=False)
        print(f"[{label}] Connected.")

        # Build the kill command using 'sudo pkill -f'
        command = f'sudo pkill -f {process_name}'
        print(f"[{label}] Executing: {command}")

        client.exec_command(command, get_pty=True)
        print(f"[{label}] Kill command sent.")

    except Exception as e:
        print(f"[{label} ERROR] {e}")
    finally:
        client.close()
        print(f"[{label}] SSH connection closed.")

def main():
    if len(sys.argv) != 2:
        print("Usage: python kill_process.py <process_name>")
        print("Example: python kill_process.py my_executable")
        sys.exit(1)

    process_name = sys.argv[1]
    threads = []

    for label in RPIS:
        thread = threading.Thread(target=kill_remote_process, args=(label, process_name))
        thread.start()
        threads.append(thread)
    
    for t in threads:
        t.join()

if __name__ == "__main__":
    main()


# -------------------------------------------------------

def kill_remote_process(label, process_name):
    rpi = RPIS[label]
    ip = rpi["ip"]
    username = rpi["username"]
    password = rpi["password"]

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        print(f"[{label}] Connecting to {ip}...")
        client.connect(ip, username=username, password=password)
        print(f"[{label}] Connected.")

        # Build the kill command; using 'sudo pkill -f' to kill by process name
        command = f'sudo pkill -f {process_name}'
        print(f"[{label}] Executing: {command}")

        # Execute the remote kill command.
        # We use get_pty=True if sudo might require a pseudo-terminal.
        client.exec_command(command, get_pty=True)
        print(f"[{label}] Kill command sent.")

    except Exception as e:
        print(f"[{label} ERROR] {e}")
    finally:
        client.close()
        print(f"[{label}] SSH connection closed.")

def main():
    if len(sys.argv) != 2:
        print("Usage: python kill_process.py <process_name>")
        print("Example: python kill_process.py my_executable")
        sys.exit(1)

    process_name = sys.argv[1]
    threads = []

    # Create a thread for each RPi
    for label in RPIS:
        thread = threading.Thread(target=kill_remote_process, args=(label, process_name))
        thread.start()
        threads.append(thread)
    
    for t in threads:
        t.join()

if __name__ == "__main__":
    main()
