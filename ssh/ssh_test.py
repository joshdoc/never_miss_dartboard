#!/usr/bin/env python3
import paramiko
import os
import sys
import time

# ------------- Configuration Options -------------
PI_IP = "172.20.10.8"           # Raspberry Pi IP address
PI_USERNAME = "pi"                # Raspberry Pi username
PI_PASSWORD = "dart"         # Raspberry Pi password
CPP_EXECUTABLE_PATH = "/home/pi/outputfilename"  # Full path to the C++ executable
# ---------------------------------------------------

def run_remote_command():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        print(f"Connecting to {PI_IP} as {PI_USERNAME}...")
        client.connect(PI_IP, username=PI_USERNAME, password=PI_PASSWORD)
        print("Connected successfully!")

        # Determine the directory and executable name.
        exec_dir = os.path.dirname(CPP_EXECUTABLE_PATH)
        exec_name = os.path.basename(CPP_EXECUTABLE_PATH)

        # Build the command to change directory and run the executable.
        command = f'cd "{exec_dir}" && sudo ./{exec_name}'
        print(f"Running command: {command}")

        # Execute the remote command with a pseudo-terminal.
        stdin, stdout, stderr = client.exec_command(command, get_pty=True)

        # Get the underlying channel and set nonblocking mode.
        channel = stdout.channel
        channel.settimeout(0.0)

        # Loop until the command completes.
        while True:
            # Read and print any standard output.
            if channel.recv_ready():
                out = channel.recv(1024).decode('utf-8')
                if out:
                    for line in out.splitlines():
                        print(line)

            # Read and print any error output.
            if channel.recv_stderr_ready():
                err = channel.recv_stderr(1024).decode('utf-8')
                if err:
                    for line in err.splitlines():
                        print(f"[ERROR] {line}", file=sys.stderr)

            # If the command has finished, process any remaining output and break.
            if channel.exit_status_ready():
                while channel.recv_ready():
                    out = channel.recv(1024).decode('utf-8')
                    if out:
                        for line in out.splitlines():
                            print(line)
                while channel.recv_stderr_ready():
                    err = channel.recv_stderr(1024).decode('utf-8')
                    if err:
                        for line in err.splitlines():
                            print(f"[ERROR] {line}", file=sys.stderr)
                break

            # Avoid high CPU usage.
            time.sleep(0.1)

        exit_status = channel.recv_exit_status()
        print(f"Command exited with status: {exit_status}")

    except Exception as e:
        print("An error occurred:", e, file=sys.stderr)
    finally:
        client.close()
        print("SSH connection closed.")

if __name__ == "__main__":
    run_remote_command()
