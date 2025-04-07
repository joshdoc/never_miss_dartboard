import paramiko

# Configuration
pis = [
    {"host": "172.20.10.7", "id": 1},
    {"host": "172.20.10.8", "id": 2}
]
username = "pi"
password = "raspberry"  # Update if needed
remote_binary_path = "./udp_test"  # Adjust path if it's somewhere else

def ssh_run_command(client, command):
    stdin, stdout, stderr = client.exec_command(command)
    stdout_text = stdout.read().decode()
    stderr_text = stderr.read().decode()
    return stdout_text, stderr_text

for pi in pis:
    print(f"\nConnecting to Pi {pi['id']} at {pi['host']}...")

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(pi['host'], username=username, password=password)

    run_cmd = f"{remote_binary_path} &"
    print(f"Running executable: {run_cmd}")
    stdout, stderr = ssh_run_command(ssh, run_cmd)

    if stderr:
        print(f"Error from Pi {pi['id']}:\n{stderr}")
    else:
        print(f"Pi {pi['id']} launched successfully.")

    ssh.close()
