import subprocess
from typing import Iterable
import serial
import serial.tools.list_ports

def change_permissions(file_path, permissions, password):
    """ Changes permission of the USB ports to allow automated CAN bus connection. """
    try:
        # Construct the chmod command
        command = ["sudo", "-S", "chmod", permissions, file_path]
        
        # Pass the password via stdin
        subprocess.run(
            command,
            input=f"{password}\n",
            text=True,
            capture_output=True,
            check=True
        )
        
    except subprocess.CalledProcessError as e:
        pass

# Locate the CAN port for USB2CAN or similar adapter
def find_port(descriptors: Iterable[str]):
    """ Finds a CAN connection from USB ports. """
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if any([descriptor in port.description for descriptor in descriptors]):
            return port.device
    return None