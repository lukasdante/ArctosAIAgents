import pyudev

def get_camera():
    context = pyudev.Context()
    for device in context.list_devices(subsystem="video4linux"):
        data = device.properties

        if 'GENERAL' in data['ID_V4L_PRODUCT']:

            return device.device_node

print(get_camera())