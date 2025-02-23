# ArctosHumble

This is a ROS 2 Humble Hawksbill implementation of the Arctos robot arm. 

## Getting Started

To get started, clone the repository and source the setup file. This assumes that you are running in an Ubuntu 22.04 x86_64 machine with ROS 2 Humble installed. 



```bash
git clone https://github.com/lukasdante/ArctosHumble.git
echo "source ArctosHumble/arctos_ws/install/setup.bash" >> ~/.bashrc
```

You must also install all the dependencies to run this nodes. Make sure you have Python>=3.9 and pip installed. You should at least have at least 20GB of free space to run the local AI nodes. If you want to run the CAN control nodes only, 
```bash
cd ArctosHumble
sudo apt install portaudio19-dev
pip install -r requirements.txt
```

You can test if all the nodes are working by running the launch files.

```bash
ros2 launch tests launch_tests.py
```


## Contributing



## Known Issues

