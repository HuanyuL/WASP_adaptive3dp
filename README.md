# WASP Crane Web Service
This is the repository of Octoprint interface built in ROS for WASP crane. The node provide ROS service of control parameter(print speed and extrusion speed), subscriptoion for printing status. 

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Installation
**Supported Operating Systems:** Ubuntu 20.04 and Ubuntu 22.04
### Docker Engine Installation
This project uses Docker for containerization. If you don't have Docker Engine installed, follow this [link](https://docs.docker.com/engine/install/ubuntu/)
To manage docker as a non-root user, please follow the post-installation steps in this [link](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

Add the following to your bashrc file for ros and the workspace to be sourced when opening a new terminal in dev_ws, make sure your are not sourcing ROS in your bashrc file.
```
if [ -f "/dev_ws/setup.bash" ]; then
    source /dev_ws/setup.bash
fi
```
### Foxglove Studio Installation
If you want to visualize the pointcloud and images from husky robot please install foxglove studio.
```
sudo snap install foxglove-studio
```

### Vscode Extensions
To access the files inside container you need to install **Remote Development** and **Docker** extension in your vscode.
To modify the files in container:
  - click the Docker icon in vscode
  - Right click the container you want to access
  - Attach vscode
  - Open the folder, type /dev_ws/src

## Usage
**Clone this repository to your computer** 
```
git@github.com:HuanyuL/wasp_crane_webservice.git
```
**Build the image**
```
cd wasp_crane_webserivce
.docker/build_image.sh
```
**Run the container**
```
.docker/run_user.sh
```
Once you run the container, you will see the terminal return the request to change the ownership of the folder, copy this line from the terminal
```
sudo chown -R $USER /dev_ws
```
Then run the terminator
```
Terminator
```
**Run the node**
```
roslaunch wasp_crane_web_service web_service.launch
```

**Import the panel json to Foxglove**
Foxglove studio should run in your machine instead of container, chose the ROS1 display and import the template from the ``wasp_crane_monitor.json``

### License
This project is licensed under the [MIT License](./LICENSE).
