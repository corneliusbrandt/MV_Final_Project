
# Instructions on how to run project (test)

## Install docker

Install docker from the official webpage by following their tutorials for your specific OS: 
[Docker Install](https://docs.docker.com/engine/install/).

## Utilize Docker Compose to run the program

### (We need to define workspace and name, what volumes do we need?)

Now run the following in the terminal of your choice:

1. ```bash
    sudo docker compose build
    ```

2. ```bash
    sudo docker compose up -d
    ```

3. ```bash
    sudo docker compose exec gesturebot bash
    ```

### What do these commands do?

1. Builds (or rebuilds) the Docker Image service defined in the Docker Compose file

2. Pulls the image into the compose commands and starts the container. `-d` makes the container run in a fetached mode, making it run in the background instead of taking up the entire terminal. To check the running services use `sudo docker compose ps` or to follow the output logs use `sudo docker compose logs`. 

3. This command is used to run a specific command inside a running container managed by Docker Compose. This activates an interactive shell from within the running container which can run ROS2 commands. 

## Run commands from project Docker Image 

```bash
cd ~/gesturebot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Finished?

Simply run the following to shut down the container:

```bash
sudo docker compose down
```
