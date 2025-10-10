
# Instructions on how to run project (test)

## Install docker
    
Install docker from the official webpage by following their tutorials for your specific OS: 
[Docker Install]{https://docs.docker.com/engine/install/}.

## Utilize Docker Compose to run the program

First things first, make sure the project ROS2 workspace is in a folder named `gesturebot\_ws`.
This is what will be mounted in the 'volumes' directive!

Now run the following in the terminal of your choice:

1. ```console
    user@console:~$ docker-compose build
    ```
2. ```console
    user@console:~$ docker-compose up -d
    ```
3. ```console
    user@console:~$ docker-compose exec gesturebot bash
    ```
4. ```console
    user@console:~$ docker-compose exec gesturebot bash
    ```


