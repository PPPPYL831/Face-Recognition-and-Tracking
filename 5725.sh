#!/bin/bash

# sudo nano /etc/rc.local
# /home/pi/project/5725.sh

cd "$(dirname "$0")"

echo "IP: $1"
export IP=$1

# Stop previous instance
sudo killall --signal SIGINT --wait python3
sudo killall --wait mjpg_streamer

# Start pigpio deamon (to control GPIO)
sudo pigpiod

# Start mjpg-streamer to stream UVC camera.
(
    killall -9 mjpg_streamer
    cd ~/mjpg-streamer-master/mjpg-streamer-experimental
    export LD_LIBRARY_PATH=.
    sudo /etc/init.d/motion stop
    ./mjpg_streamer -i "./input_raspicam.so -x 320 -y 240" -o "./output_http.so -w ./www" &
)

# Start rpi-robot web server
python3 main.py
