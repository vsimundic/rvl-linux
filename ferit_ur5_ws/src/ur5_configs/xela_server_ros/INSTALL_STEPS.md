# Docker start

For each Docker start, you need to do this:

1. Plug in the sensor.
2. Run this:
    ```sh
    ls /dev/ttyUSB* (then check for the ttyUSB)
    slcand -o -s8 -t hw -S 3000000 /dev/ttyUSBX
    ifconfig slcan0 up
    ```
3. On the host machine, you you need this:
    ```sh
    sudo udevadm control --reload-rules
    sudo systemctl daemon-reload
    ```

# Docker Build and Run
For each Docker build and run, you need this:

1. Plug in the sensor.
2. Run this:
    ```sh
    slcand -o -s8 -t hw -S 3000000 /dev/ttyUSBX
    ifconfig slcan0 up
    ```
3. On the host machine, you you need this:
    ```sh
    sudo udevadm control --reload-rules
    sudo systemctl daemon-reload
    ```
4. Run:
    ```sh
    xela_conf -c slcan0
    ```

5. Run:
    ```sh
    export PATH="/xela_suite_linux:/etc/xela:${PATH}"
    ```

5. Edit the `xServ.ini` file located at `/etc/xela/xServ.ini` and add the following content:
    ```ini
    [sensor]
    ...
    rotation = 1
    calibration = on
    ``` 

# Running the Sensor Visualization
When you want to run the sensor visualization, follow these steps:

1. Start the server:
    ```sh
    xela_server -f /etc/xela/xServ.ini
    ```
2. Start the visualization:
    ```sh
    xela_viz -f /etc/xela/xServ.ini
    ```