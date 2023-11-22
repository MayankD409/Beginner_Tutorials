# ROS2 Beginner_Tutorials
## ROS2 Pub/Sub Package:

This ROS2 package demonstrates a simple publisher-subscriber architecture using the ROS2 middleware. The package contains a publisher node (`talker`) and a subscriber node (`listener`) that communicate over the `topic` topic.

### Assumptions/Dependencies

- This package assumes that you have ROS2 installed on your system.
- The code is written in ROS2 Humble.
- Both the publisher and subscriber nodes (`talker` and `listener`) will run simultaneously in different terminals.

### Build and Run Steps

1. **Clone the ROS2 Package into an Existing Workspace:**

    ```bash
    cd path/to/existing/ros2_ws/src
    git clone https://github.com/MayankD409/beginner_tutorials.git
    cp -a beginner_tutorials/cpp_srvcli ./
    ```

2. **Build the ROS2 Package:**

    ```bash
    colcon build
    ```

3. **Source the ROS2 Workspace:**

    ```bash
    source install/setup.bash
    ```

4. **Run the Publisher (`talker`) Node:**

    ```bash
    ros2 run cpp_srvcli talker
    ```

5. **Run the Subscriber (`listener`) Node:**

    ```bash
    ros2 run cpp_srvcli listener
    ```

6. **Call the Service using ros2 service Command-Line Tool:**

    ```bash
    ros2 service call /change_string cpp_srvcli/srv/talkerr "{new_string: 'YourNewString'}"
    ```

7. **Use Launch File for Convenience:**

    ```bash
    ros2 launch cpp_srvcli service.launch.py publish_freq:=590
    ```

    This will automatically run both the `talker` and `listener` nodes.

#### Inspecting TF Frames

To inspect TF frames, you can use the following command:

```bash
ros2 run tf2_tools view_frames.py
```

#### ROS2 Bag

ROS2 bag is used to record the data published by the talker node. To record the data the launch file can be run with `record_bag` parameter.
```bash
ros2 launch cpp_srvcli service.launch.py publish_freq:=590 record_bag:=1
```

On running this command the recorded data will be saved in the root of your workspace in a folder called `my_bag`. To launch the talker node without recording the bag file use the following command.

```bash
ros2 launch cpp_srvcli service.launch.py publish_freq:=590 record_bag:=0
```
To check if the bag files have been recorded properly
```bash
# Check whether the bag has been written to correctly:
  ros2 bag info my_bag
```
To play the recorded data use the following command:
```bash
ros2 bag play my_bag
ros2 run cpp_srvcli listener
```

#### Testing

To run the ROS2 Integration tests use the following command:

```bash
# Go to the root of the Workspace
cd ros2_ws/
colcon test
# View the results of the tests:
cat log/latest_test/cpp_srvcli/stdout_stderr.log
```

### Code Explanation

#### `publisher_member_function.cpp`

- `MinimalPublisher` class creates a publisher node that publishes messages to the `topic` topic.
- `timer_callback()` function is called at regular intervals and publishes a message with an incrementing count.
- The publisher node is created using `create_publisher()` and a timer using `create_wall_timer()`.

#### `subscriber_member_function.cpp`

- `MinimalSubscriber` class creates a subscriber node that subscribes to the `topic` topic.
- `topic_callback()` function is called whenever a new message is received on the `topic` topic, and it prints the received message.

### Service Explanation

- These files demonstrate the interaction with a service (`change_string`), allowing dynamic modification of the message content.
- The `talker` node sends a service request to change the message content.
- The `listener` node receives the updated message and prints it to the console.

### Additional Notes

- Ensure that your ROS2 environment is properly set up before building and running the package.
- Check the ROS2 documentation for troubleshooting and additional information: [ROS2 Documentation](https://docs.ros.org/en/galactic/index.html)

Feel free to modify and extend this package according to your specific use case and requirements. Happy coding!
