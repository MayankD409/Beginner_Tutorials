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
    ros2 launch cpp_srvcli service.launch.py
    ```

    This will automatically run both the `talker` and `listener` nodes.

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
