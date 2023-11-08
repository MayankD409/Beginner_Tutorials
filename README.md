# ROS2 Beginner_Tutorials
## ROS2 Pub/Sub Package: `cpp_pubsub`

This ROS2 package demonstrates a simple publisher-subscriber architecture using the ROS2 middleware. The package contains a publisher node (`talker`) and a subscriber node (`listener`) that communicate over the `topic` topic.

### Assumptions/Dependencies

- This package assumes that you have ROS2 installed on your system.
- The code is written in ROS2 Humble.
- Both the publisher and subscriber nodes (`talker` and `listener`) will run simultaneously in different terminals.

### Build and Run Steps

1. **Create a ROS2 workspace and clone the repo:**

    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    ros2 pkg create --build-type ament_cmake cpp_pubsub
    cd cpp_pubsub
    git clone https://github.com/MayankD409/beginner_tutorials.git
    cp -a /beginner_tutorials/ . /cpp_pubsub/
    rm -rf beginner_tutorials
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
    ros2 run cpp_pubsub talker
    ```

5. **Run the Subscriber (`listener`) Node:**

    ```bash
    ros2 run cpp_pubsub listener
    ```

The `talker` node will publish messages to the `topic` topic, and the `listener` node will receive and print those messages.

### Code Explanation

#### `publisher_member_function.cpp`

- `MinimalPublisher` class creates a publisher node that publishes messages to the `topic` topic.
- `timer_callback()` function is called at regular intervals and publishes a message with an incrementing count.
- The publisher node is created using `create_publisher()` and a timer using `create_wall_timer()`.

#### `subscriber_member_function.cpp`

- `MinimalSubscriber` class creates a subscriber node that subscribes to the `topic` topic.
- `topic_callback()` function is called whenever a new message is received on the `topic` topic, and it prints the received message.

### Additional Notes

- Ensure that your ROS2 environment is properly set up before building and running the package.
- Check the ROS2 documentation for troubleshooting and additional information: [ROS2 Documentation](https://docs.ros.org/en/galactic/index.html)

Feel free to modify and extend this package according to your specific use case and requirements. Happy coding!
