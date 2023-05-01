# ros2_single_node_replayer
A tool to run a single ROS 2 node with the same logic as when the entire system is operating.

## About the tool
This is a tool for running a specific node of a ROS 2 App in isolation.
A key feature of this tool is that it can run a specific node in isolation with the same logic as when the entire ROS 2 App is operating.
This solves problems such as "I want to run a single node for performance analysis, but I need the preceding nodes to operate with the same logic as in production."
As a means of achieving this, the tool saves the parameters when the specified node is launched and records all data entering the input topic in a rosbag.
This allows the same operation to be performed later on the single node.

*Note*: Currently, there is no way to record services and actions in ROS 2 Humble, so this tool cannot record them either.
Therefore, if you'd like to run some node on the client side of a service or action, you may encounter difficulties when running a single node.

## How to use
First, clone the repository.
```shell
git clone https://github.com/sykwer/ros2_single_node_replayer.git
cd ros2_single_node_replayer
```

After launching a whole ROS 2 App, start the tool before playing the rosbag (i.e., launch a ROS 2 App → start the tool → play the rosbag to start the ROS 2 App).
In the terminal where you run the tool, do not forget to run the script that sets the environment variables required for the ROS 2 App to work (i.e., `setup.bash`).

The tool is started in the format below.
```shell
python3 recorder.py <package name> <executable name> <namespace> <node name> <remapping file path>
```

Once the playback of the ROS 2 App's rosbag is complete, stop the operation with Control-C in the terminal of the tool.
An output directory will be created in the tool's directory.
In this output directory, directories in the format `%Y-%m-%d-%H-%M-%S_<namespace>_<nodename>` will be created by the tool.
Each of these directories corresponds to one measurement.

In the directory for a single measurement, two files and one directory are created.
```shell
ls %Y-%m-%d-%H-%M-%S_<namespace>_<nodename>
# ros2_run_<package_name>_<executable_name>
# <namespace>_<nodename>.yaml
# rosbag2_%Y_%m_%d_%H_%M_%S
```

The file in the format `ros2_run_<package name>_<executable name>` is the command to run the single node.
The yaml file contains the parameters at startup.
The directory contains the recorded rosbag.

To run the single node, open two terminals and enter the startup command and rosbag playback command, respectively.
```shell
bash ros2_run_<package_name>_<executable_name>
```
```shell
ros2 bag play rosbag2_%Y_%m_%d_%H_%M_%S
```


Do not forget to source the ROS 2 App's `setup.bash` in each terminal.
Note that if you set, for example, `-r 0.2` when running the whole ROS 2 App to play the rosbag at 1/5 the frequency, the single node operation will run at that same 1/5 frequency without specifying `-r 0.2` when playing the generated rosbag.

## Notice
### Early return due to subscription count
Codes in a ROS 2 App sometimes have a guard at the beginning of the callback like below.
```cpp
if (some_publisher_->get_subscription_count() == 0) return;
```

In the single node operation mechanism, no other nodes subscribe to the topic, so the code will always return at this point.
You will need to remove this part or make necessary adjustments before running the single node.

### Timing issue
Note that topic messages that flow into the target node cannot be recorded between the start of the ROS 2 App and the start of the tool.
If there are any missed important data, the internal state of the node may become unexpected.
At the time of writing this document, it is unclear whether a mechanism to prevent data loss can be implemented, but ideally, no data should be missed.
