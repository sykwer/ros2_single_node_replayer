import rclpy
import os
import argparse
import yaml
from datetime import datetime


def get_params_file_name(ns, node_name):
    ns_modified = ns
    if ns_modified[0] == '/':
        ns_modified = ns_modified[1:]
    params_file_name = ns_modified + "/" + node_name + ".yaml"
    return  params_file_name.replace("/", "__")


def prepare_output_dir(params_file_name):
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    if not os.path.isdir("output"):
        os.mkdir("output")
    os.chdir("output")

    logdir = datetime.now().strftime("%Y-%m-%d-%H-%M-%S_") + params_file_name[:-len(".yaml")]
    os.mkdir(logdir)
    os.chdir(logdir)


def main(node_name, ns, package_name, executable_name, remapping_file):
    rclpy.init()
    node = rclpy.create_node("node_input_topic_recorder")
    # names = node.get_node_names()
    info = node.get_subscriber_names_and_types_by_node(node_name, ns)

    remappings = {}
    remapping_file_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), remapping_file)
    with open(remapping_file_path, 'r') as f:
        remappings = yaml.safe_load(f)

    params_file_name = get_params_file_name(ns, node_name)
    prepare_output_dir(params_file_name)

    os.system("ros2 param dump " + ns + "/" + node_name)

    record_command = "ros2 bag record"
    for topic_and_types in info:
        topic, types = topic_and_types
        record_command = record_command + " " + topic
    os.system(record_command)

    with open("ros2_run_" + package_name + "_" + executable_name, "w") as f:
        exec_command = "ros2 run " + package_name + " " + executable_name + \
                       " --ros-args --params-file " + params_file_name + \
                       " -r __ns:=" + ns + " -r __node:=" + node_name

        for k, v in remappings.items():
            exec_command = exec_command + " -r " + k + ":=" + v

        f.write(exec_command)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record input topic data for the target node")
    parser.add_argument("package", type=str, help="package name")
    parser.add_argument("executable", type=str, help="executable name")
    parser.add_argument("ns", type=str, help="namespace which target node belongs to")
    parser.add_argument("node", type=str, help="target node name")
    parser.add_argument("remapping", type=str, help="topic remapping yaml file")
    args = parser.parse_args()

    main(args.node, args.ns, args.package, args.executable, args.remapping)
