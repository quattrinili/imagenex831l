# To create udev rules for the ft232h.
rosrun depth_node_py create_udev_rules

# To run the node.
rosrun depth_node_py run_node.sh

# Disable password.
# http://askubuntu.com/questions/147241/execute-sudo-without-password
Add 
afrl ALL=(ALL) NOPASSWD: ALL
