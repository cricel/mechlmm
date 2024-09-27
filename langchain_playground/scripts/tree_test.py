import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import py_trees
import time

# ROS2 Node
class CommandListener(Node):
    def __init__(self, tree):
        super().__init__('command_listener')
        self.subscription = self.create_subscription(String, 'command', self.listener_callback, 10)
        self.behavior_tree = tree
        self.behavior_tree.setup(timeout=15)

        # Initialize blackboard and set default value for 'command'
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.command = None  # Set a default value for the command key

    def listener_callback(self, msg):
        py_trees.console.logdebug(f"Received command: {msg.data}")
        if msg.data == "1" or msg.data == "2":
            self.blackboard.command = msg.data
        else:
            py_trees.console.logwarn(f"Unknown command received: {msg.data}")

    def run_tree(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            py_trees.console.logdebug("Ticking the behavior tree")
            self.behavior_tree.tick()

# Behavior Tree Nodes
class Action1(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action1"):
        super(Action1, self).__init__(name)
        # Attach the blackboard client to this node
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="command", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="action1_status", access=py_trees.common.Access.WRITE)

    def update(self):
        if self.blackboard.command == "1":
            py_trees.console.logdebug("Executing Action1")
            # Simulating success or failure
            success = self.simulate_action()
            if success:
                py_trees.console.logdebug("Action1 succeeded")
                self.blackboard.action1_status = "success"
                return py_trees.common.Status.SUCCESS
            else:
                py_trees.console.logdebug("Action1 failed")
                self.blackboard.action1_status = "failure"
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE

    def simulate_action(self):
        time.sleep(2)  # Simulate time delay
        return True  # For this example, always succeed

class SubAction1(py_trees.behaviour.Behaviour):
    def __init__(self, name="SubAction1"):
        super(SubAction1, self).__init__(name)
        # Attach the blackboard client to this node
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="action1_status", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.action1_status == "success":
            py_trees.console.logdebug("Executing SubAction1")
            success = self.simulate_sub_action()
            if success:
                py_trees.console.logdebug("SubAction1 succeeded")
                return py_trees.common.Status.SUCCESS
            else:
                py_trees.console.logdebug("SubAction1 failed")
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE

    def simulate_sub_action(self):
        time.sleep(2)
        return True  # For this example, always succeed

class Action2(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action2"):
        super(Action2, self).__init__(name)
        # Attach the blackboard client to this node
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="command", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.command == "2":
            py_trees.console.logdebug("Executing Action2")
            time.sleep(2)  # Simulate action
            py_trees.console.logdebug("Action2 succeeded")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

# Main Function
def main(args=None):
    rclpy.init(args=args)

    # Set py_trees logging level to DEBUG
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # Behavior Tree setup
    idle = py_trees.composites.Selector(name="Idle", memory=False)

    action_1 = Action1()
    sub_action_1 = SubAction1()
    action_2 = Action2()

    # Subtree for action_1 and sub_action_1
    sequence_1 = py_trees.composites.Sequence(name="Sequence1", memory=False)
    sequence_1.add_children([action_1, sub_action_1])

    idle.add_children([sequence_1, action_2])

    # Create a behavior tree with idle state as root
    behavior_tree = py_trees.trees.BehaviourTree(root=idle)

    # ROS2 Node initialization
    node = CommandListener(behavior_tree)

    py_trees.console.logdebug("Starting the behavior tree execution")
    
    # Run the behavior tree and listen for commands
    node.run_tree()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
