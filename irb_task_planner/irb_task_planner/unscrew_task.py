"""Unscrew task planner and dispatcher node."""

import networkx as nx
import rclpy
import unified_planning as up  # type: ignore
from rclpy.node import Node
from up_esb.plexmo import PlanDispatcher  # type: ignore

from irb_task_planner.action_clients import (
    ChangeToolClient,
    DetectScrewClient,
    DropScrewClient,
    GoToClient,
    UnscrewClient,
)
from irb_task_planner.problem import *  # pylint: disable=unused-wildcard-import, wildcard-import


class UnscrewTaskRunner(Node):
    """Task runner for unscrew task."""

    def __init__(self):
        """Initialize the task runner."""
        super().__init__("unscrew_task_runner")

        # Setup action clients
        RobotArm.set_change_tool_client(ChangeToolClient(self))
        RobotArm.set_detect_screw_client(DetectScrewClient(self))
        RobotArm.set_drop_screw_client(DropScrewClient(self))
        RobotArm.set_goto_client(GoToClient(self))
        RobotArm.set_unscrew_client(UnscrewClient(self))

        self.problem, self.bridge = define_problem()
        self.dispatcher = PlanDispatcher()
        self.plan = None

        self.run_task()

    def get_plan(self):
        """Plan the task."""
        self.plan = self.bridge.solve(self.problem, planner_name="pyperplan")

        print("\033[94m" + "*" * 30 + "\033[0m")
        print("\033[94m* Plan *\033[0m")
        print(self.plan)
        print("\033[94m" + "*" * 30 + "\033[0m")

    @staticmethod
    def execute_graph(graph: nx.DiGraph):
        """Execute the graph."""
        for _, node in dict(graph.nodes(data=True)).items():
            if node["node_name"] in ["start", "end"]:
                continue
            context = node["context"]
            executor = context[node["action"]]
            if node["node_name"] in ["start", "end"]:
                continue
            print("\033[38;2;255;165;0m" + f"Executing {node['node_name']}" + "\033[0m")

            parameters = node["parameters"]
            executor(**parameters)
            time.sleep(1)

    def run_task(self):
        """Run the task."""
        self.get_plan()

        graph_executor = self.bridge.get_executable_graph(self.plan)
        self.execute_graph(graph_executor)

        print("\033[92m" + "*" * 30 + "\033[0m")
        print("\033[92m* Task completed *\033[0m")
        print("\033[92m" + "*" * 30 + "\033[0m")


def main(args=None):
    """Run the unscrew task planner and dispatcher node."""
    up.shortcuts.get_environment().credits_stream = None
    rclpy.init(args=args)

    unscrew_task_runner = UnscrewTaskRunner()

    rclpy.spin(unscrew_task_runner)

    unscrew_task_runner.destroy_node()


if __name__ == "__main__":
    main()
