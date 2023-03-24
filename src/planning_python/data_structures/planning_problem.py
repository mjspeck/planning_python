"""Struct to define a planning problem which will be solved by a planner"""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Dict, Optional

    from planning_python.cost_functions.cost_function import CostFunction
    from planning_python.environment_interface.env_2d import Env2D
    from planning_python.heuristic_functions.heuristic_function import HeuristicFunction
    from planning_python.state_lattices.state_lattice import StateLattice
    from planning_python.utils.types import Node


class PlanningProblem:
    def __init__(self, params: Dict):
        self.initialized: bool = False
        self.params = params

    def initialize(
        self,
        env: Optional[Env2D] = None,
        lattice: Optional[StateLattice] = None,
        cost: Optional[CostFunction] = None,
        heuristic: Optional[HeuristicFunction] = None,
        start_n: Optional[Node] = None,
        goal_n: Optional[Node] = None,
        visualize: bool = False,
    ):
        self.env = env
        self.lattice = lattice
        self.cost = cost
        self.heuristic = heuristic
        self.start_n = start_n
        self.goal_n = goal_n
        self.visualize = visualize
        self.initialized = True
        print("Planning Problem Initialized")

    def reset_env(self, env: Env2D):
        """Given the same lattice, cost, heuristic and params, reset the underlying environment"""
        self.env = env

    def reset_heuristic(self, heuristic: HeuristicFunction):
        """Reset the heuristic function being used"""
        self.heuristic = heuristic
