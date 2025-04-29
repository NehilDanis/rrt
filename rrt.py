import numpy as np

from typing import Tuple, List, Optional

class Node:
    def __init__(self, x: float, y: float, parent = None):
        self._x: float = x
        self._y: float = y
        self._parent: Node = parent

class RRT:
    def __init__(self, start: Tuple[float, float], goal: Tuple[float, float], map_size: Tuple[float, float], obstacles: List[Tuple[float, float, float, float]] = [], step_size=1.0, max_iter=500):
        self._start = Node(*start)
        self._goal = Node(*goal)
        self._map_width, self._map_height = map_size
        self._obstacles = obstacles
        self._step_size = step_size
        self._max_iter = max_iter
        self._nodes: List[Node] = [self._start]

    def get_random_node(self) -> Node:
        return Node(x = np.random.uniform(0, self._map_width), y = np.random.uniform(0.0, self._map_height))
    
    def distance(self, node1: Node, node2: Node) -> float:
        return np.hypot(node1.x - node2.x, node1.y - node2.y)
    
    def get_nearest_node(self, rand_node:Node) -> Node:
        return min(self._nodes, key= lambda node: self.distance(node, rand_node))
    
    def steer(self, nearest_node: Node, random_node: Node):
        pass

    def plan(self) -> Optional[List[Tuple[float, float]]]:
        for _ in self._max_iter:
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(rand_node)

            # from the nearest node move in the direction of the random node, as much as the step size

        

        return None


if __name__ == "__main__":
    start = (1, 1)
    goal = (18, 18)

    obstacles = [
        (5, 5, 2, 12),
        (12, 0, 2, 10),
    ]
    map_size = (20, 20)

    rrt = RRT(start, goal, map_size, obstacles)
    path = rrt.plan()
    plot_rrt(rrt, path)