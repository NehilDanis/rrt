import numpy as np
from shapely import LineString, box
import matplotlib.pyplot as plt

from typing import Tuple, List, Optional

class Node:
    def __init__(self, x: float, y: float, parent = None):
        self._x: float = x
        self._y: float = y
        self._parent: Node = parent

class RRT:
    def __init__(self, start: Tuple[float, float], goal: Tuple[float, float], map_size: Tuple[float, float], obstacles: List[Tuple[float, float, float, float]] = [], step_size=1.0, max_iter=5000):
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
        return np.hypot(node1._x - node2._x, node1._y - node2._y)
    
    def get_nearest_node(self, rand_node:Node) -> Node:
        return min(self._nodes, key= lambda node: self.distance(node, rand_node))
    
    def steer(self, from_node: Node, to_node: Node) -> Node:
        theta = np.arctan2(to_node._x - from_node._x, to_node._y - from_node._y)
        return Node(
            x = from_node._x + self._step_size * np.cos(theta),
            y = from_node._y + self._step_size * np.sin(theta),
            parent = from_node
            )
    
    def check_collision(self, from_node: Node, to_node: Node) -> bool:
        for obstacle in self._obstacles:
            if(self.is_line_instersect_with_obstacle(from_node._x, from_node._y, to_node._x, to_node._y, obstacle)): return True
        return False

    def is_line_instersect_with_obstacle(self, x_from: float, y_from: float, x_to: float, y_to: float, obstacle: Tuple[float, float, float, float]) -> bool:
        line = LineString([(x_from, y_from), (x_to, y_to)])
        rx, ry, rw, rh = obstacle
        rect = box(rx, ry, rx + rw, ry + rh)
        return line.intersects(rect)
    
    def generate_path(self, end_node: Node) -> List[Tuple[float, float]]: 
        path = [(self._goal._x, self._goal._y)]
        node = end_node
        while(node):
            path.append((node._x, node._y))
            node = node._parent
        return path[::-1]


    def plan(self) -> Optional[List[Tuple[float, float]]]:
        for _ in range(self._max_iter):
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(rand_node)
            # from the nearest node move in the direction of the random node, as much as the step size
            new_node = self.steer(nearest_node, rand_node)
            # now check for collision while going from nearest_node to the new node
            if(not self.check_collision(nearest_node, new_node)):
                self._nodes.append(new_node)
                if self.distance(new_node, self._goal) < self._step_size:
                    return self.generate_path(new_node)
        return None

def plot_rrt(rrt: RRT, path=None):
    plt.figure(figsize=(8, 8))
    for ox, oy, w, h in rrt._obstacles:
        plt.gca().add_patch(plt.Rectangle((ox, oy), w, h, color='gray'))

    for node in rrt._nodes:
        if node._parent:
            plt.plot([node._x, node._parent._x], [node._y, node._parent._y], "-g")

    if path:
        px, py = zip(*path)
        plt.plot(px, py, '-r', linewidth=2)

    plt.plot(rrt._start._x, rrt._start._y, "bo", label="Start")
    plt.plot(rrt._goal._x, rrt._goal._y, "ro", label="Goal")
    plt.xlim(0, rrt._map_width)
    plt.ylim(0, rrt._map_height)
    plt.grid(True)
    plt.legend()
    plt.title("Basic RRT Path Planning")
    plt.show()

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