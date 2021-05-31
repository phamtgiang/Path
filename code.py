from __future__ import annotations
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional
import heapq
import collections
import numpy as np

T = TypeVar('T')

Location = TypeVar('Location')
# Creat abtract class
class Graph(Protocol):
    def neighbors(self, id: Location) -> List[Location]: pass
# Creat Queue
class Queue:
    def __init__(self):
        self.elements = collections.deque()

    def empty(self) -> bool:
        return not self.elements

    def put(self, x: T):
        self.elements.append(x)

    def get(self) -> T:
        return self.elements.popleft()

def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = " * "
    if 'start' in style and id == style['start']: r = " S "
    if 'goal' in style and id == style['goal']:   r = " E "
    if id in graph.walls: r = " # "
    return r

def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)


GridLocation = Tuple[int, int]

class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: List[GridLocation] = []

    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls

    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]
        if (x + y) % 2 == 0: neighbors.reverse()
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

class WeightedGraph(Graph):
    def cost(self, from_id: Location, to_id: Location) -> float: pass

class GridWithWeights(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: Dict[GridLocation, float] = {}

    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        return self.weights.get(to_node, 1)

class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []

    def empty(self) -> bool:
        return not self.elements

    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

def reconstruct_path(came_from: Dict[Location, Location],
                     start: Location, goal: Location) -> List[Location]:
    current: Location = goal
    path: List[Location] = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

# TODO: Heuristic function
# Heuristic function tells us how close we are to the goal.
# On other word, this func tion show distance from node/location n to destination (goal)
# based on GridLocation (the coordinates on the map)

def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

# TODO: A* algorithm
# TODO:frontier (Open list):
# consists on nodes that have been visited but not expanded (meaning that sucessors have not been
# explored yet). This is the list of pending tasks.
# TODO:came_from(Close list):
# consists on nodes that have been visited and expanded (sucessors have been explored already and
# included in the open list, if this was the case).
def a_star_search(graph: WeightedGraph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Location, Optional[Location]] = {}
    cost_so_far: Dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current: Location = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far

# TODO: Test following data with generate random
# Make a map with the coordinates of each location on the map. In this test, we make map like squared matrix (10 x 10).
N = np.random.randint(10, 50)
diagram4 = GridWithWeights(N, N)

# Make locations have wall.
def location():
    return (np.random.randint(0,N),np.random.randint(0,N))


_walls = []
_sw = np.random.randint(0,N*N)
for i in range(_sw,-1,-1):
  _walls.append(location())

diagram4.walls = _walls
# Make weight for some locations. By default, weight is equal to 1.
_weights = []
_swe = np.random.randint(0,N*N)
for i in range(_swe,-1,-1):
    _weights.append(location())


diagram4.weights = {loc: np.random.randint(0,N) for loc in _weights}

# Make location for source and destination.
start = location()
goal = location()
came_from, cost_so_far = a_star_search(diagram4, start, goal)

draw_grid(diagram4, number=cost_so_far, start=start, goal=goal)
draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=goal))
draw_grid(diagram4, point_to=came_from, start=start, goal=goal)
