# astar.py
# Core A* engine and Grid class

import heapq
import time
import math
import itertools
from collections import namedtuple

Result = namedtuple('Result', ['found','path','cost','nodes_expanded','time'])

class Grid:
    def __init__(self, cost_map, allow_diagonal=False):
        self.cost = cost_map  # 2D list of positive numbers or math.inf for blocked
        self.h = len(cost_map)
        self.w = len(cost_map[0]) if self.h>0 else 0
        self.allow_diag = allow_diagonal
        # compute minimum traversable cell cost for admissible heuristics
        self._min_cost = min(cell for row in cost_map for cell in row if cell != math.inf)

    def in_bounds(self, x, y):
        return 0 <= x < self.h and 0 <= y < self.w

    def passable(self, x, y):
        return self.cost[x][y] != math.inf

    def neighbors(self, x, y):
        dirs = [(1,0),(-1,0),(0,1),(0,-1)]
        if self.allow_diag:
            dirs += [(1,1),(1,-1),(-1,1),(-1,-1)]
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if self.in_bounds(nx, ny) and self.passable(nx, ny):
                dist = math.hypot(dx, dy)
                step_cost = (self.cost[x][y] + self.cost[nx][ny]) / 2.0 * dist
                yield (nx, ny, step_cost)

class AStar:
    def __init__(self, grid, heuristic):
        self.grid = grid
        self.h = heuristic

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def search(self, start, goal):
        start_t = time.perf_counter()
        open_heap = []
        counter = itertools.count()
        gscore = {start: 0.0}
        fscore_start = self.h(start, goal)
        heapq.heappush(open_heap, (fscore_start, next(counter), start))
        came_from = {}
        closed = set()
        nodes_expanded = 0

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            nodes_expanded += 1
            if current == goal:
                path = self.reconstruct_path(came_from, current)
                total_time = time.perf_counter() - start_t
                return Result(True, path, gscore[current], nodes_expanded, total_time)
            closed.add(current)
            x, y = current
            for nx, ny, step_cost in self.grid.neighbors(x, y):
                neighbor = (nx, ny)
                tentative_g = gscore[current] + step_cost
                if neighbor in closed and tentative_g >= gscore.get(neighbor, float('inf')):
                    continue
                if tentative_g < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g
                    f = tentative_g + self.h(neighbor, goal)
                    heapq.heappush(open_heap, (f, next(counter), neighbor))
        total_time = time.perf_counter() - start_t
        return Result(False, [], float('inf'), nodes_expanded, total_time)

# small util for rendering maps in console
def print_map(cost_map, path=None, start=None, goal=None):
    h = len(cost_map); w = len(cost_map[0]) if h>0 else 0
    pathset = set(path) if path else set()
    for i in range(h):
        row = ''
        for j in range(w):
            if (i,j) == start:
                row += 'S'
            elif (i,j) == goal:
                row += 'G'
            elif (i,j) in pathset:
                row += '*'
            else:
                c = cost_map[i][j]
                if c == math.inf:
                    row += '#'
                elif c > 9:
                    row += 'M'
                elif c > 1.5:
                    row += '+'
                else:
                    row += '.'
        print(row)
