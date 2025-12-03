import math

def manhattan_heuristic(grid):
    """
    Returns a function h(a,b) that computes Manhattan distance times grid minimum cost.
    This keeps the heuristic admissible for weighted maps.
    """
    minc = grid._min_cost
    def h(a, b):
        (x1,y1),(x2,y2) = a,b
        return (abs(x1-x2) + abs(y1-y2)) * minc
    return h

def euclidean_heuristic(grid):
    """
    Returns Euclidean distance times grid minimum cost.
    Admissible because we multiply by minimum per-cell cost.
    """
    minc = grid._min_cost
    def h(a, b):
        (x1,y1),(x2,y2) = a,b
        return math.hypot(x1-x2, y1-y2) * minc
    return h
