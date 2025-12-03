# scenarios.py
# Test harness that runs three scenarios and compares heuristics.
from astar import Grid, AStar, print_map
from heuristics import manhattan_heuristic, euclidean_heuristic
import math
import random

def scenario_1():
    grid = [
        [1,1,1,1,1,1,1,1,1,1],
        [1,math.inf,math.inf,1,math.inf,math.inf,math.inf,math.inf,math.inf,1],
        [1,1,1,1,1,1,1,1,math.inf,1],
        [1,math.inf,math.inf,math.inf,math.inf,math.inf,1,1,math.inf,1],
        [1,1,1,1,1,1,1,math.inf,math.inf,1],
        [1,math.inf,1,math.inf,1,math.inf,1,1,1,1],
        [1,math.inf,1,math.inf,1,math.inf,math.inf,math.inf,math.inf,1],
        [1,1,1,1,1,1,1,1,1,1],
    ]
    return Grid(grid, allow_diagonal=False), (0,0), (7,9)

def scenario_2():
    random.seed(2)
    grid = []
    for i in range(20):
        row = []
        for j in range(20):
            r = random.random()
            if r < 0.10:
                row.append(10.0)
            elif r < 0.20:
                row.append(4.0)
            else:
                row.append(1.0)
        grid.append(row)
    for i in range(20):
        grid[10][i] = 10.0
    grid[10][9] = 1.0
    return Grid(grid, allow_diagonal=False), (0,0), (19,19)

def scenario_3():
    grid = [[1 for _ in range(30)] for __ in range(30)]
    for c in range(5,25):
        grid[6][c] = math.inf
    for r in range(6,25):
        grid[r][24] = math.inf
    for r in range(12,20):
        for c in range(2,12):
            grid[r][c] = 3.0
    for r in range(2,8):
        for c in range(2,8):
            grid[r][c] = 8.0
    return Grid(grid, allow_diagonal=False), (0,0), (29,29)

def run_all():
    scenarios = [scenario_1(), scenario_2(), scenario_3()]
    records = []
    out_lines = []
    for idx, (grid, start, goal) in enumerate(scenarios, start=1):
        mh = manhattan_heuristic(grid)
        eh = euclidean_heuristic(grid)
        ast_m = AStar(grid, mh)
        ast_e = AStar(grid, eh)
        res_m = ast_m.search(start, goal)
        res_e = ast_e.search(start, goal)

        print(f"\n=== Scenario {idx} ===")
        print(f"Grid size: {grid.h}x{grid.w} | start={start} goal={goal} | min_cell_cost={grid._min_cost}")
        if res_m.found:
            print(f"\nManhattan: cost={res_m.cost:.3f}, nodes={res_m.nodes_expanded}, time={res_m.time:.4f}s, path_len={len(res_m.path)}")
        else:
            print("\nManhattan: No path found.")
        print_map(grid.cost, res_m.path if res_m.found else None, start, goal)

        if res_e.found:
            print(f"\nEuclidean: cost={res_e.cost:.3f}, nodes={res_e.nodes_expanded}, time={res_e.time:.4f}s, path_len={len(res_e.path)}")
        else:
            print("\nEuclidean: No path found.")
        print_map(grid.cost, res_e.path if res_e.found else None, start, goal)

        records.append((f"Scenario {idx}", res_m, res_e))
        out_lines.append(f"Scenario {idx}: Manhattan cost={res_m.cost:.3f}, nodes={res_m.nodes_expanded}, time={res_m.time:.4f}")
        out_lines.append(f"Scenario {idx}: Euclidean cost={res_e.cost:.3f}, nodes={res_e.nodes_expanded}, time={res_e.time:.4f}")

    # write results to results.txt
    with open('results.txt', 'w') as f:
        f.write('\n'.join(out_lines))
    print("\nWrote results to results.txt")
    return records

if __name__ == "__main__":
    run_all()
