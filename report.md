# Advanced Implementation of A* with Heuristic Tuning
**Author:** Mohan  
**Project:** Astar-Heuristic-Project

---

## Project overview
Implemented A* search engine for grid-based pathfinding with support for arbitrary cost maps and blocked cells. The system supports pluggable admissible heuristics. Two heuristics implemented: Manhattan and Euclidean (both multiplied by grid minimum cost to preserve admissibility on weighted maps).

---

## Files
- `astar.py` — Grid and AStar engine
- `heuristics.py` — Manhattan and Euclidean heuristic factories
- `scenarios.py` — Three test scenarios and a run harness (produces `results.txt`)
- `report.md` — This document

---

## Heuristics
- **Manhattan heuristic (admissible):** `(abs(dx)+abs(dy)) * min_cost`
- **Euclidean heuristic (admissible):** `sqrt(dx^2 + dy^2) * min_cost`

Multiplying by `min_cost` keeps heuristics lower-bounding true remaining cost even when cell costs vary.

---

## Movement cost model
Move cost from cell A to neighbor B = `(cost[A] + cost[B]) / 2 * distance`, where distance is `1` for orthogonal and `sqrt(2)` for diagonal. This models terrain transition cost.

---

## Test scenarios
1. **Scenario 1:** Maze obstacles (8x10), uniform cost = 1  
2. **Scenario 2:** 20x20 weighted terrain, mountain rows + small gap  
3. **Scenario 3:** 30x30 mixed map with large blocked walls and rough/mountain patches

---

## Execution results (from `scenarios.py` run)
(Saved in `results.txt`.)

Summary comparison (heuristic: cost, nodes_expanded, time):

- Scenario 1:
  - Manhattan -> cost=16.000, nodes=55, time≈0.0003s
  - Euclidean -> cost=16.000, nodes=51, time≈0.0002s

- Scenario 2:
  - Manhattan -> cost=50.000, nodes=216, time≈0.0014s
  - Euclidean -> cost=50.000, nodes=295, time≈0.0019s

- Scenario 3:
  - Manhattan -> cost=58.000, nodes=733, time≈0.0069s
  - Euclidean -> cost=58.000, nodes=853, time≈0.0050s

---

## Analysis and conclusions
- Both heuristics are admissible (they return the same optimal path cost).
- Node expansions vary: Euclidean sometimes explores fewer nodes in tight open spaces (Scenario 1), but more nodes on heavily weighted or maze-like maps (Scenarios 2 and 3). This demonstrates heuristic behavior depends on map structure and cost distribution.
- For larger maps or production use, consider memory-bounded A* variants or more informed heuristics (landmarks/ALT, pattern databases) and more advanced priority queues if decrease-key becomes a bottleneck.

---

## How to run
1. Clone repo or download files.
2. Run:
