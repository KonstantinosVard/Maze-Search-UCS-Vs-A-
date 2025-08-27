# Maze Pathfinding AI: UCS vs A* Search Algorithms
### MSc Artificial Intelligence  Project
A comprehensive implementation comparing Uniform Cost Search (UCS) and A* pathfinding algorithms for robot navigation in randomly generated mazes with special teleportation mechanics.

## Overview

This project implements two classical AI search algorithms to solve robot navigation problems in N×N mazes. The maze features randomly distributed obstacles, 8-directional movement, and special teleportation between designated cells A and B, providing a complex pathfinding challenge for algorithm comparison.

## Problem Definition

### Maze Environment
- **Grid size**: N×N configurable maze (up to 100×100)
- **Cell types**: Free (probability p), Obstacles (1-p), Start (S), Goal (G)
- **Special cells**: A (bottom-left corner), B (top-right corner) - always free
- **Movement**: 8-directional (horizontal, vertical, diagonal)

### Movement Costs
- **Regular moves**: Cost = 1 (all 8 directions)
- **Teleportation**: Cost = 2 (A ↔ B bidirectional)
- **Objective**: Find optimal path from Start to Goal

### Constraints
- All cells except S, G, A, B generated with probability p of being free
- Robot cannot move through obstacles
- Must find minimum cost path considering both regular and teleportation moves

## Algorithms Implemented

### 1. Uniform Cost Search (UCS)
- **Strategy**: Breadth-first expansion by path cost
- **Priority**: Nodes ordered by g(n) = cost from start
- **Guarantee**: Finds optimal solution
- **Implementation**: Min-heap priority queue

### 2. A* Search
- **Strategy**: Best-first search with heuristic guidance
- **Priority**: Nodes ordered by f(n) = g(n) + h(n)
- **Heuristic**: Admissible distance estimation
- **Guarantee**: Finds optimal solution (with admissible heuristic)

## Heuristic Function Design

The A* heuristic calculates minimum estimated cost considering three possible routes:

1. **Direct path**: Chebyshev distance to goal
2. **Via A then B**: Distance to A + 2 + Distance from B to goal  
3. **Via B then A**: Distance to B + 2 + Distance from A to goal

```c
float heuristic(int x, int y, int gx, int gy) {
    float direct = chebyshev(x, y, gx, gy);
    float via_A = chebyshev(x, y, N-1, 0) + 2 + chebyshev(0, N-1, gx, gy);
    float via_B = chebyshev(x, y, 0, N-1) + 2 + chebyshev(N-1, 0, gx, gy);
    return min(direct, min(via_A, via_B));
}
```

### Heuristic Properties
- **Admissible**: Never overestimates actual cost
- **Consistent**: Satisfies triangle inequality h(n) ≤ c(n,n') + h(n')
- **Optimality**: Guarantees A* finds optimal solution


## Performance Analysis

Based on testing with N=10, p=0.7 mazes:

| Algorithm | Path Cost | Avg Expansions | Efficiency Gain |
|-----------|-----------|----------------|-----------------|
| **UCS** | Optimal | 58.0 | Baseline |
| **A*** | Optimal | 15.8 | **73% fewer expansions** |

### Key Results
- **Optimality**: Both algorithms find optimal solutions
- **Efficiency**: A* reduces node expansions by ~73%
- **Consistency**: Performance advantage maintained across different maze configurations


## Compilation and Usage

### Compile
```bash
gcc -o labsearch labsearch.c -lm
```

### Run
```bash
./labsearch
```

### Input Parameters
```
Enter N (grid size): 10
Enter p (probability of free cell): 0.7
Enter start cell coordinates (sx sy): 9 3
Enter goal cell coordinates (gx gy): 2 8
```

### Sample Output
```
Initial Maze:
<maze diagram>

UCS Results:
<maze with UCS path>
Path cost: 7.0
Expansions: 55

A* Results:
<maze with A* path>
Path cost: 7.0
Expansions: 14
```


## Algorithmic Insights

### UCS Characteristics
- **Complete**: Always finds solution if one exists
- **Optimal**: Guaranteed shortest path
- **Uninformed**: No domain knowledge utilization
- **Systematic**: Explores nodes in cost order

### A* Advantages
- **Informed search**: Uses heuristic guidance
- **Goal-directed**: Focuses search toward target
- **Efficient**: Significantly fewer node expansions
- **Optimal**: Maintains optimality with admissible heuristic

### Practical Applications
- **Robotics**: Path planning in known environments
- **Gaming**: NPC navigation and pathfinding
- **Network routing**: Optimal route discovery
- **Logistics**: Delivery route optimization

## Author

**Konstantinos Vardakas**  

## Course Information

Artificial Intelligence - Project 1: Labyrinth Search

---

*This implementation demonstrates the practical advantages of informed search strategies and the importance of well-designed heuristic functions in AI pathfinding applications.*
