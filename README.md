# Graph Movement Simulation – OS Project

## Build & Run

### Milestone 1 – Dijkstra (CLI)
```bash
make milestone1
./dijkstra input.txt
```

### Milestone 2 – Static Graph GUI (raylib)
```bash
make milestone2
./sim input.txt
```

### Milestone 3 – Animated traversal (raylib)
```bash
make milestone3
./sim input.txt
```

### Clean
```bash
make clean
```

---

## Input File Format
```
N M          # N nodes, M edges
src dst w    # directed edge (repeated M times)
...
src dst      # query: find shortest path from src to dst
```

Example (`input.txt`):
```
6 8
0 1 4
0 2 2
1 3 5
2 1 1
2 3 8
3 4 2
4 5 3
2 5 10
0 5
```

---

## Implementation

### Milestone 1
Directed weighted graph stored as an **adjacency list**.
Shortest path found with **Dijkstra's algorithm** using a binary min-heap.
Reads graph and query from one file. Handles disconnected graphs, negative weight rejection, and src == dst.

### Milestone 2
Static visual display using **raylib**.
Nodes drawn as circles arranged in a circle layout with ID labels.
Edges drawn as directional arrows with weight labels.
Shortest path highlighted in yellow; source/destination nodes in green.

### Milestone 3
Animated traversal of the Dijkstra path using **raylib**.
An orange entity starts at the source and moves along the shortest path.
- Each edge with weight W is divided into W discrete jumps of 300 ms each.
- The entity waits 1 second at every intermediate node.
- A PLAY / STOP / REPLAY button controls the animation.
- An "Arrived at destination!" message is shown on arrival.