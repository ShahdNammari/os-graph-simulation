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

### Milestone 4 – Multiple processes (fork)
```bash
make milestone4
./sim input.txt
```

### Milestone 5 – IPC via pipes
```bash
make milestone5
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

Example (`input.txt`) — milestone 4+ format:
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
# travelers
3
0 5
1 4
2 3
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

### Milestone 4
Multiple travelers moving simultaneously using `fork()`.
Input file extended with a `# travelers` section listing N source/destination pairs.
- Parent reads file, computes Dijkstra for every traveler, then `fork()`s N child processes.
- Each child prints `[PID] started` and sleeps until killed.
- Parent runs the raylib GUI showing all travelers in different colors moving in parallel.
- When a traveler's animation finishes, parent sends `SIGTERM` to that child.
- Parent `waitpid()`s all children before exiting.

### Milestone 5
IPC via **pipes** — children are now autonomous: each child independently reads the graph, computes its own Dijkstra path, and reports its position to the parent.
- **IPC mechanism chosen: anonymous pipes** (`pipe()`). One pipe per child (write end in child, read end in parent). Chosen because: simple API, kernel-buffered, naturally serializes messages, and works well with non-blocking reads in the GUI loop.
- Parent forks children **before** computing any path data, so no path information exists in parent memory at fork time — clean separation.
- Each child sends a `NodeMsg {pid, node, next}` when it arrives at each node, then sleeps `(1000 + weight×300) ms` before moving on.
- Parent reads pipes non-blocking each frame, updates GUI positions, and prints the log:
  ```
  [PID=1021] arrived at node 0 | next node: 2
  [PID=1021] arrived at node 2 | DESTINATION
  [PID=1021] finished
  ```
- After closing the window the parent kills any remaining children and `waitpid()`s all.