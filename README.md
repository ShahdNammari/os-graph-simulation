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

### Milestone 6 – Node synchronization (semaphores)
```bash
make milestone6
./sim input.txt
```

### Clean
```bash
make clean
```

---

## Input File Format

**Milestones 1–3** (`input.txt`):
```
N M          # N nodes, M edges
src dst w    # directed edge (repeated M times)
...
src dst      # single query: find shortest path
```

**Milestones 4–6** (`input_travelers.txt`):
```
N M
src dst w
...
# travelers
K
src1 dst1
src2 dst2
...
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
- Each edge with weight W is traversed in W × 300 ms.
- The entity waits 1 second at every intermediate node.
- A PLAY / STOP / REPLAY button controls the animation.

### Milestone 4
Multiple travelers moving simultaneously using `fork()`.
Input extended with a `# travelers` section.
- Parent reads file, computes Dijkstra for every traveler, then forks N child processes.
- Each child prints `[PID] started` and sleeps until killed.
- Parent runs the raylib GUI; sends SIGTERM when animation finishes.

### Milestone 5
IPC via **pipes** — children are now autonomous.

**IPC mechanism: anonymous pipes** (`pipe()`). One pipe per child (write end in child, read end in parent). Chosen because: simple API, kernel-buffered, naturally serializes messages, and works well with non-blocking reads in the GUI loop.

- Parent forks children **before** computing any paths — clean separation.
- Each child independently reads the graph, runs Dijkstra, and sends a `NodeMsg {pid, node, next}` per node.
- Parent reads pipes non-blocking each frame, updates GUI and prints the log.

### Milestone 6
Node access synchronization via **POSIX named semaphores**.

**Synchronization mechanism: named semaphores** (`sem_open` / `sem_wait` / `sem_post`). One binary semaphore per node, named `/grsim_<ppid>_<node_id>`. Named semaphores are used because they work across separate processes (unlike `pthread_mutex`, which requires shared memory). The parent creates all semaphores before forking and unlinks them on exit.

**Invariant: at most one traveler inside any node at a given time.**

Child behavior at intermediate nodes (not source/destination):
1. Send `MSG_WAITING` → parent shows traveler as a hollow ring ("W") outside the node
2. Call `sem_wait()` → blocks if another traveler is inside (no starvation: Linux POSIX semaphores are FIFO)
3. Send `MSG_ARRIVED` → parent shows traveler as a filled circle inside the node
4. Sleep 1 second (critical section)
5. Call `sem_post()` → releases the node for the next waiting traveler

**GUI distinction:**
- **Waiting** (blocked outside): hollow double-ring in traveler color with "W" label
- **Moving / inside node**: filled circle with white outline
- Status bar shows `[WAITING]` tag next to traveler info

**Demo:** run with `input.txt` — three travelers (0→4, 1→4, 3→4) all route through node 2 with equal edge weights, arriving simultaneously and queuing one by one.