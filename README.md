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
./sim input_travelers.txt
```

### Milestone 5 – IPC via pipes
```bash
make milestone5
./sim input_travelers.txt
```

### Milestone 6 – Node synchronization (semaphores)
```bash
make milestone6
./sim input.txt
```

### Milestone 7 – Scheduling algorithms (FCFS / SJF)
```bash
make milestone7
./sim -schd fcfs input.txt
./sim -schd sjf  input.txt
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

**Milestones 4–7** (`input_travelers.txt`):
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

### Milestone 2
Static visual display using **raylib**. Nodes as circles, edges as arrows with weight labels.

### Milestone 3
Animated traversal of the Dijkstra path. PLAY / REPLAY button.

### Milestone 4
Multiple travelers moving simultaneously using `fork()`.

### Milestone 5
IPC via **anonymous pipes** — children are autonomous: each independently reads the graph, runs Dijkstra, and reports `NodeMsg {pid, node, next}` to the parent per node.

### Milestone 6
Node access synchronization via **POSIX named semaphores** — at most one traveler inside any node at a time. Travelers blocked outside a node are shown with a hollow "W" ring.

### Milestone 7
**Scheduling algorithms for node entry order.**

In Milestone 6, the order in which waiting travelers entered a node was decided by the kernel (FIFO order of `sem_wait()` calls on a shared per-node semaphore) — the parent had no say in it. Milestone 7 replaces that with **parent-controlled scheduling**:

- The shared per-node semaphore is replaced by **one private "permission" semaphore per traveler**, created with initial value `0` (locked). A child blocks on its *own* semaphore until the parent explicitly grants it.
- The parent maintains a **software waiting queue per node** (`NodeState.queue[]`). When a child sends `MSG_WAITING`, the parent appends that traveler's index to the queue for that node — it does **not** grant entry yet.
- Each frame, for every node that is currently free, the parent calls the active **scheduler** to pick the next traveler from that node's queue, then `sem_post()`s that specific traveler's permission semaphore.
- The node is marked occupied for a fixed `WAIT_MS` (1 second) using the parent's own simulation clock — no extra "I'm leaving" message is needed from the child, since the critical-section duration is constant and known in advance.

**Run command:**
```bash
./sim -schd fcfs <file_name>
./sim -schd sjf  <file_name>
```

**Algorithm 1 — FCFS (First-Come-First-Served):** the traveler that has been waiting longest (front of the queue) is admitted first. Pure arrival order, no other factor.

**Algorithm 2 — SJF (Shortest-Job-First):** the traveler with the **smallest remaining cost to its destination** is admitted first. "Job length" is computed from the Dijkstra path the parent already has for that traveler — no new input-file field is needed: the parent finds the traveler's current node in its stored path and sums the edge weights from there to the end. Ties are broken by arrival order (so SJF degrades to FCFS among equal-length jobs).

**GUI:** the active scheduler is shown as a label in the top-left corner ("Scheduler: FCFS" / "Scheduler: SJF") and in the window title.

**No starvation:** every traveler is removed from a node's queue exactly once it is admitted; the queue is re-scanned every frame, so a traveler is never skipped forever — under SJF a traveler with a long remaining path could wait longer if shorter jobs keep arriving, but it is never starved indefinitely since the demo's job set is finite and each queue empties out.

#### Demo input (`input.txt`)
Three travelers — `T0: 0→4`, `T1: 1→5`, `T2: 2→6` — all reach the shared bottleneck node **3** at the same time (each source edge has weight 1), then diverge with very different remaining costs: T0 still has 5 to go, T1 has 1, T2 has 3.

| Scheduler | Entry order at node 3 | Why |
|---|---|---|
| FCFS | T0 → T1 → T2 | Matches arrival order (all arrive together; ties broken by traveler index) |
| SJF  | T1 → T2 → T0 | Ordered by remaining cost: 1 → 3 → 5 |

#### Comparison: effect on waiting time
Run the same `input.txt` with both schedulers and compare the `[PID=...] waiting for node 3` → `arrived at node 3` gaps printed to the terminal:
- Under **FCFS**, T1 (the traveler with the shortest remaining trip) is forced to wait behind T0 even though T0 has much further left to travel — T1's total completion time is delayed by T0's full 1-second critical section.
- Under **SJF**, T1 enters first and finishes its short remaining trip quickly; T0 (the longest remaining job) is pushed to the back of the queue, so it waits the longest before entering — but since T0 was already going to take the most overall travel time, this ordering minimizes the *average* completion time across all three travelers, at the cost of T0's wait.

This matches the classic FCFS-vs-SJF tradeoff: FCFS is simple and fair by arrival order, while SJF minimizes average wait time but can leave longer jobs waiting disproportionately long.
