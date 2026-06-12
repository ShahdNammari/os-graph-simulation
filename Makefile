CC     = gcc
CFLAGS = -Wall -Wextra -std=c11

RAYLIB_INC = /usr/local/include
RAYLIB_LIB = /usr/local/lib
RAYLIB_FLAGS = -I$(RAYLIB_INC) -L$(RAYLIB_LIB) -Wl,-rpath,$(RAYLIB_LIB) \
               -lraylib -lm -lpthread -ldl -lrt -lX11

SRCS_1 = main.c graph.c dijkstra.c
SRCS_2 = main_sim.c graph.c dijkstra.c
HDRS   = graph.h dijkstra.h

.PHONY: milestone1 milestone2 milestone3 milestone4 milestone5 milestone6 clean

# ── Milestone 1 : Dijkstra CLI ───────────────────────────────────────────────
milestone1: dijkstra

dijkstra: $(SRCS_1) $(HDRS)
	$(CC) $(CFLAGS) -o dijkstra $(SRCS_1)

# ── Milestone 2 : raylib GUI ─────────────────────────────────────────────────
milestone2: sim

# ── Milestone 3 : animation ──────────────────────────────────────────────────
milestone3: sim

# ── Milestone 4 : multiple processes (fork) ──────────────────────────────────
milestone4: sim

# ── Milestone 5 : IPC via pipes ──────────────────────────────────────────────
milestone5: sim

# ── Milestone 6 : node synchronization (semaphores) ─────────────────────────
milestone6: sim

sim: $(SRCS_2) $(HDRS)
	$(CC) $(CFLAGS) -o sim $(SRCS_2) $(RAYLIB_FLAGS)

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -f dijkstra sim *.o
