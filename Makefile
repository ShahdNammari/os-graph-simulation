CC     = gcc
CFLAGS = -Wall -Wextra -std=c11

SRCS_1 = main.c graph.c dijkstra.c
HDRS   = graph.h dijkstra.h

.PHONY: milestone1 clean

# ── Milestone 1 : Dijkstra CLI ───────────────────────────────────────────────
milestone1: dijkstra

dijkstra: $(SRCS_1) $(HDRS)
	$(CC) $(CFLAGS) -o dijkstra $(SRCS_1)

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -f dijkstra sim *.o