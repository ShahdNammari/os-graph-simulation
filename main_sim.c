#define _DEFAULT_SOURCE
#include "raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <semaphore.h>
#include "graph.h"
#include "dijkstra.h"

#define SCREEN_W      900
#define SCREEN_H      700
#define NODE_R         25
#define MAX_NODES      15
#define MAX_TRAVELERS  10
#define JUMP_MS      300.0f
#define WAIT_MS     1000.0f

#define MSG_ARRIVED  0
#define MSG_WAITING  1

typedef enum { SCHED_FCFS, SCHED_SJF } SchedAlgo;

/* message sent child → parent via pipe */
typedef struct {
    int pid;
    int type;   /* MSG_ARRIVED or MSG_WAITING */
    int node;
    int next;   /* -1 = destination reached */
} NodeMsg;

typedef struct {
    int            src, dst;
    DijkstraResult *result;    /* parent-computed for path highlighting + SJF cost */
    pid_t          pid;
    int            pipe_fd;
    Color          color;
    int            cur_node;
    int            nxt_node;
    float          anim_t;
    float          anim_dur;
    Vector2        entity_pos;
    int            initialized;
    int            done;
    int            waiting;       /* 1 = queued outside a node, not yet granted entry */
} Traveler;

/* per-node software queue managed entirely by the parent (the scheduler) */
typedef struct {
    int    occupied;
    double free_at_ms;             /* sim clock value at which the occupant leaves */
    int    queue[MAX_TRAVELERS];   /* traveler indices, in arrival order            */
    int    queue_len;
} NodeState;

static Color PALETTE[MAX_TRAVELERS] = {
    {255, 140,   0, 255},
    {  0, 200, 255, 255},
    {  0, 220,   0, 255},
    {220,   0, 220, 255},
    {255, 200,   0, 255},
    {255, 100, 150, 255},
    {150,  50, 255, 255},
    {255,  50,  50, 255},
    {  0, 200, 150, 255},
    {200, 200,   0, 255},
};

static const char *algo_name(SchedAlgo a) { return (a == SCHED_FCFS) ? "FCFS" : "SJF"; }

static void compute_positions(int n, Vector2 *pos) {
    float cx = SCREEN_W / 2.0f, cy = SCREEN_H / 2.0f;
    float r  = (n <= 1) ? 0.0f : 260.0f;
    for (int i = 0; i < n; i++) {
        float a = 2.0f * PI * i / (float)n - PI / 2.0f;
        pos[i] = (Vector2){ cx + r * cosf(a), cy + r * sinf(a) };
    }
}

static void draw_arrow(Vector2 from, Vector2 to, Color color, float thickness) {
    float dx = to.x - from.x, dy = to.y - from.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1.0f) return;
    float ux = dx / len, uy = dy / len;
    Vector2 s = { from.x + ux * NODE_R, from.y + uy * NODE_R };
    Vector2 e = { to.x   - ux * NODE_R, to.y   - uy * NODE_R };
    DrawLineEx(s, e, thickness, color);
    float ang = atan2f(uy, ux);
    float as = 14.0f, aa = 0.42f;
    Vector2 p1 = { e.x - as * cosf(ang - aa), e.y - as * sinf(ang - aa) };
    Vector2 p2 = { e.x - as * cosf(ang + aa), e.y - as * sinf(ang + aa) };
    DrawTriangle(e, p1, p2, color);
}

static int edge_weight(Graph *g, int src, int dst) {
    for (EdgeNode *e = g->adj[src]; e; e = e->next)
        if (e->dst == dst) return (e->weight > 0) ? e->weight : 1;
    return 1;
}

/* SJF "job length": remaining cost from `node` to the traveler's destination,
 * read off the path Dijkstra already computed for it. */
static int remaining_cost(Graph *g, DijkstraResult *res, int node) {
    if (!res || !res->found) return 0;
    int idx = -1;
    for (int k = 0; k < res->path_len; k++)
        if (res->path[k] == node) { idx = k; break; }
    if (idx < 0) return 0;
    int cost = 0;
    for (int k = idx; k + 1 < res->path_len; k++)
        cost += edge_weight(g, res->path[k], res->path[k + 1]);
    return cost;
}

static void skip_to_token(FILE *fp) {
    int c;
    while ((c = fgetc(fp)) != EOF) {
        if (c == '#') { while ((c = fgetc(fp)) != EOF && c != '\n'); }
        else if (!isspace((unsigned char)c)) { ungetc(c, fp); return; }
    }
}

static Graph *read_graph_only(const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) return NULL;
    skip_to_token(fp);
    int n, m;
    if (fscanf(fp, "%d %d", &n, &m) != 2 || n <= 0 || m < 0) {
        fclose(fp); return NULL;
    }
    Graph *g = graph_create(n);
    for (int i = 0; i < m; i++) {
        int src, dst, w;
        if (fscanf(fp, "%d %d %d", &src, &dst, &w) != 3) {
            graph_free(g); fclose(fp); return NULL;
        }
        graph_add_edge(g, src, dst, w);
    }
    fclose(fp);
    return g;
}

static Graph *read_graph(const char *filename, Traveler *travelers, int *num_travelers) {
    FILE *fp = fopen(filename, "r");
    if (!fp) { fprintf(stderr, "Error: cannot open '%s'\n", filename); return NULL; }

    skip_to_token(fp);
    int n, m;
    if (fscanf(fp, "%d %d", &n, &m) != 2 || n <= 0 || m < 0) {
        fprintf(stderr, "Error: invalid header\n"); fclose(fp); return NULL;
    }

    Graph *g = graph_create(n);
    for (int i = 0; i < m; i++) {
        int src, dst, w;
        if (fscanf(fp, "%d %d %d", &src, &dst, &w) != 3 || w < 0 ||
            src < 0 || src >= n || dst < 0 || dst >= n) {
            fprintf(stderr, "Error: invalid edge\n");
            graph_free(g); fclose(fp); return NULL;
        }
        graph_add_edge(g, src, dst, w);
    }

    skip_to_token(fp);
    *num_travelers = 0;
    if (fscanf(fp, "%d", num_travelers) != 1 || *num_travelers <= 0) {
        fprintf(stderr, "Error: missing travelers count\n");
        graph_free(g); fclose(fp); return NULL;
    }
    if (*num_travelers > MAX_TRAVELERS) *num_travelers = MAX_TRAVELERS;

    for (int i = 0; i < *num_travelers; i++) {
        if (fscanf(fp, "%d %d", &travelers[i].src, &travelers[i].dst) != 2) {
            fprintf(stderr, "Error: invalid traveler %d\n", i);
            *num_travelers = i;
            break;
        }
        travelers[i].result      = NULL;
        travelers[i].pid         = -1;
        travelers[i].pipe_fd     = -1;
        travelers[i].cur_node    = -1;
        travelers[i].nxt_node    = -1;
        travelers[i].anim_t      = 0.0f;
        travelers[i].anim_dur    = 1.0f;
        travelers[i].initialized = 0;
        travelers[i].done        = 0;
        travelers[i].waiting     = 0;
    }

    fclose(fp);
    return g;
}

/*
 * Child: reads graph independently, computes own Dijkstra path, then travels.
 * Intermediate nodes (not source, not destination) require permission from the
 * PARENT (the scheduler) before entering — the parent decides ordering (FCFS/SJF),
 * not the OS:
 *   1. Send MSG_WAITING  → parent enqueues this traveler for the node
 *   2. sem_wait(own sem) → blocks until the parent's scheduler grants this traveler entry
 *   3. Send MSG_ARRIVED  → parent shows traveler inside node
 *   4. sleep 1 second    → critical section (parent tracks release via its own clock)
 */
static void child_run(int write_fd, int src, int dst, const char *filename,
                       pid_t ppid, int my_idx) {
    Graph *g = read_graph_only(filename);
    if (!g) { close(write_fd); exit(1); }

    DijkstraResult *res = dijkstra(g, src, dst);
    if (!res || !res->found) {
        graph_free(g);
        close(write_fd);
        exit(0);
    }

    for (int i = 0; i < res->path_len; i++) {
        int node   = res->path[i];
        int next_n = (i + 1 < res->path_len) ? res->path[i + 1] : -1;
        int is_intermediate = (i > 0) && (next_n != -1);

        if (is_intermediate) {
            /* step 1: tell the parent we want to enter this node */
            NodeMsg wait_msg = { (int)getpid(), MSG_WAITING, node, next_n };
            write(write_fd, &wait_msg, sizeof(wait_msg));

            /* step 2: block until the parent's scheduler grants permission */
            char sem_name[64];
            snprintf(sem_name, sizeof(sem_name), "/grsim_%d_t%d", (int)ppid, my_idx);
            sem_t *sem = sem_open(sem_name, 0);
            sem_wait(sem);
            sem_close(sem);

            /* step 3: inside the node — signal parent */
            NodeMsg arrive_msg = { (int)getpid(), MSG_ARRIVED, node, next_n };
            write(write_fd, &arrive_msg, sizeof(arrive_msg));

            /* step 4: critical section — wait 1 second at this node */
            usleep((useconds_t)(WAIT_MS * 1000.0f));

            /* travel to next node (no release message needed: the parent
             * tracks the fixed 1-second critical section with its own clock) */
            int w = edge_weight(g, node, next_n);
            usleep((useconds_t)(w * JUMP_MS * 1000.0f));
        } else {
            /* source or destination: no queueing needed */
            NodeMsg msg = { (int)getpid(), MSG_ARRIVED, node, next_n };
            write(write_fd, &msg, sizeof(msg));
            if (next_n != -1) {
                int w = edge_weight(g, node, next_n);
                usleep((useconds_t)((WAIT_MS + w * JUMP_MS) * 1000.0f));
            }
        }
    }

    dijkstra_result_free(res);
    graph_free(g);
    close(write_fd);
    exit(0);
}

/* Create per-traveler permission semaphores, pipes, fork children,
 * reset traveler + node-queue state. */
static void launch_travelers(Traveler *travelers, int num_travelers,
                              int pipes[][2], const char *filename,
                              Vector2 *pos, int n, pid_t my_pid,
                              NodeState *node_state, double *sim_clock_ms) {
    /* one "permission slip" semaphore per traveler, starts at 0 (locked) */
    for (int i = 0; i < num_travelers; i++) {
        char name[64];
        snprintf(name, sizeof(name), "/grsim_%d_t%d", (int)my_pid, i);
        sem_unlink(name);
        sem_t *s = sem_open(name, O_CREAT | O_EXCL, 0644, 0);
        if (s == SEM_FAILED) { perror("sem_open"); return; }
        sem_close(s);
    }

    for (int i = 0; i < num_travelers; i++) {
        if (pipe(pipes[i]) != 0) { perror("pipe"); return; }
        travelers[i].pipe_fd = pipes[i][0];
    }

    /* fork children BEFORE parent computes any paths */
    for (int i = 0; i < num_travelers; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            for (int j = 0; j < num_travelers; j++) {
                close(pipes[j][0]);
                if (j != i) close(pipes[j][1]);
            }
            child_run(pipes[i][1], travelers[i].src, travelers[i].dst, filename, my_pid, i);
            /* never returns */
        } else if (pid > 0) {
            travelers[i].pid = pid;
        } else {
            perror("fork");
        }
    }

    for (int i = 0; i < num_travelers; i++) {
        close(pipes[i][1]);
        int flags = fcntl(pipes[i][0], F_GETFL, 0);
        fcntl(pipes[i][0], F_SETFL, flags | O_NONBLOCK);
    }

    for (int i = 0; i < num_travelers; i++) {
        travelers[i].done        = 0;
        travelers[i].initialized = 0;
        travelers[i].cur_node    = -1;
        travelers[i].nxt_node    = -1;
        travelers[i].anim_t      = 0.0f;
        travelers[i].waiting     = 0;
        if (travelers[i].src >= 0 && travelers[i].src < n)
            travelers[i].entity_pos = pos[travelers[i].src];
    }

    for (int i = 0; i < n; i++) {
        node_state[i].occupied   = 0;
        node_state[i].free_at_ms = 0.0;
        node_state[i].queue_len  = 0;
    }
    *sim_clock_ms = 0.0;
}

/* Remove the queue entry at `slot`, preserving relative arrival order. */
static void node_queue_remove_at(NodeState *ns, int slot) {
    for (int k = slot; k + 1 < ns->queue_len; k++)
        ns->queue[k] = ns->queue[k + 1];
    ns->queue_len--;
}

/* Pick the next traveler to admit into `node`, per the active scheduler.
 * FCFS: earliest-arrived (front of queue). SJF: smallest remaining cost
 * to destination, ties broken by arrival order. Returns traveler index or -1. */
static int node_queue_pick(NodeState *ns, Traveler *travelers, Graph *g,
                            int node, SchedAlgo algo) {
    if (ns->queue_len == 0) return -1;
    int best_slot = 0;
    if (algo == SCHED_SJF) {
        int best_cost = remaining_cost(g, travelers[ns->queue[0]].result, node);
        for (int k = 1; k < ns->queue_len; k++) {
            int c = remaining_cost(g, travelers[ns->queue[k]].result, node);
            if (c < best_cost) { best_cost = c; best_slot = k; }
        }
    }
    int idx = ns->queue[best_slot];
    node_queue_remove_at(ns, best_slot);
    return idx;
}

int main(int argc, char *argv[]) {
    if (argc != 4 || strcmp(argv[1], "-schd") != 0) {
        fprintf(stderr, "Usage: %s -schd <fcfs|sjf> <input_file>\n", argv[0]);
        return 1;
    }
    SchedAlgo algo;
    if      (strcmp(argv[2], "fcfs") == 0) algo = SCHED_FCFS;
    else if (strcmp(argv[2], "sjf")  == 0) algo = SCHED_SJF;
    else { fprintf(stderr, "Error: unknown scheduler '%s' (use fcfs or sjf)\n", argv[2]); return 1; }
    const char *filename = argv[3];

    Traveler travelers[MAX_TRAVELERS] = {0};
    int num_travelers = 0;

    Graph *g = read_graph(filename, travelers, &num_travelers);
    if (!g) return 1;
    int n = g->num_nodes;

    for (int i = 0; i < num_travelers; i++)
        travelers[i].color = PALETTE[i % MAX_TRAVELERS];

    pid_t my_pid = getpid();
    int pipes[MAX_TRAVELERS][2];
    memset(pipes, -1, sizeof(pipes));

    NodeState node_state[MAX_NODES] = {0};
    double sim_clock_ms = 0.0;

    Vector2 pos[MAX_NODES] = {0};
    compute_positions(n, pos);

    for (int i = 0; i < num_travelers; i++)
        if (travelers[i].src >= 0 && travelers[i].src < n)
            travelers[i].entity_pos = pos[travelers[i].src];

    char title[64];
    snprintf(title, sizeof(title), "Graph Simulation - Milestone 7 [%s]", algo_name(algo));
    InitWindow(SCREEN_W, SCREEN_H, title);
    SetTargetFPS(60);

    int simulation_started = 0;
    Rectangle btn = { SCREEN_W - 130.0f, 10.0f, 120.0f, 40.0f };

    while (!WindowShouldClose()) {
        float dt_ms = GetFrameTime() * 1000.0f;

        int all_done = simulation_started;
        for (int i = 0; i < num_travelers; i++)
            if (!travelers[i].done) { all_done = 0; break; }

        /* button click */
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Vector2 mp = GetMousePosition();
            if (CheckCollisionPointRec(mp, btn)) {
                if (!simulation_started) {
                    launch_travelers(travelers, num_travelers, pipes, filename, pos,
                                      n, my_pid, node_state, &sim_clock_ms);
                    simulation_started = 1;
                    for (int i = 0; i < num_travelers; i++) {
                        if (travelers[i].result) {
                            dijkstra_result_free(travelers[i].result);
                            travelers[i].result = NULL;
                        }
                        if (travelers[i].src >= 0 && travelers[i].src < n &&
                            travelers[i].dst >= 0 && travelers[i].dst < n)
                            travelers[i].result = dijkstra(g, travelers[i].src, travelers[i].dst);
                    }
                } else if (all_done) {
                    for (int i = 0; i < num_travelers; i++) {
                        if (travelers[i].pid > 0) {
                            if (!travelers[i].done) kill(travelers[i].pid, SIGTERM);
                            waitpid(travelers[i].pid, NULL, 0);
                            travelers[i].pid = -1;
                        }
                        if (travelers[i].pipe_fd >= 0) {
                            close(travelers[i].pipe_fd);
                            travelers[i].pipe_fd = -1;
                        }
                    }
                    launch_travelers(travelers, num_travelers, pipes, filename, pos,
                                      n, my_pid, node_state, &sim_clock_ms);
                }
            }
        }

        if (simulation_started) {
            sim_clock_ms += dt_ms;

            /* poll pipes for messages from children */
            for (int i = 0; i < num_travelers; i++) {
                if (travelers[i].pipe_fd < 0 || travelers[i].done) continue;

                NodeMsg msg;
                ssize_t r = read(travelers[i].pipe_fd, &msg, sizeof(msg));

                if (r == (ssize_t)sizeof(msg)) {
                    if (msg.type == MSG_WAITING) {
                        printf("[PID=%d] waiting for node %d\n", msg.pid, msg.node);
                        fflush(stdout);
                        travelers[i].waiting  = 1;
                        travelers[i].nxt_node = -1; /* freeze animation */
                        NodeState *ns = &node_state[msg.node];
                        ns->queue[ns->queue_len++] = i;
                    } else { /* MSG_ARRIVED */
                        travelers[i].waiting = 0;
                        if (msg.next == -1) {
                            printf("[PID=%d] arrived at node %d | DESTINATION\n",
                                   msg.pid, msg.node);
                            printf("[PID=%d] finished\n", msg.pid);
                            fflush(stdout);
                            travelers[i].entity_pos = pos[msg.node];
                            travelers[i].done       = 1;
                            close(travelers[i].pipe_fd);
                            travelers[i].pipe_fd    = -1;
                        } else {
                            printf("[PID=%d] arrived at node %d | next node: %d\n",
                                   msg.pid, msg.node, msg.next);
                            fflush(stdout);
                            travelers[i].cur_node    = msg.node;
                            travelers[i].nxt_node    = msg.next;
                            travelers[i].anim_t      = 0.0f;
                            int w = edge_weight(g, msg.node, msg.next);
                            travelers[i].anim_dur    = WAIT_MS + w * JUMP_MS;
                            travelers[i].entity_pos  = pos[msg.node];
                            travelers[i].initialized = 1;
                        }
                    }
                } else if (r == 0) {
                    close(travelers[i].pipe_fd);
                    travelers[i].pipe_fd = -1;
                    travelers[i].done    = 1;
                }
            }

            /* scheduler: release finished occupants, then admit next in line */
            for (int node = 0; node < n; node++) {
                NodeState *ns = &node_state[node];
                if (ns->occupied && sim_clock_ms >= ns->free_at_ms)
                    ns->occupied = 0;

                if (!ns->occupied) {
                    int idx = node_queue_pick(ns, travelers, g, node, algo);
                    if (idx >= 0) {
                        ns->occupied   = 1;
                        ns->free_at_ms = sim_clock_ms + WAIT_MS;
                        travelers[idx].waiting = 0;
                        char name[64];
                        snprintf(name, sizeof(name), "/grsim_%d_t%d", (int)my_pid, idx);
                        sem_t *sem = sem_open(name, 0);
                        if (sem != SEM_FAILED) { sem_post(sem); sem_close(sem); }
                    }
                }
            }
        }

        /* update animations — skip if waiting or not yet started */
        for (int i = 0; i < num_travelers; i++) {
            if (!travelers[i].initialized || travelers[i].done || travelers[i].waiting)
                continue;
            if (travelers[i].nxt_node < 0) continue;
            travelers[i].anim_t += dt_ms / travelers[i].anim_dur;
            if (travelers[i].anim_t > 1.0f) travelers[i].anim_t = 1.0f;
            float t  = travelers[i].anim_t;
            int   c  = travelers[i].cur_node;
            int   nx = travelers[i].nxt_node;
            travelers[i].entity_pos.x = pos[c].x + t * (pos[nx].x - pos[c].x);
            travelers[i].entity_pos.y = pos[c].y + t * (pos[nx].y - pos[c].y);
        }

        BeginDrawing();
        ClearBackground((Color){25, 25, 35, 255});

        for (int i = 0; i < n; i++)
            for (EdgeNode *e = g->adj[i]; e; e = e->next)
                draw_arrow(pos[i], pos[e->dst], (Color){160, 160, 160, 255}, 1.5f);

        for (int t = 0; t < num_travelers; t++) {
            DijkstraResult *res = travelers[t].result;
            if (!res || !res->found) continue;
            for (int k = 0; k + 1 < res->path_len; k++)
                draw_arrow(pos[res->path[k]], pos[res->path[k + 1]],
                           travelers[t].color, 3.0f);
        }

        for (int i = 0; i < n; i++) {
            for (EdgeNode *e = g->adj[i]; e; e = e->next) {
                float mx = (pos[i].x + pos[e->dst].x) * 0.5f;
                float my = (pos[i].y + pos[e->dst].y) * 0.5f;
                float ex = pos[e->dst].x - pos[i].x;
                float ey = pos[e->dst].y - pos[i].y;
                float el = sqrtf(ex * ex + ey * ey);
                if (el > 0) { mx += (-ey / el) * 14.0f; my += (ex / el) * 14.0f; }
                char ws[16]; snprintf(ws, sizeof(ws), "%d", e->weight);
                DrawText(ws, (int)mx - 6, (int)my - 8, 16, WHITE);
            }
        }

        for (int i = 0; i < n; i++) {
            Color nc = (Color){40, 80, 160, 255};
            for (int t = 0; t < num_travelers; t++)
                if (i == travelers[t].src || i == travelers[t].dst) { nc = GREEN; break; }
            DrawCircleV(pos[i], NODE_R, nc);
            DrawCircleLinesV(pos[i], NODE_R, WHITE);
            char lbl[8]; snprintf(lbl, sizeof(lbl), "%d", i);
            int tw = MeasureText(lbl, 20);
            DrawText(lbl, (int)pos[i].x - tw / 2, (int)pos[i].y - 10, 20, WHITE);
        }

        for (int t = 0; t < num_travelers; t++) {
            if (!simulation_started) continue;
            if (travelers[t].waiting) {
                DrawCircleLinesV(travelers[t].entity_pos, 14.0f, travelers[t].color);
                DrawCircleLinesV(travelers[t].entity_pos, 10.0f, travelers[t].color);
                int ww = MeasureText("W", 14);
                DrawText("W", (int)travelers[t].entity_pos.x - ww / 2,
                         (int)travelers[t].entity_pos.y - 7, 14, travelers[t].color);
            } else {
                DrawCircleV(travelers[t].entity_pos, 14.0f, travelers[t].color);
                DrawCircleLinesV(travelers[t].entity_pos, 14.0f, WHITE);
            }
        }

        if (all_done) {
            const char *msg_str = "All travelers arrived!";
            int mw = MeasureText(msg_str, 28);
            DrawRectangle(SCREEN_W / 2 - mw / 2 - 20, SCREEN_H / 2 - 35,
                          mw + 40, 70, (Color){0, 0, 0, 210});
            DrawText(msg_str, SCREEN_W / 2 - mw / 2, SCREEN_H / 2 - 14, 28, GREEN);
        }

        /* scheduler indicator — clearly visible, required by spec */
        {
            char sched_lbl[32];
            snprintf(sched_lbl, sizeof(sched_lbl), "Scheduler: %s", algo_name(algo));
            DrawRectangle(5, 5, MeasureText(sched_lbl, 22) + 14, 30, (Color){0, 0, 0, 180});
            DrawText(sched_lbl, 12, 9, 22, YELLOW);
        }

        {
            const char *lbl = !simulation_started ? "PLAY"
                            : all_done            ? "REPLAY"
                            : NULL;
            if (lbl) {
                DrawRectangleRec(btn, (Color){0, 160, 0, 255});
                DrawRectangleLinesEx(btn, 2.0f, WHITE);
                int lw = MeasureText(lbl, 20);
                DrawText(lbl, (int)(btn.x + (btn.width - lw) / 2),
                         (int)(btn.y + (btn.height - 20) / 2), 20, WHITE);
            }
        }

        {
            int bar_h = 20 * num_travelers + 4;
            DrawRectangle(0, SCREEN_H - bar_h, SCREEN_W, bar_h, (Color){0, 0, 0, 200});
            for (int t = 0; t < num_travelers; t++) {
                DijkstraResult *res = travelers[t].result;
                int y = SCREEN_H - bar_h + 2 + t * 20;
                char info[256];
                if (res && res->found) {
                    int off = snprintf(info, sizeof(info), "T%d [pid %d]%s: ",
                                       t, (int)travelers[t].pid,
                                       travelers[t].waiting ? " [WAITING]" : "");
                    for (int k = 0; k < res->path_len && off < (int)sizeof(info) - 1; k++) {
                        if (k > 0) off += snprintf(info + off, sizeof(info) - off, " -> ");
                        off += snprintf(info + off, sizeof(info) - off, "%d", res->path[k]);
                    }
                    snprintf(info + off, sizeof(info) - off, "  (cost: %d)", res->total_weight);
                } else {
                    snprintf(info, sizeof(info), "T%d: %s", t,
                             simulation_started ? "No path found" : "Press PLAY to start");
                }
                DrawText(info, 10, y, 16, travelers[t].color);
            }
        }

        EndDrawing();
    }

    CloseWindow();

    for (int i = 0; i < num_travelers; i++) {
        if (travelers[i].pid > 0) {
            if (!travelers[i].done) kill(travelers[i].pid, SIGTERM);
            waitpid(travelers[i].pid, NULL, 0);
            if (travelers[i].pipe_fd >= 0) {
                close(travelers[i].pipe_fd);
                travelers[i].pipe_fd = -1;
            }
        }
    }

    for (int i = 0; i < num_travelers; i++) {
        char name[64];
        snprintf(name, sizeof(name), "/grsim_%d_t%d", (int)my_pid, i);
        sem_unlink(name);
    }

    for (int i = 0; i < num_travelers; i++)
        if (travelers[i].result) dijkstra_result_free(travelers[i].result);
    graph_free(g);
    return 0;
}
