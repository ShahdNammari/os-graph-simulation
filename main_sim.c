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
#include "graph.h"
#include "dijkstra.h"

#define SCREEN_W      900
#define SCREEN_H      700
#define NODE_R         25
#define MAX_NODES      15
#define MAX_TRAVELERS  10
#define JUMP_MS      300.0f
#define WAIT_MS     1000.0f

/* message kinds sent child → parent via pipe */
#define MSG_KIND_NODE    0
#define MSG_KIND_SPECIAL 1

/* unified pipe message:
 *   MSG_KIND_NODE:    pid, node, extra = next node (-1 if destination)
 *   MSG_KIND_SPECIAL: pid, node, extra = 1 if node can reach itself, else 0
 */
typedef struct {
    int kind;
    int pid;
    int node;
    int extra;
} PipeMsg;

typedef struct {
    int            src, dst;
    DijkstraResult *result;    /* parent-computed for edge highlighting only */
    pid_t          pid;
    int            pipe_fd;    /* parent read end; -1 = none */
    Color          color;
    /* message-driven animation */
    int            cur_node;
    int            nxt_node;   /* -1 = not moving */
    float          anim_t;
    float          anim_dur;
    Vector2        entity_pos;
    int            initialized;
    int            done;
} Traveler;

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

/* ── drawing helpers ─────────────────────────────────────────────────────────*/

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

/* ── file I/O ────────────────────────────────────────────────────────────────*/

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
    }

    fclose(fp);
    return g;
}

/* ── child process ───────────────────────────────────────────────────────────*/

static void child_run(int write_fd, int src, int dst, const char *filename) {
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

        /* special message sent first: can this node reach itself? */
        int can_reach_self = 0;
        for (EdgeNode *e = g->adj[node]; e; e = e->next)
            if (e->dst == node) { can_reach_self = 1; break; }
        PipeMsg special = { MSG_KIND_SPECIAL, (int)getpid(), node, can_reach_self };
        write(write_fd, &special, sizeof(special));

        /* regular traversal message sent after */
        PipeMsg msg = { MSG_KIND_NODE, (int)getpid(), node, next_n };
        write(write_fd, &msg, sizeof(msg));

        if (next_n != -1) {
            int w = edge_weight(g, node, next_n);
            usleep((useconds_t)((WAIT_MS + w * JUMP_MS) * 1000.0f));
        }
    }

    dijkstra_result_free(res);
    graph_free(g);
    close(write_fd);
    exit(0);
}

/* ── launch all travelers (fork + pipes + reset state) ───────────────────────*/

static void launch_travelers(Traveler *travelers, int num_travelers,
                              int pipes[][2], const char *filename,
                              Vector2 *pos) {
    for (int i = 0; i < num_travelers; i++) {
        if (pipe(pipes[i]) != 0) { perror("pipe"); return; }
        travelers[i].pipe_fd = pipes[i][0];
    }

    for (int i = 0; i < num_travelers; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            for (int j = 0; j < num_travelers; j++) {
                close(pipes[j][0]);
                if (j != i) close(pipes[j][1]);
            }
            child_run(pipes[i][1], travelers[i].src, travelers[i].dst, filename);
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

    /* reset animation state for all travelers */
    for (int i = 0; i < num_travelers; i++) {
        travelers[i].done        = 0;
        travelers[i].initialized = 0;
        travelers[i].cur_node    = -1;
        travelers[i].nxt_node    = -1;
        travelers[i].anim_t      = 0.0f;
        travelers[i].entity_pos  = pos[travelers[i].src];
    }
}

/* ── main ────────────────────────────────────────────────────────────────────*/

int main(int argc, char *argv[]) {
    if (argc != 2) { fprintf(stderr, "Usage: %s <input_file>\n", argv[0]); return 1; }

    Traveler travelers[MAX_TRAVELERS] = {0};
    int num_travelers = 0;

    Graph *g = read_graph(argv[1], travelers, &num_travelers);
    if (!g) return 1;
    int n = g->num_nodes;

    for (int i = 0; i < num_travelers; i++)
        travelers[i].color = PALETTE[i % MAX_TRAVELERS];

    Vector2 pos[MAX_NODES] = {0};
    compute_positions(n, pos);

    for (int i = 0; i < num_travelers; i++)
        if (travelers[i].src >= 0 && travelers[i].src < n)
            travelers[i].entity_pos = pos[travelers[i].src];

    int pipes[MAX_TRAVELERS][2];
    int simulation_started = 0;

    Rectangle btn = { SCREEN_W - 130.0f, 10.0f, 110.0f, 40.0f };

    InitWindow(SCREEN_W, SCREEN_H, "Graph Simulation");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt_ms = GetFrameTime() * 1000.0f;

        /* check all done */
        int all_done = 1;
        for (int i = 0; i < num_travelers; i++)
            if (!travelers[i].done) { all_done = 0; break; }

        /* ── button input ── */
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) &&
            CheckCollisionPointRec(GetMousePosition(), btn)) {

            if (!simulation_started) {
                /* PLAY — fork first, then compute parent paths (children won't inherit) */
                launch_travelers(travelers, num_travelers, pipes, argv[1], pos);
                simulation_started = 1;
                for (int i = 0; i < num_travelers; i++) {
                    if (travelers[i].src >= 0 && travelers[i].src < n &&
                        travelers[i].dst >= 0 && travelers[i].dst < n)
                        travelers[i].result = dijkstra(g, travelers[i].src, travelers[i].dst);
                }

            } else if (all_done) {
                /* REPLAY — clean up old children then re-fork */
                for (int i = 0; i < num_travelers; i++) {
                    if (travelers[i].pid > 0) {
                        waitpid(travelers[i].pid, NULL, 0);
                        travelers[i].pid = -1;
                    }
                    if (travelers[i].pipe_fd >= 0) {
                        close(travelers[i].pipe_fd);
                        travelers[i].pipe_fd = -1;
                    }
                }
                launch_travelers(travelers, num_travelers, pipes, argv[1], pos);
            }
        }

        /* ── poll pipes ── */
        if (simulation_started) {
            for (int i = 0; i < num_travelers; i++) {
                if (travelers[i].pipe_fd < 0 || travelers[i].done) continue;

                PipeMsg msg;
                ssize_t r = read(travelers[i].pipe_fd, &msg, sizeof(msg));

                if (r == (ssize_t)sizeof(msg)) {
                    if (msg.kind == MSG_KIND_SPECIAL) {
                        /* parent handles special message separately */
                        printf("[SPECIAL][PID=%d] node %d can reach itself: %s\n",
                               msg.pid, msg.node, msg.extra ? "YES" : "NO");
                        fflush(stdout);
                    } else if (msg.extra == -1) {   /* MSG_KIND_NODE, destination */
                        printf("[PID=%d] arrived at node %d | DESTINATION\n",
                               msg.pid, msg.node);
                        printf("[PID=%d] finished\n", msg.pid);
                        fflush(stdout);
                        travelers[i].entity_pos = pos[msg.node];
                        travelers[i].done       = 1;
                        close(travelers[i].pipe_fd);
                        travelers[i].pipe_fd = -1;
                    } else {                        /* MSG_KIND_NODE, in transit */
                        printf("[PID=%d] arrived at node %d | next node: %d\n",
                               msg.pid, msg.node, msg.extra);
                        fflush(stdout);
                        travelers[i].cur_node    = msg.node;
                        travelers[i].nxt_node    = msg.extra;
                        travelers[i].anim_t      = 0.0f;
                        int w = edge_weight(g, msg.node, msg.extra);
                        travelers[i].anim_dur    = WAIT_MS + w * JUMP_MS;
                        travelers[i].entity_pos  = pos[msg.node];
                        travelers[i].initialized = 1;
                    }
                } else if (r == 0) {
                    close(travelers[i].pipe_fd);
                    travelers[i].pipe_fd = -1;
                    travelers[i].done    = 1;
                }
            }
        }

        /* ── update animations ── */
        if (simulation_started) {
            for (int i = 0; i < num_travelers; i++) {
                if (!travelers[i].initialized || travelers[i].done ||
                    travelers[i].nxt_node < 0) continue;
                travelers[i].anim_t += dt_ms / travelers[i].anim_dur;
                if (travelers[i].anim_t > 1.0f) travelers[i].anim_t = 1.0f;
                float t  = travelers[i].anim_t;
                int   c  = travelers[i].cur_node;
                int   nx = travelers[i].nxt_node;
                travelers[i].entity_pos.x = pos[c].x + t * (pos[nx].x - pos[c].x);
                travelers[i].entity_pos.y = pos[c].y + t * (pos[nx].y - pos[c].y);
            }
        }

        /* ── draw ── */
        BeginDrawing();
        ClearBackground((Color){25, 25, 35, 255});

        /* base edges */
        for (int i = 0; i < n; i++)
            for (EdgeNode *e = g->adj[i]; e; e = e->next)
                draw_arrow(pos[i], pos[e->dst], (Color){160, 160, 160, 255}, 1.5f);

        /* path edges per traveler */
        for (int t = 0; t < num_travelers; t++) {
            DijkstraResult *res = travelers[t].result;
            if (!res || !res->found) continue;
            for (int k = 0; k + 1 < res->path_len; k++)
                draw_arrow(pos[res->path[k]], pos[res->path[k + 1]],
                           travelers[t].color, 3.0f);
        }

        /* weight labels */
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

        /* nodes */
        for (int i = 0; i < n; i++) {
            Color nc = (Color){40, 80, 160, 255};
            for (int t = 0; t < num_travelers; t++)
                if (i == travelers[t].src || i == travelers[t].dst)
                    { nc = GREEN; break; }
            DrawCircleV(pos[i], NODE_R, nc);
            DrawCircleLinesV(pos[i], NODE_R, WHITE);
            char lbl[8]; snprintf(lbl, sizeof(lbl), "%d", i);
            int tw = MeasureText(lbl, 20);
            DrawText(lbl, (int)pos[i].x - tw / 2, (int)pos[i].y - 10, 20, WHITE);
        }

        /* traveler entities */
        for (int t = 0; t < num_travelers; t++) {
            DrawCircleV(travelers[t].entity_pos, 14.0f, travelers[t].color);
            DrawCircleLinesV(travelers[t].entity_pos, 14.0f, WHITE);
        }

        /* PLAY / REPLAY button */
        {
            const char *lbl = !simulation_started ? "PLAY"
                            : all_done            ? "REPLAY"
                            : NULL;
            if (lbl) {
                DrawRectangleRec(btn, (Color){0, 160, 0, 255});
                DrawRectangleLinesEx(btn, 2.0f, WHITE);
                int lw = MeasureText(lbl, 20);
                DrawText(lbl,
                         (int)(btn.x + (btn.width  - lw) / 2),
                         (int)(btn.y + (btn.height - 20) / 2),
                         20, WHITE);
            }
        }

        /* all-arrived banner */
        if (simulation_started && all_done) {
            const char *msg_str = "All travelers arrived!";
            int mw = MeasureText(msg_str, 28);
            DrawRectangle(SCREEN_W / 2 - mw / 2 - 20, SCREEN_H / 2 - 35,
                          mw + 40, 70, (Color){0, 0, 0, 210});
            DrawText(msg_str, SCREEN_W / 2 - mw / 2, SCREEN_H / 2 - 14, 28, GREEN);
        }

        /* status bar */
        {
            int bar_h = 20 * num_travelers + 4;
            DrawRectangle(0, SCREEN_H - bar_h, SCREEN_W, bar_h, (Color){0, 0, 0, 200});
            for (int t = 0; t < num_travelers; t++) {
                DijkstraResult *res = travelers[t].result;
                int y = SCREEN_H - bar_h + 2 + t * 20;
                char info[256];
                if (res && res->found) {
                    int off = snprintf(info, sizeof(info), "T%d [pid %d]: ",
                                       t, (int)travelers[t].pid);
                    for (int k = 0; k < res->path_len && off < (int)sizeof(info) - 1; k++) {
                        if (k > 0) off += snprintf(info + off, sizeof(info) - off, " -> ");
                        off += snprintf(info + off, sizeof(info) - off, "%d", res->path[k]);
                    }
                    snprintf(info + off, sizeof(info) - off,
                             "  (cost: %d)", res->total_weight);
                } else {
                    snprintf(info, sizeof(info), "T%d: No path found", t);
                }
                DrawText(info, 10, y, 16, travelers[t].color);
            }
        }

        EndDrawing();
    }

    CloseWindow();

    /* kill any remaining children and wait */
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

    for (int i = 0; i < num_travelers; i++)
        if (travelers[i].result) dijkstra_result_free(travelers[i].result);
    graph_free(g);
    return 0;
}