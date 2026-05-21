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
#include "graph.h"
#include "dijkstra.h"

#define SCREEN_W      900
#define SCREEN_H      700
#define NODE_R         25
#define MAX_NODES      15
#define MAX_TRAVELERS  10
#define JUMP_MS      300.0f
#define WAIT_MS     1000.0f

typedef enum { ANIM_IDLE, ANIM_MOVING, ANIM_WAITING, ANIM_DONE } AnimState;

typedef struct {
    AnimState state;
    int       playing;
    int       path_idx;
    int       jump;
    float     timer_ms;
    Vector2   entity_pos;
} Anim;

typedef struct {
    int            src, dst;
    DijkstraResult *result;
    pid_t          pid;
    int            signaled;
    Anim           anim;
    Color          color;
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

/* drawing helpers */

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

/* animation helpers */

static int edge_weight(Graph *g, int src, int dst) {
    for (EdgeNode *e = g->adj[src]; e; e = e->next)
        if (e->dst == dst) return (e->weight > 0) ? e->weight : 1;
    return 1;
}

static void anim_reset(Anim *a, Vector2 *pos, DijkstraResult *result) {
    a->state      = ANIM_IDLE;
    a->playing    = 0;
    a->path_idx   = 0;
    a->jump       = 0;
    a->timer_ms   = 0.0f;
    a->entity_pos = (result && result->found && result->path_len > 0)
                    ? pos[result->path[0]] : (Vector2){0, 0};
}

static void anim_update(Anim *a, DijkstraResult *result, Graph *g,
                        Vector2 *pos, float dt_ms) {
    if (!a->playing || !result || !result->found) return;

    if (a->state == ANIM_IDLE) {
        if (result->path_len <= 1) { a->state = ANIM_DONE; a->playing = 0; return; }
        a->state = ANIM_MOVING;
    }

    if (a->state == ANIM_MOVING) {
        int from = result->path[a->path_idx];
        int to   = result->path[a->path_idx + 1];
        int w    = edge_weight(g, from, to);

        float t = (float)a->jump / (float)w;
        a->entity_pos.x = pos[from].x + t * (pos[to].x - pos[from].x);
        a->entity_pos.y = pos[from].y + t * (pos[to].y - pos[from].y);

        a->timer_ms += dt_ms;
        if (a->timer_ms >= JUMP_MS) {
            a->timer_ms -= JUMP_MS;
            a->jump++;
            if (a->jump >= w) {
                a->entity_pos = pos[to];
                a->path_idx++;
                a->jump     = 0;
                a->timer_ms = 0.0f;
                if (a->path_idx + 1 >= result->path_len) {
                    a->state   = ANIM_DONE;
                    a->playing = 0;
                } else {
                    a->state = ANIM_WAITING;
                }
            }
        }
        return;
    }

    if (a->state == ANIM_WAITING) {
        a->timer_ms += dt_ms;
        if (a->timer_ms >= WAIT_MS) {
            a->timer_ms = 0.0f;
            a->state    = ANIM_MOVING;
        }
    }
}

/* file I/O */

static void skip_to_token(FILE *fp) {
    int c;
    while ((c = fgetc(fp)) != EOF) {
        if (c == '#') { while ((c = fgetc(fp)) != EOF && c != '\n'); }
        else if (!isspace((unsigned char)c)) { ungetc(c, fp); return; }
    }
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
        travelers[i].result   = NULL;
        travelers[i].pid      = -1;
        travelers[i].signaled = 0;
    }

    fclose(fp);
    return g;
}

/* main */

int main(int argc, char *argv[]) {
    if (argc != 2) { fprintf(stderr, "Usage: %s <input_file>\n", argv[0]); return 1; }

    Traveler travelers[MAX_TRAVELERS] = {0};
    int num_travelers = 0;

    Graph *g = read_graph(argv[1], travelers, &num_travelers);
    if (!g) return 1;

    int n = g->num_nodes;

    /* compute Dijkstra for every traveler (parent does all path work) */
    for (int i = 0; i < num_travelers; i++) {
        travelers[i].color = PALETTE[i % MAX_TRAVELERS];
        if (travelers[i].src >= 0 && travelers[i].src < n &&
            travelers[i].dst >= 0 && travelers[i].dst < n)
            travelers[i].result = dijkstra(g, travelers[i].src, travelers[i].dst);
    }

    /* fork ALL children before touching raylib or animations */
    for (int i = 0; i < num_travelers; i++) {
        if (!travelers[i].result || !travelers[i].result->found) continue;
        pid_t pid = fork();
        if (pid == 0) {
            /* child: print and sleep until signaled */
            printf("[%d] started\n", getpid());
            fflush(stdout);
            while (1) sleep(1);
            exit(0);
        } else if (pid > 0) {
            travelers[i].pid = pid;
        } else {
            perror("fork");
        }
    }

    /* compute positions */
    Vector2 pos[MAX_NODES] = {0};
    compute_positions(n, pos);

    /* initialise ALL animations together, after all forks are done */
    for (int i = 0; i < num_travelers; i++) {
        anim_reset(&travelers[i].anim, pos, travelers[i].result);

    }

    Rectangle btn = { SCREEN_W - 130.0f, 10.0f, 110.0f, 40.0f };

    InitWindow(SCREEN_W, SCREEN_H, "Graph Simulation");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt_ms = GetFrameTime() * 1000.0f;

        /* input */
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) &&
            CheckCollisionPointRec(GetMousePosition(), btn)) {
            int all_done = 1, any_playing = 0;
            for (int i = 0; i < num_travelers; i++) {
                if (travelers[i].anim.state != ANIM_DONE) all_done = 0;
                if (travelers[i].anim.playing)            any_playing = 1;
            }
            if (all_done) {
                /* REPLAY: reset all animations (children are already dead, that's OK) */
                for (int i = 0; i < num_travelers; i++) {
                    anim_reset(&travelers[i].anim, pos, travelers[i].result);
                    if (travelers[i].result && travelers[i].result->found) {
                        travelers[i].anim.playing = 1;
                        travelers[i].anim.state   = ANIM_IDLE;
                    }
                    travelers[i].signaled = 0;
                }
            } else {
                /* PLAY / STOP toggle */
                for (int i = 0; i < num_travelers; i++)
                    if (travelers[i].anim.state != ANIM_DONE)
                        travelers[i].anim.playing = !any_playing;
            }
        }

        /* update + signal finished children */
        for (int i = 0; i < num_travelers; i++) {
            anim_update(&travelers[i].anim, travelers[i].result, g, pos, dt_ms);
            if (travelers[i].anim.state == ANIM_DONE &&
                !travelers[i].signaled && travelers[i].pid > 0) {
                kill(travelers[i].pid, SIGTERM);
                travelers[i].signaled = 1;
            }
        }

        /* draw */
        BeginDrawing();
        ClearBackground((Color){25, 25, 35, 255});

        /* base edges (gray) */
        for (int i = 0; i < n; i++)
            for (EdgeNode *e = g->adj[i]; e; e = e->next)
                draw_arrow(pos[i], pos[e->dst], (Color){160, 160, 160, 255}, 1.5f);

        /* path edges per traveler (colored overlay) */
        for (int t = 0; t < num_travelers; t++) {
            DijkstraResult *res = travelers[t].result;
            if (!res || !res->found) continue;
            for (int k = 0; k + 1 < res->path_len; k++)
                draw_arrow(pos[res->path[k]], pos[res->path[k + 1]],
                           travelers[t].color, 3.0f);
        }

        /* weight labels on top */
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
                if (i == travelers[t].src || i == travelers[t].dst) { nc = GREEN; break; }
            DrawCircleV(pos[i], NODE_R, nc);
            DrawCircleLinesV(pos[i], NODE_R, WHITE);
            char lbl[8]; snprintf(lbl, sizeof(lbl), "%d", i);
            int tw = MeasureText(lbl, 20);
            DrawText(lbl, (int)pos[i].x - tw / 2, (int)pos[i].y - 10, 20, WHITE);
        }

        /* traveler entities */
        for (int t = 0; t < num_travelers; t++) {
            if (!travelers[t].result || !travelers[t].result->found) continue;
            DrawCircleV(travelers[t].anim.entity_pos, 14.0f, travelers[t].color);
            DrawCircleLinesV(travelers[t].anim.entity_pos, 14.0f, WHITE);
        }

        /* PLAY / STOP / REPLAY button */
        {
            int all_done = 1, any_playing = 0;
            for (int i = 0; i < num_travelers; i++) {
                if (travelers[i].anim.state != ANIM_DONE) all_done = 0;
                if (travelers[i].anim.playing)            any_playing = 1;
            }
            const char *lbl = all_done ? "REPLAY" : (any_playing ? "STOP" : "PLAY");
            Color bc = any_playing ? RED : (Color){0, 160, 0, 255};
            DrawRectangleRec(btn, bc);
            DrawRectangleLinesEx(btn, 2.0f, WHITE);
            int lw = MeasureText(lbl, 20);
            DrawText(lbl,
                     (int)(btn.x + (btn.width  - lw) / 2),
                     (int)(btn.y + (btn.height - 20) / 2),
                     20, WHITE);

            if (all_done) {
                const char *msg = "All travelers arrived!";
                int mw = MeasureText(msg, 28);
                DrawRectangle(SCREEN_W / 2 - mw / 2 - 20, SCREEN_H / 2 - 35,
                              mw + 40, 70, (Color){0, 0, 0, 210});
                DrawText(msg, SCREEN_W / 2 - mw / 2, SCREEN_H / 2 - 14, 28, GREEN);
            }
        }

        /* status bar — one line per traveler */
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

    /* signal any remaining children and wait for all */
    for (int i = 0; i < num_travelers; i++) {
        if (travelers[i].pid > 0) {
            if (!travelers[i].signaled) kill(travelers[i].pid, SIGTERM);
            waitpid(travelers[i].pid, NULL, 0);
        }
    }

    for (int i = 0; i < num_travelers; i++)
        if (travelers[i].result) dijkstra_result_free(travelers[i].result);
    graph_free(g);
    return 0;
}