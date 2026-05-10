#include "raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "graph.h"
#include "dijkstra.h"

#define SCREEN_W   900
#define SCREEN_H   700
#define NODE_R      25
#define MAX_NODES   15
#define JUMP_MS   300.0f   /* ms per jump on an edge          */
#define WAIT_MS  1000.0f   /* ms waiting at intermediate node */

/* ── animation state ────────────────────────────────────────────────────────*/

typedef enum { ANIM_IDLE, ANIM_MOVING, ANIM_WAITING, ANIM_DONE } AnimState;

typedef struct {
    AnimState state;
    int       playing;
    int       path_idx;    /* entity moves from path[path_idx] to path[path_idx+1] */
    int       jump;        /* discrete jump index within current edge (0..weight) */
    float     timer_ms;    /* time accumulated in current phase */
    Vector2   entity_pos;  /* current on-screen position of the entity */
} Anim;

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

/* ── animation helpers ───────────────────────────────────────────────────────*/

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

    /* kick off */
    if (a->state == ANIM_IDLE) {
        if (result->path_len <= 1) { a->state = ANIM_DONE; a->playing = 0; return; }
        a->state = ANIM_MOVING;
    }

    if (a->state == ANIM_MOVING) {
        int from = result->path[a->path_idx];
        int to   = result->path[a->path_idx + 1];
        int w    = edge_weight(g, from, to);

        /* place entity at current discrete jump position */
        float t = (float)a->jump / (float)w;
        a->entity_pos.x = pos[from].x + t * (pos[to].x - pos[from].x);
        a->entity_pos.y = pos[from].y + t * (pos[to].y - pos[from].y);

        a->timer_ms += dt_ms;
        if (a->timer_ms >= JUMP_MS) {
            a->timer_ms -= JUMP_MS;
            a->jump++;
            if (a->jump >= w) {
                /* arrived at next node */
                a->entity_pos = pos[to];
                a->path_idx++;
                a->jump     = 0;
                a->timer_ms = 0.0f;
                if (a->path_idx + 1 >= result->path_len) {
                    a->state   = ANIM_DONE;
                    a->playing = 0;
                } else {
                    a->state = ANIM_WAITING; /* wait 1 s at intermediate node */
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

/* ── file I/O ────────────────────────────────────────────────────────────────*/

static Graph *read_graph(const char *filename, int *q_src, int *q_dst) {
    FILE *fp = fopen(filename, "r");
    if (!fp) { fprintf(stderr, "Error: cannot open '%s'\n", filename); return NULL; }
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
    *q_src = *q_dst = -1;
    fscanf(fp, "%d %d", q_src, q_dst);
    fclose(fp);
    return g;
}

/* ── main ────────────────────────────────────────────────────────────────────*/

int main(int argc, char *argv[]) {
    if (argc != 2) { fprintf(stderr, "Usage: %s <input_file>\n", argv[0]); return 1; }

    int q_src, q_dst;
    Graph *g = read_graph(argv[1], &q_src, &q_dst);
    if (!g) return 1;

    int n = g->num_nodes;
    DijkstraResult *result = NULL;
    if (q_src >= 0 && q_src < n && q_dst >= 0 && q_dst < n)
        result = dijkstra(g, q_src, q_dst);

    Vector2 pos[MAX_NODES] = {0};
    compute_positions(n, pos);

    Anim anim = {0};
    anim_reset(&anim, pos, result);

    Rectangle btn = { SCREEN_W - 130.0f, 10.0f, 110.0f, 40.0f };

    InitWindow(SCREEN_W, SCREEN_H, "Graph Simulation");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt_ms = GetFrameTime() * 1000.0f;

        /* ── input ── */
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) &&
            CheckCollisionPointRec(GetMousePosition(), btn) &&
            result && result->found) {
            if (anim.state == ANIM_DONE) {
                anim_reset(&anim, pos, result);
                anim.playing = 1;
                anim.state   = ANIM_IDLE;
            } else {
                anim.playing = !anim.playing;
            }
        }

        anim_update(&anim, result, g, pos, dt_ms);

        /* ── draw ── */
        BeginDrawing();
        ClearBackground((Color){25, 25, 35, 255});

        /* edges */
        for (int i = 0; i < n; i++) {
            for (EdgeNode *e = g->adj[i]; e; e = e->next) {
                int on_path = 0;
                if (result && result->found)
                    for (int k = 0; k + 1 < result->path_len; k++)
                        if (result->path[k] == i && result->path[k + 1] == e->dst)
                            { on_path = 1; break; }

                draw_arrow(pos[i], pos[e->dst],
                           on_path ? YELLOW : (Color){160, 160, 160, 255},
                           on_path ? 3.0f   : 1.5f);

                /* weight label, offset perpendicular to edge */
                float mx = (pos[i].x + pos[e->dst].x) * 0.5f;
                float my = (pos[i].y + pos[e->dst].y) * 0.5f;
                float ex = pos[e->dst].x - pos[i].x;
                float ey = pos[e->dst].y - pos[i].y;
                float el = sqrtf(ex * ex + ey * ey);
                if (el > 0) { mx += (-ey / el) * 14.0f; my += (ex / el) * 14.0f; }
                char ws[16]; snprintf(ws, sizeof(ws), "%d", e->weight);
                DrawText(ws, (int)mx - 6, (int)my - 8, 16,
                         on_path ? YELLOW : WHITE);
            }
        }

        /* nodes */
        for (int i = 0; i < n; i++) {
            Color nc = (Color){40, 80, 160, 255};
            if (result && result->found) {
                if (i == q_src || i == q_dst) nc = GREEN;
                else for (int k = 0; k < result->path_len; k++)
                    if (result->path[k] == i) { nc = (Color){0, 160, 255, 255}; break; }
            }
            DrawCircleV(pos[i], NODE_R, nc);
            DrawCircleLinesV(pos[i], NODE_R, WHITE);
            char lbl[8]; snprintf(lbl, sizeof(lbl), "%d", i);
            int tw = MeasureText(lbl, 20);
            DrawText(lbl, (int)pos[i].x - tw / 2, (int)pos[i].y - 10, 20, WHITE);
        }

        /* entity — always visible while path exists */
        if (result && result->found) {
            DrawCircleV(anim.entity_pos, 14.0f, ORANGE);
            DrawCircleLinesV(anim.entity_pos, 14.0f, WHITE);
        }

        /* play / stop / replay button */
        if (result && result->found) {
            const char *lbl = anim.playing              ? "STOP"
                            : (anim.state == ANIM_DONE)  ? "REPLAY"
                                                         : "PLAY";
            Color bc = anim.playing ? RED : (Color){0, 160, 0, 255};
            DrawRectangleRec(btn, bc);
            DrawRectangleLinesEx(btn, 2.0f, WHITE);
            int lw = MeasureText(lbl, 20);
            DrawText(lbl,
                     (int)(btn.x + (btn.width  - lw) / 2),
                     (int)(btn.y + (btn.height - 20) / 2),
                     20, WHITE);
        }

        /* arrived message */
        if (anim.state == ANIM_DONE) {
            const char *msg = "Arrived at destination!";
            int mw = MeasureText(msg, 28);
            DrawRectangle(SCREEN_W / 2 - mw / 2 - 20, SCREEN_H / 2 - 35,
                          mw + 40, 70, (Color){0, 0, 0, 210});
            DrawText(msg, SCREEN_W / 2 - mw / 2, SCREEN_H / 2 - 14, 28, GREEN);
        }

        /* status bar */
        DrawRectangle(0, SCREEN_H - 40, SCREEN_W, 40, (Color){0, 0, 0, 180});
        if (result && result->found) {
            char info[256] = "Path: ";
            char tmp[16];
            for (int k = 0; k < result->path_len; k++) {
                if (k > 0) strcat(info, " -> ");
                snprintf(tmp, sizeof(tmp), "%d", result->path[k]);
                strcat(info, tmp);
            }
            char cost[32];
            snprintf(cost, sizeof(cost), "  (cost: %d)", result->total_weight);
            strcat(info, cost);
            DrawText(info, 10, SCREEN_H - 28, 18, YELLOW);
        } else if (result) {
            DrawText("No path found", 10, SCREEN_H - 28, 18, RED);
        }

        EndDrawing();
    }

    CloseWindow();
    if (result) dijkstra_result_free(result);
    graph_free(g);
    return 0;
}