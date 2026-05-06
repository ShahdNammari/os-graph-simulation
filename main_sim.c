#include "raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "graph.h"
#include "dijkstra.h"

#define SCREEN_W  900
#define SCREEN_H  700
#define NODE_R     25
#define MAX_NODES  15

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

    InitWindow(SCREEN_W, SCREEN_H, "Graph Simulation");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
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