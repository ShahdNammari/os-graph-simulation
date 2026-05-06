#include <stdio.h>
#include <stdlib.h>
#include "graph.h"
#include "dijkstra.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <input_file>\n", argv[0]);
        return 1;
    }

    FILE *fp = fopen(argv[1], "r");
    if (!fp) {
        fprintf(stderr, "Error: cannot open file '%s'\n", argv[1]);
        return 1;
    }

    int n, m;
    if (fscanf(fp, "%d %d", &n, &m) != 2 || n <= 0 || m < 0) {
        fprintf(stderr, "Error: invalid graph header\n");
        fclose(fp);
        return 1;
    }

    Graph *g = graph_create(n);

    for (int i = 0; i < m; i++) {
        int src, dst, weight;
        if (fscanf(fp, "%d %d %d", &src, &dst, &weight) != 3) {
            fprintf(stderr, "Error: invalid edge format\n");
            graph_free(g);
            fclose(fp);
            return 1;
        }
        if (weight < 0) {
            fprintf(stderr, "Error: negative weights are not allowed\n");
            graph_free(g);
            fclose(fp);
            return 1;
        }
        if (src < 0 || src >= n || dst < 0 || dst >= n) {
            fprintf(stderr, "Error: node index out of range\n");
            graph_free(g);
            fclose(fp);
            return 1;
        }
        graph_add_edge(g, src, dst, weight);
    }

    int src, dst;
    if (fscanf(fp, "%d %d", &src, &dst) != 2) {
        fprintf(stderr, "Error: missing query (src dst)\n");
        graph_free(g);
        fclose(fp);
        return 1;
    }
    fclose(fp);

    if (src < 0 || src >= n || dst < 0 || dst >= n) {
        fprintf(stderr, "Error: query node index out of range\n");
        graph_free(g);
        return 1;
    }

    DijkstraResult *result = dijkstra(g, src, dst);

    if (!result->found) {
        printf("No path found\n");
    } else {
        for (int i = 0; i < result->path_len; i++) {
            if (i > 0) printf(" -> ");
            printf("%d", result->path[i]);
        }
        printf("\n%d\n", result->total_weight);
    }

    dijkstra_result_free(result);
    graph_free(g);
    return 0;
}

