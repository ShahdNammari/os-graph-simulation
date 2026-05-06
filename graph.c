#include <stdlib.h>
#include "graph.h"

Graph *graph_create(int num_nodes) {
    Graph *g = malloc(sizeof(Graph));
    g->num_nodes = num_nodes;
    g->adj = calloc(num_nodes, sizeof(EdgeNode *));
    return g;
}

void graph_add_edge(Graph *g, int src, int dst, int weight) {
    EdgeNode *node = malloc(sizeof(EdgeNode));
    node->dst = dst;
    node->weight = weight;
    node->next = g->adj[src];
    g->adj[src] = node;
}

void graph_free(Graph *g) {
    for (int i = 0; i < g->num_nodes; i++) {
        EdgeNode *curr = g->adj[i];
        while (curr) {
            EdgeNode *next = curr->next;
            free(curr);
            curr = next;
        }
    }
    free(g->adj);
    free(g);
}

