#ifndef GRAPH_H
#define GRAPH_H

typedef struct EdgeNode {
    int dst;
    int weight;
    struct EdgeNode *next;
} EdgeNode;

typedef struct {
    int num_nodes;
    EdgeNode **adj;
} Graph;

Graph *graph_create(int num_nodes);
void graph_add_edge(Graph *g, int src, int dst, int weight);
void graph_free(Graph *g);

#endif

