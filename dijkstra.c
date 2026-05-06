#include <stdlib.h>
#include <limits.h>
#include "dijkstra.h"

typedef struct {
    int node;
    int dist;
} HeapNode;

typedef struct {
    HeapNode *data;
    int size;
    int capacity;
} MinHeap;

static MinHeap *heap_create(int capacity) {
    MinHeap *h = malloc(sizeof(MinHeap));
    h->data = malloc(capacity * sizeof(HeapNode));
    h->size = 0;
    h->capacity = capacity;
    return h;
}

static void heap_free(MinHeap *h) {
    free(h->data);
    free(h);
}

static void swap(HeapNode *a, HeapNode *b) {
    HeapNode tmp = *a;
    *a = *b;
    *b = tmp;
}

static void heap_push(MinHeap *h, int node, int dist) {
    if (h->size == h->capacity) {
        h->capacity *= 2;
        h->data = realloc(h->data, h->capacity * sizeof(HeapNode));
    }
    h->data[h->size] = (HeapNode){node, dist};
    int i = h->size++;
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (h->data[parent].dist > h->data[i].dist) {
            swap(&h->data[parent], &h->data[i]);
            i = parent;
        } else {
            break;
        }
    }
}

static HeapNode heap_pop(MinHeap *h) {
    HeapNode top = h->data[0];
    h->data[0] = h->data[--h->size];
    int i = 0;
    while (1) {
        int left = 2 * i + 1, right = 2 * i + 2, smallest = i;
        if (left < h->size && h->data[left].dist < h->data[smallest].dist)
            smallest = left;
        if (right < h->size && h->data[right].dist < h->data[smallest].dist)
            smallest = right;
        if (smallest == i) break;
        swap(&h->data[i], &h->data[smallest]);
        i = smallest;
    }
    return top;
}

DijkstraResult *dijkstra(Graph *g, int src, int dst) {
    int n = g->num_nodes;
    int *dist = malloc(n * sizeof(int));
    int *prev = malloc(n * sizeof(int));

    for (int i = 0; i < n; i++) {
        dist[i] = INT_MAX;
        prev[i] = -1;
    }
    dist[src] = 0;

    MinHeap *heap = heap_create(n * 2 + 1);
    heap_push(heap, src, 0);

    while (heap->size > 0) {
        HeapNode curr = heap_pop(heap);
        if (curr.dist > dist[curr.node]) continue;

        EdgeNode *edge = g->adj[curr.node];
        while (edge) {
            int new_dist = dist[curr.node] + edge->weight;
            if (new_dist < dist[edge->dst]) {
                dist[edge->dst] = new_dist;
                prev[edge->dst] = curr.node;
                heap_push(heap, edge->dst, new_dist);
            }
            edge = edge->next;
        }
    }

    heap_free(heap);

    DijkstraResult *result = malloc(sizeof(DijkstraResult));

    if (dist[dst] == INT_MAX) {
        result->found = 0;
        result->path = NULL;
        result->path_len = 0;
        result->total_weight = 0;
    } else {
        result->found = 1;
        result->total_weight = dist[dst];

        int len = 0;
        int cur = dst;
        while (cur != -1) {
            len++;
            cur = prev[cur];
        }

        result->path = malloc(len * sizeof(int));
        result->path_len = len;
        cur = dst;
        for (int i = len - 1; i >= 0; i--) {
            result->path[i] = cur;
            cur = prev[cur];
        }
    }

    free(dist);
    free(prev);
    return result;
}

void dijkstra_result_free(DijkstraResult *result) {
    free(result->path);
    free(result);
}

