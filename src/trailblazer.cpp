#include "trailblazer.h"
#include <queue>
#include<stack>
#include <iostream>
#include "pqueue.h"
#include <limits>
#include "map.h"

using namespace std;

Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {

    graph.resetData();

    Vector<Vertex*> path;

    stack<Vertex*> stack;
    stack.push(start);
    start->visited = true;
    start->setColor(GREEN);

    while (!stack.empty()) {

        Vertex* tmp = stack.top();
        stack.pop();
        tmp->setColor(GREEN);

        if (tmp->name == end->name) {

            while (tmp!=NULL) {
                path.insert(0,tmp);
                tmp = tmp->previous;
            }
            return path;

        } else {
            for (Vertex* neighbour : graph.getNeighbors(tmp)) {

                if (!neighbour->visited) {

                    neighbour->visited = true;

                    neighbour->previous = tmp;
                    neighbour->setColor(YELLOW);
                    stack.push(neighbour);

                }
            }
        }
    }
    return path;

}



Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {

    graph.resetData();

    Vector<Vertex*> path;

    queue<Vertex*> queue;
    queue.push(start);
    start->visited = true;
    start->setColor(GREEN);

    while (!queue.empty()) {

        Vertex* tmp = queue.front();
        queue.pop();
        tmp->setColor(GREEN);

        if (tmp->name == end->name) {

            while (tmp!=NULL) {
                path.insert(0,tmp);
                tmp = tmp->previous;
            }
            return path;

        } else {
            for (Vertex* neighbour : graph.getNeighbors(tmp)) {

                if (!neighbour->visited) {

                    neighbour->visited = true;
                    neighbour->previous = tmp;
                    neighbour->setColor(YELLOW);
                    queue.push(neighbour);
                }
            }
        }
    }
    return path;
}

Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Set<Vertex*> vertices = graph.getVertexSet();
    for (Vertex* v : vertices) {
        v->cost = INT_MAX;
    }

    Vector<Vertex*> path;
    PriorityQueue<Vertex*> queue;
    start->cost = 0;
    queue.enqueue(start, start->cost);

    while (!queue.isEmpty()) {

        Vertex* tmp = queue.dequeue();
        tmp->visited = true;
        tmp->setColor(GREEN);

        if ((tmp->name == end->name) && (end->getColor()==GREEN)) {

            while (tmp!=NULL) {
                path.insert(0,tmp);
                tmp = tmp->previous;
            }
            return path;

        } else {

            for (Vertex* neighbour : graph.getNeighbors(tmp)) {
                if (!neighbour->visited) {

                    double cost = tmp->cost + graph.getEdge(tmp,neighbour)->cost;

                    if (cost < neighbour->cost) {

                        if (neighbour->previous!=NULL) {
                            neighbour->previous = tmp;
                            neighbour->cost = cost;
                            queue.changePriority(neighbour,cost);

                        } else {
                            neighbour->cost = cost;
                            neighbour->previous = tmp;
                            neighbour->setColor(YELLOW);
                            queue.enqueue(neighbour, cost);

                        }
                    }
                }
            }
        }
    }

    return path;
}

Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {

    graph.resetData();
    Set<Vertex*> vertices = graph.getVertexSet();
    for (Vertex* v : vertices) {
        v->cost = INT_MAX;
    }

    Vector<Vertex*> path;
    PriorityQueue<Vertex*> queue;
    start->cost = heuristicFunction(start,end);
    queue.enqueue(start, start->cost);

    while (!queue.isEmpty()) {

        Vertex* tmp = queue.dequeue();
        tmp->visited = true;
        tmp->setColor(GREEN);

        if ((tmp->name == end->name) && (end->getColor()==GREEN)) {

            while (tmp!=NULL) {
                path.insert(0,tmp);
                tmp = tmp->previous;
            }
            return path;

        } else {

            for (Vertex* neighbour : graph.getNeighbors(tmp)) {
                if (!neighbour->visited) {
                    double cost = tmp->cost + graph.getEdge(tmp,neighbour)->cost;

                    if (cost < neighbour->cost) {

                        if (neighbour->previous!=NULL) {
                            neighbour->previous = tmp;
                            neighbour->cost = cost;

                            queue.changePriority(neighbour, cost + heuristicFunction(neighbour,end));

                        } else {
                            neighbour->cost = cost;
                            neighbour->previous = tmp;
                            neighbour->setColor(YELLOW);
                            queue.enqueue(neighbour, cost + heuristicFunction(neighbour,end));

                        }
                    }
                }
            }
        }
    }

    return path;

}


//Union find data structure used to keep track of disjoint sets

struct UnionFind {

    int *parents;
    int *size;

    //constructor
    UnionFind (int N) {

        parents = new int[N];
        size = new int[N];

        for (int i=0; i < N; i++) {
            parents[i] = i;
            size[i] = 1;
        }
    }

    int find (int p) {
        while (p!= parents[p]) {
            p = parents[p];
        }
        return p;
    }

    void join (int p, int q) {
        int i = find(p);
        int j = find(q);
        if (i == j) {
            return;
        }

        if (size[i] < size[j]) {
            parents[i] = j;
            size[j] += size[i];

        } else {
            parents[j] = i;
            size[i] += size[j];
        }

    }

    bool connected (int p, int q) {
        return find(p) == find(q);
    }
};

//Kruskal algorithm to find minimum spanning tree used for generating new random mazes.

Set<Edge*> kruskal(BasicGraph& graph) {

    graph.resetData();
    PriorityQueue<Edge*> queue;
    Set<Edge*> mst;
    UnionFind uf (graph.size());
    Map<Vertex*,int> bijection;

    int index = 0;

    for (Vertex* v : graph.getVertexSet()) {
        bijection[v] = index;
        index++;
    }

    for (Edge* edge : graph.getEdgeSet()) {
        queue.enqueue(edge, edge->cost);
    }

    while (!queue.isEmpty()) {

        Edge* edge = queue.dequeue();
        if (!uf.connected(bijection[edge->start], bijection[edge->finish])) {

            uf.join(bijection[edge->start], bijection[edge->finish]);
            mst.add(edge);

        }
    }

    return mst;
}
