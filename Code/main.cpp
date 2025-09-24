#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <random>
#include <algorithm>
#include <string>
#include "bellman.h"
#include "tsm.h"

using namespace std;

const int MAX_GRAPH_EDGES = 1000;

void printEdgeFormatted(const int edge[3]) {
    cout << (char)edge[0] << " " << (char)edge[1] << " " << edge[2] << "\n";
}

int createConnectedGraph(int graph[][3], int vertices, int edges, int maxCost) {
    if (edges < vertices - 1 || edges > MAX_GRAPH_EDGES) {
        cerr << "Edge count invalid.\n";
        return -1;
    }

    vector<int> nodeList;
    for (int c = 65; c <= 90; ++c) nodeList.push_back(c); // A-Z
    shuffle(nodeList.begin(), nodeList.end(), mt19937{random_device{}()});

    vector<int> selected(nodeList.begin(), nodeList.begin() + vertices);
    mt19937 rng(random_device{}());
    uniform_int_distribution<int> costDist(1, maxCost);
    uniform_int_distribution<int> pickNode(0, vertices - 1);

    int count = 0;
    set<pair<int, int>> usedEdges;

    for (int i = 0; i < vertices - 1; ++i) {
        int u = selected[i];
        int v = selected[i + 1];
        usedEdges.insert({u, v});
        graph[count][0] = u;
        graph[count][1] = v;
        graph[count][2] = costDist(rng);
        count++;
    }

    while (count < edges) {
        int u = selected[pickNode(rng)];
        int v = selected[pickNode(rng)];
        if (u == v || usedEdges.count({u, v})) continue;
        usedEdges.insert({u, v});
        graph[count][0] = u;
        graph[count][1] = v;
        graph[count][2] = costDist(rng);
        count++;
    }

    ofstream fout("EdgeList.txt");
    for (int i = 0; i < count; ++i)
        fout << graph[i][0] << " " << graph[i][1] << " " << graph[i][2] << "\n";
    fout.close();

    return count;
}

void runBellmanStep(int graph[][3], int edgeCount) {
    char start;
    cout << "Start vertex: ";
    cin >> start;

    static int round = 0;
    static int distance[256], parent[256];
    if (round == 0) {
        fill(distance, distance + 256, -1);
        fill(parent, parent + 256, -1);
    }

    round++;
    BF(graph, edgeCount, start, distance, parent);
    cout << "Step #" << round << ":\n";
    for (int i = 0; i < 128; ++i) {
        if (distance[i] != -1)
            cout << (char)i << ": cost=" << distance[i] << ", from=" << parent[i] << "\n";
    }
}

void runShortestPath(int graph[][3], int edgeCount) {
    char source, dest;
    cout << "Start vertex: ";
    cin >> source;
    cout << "End vertex: ";
    cin >> dest;

    char* pathResult = BF_Path(graph, edgeCount, source, dest);
    cout << "Path: " << pathResult << "\n";
}

void runTSP(int graph[][3], int edgeCount) {
    char start;
    cout << "Starting point for TSP: ";
    cin >> start;

    string route = Traveling(graph, edgeCount, start);
    if (route.empty())
        cout << "No valid cycle found.\n";
    else
        cout << "TSP Route: " << route << "\n";
}

int main() {
    int graph[MAX_GRAPH_EDGES][3];
    int totalEdges = 0;

    cout << "==== GRAPH SETUP ====\n";
    cout << "1. Load from file\n";
    cout << "2. Generate new graph\n";
    cout << "Select option: ";
    int option;
    cin >> option;

    if (option == 1) {
        ifstream fin("EdgeList.txt");
        while (fin >> graph[totalEdges][0] >> graph[totalEdges][1] >> graph[totalEdges][2])
            totalEdges++;
        fin.close();
        cout << "Loaded " << totalEdges << " edges.\n";
    } else if (option == 2) {
        int vCount, eCount, maxWeight;
        cout << "Number of vertices: "; cin >> vCount;
        cout << "Number of edges: "; cin >> eCount;
        cout << "Max edge weight: "; cin >> maxWeight;
        totalEdges = createConnectedGraph(graph, vCount, eCount, maxWeight);
        if (totalEdges < 0) return 1;
        cout << "Graph generated and saved.\n";
    } else {
        cerr << "Invalid choice.\n";
        return 1;
    }

    while (true) {
        cout << "\n==== MAIN MENU ====\n";
        cout << "1. Bellman-Ford (1 step)\n";
        cout << "2. Shortest path\n";
        cout << "3. TSP cycle\n";
        cout << "0. Exit\n";
        cout << "Choose: ";
        int choice;
        cin >> choice;

        switch (choice) {
            case 0: cout << "Goodbye!\n"; return 0;
            case 1: runBellmanStep(graph, totalEdges); break;
            case 2: runShortestPath(graph, totalEdges); break;
            case 3: runTSP(graph, totalEdges); break;
            default: cout << "Invalid option.\n"; break;
        }
    }
}
