#include "tsm.h"
#include <vector>
#include <string>
#include <utility>

std::string Traveling(int edges[][3], int eCnt, char start) {
    const int INF = 0x3f3f3f3f;

    bool present[256] = {0};
    present[(unsigned char)start] = true;
    for (int i = 0; i < eCnt; i++) {
        present[(unsigned char)edges[i][0]] = true;
        present[(unsigned char)edges[i][1]] = true;
    }

    int mapC[256];
    for (int i = 0; i < 256; i++) mapC[i] = -1;
    std::vector<char> rev;
    for (int c = 0; c < 256; c++) {
        if (present[c]) {
            mapC[c] = rev.size();
            rev.push_back((char)c);
        }
    }
    int n = rev.size(), startId = mapC[(unsigned char)start];

    std::vector<std::vector<std::pair<int,int>>> graph(n);
    for (int i = 0; i < eCnt; i++) {
        int u = mapC[(unsigned char)edges[i][0]];
        int v = mapC[(unsigned char)edges[i][1]];
        int w = edges[i][2];
        graph[u].push_back({v, w});
    }

    for (int u = 0; u < n; u++) {
        int m = graph[u].size();
        for (int i = 0; i < m; i++) {
            for (int j = i + 1; j < m; j++) {
                if (graph[u][i].first == graph[u][j].first) {
                    int bestW = std::min(graph[u][i].second, graph[u][j].second);
                    graph[u][i].second = bestW;
                    graph[u].erase(graph[u].begin() + j);
                    m--; j--;
                }
            }
        }
    }

    int ALL = (1 << n) - 1;
    std::vector<std::vector<int>> memo(1<<n, std::vector<int>(n, -1));
    std::vector<std::vector<int>> pick(1<<n, std::vector<int>(n, -1));

    struct Solver {
        int n, startId, ALL, INF;
        std::vector<std::vector<std::pair<int,int>>>* g;
        std::vector<std::vector<int>>* memo;
        std::vector<std::vector<int>>* pick;

        int dfs(int mask, int last) {
            if (mask == ALL) {
                int best = INF;
                for (auto &e : (*g)[last]) {
                    if (e.first == startId && e.second < best)
                        best = e.second;
                }
                return best;
            }
            int &res = (*memo)[mask][last];
            if (res != -1) return res;

            int best = INF;
            for (auto &e : (*g)[last]) {
                int to = e.first, w = e.second;
                if (mask & (1<<to)) continue;
                int sub = dfs(mask | (1<<to), to);
                if (sub == INF) continue;
                sub += w;
                if (sub < best) {
                    best = sub;
                    (*pick)[mask][last] = to;
                }
            }
            return res = best;
        }
    } solver { n, startId, ALL, INF, &graph, &memo, &pick };

    int totalCost = solver.dfs(1<<startId, startId);
    if (totalCost >= INF) return "";

    std::vector<int> order;
    int curMask = 1<<startId, cur = startId;
    while (true) {
        order.push_back(cur);
        int nxt = pick[curMask][cur];
        if (nxt < 0) break;
        curMask |= 1<<nxt;
        cur = nxt;
    }
    order.push_back(startId);

    std::string out;
    for (int i = 0; i < (int)order.size(); i++) {
        if (i) out.push_back(' ');
        out.push_back(rev[order[i]]);
    }
    return out;
}
