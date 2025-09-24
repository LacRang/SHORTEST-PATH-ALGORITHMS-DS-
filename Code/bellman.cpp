#include "bellman.h"
#include <cstdlib> // qsort
#include <cstring>

// Hàm chính Bellman-Ford cơ bản (mỗi lần gọi chỉ thực hiện 1 step relax)
void BF(int edgeList[][3], int numEdges, char start, int BFValue[], int BFPrev[]) {
    char vertices[200];
    int vCount = 0;
    for (int e = 0; e < numEdges; ++e) {
        char u = (char)edgeList[e][0];
        char v = (char)edgeList[e][1];
        bool found = false;
        for (int i = 0; i < vCount; ++i)
            if (vertices[i] == u) { found = true; break; }
        if (!found) vertices[vCount++] = u;
        found = false;
        for (int i = 0; i < vCount; ++i)
            if (vertices[i] == v) { found = true; break; }
        if (!found) vertices[vCount++] = v;
    }

    for (int i = 0; i < vCount - 1; ++i)
        for (int j = i + 1; j < vCount; ++j)
            if (vertices[i] > vertices[j]) {
                char t = vertices[i];
                vertices[i] = vertices[j];
                vertices[j] = t;
            }

    int charToIdx[256];
    for (int i = 0; i < 256; ++i) charToIdx[i] = -1;
    for (int i = 0; i < vCount; ++i)
        charToIdx[(unsigned char)vertices[i]] = i;

    struct E { int u, v, w; } edges2[2000];
    for (int e = 0; e < numEdges; ++e) {
        edges2[e].u = charToIdx[(unsigned char)edgeList[e][0]];
        edges2[e].v = charToIdx[(unsigned char)edgeList[e][1]];
        edges2[e].w =              edgeList[e][2];
    }
    for (int i = 0; i < numEdges - 1; ++i)
        for (int j = i + 1; j < numEdges; ++j)
            if (edges2[j].u < edges2[i].u ||
               (edges2[j].u == edges2[i].u && edges2[j].v < edges2[i].v)) {
                E tmp = edges2[i];
                edges2[i] = edges2[j];
                edges2[j] = tmp;
            }

    int oldVal[200];
    for (int i = 0; i < vCount; ++i)
        oldVal[i] = BFValue[i];

    int s = charToIdx[(unsigned char)start];
    if (s >= 0 && BFValue[s] == -1) {
        BFValue[s] = 0;
        oldVal[s] = 0;
    }

    for (int e = 0; e < numEdges; ++e) {
        int u = edges2[e].u;
        int v = edges2[e].v;
        int w = edges2[e].w;
        if (u < 0 || v < 0) continue;
        if (oldVal[u] != -1) {
            int cand = oldVal[u] + w;
            if (BFValue[v] == -1 || cand < BFValue[v]) {
                BFValue[v] = cand;
                BFPrev[v] = u;
            }
        }
    }
}

// ===============================
// Các hàm phụ cho BF_Path

int edgeCompareFunc(const void* a, const void* b) {
    const int* ea = (const int*)a;
    const int* eb = (const int*)b;

    if (ea[0] != eb[0]) return ea[0] - eb[0];
    if (ea[1] != eb[1]) return ea[1] - eb[1];
    return ea[2] - eb[2];
}

void sortChars(char a[], int n) {
    for (int i = 0; i < n - 1; ++i) {
        int minIdx = i;
        for (int j = i + 1; j < n; ++j) {
            if (a[j] < a[minIdx]) {
                minIdx = j;
            }
        }
        if (minIdx != i) {
            char tmp = a[i];
            a[i] = a[minIdx];
            a[minIdx] = tmp;
        }
    }
}

void reverseIntArray(int arr[], int len) {
    for (int i = 0; i < len / 2; ++i) {
        int tmp = arr[i];
        arr[i] = arr[len - 1 - i];
        arr[len - 1 - i] = tmp;
    }
}

void BellmanFord(int edgeList[][3], int numEdges,
                 int BFValue[], int BFPrev[],
                 int charToIndex[256], char indexToChar[], int n,
                 char start_vertex) {
    for (int i = 0; i < n; ++i) {
        BFValue[i] = -1;
        BFPrev[i] = -1;
    }

    int startIdx = charToIndex[(int)start_vertex];
    BFValue[startIdx] = 0;

    for (int k = 0; k < n - 1; ++k) {
        for (int i = 0; i < numEdges; ++i) {
            int u = charToIndex[edgeList[i][0]];
            int v = charToIndex[edgeList[i][1]];
            int w = edgeList[i][2];

            if (BFValue[u] == -1) continue;

            int newCost = BFValue[u] + w;

            if (BFValue[v] == -1 || newCost < BFValue[v]) {
                BFValue[v] = newCost;
                BFPrev[v] = u;
            } else if (newCost == BFValue[v]) {
                if (indexToChar[u] < indexToChar[BFPrev[v]]) {
                    BFPrev[v] = u;
                }
            }
        }
    }
}

// ===============================
// Hàm trả về đường đi dạng chuỗi từ start đến goal

char* BF_Path(int edgeList[][3], int numEdges,
              char start_vertex, char goal_vertex) {
    int charExist[256] = {0};
    for (int i = 0; i < numEdges; ++i) {
        charExist[edgeList[i][0]] = 1;
        charExist[edgeList[i][1]] = 1;
    }

    if (!charExist[(int)start_vertex] || !charExist[(int)goal_vertex]) {
        return (char*)"No path";
    }

    char indexToChar[100];
    int  charToIndex[256];
    int n = 0;

    for (int i = 0; i < 256; ++i) {
        if (charExist[i]) {
            indexToChar[n] = (char)i;
            charToIndex[i] = n;
            ++n;
        }
    }

    sortChars(indexToChar, n);
    for (int i = 0; i < n; ++i) {
        charToIndex[(int)indexToChar[i]] = i;
    }

    qsort(edgeList, numEdges, sizeof(edgeList[0]), edgeCompareFunc);

    int BFValue[100], BFPrev[100];
    BellmanFord(edgeList, numEdges, BFValue, BFPrev, charToIndex, indexToChar, n, start_vertex);

    int startIdx = charToIndex[(int)start_vertex];
    int goalIdx = charToIndex[(int)goal_vertex];

    if (BFValue[goalIdx] == -1) {
        return (char*)"No path";
    }

    int path[100];
    int pathLength = 0;

    for (int v = goalIdx; v != -1; v = BFPrev[v]) {
        if (pathLength >= 100) {
            return (char*)"Path too long";
        }
        path[pathLength++] = v;
        if (v == startIdx) break;
    }

    if (path[pathLength - 1] != startIdx) {
        return (char*)"No path";
    }

    reverseIntArray(path, pathLength);

    static char result[500];
    int pos = 0;

    for (int i = 0; i < pathLength; ++i) {
        result[pos++] = indexToChar[path[i]];
        if (i < pathLength - 1) result[pos++] = ' ';
        if (pos >= sizeof(result) - 1) return (char*)"Result too long";
    }

    result[pos] = '\0';
    return result;
}
