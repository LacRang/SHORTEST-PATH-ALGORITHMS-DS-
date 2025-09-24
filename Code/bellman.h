#ifndef BELLMAN_H
#define BELLMAN_H

void BF(int edgeList[][3], int numEdges, char start, int BFValue[], int BFPrev[]);
char* BF_Path(int edgeList[][3], int numEdges, char start_vertex, char goal_vertex);

#endif
