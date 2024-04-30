#include "graph.cpp"


void main() {
	Graph<int> g;

	for (int i = 0; i < 5; i++)
		g.add_vertex(i);
	g.add_edge(1, 4, 4, false);
	g.add_edge(4, 2, 2, false);
	g.add_edge(2, 1, 5, false);
	g.add_edge(2, 3, 3, false);
	g.add_edge(3, 0, 1, false);

	int v = g.find_centroid();
}