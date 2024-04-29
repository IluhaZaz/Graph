#include <gtest/gtest.h>
#include "graph.cpp"

TEST(EdgeTests, ConstructorCheck) {
	Edge<int> e(2, 6);
	EXPECT_EQ(e.from, 2);
	EXPECT_EQ(e.to, 6);
	EXPECT_EQ(e.dist, 0);
}


TEST(EdgeTests, ConstructorCheck2) {
	Edge<int, int> e(2, 6, 12);
	EXPECT_EQ(e.from, 2);
	EXPECT_EQ(e.to, 6);
	EXPECT_EQ(e.dist, 12);
}

TEST(GraphTests, VertexCheck) {
	Graph<int> g;
	g.add_vertex(2);
	g.add_vertex(4);
	g.add_vertex(8);
	EXPECT_TRUE(g.has_vertex(2));
	EXPECT_TRUE(g.has_vertex(4));
	EXPECT_TRUE(g.has_vertex(8));
	EXPECT_FALSE(g.has_vertex(-2));
	EXPECT_FALSE(g.has_vertex(11));

	g.remove_vertex(2);
	g.remove_vertex(4);
	EXPECT_TRUE(g.has_vertex(8));
	EXPECT_FALSE(g.has_vertex(2));
	EXPECT_FALSE(g.has_vertex(4));
}

TEST(GraphTests, EdgeCheck) {
	Graph<int> g;
	g.add_vertex(1);
	g.add_vertex(2);
	g.add_vertex(3);
	g.add_vertex(4);
	g.add_vertex(5);

	g.add_edge(1, 2, 3);
	g.add_edge(2, 1, 6);
	g.add_edge(2, 3, 1);
	g.add_edge(3, 4, 12);

	EXPECT_TRUE(g.has_edge(1, 2));
	EXPECT_TRUE(g.has_edge(Edge<int, double>(1, 2, 3)));
	EXPECT_TRUE(g.has_edge(2, 1));
	EXPECT_TRUE(g.has_edge(2, 3));
	EXPECT_TRUE(g.has_edge(3, 4));
	EXPECT_TRUE(g.has_edge(Edge<int, double>(3, 4, 12)));

	EXPECT_FALSE(g.has_edge(5, 4));
	EXPECT_FALSE(g.has_edge(3, 2));
	EXPECT_FALSE(g.has_edge(6, 7));
	EXPECT_FALSE(g.has_edge(Edge<int, double>(4, 3, 12)));
	EXPECT_FALSE(g.has_edge(Edge<int, double>(1, 2, 9)));

	g.remove_edge(1, 2);
	EXPECT_FALSE(g.has_edge(Edge<int, double>(1, 2, 3)));

	g.remove_vertex(2);
	EXPECT_FALSE(g.has_edge(Edge<int, double>(2, 1, 6)));
	EXPECT_FALSE(g.has_edge(2, 3));
	EXPECT_TRUE(g.has_edge(Edge<int, double>(3, 4, 12)));
}

TEST(GraphTests, WalkCheck) {
	Graph<int> g;
	for (int i = 0; i < 9; i++)
		g.add_vertex(i);
	g.add_edge(1, 2, 3);
	g.add_edge(2, 1, 6);
	g.add_edge(2, 3, 1);
	g.add_edge(3, 4, 12);
	g.add_edge(4, 6, 3);

	g.walk(2);
}

TEST(GraphTests, ShortestPathCheck) {
	Graph<int> g;
	for (int i = 0; i < 8; i++)
		g.add_vertex(i);
	g.add_edge(0, 1, 3);
	g.add_edge(1, 2, 1);
	g.add_edge(1, 3, 3);
	g.add_edge(2, 6, 2);
	g.add_edge(3, 6, 1);
	g.add_edge(4, 5, 4);
	g.add_edge(5, 6, 3);
	g.add_edge(6, 7, 5);

	map<int, int> rs = g.shortest_path(0, 7);
}