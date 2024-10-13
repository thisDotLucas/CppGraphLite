#include <gtest/gtest.h>
#include "../include/CppGraphLite.h"

using namespace graphlite;

int main(int argc, char** argv) 
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(CppGraphLiteTest, EmptyDirectedGraph)
{
	Graph<int, EdgeType::Directed> graph;
	EXPECT_TRUE(graph.size() == 0);
}

TEST(CppGraphLiteTest, EmptyUnDirectedGraph)
{
	Graph<int, EdgeType::Undirected> graph;
	EXPECT_TRUE(graph.size() == 0);
}

TEST(CppGraphLiteTest, SimpleDirectedGraph) 
{
	Graph<int, EdgeType::Directed> graph;
	graph.insert(2, 4);

	EXPECT_TRUE(graph.size() == 2);
	EXPECT_TRUE(graph.inDegree(4) == 1);
	EXPECT_TRUE(graph.inDegree(2) == 0);
	EXPECT_TRUE(graph.outDegree(2) == 1);
	EXPECT_TRUE(graph.outDegree(4) == 0);
}

TEST(CppGraphLiteTest, SimpleUnDirectedGraph) 
{
	Graph<int, EdgeType::Undirected> graph;
	graph.insert(2, 4);

	EXPECT_TRUE(graph.size() == 2);
	EXPECT_TRUE(graph.inDegree(4) == 1);
	EXPECT_TRUE(graph.inDegree(2) == 1);
	EXPECT_TRUE(graph.outDegree(2) == 1);
	EXPECT_TRUE(graph.outDegree(4) == 1);
}

