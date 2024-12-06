#include <gtest/gtest.h>
#include "../include/CppGraphLite.h"
#include <random>

using namespace graphlite;

namespace
{
	Graph<int, EdgeType::Directed, int> generateRandomDirectedGraph(int n, int minWeight, int maxWeight)
	{
		Graph<int, EdgeType::Directed, int> graph;

		// Add vertices
		for (int i = 1; i <= n; ++i)
			graph.insert(i);

		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<> weightDist(minWeight, maxWeight);
		std::uniform_int_distribution<> vertexDist(1, n);

		// Ensure connectivity 
		for (int i = 1; i < n; ++i)
		{
			int weight = weightDist(gen);
			graph.insert((int)i, i + 1, weight);
		}

		// Add additional random edges
		for (int i = 0; i < n; ++i)
		{
			int from = vertexDist(gen);
			int to = vertexDist(gen);

			if (from != to)
			{
				int weight = weightDist(gen);
				graph.insert(from, to, weight);
			}
		}

		return graph;
	}

	void printGraphRepresentation(const Graph<int, EdgeType::Directed, int>& graph)
	{
		std::ostringstream oss;
		for (int vertex : graph.vertices()) 
		{
			oss << std::setw(2) << vertex << " -> ";
			for (int neighbor : graph.edges(vertex)) 
			{
				auto weight = graph.edgeData(vertex, neighbor).value();
				oss << neighbor << "(" << weight << ") ";
			}

			oss << "\n";
		}

		std::cout << oss.str() << std::endl;
	}

    void checkRandomGraph(int numVertices, int minWeight, int maxWeight)
    {
        std::cout << "Random graph: numVertices: " << numVertices << ", minWeight: " << minWeight << ", maxWeight: " << maxWeight << "\n";

		auto graph = generateRandomDirectedGraph(numVertices, minWeight, maxWeight);

		printGraphRepresentation(graph);

		{
			graphlite::algorithm::GraphLiteBFS<decltype(graph)> bfs;

			std::vector<int> visitedOrder;
			bfs.BFS(graph, 1, [&](std::optional<int> from, int to) {
				visitedOrder.push_back(to);
				return false;
			});

			EXPECT_EQ(visitedOrder.size(), numVertices); // Ensure all vertices are visited
		}

		{
			graphlite::algorithm::GraphLiteDFS<decltype(graph)> dfs;

			std::vector<int> visitedOrder;
			dfs.DFS(graph, 1, [&](std::optional<int> from, int to) {
				visitedOrder.push_back(to);
				return false;
			});

			EXPECT_EQ(visitedOrder.size(), numVertices); // Ensure all vertices are visited
		}

		{
			graphlite::algorithm::GraphLiteDijkstra<decltype(graph)> dijkstra;

			auto distances = dijkstra.Dijkstra(graph, 1, [](int vertex) { return false; });

			auto path = dijkstra.getPath(numVertices);
			EXPECT_FALSE(path.empty()); // Ensure a path exists to the last vertex
		}
    }
}

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

TEST(CppGraphLiteTest, EmptyUndirectedGraph)
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

TEST(CppGraphLiteTest, SimpleUndirectedGraph) 
{
	Graph<int, EdgeType::Undirected> graph;
	graph.insert(2, 4);

	EXPECT_TRUE(graph.size() == 2);
	EXPECT_TRUE(graph.inDegree(4) == 1);
	EXPECT_TRUE(graph.inDegree(2) == 1);
	EXPECT_TRUE(graph.outDegree(2) == 1);
	EXPECT_TRUE(graph.outDegree(4) == 1);
}

TEST(CppGraphLiteTest, SimpleUndirectedGraphErase) 
{
	Graph<int, EdgeType::Undirected> graph;
	graph.insert(2, 4);
	graph.erase(2);

	EXPECT_TRUE(graph.inDegree(2) == 0);
	EXPECT_TRUE(graph.inDegree(4) == 0);
	EXPECT_TRUE(graph.outDegree(2) == 0);
	EXPECT_TRUE(graph.outDegree(4) == 0);
	EXPECT_TRUE(graph.edges(2).empty());
	EXPECT_TRUE(graph.edges(4).empty());
}

TEST(CppGraphLiteTest, SimpleDirectedGraphErase) 
{
	Graph<int, EdgeType::Directed> graph;
	graph.insert(2, 4);
	graph.erase(2);

	EXPECT_TRUE(graph.inDegree(2) == 0);
	EXPECT_TRUE(graph.inDegree(4) == 0);
	EXPECT_TRUE(graph.outDegree(2) == 0);
	EXPECT_TRUE(graph.outDegree(4) == 0);
	EXPECT_TRUE(graph.edges(2).empty());
	EXPECT_TRUE(graph.edges(4).empty());
}

TEST(CppGraphLiteTest, GraphWithEdgeData) 
{
    Graph<int, EdgeType::Directed, std::string> graph;
    graph.insert(1, 2, "1->2");
    graph.insert(2, 3, "2->3");

    EXPECT_TRUE(graph.size() == 3);
    EXPECT_EQ(graph.edgeData(1, 2).value(), "1->2");
    EXPECT_EQ(graph.edgeData(2, 3).value(), "2->3");
}

TEST(CppGraphLiteTest, SelfLoopDirectedGraph) 
{
    Graph<int, EdgeType::Directed> graph;
    graph.insert(1, 1);

    EXPECT_TRUE(graph.size() == 1);
    EXPECT_TRUE(graph.inDegree(1) == 1);
    EXPECT_TRUE(graph.outDegree(1) == 1);
}

TEST(CppGraphLiteTest, DuplicateEdgesUndirectedGraph) 
{
    Graph<int, EdgeType::Undirected> graph;
    graph.insert(1, 2);
    graph.insert(2, 1);

    EXPECT_TRUE(graph.size() == 2);
    EXPECT_TRUE(graph.inDegree(1) == 1);
    EXPECT_TRUE(graph.inDegree(2) == 1);
}

TEST(CppGraphLiteTest, RemoveNonExistentVertex) 
{
    Graph<int, EdgeType::Directed> graph;
    graph.insert(1, 2);
    graph.erase(3);

    EXPECT_TRUE(graph.size() == 2);
    EXPECT_TRUE(graph.inDegree(1) == 0);
    EXPECT_TRUE(graph.inDegree(2) == 1);
}

TEST(CppGraphLiteTest, BFSTraversal) 
{
    Graph<int, EdgeType::Directed> graph;
    graph.insert(1, 2);
    graph.insert(1, 3);
    graph.insert(2, 4);
    graph.insert(3, 4);

    graphlite::algorithm::GraphLiteBFS<decltype(graph)> bfs;

    std::vector<int> visitedOrder;
    bfs.BFS(graph, 1, [&](std::optional<int> from, int to) {
        visitedOrder.push_back(to);
        return false;
    });

    EXPECT_EQ(visitedOrder, std::vector<int>({1, 2, 3, 4}));
}

TEST(CppGraphLiteTest, DFSTraversal) 
{
    Graph<int, EdgeType::Directed> graph;
    graph.insert(1, 2);
    graph.insert(1, 3);
    graph.insert(2, 4);
    graph.insert(3, 4);

    graphlite::algorithm::GraphLiteDFS<decltype(graph)> dfs;

    std::vector<int> visitedOrder;
    dfs.DFS(graph, 1, [&](std::optional<int> from, int to) {
        visitedOrder.push_back(to);
        return false;
    });

    EXPECT_EQ(visitedOrder, std::vector<int>({1, 2, 4, 3}));
}

TEST(CppGraphLiteTest, DijkstraShortestPath) 
{
    Graph<int, EdgeType::Directed, int> graph;
    graph.insert(1, 2, 4);
    graph.insert(1, 3, 2);
    graph.insert(2, 3, 5);
    graph.insert(2, 4, 10);
    graph.insert(3, 4, 3);

    graphlite::algorithm::GraphLiteDijkstra<decltype(graph)> dijkstra;

    auto distances = dijkstra.Dijkstra(graph, 1, [](int vertex) { return false; });

    EXPECT_EQ(distances[1], 0);
    EXPECT_EQ(distances[2], 4);
    EXPECT_EQ(distances[3], 2);
    EXPECT_EQ(distances[4], 5);

    auto path = dijkstra.getPath(4);
    EXPECT_EQ(path, std::vector<int>({1, 3, 4}));
}

/*
Graph Structure:

        1
       /|\
      2 | 3
     /| | /\
    4 | 5   6
   /  |  \  |
  7   |   8 |
       \    |
        9---10

Edges with weights:
1 -> 2 (weight: 2)
1 -> 3 (weight: 3)
1 -> 5 (weight: 6)
2 -> 4 (weight: 1)
4 -> 7 (weight: 3)
2 -> 9 (weight: 4)
5 -> 8 (weight: 5)
9 -> 10 (weight: 2)
3 -> 6 (weight: 1)
6 -> 10 (weight: 7)

*/
TEST(CppGraphLiteTest, Graph1AlgorithmTest) 
{
    Graph<int, EdgeType::Directed, int> graph;
    graph.insert(1, 2, 2);
    graph.insert(1, 3, 3);
    graph.insert(1, 5, 6);
    graph.insert(2, 4, 1);
    graph.insert(4, 7, 3);
    graph.insert(2, 9, 4);
    graph.insert(5, 8, 5);
    graph.insert(9, 10, 2);
    graph.insert(3, 6, 1);
    graph.insert(6, 10, 7);

    {
        graphlite::algorithm::GraphLiteBFS<decltype(graph)> bfs;

        std::vector<int> visitedOrder;
        bfs.BFS(graph, 1, [&](std::optional<int> from, int to) {
            visitedOrder.push_back(to);
            return false;
        });

        EXPECT_EQ(visitedOrder, std::vector<int>({1, 2, 3, 5, 4, 9, 6, 8, 7, 10}));
    }

    {
        graphlite::algorithm::GraphLiteDFS<decltype(graph)> dfs;

        std::vector<int> visitedOrder;
        dfs.DFS(graph, 1, [&](std::optional<int> from, int to) {
            visitedOrder.push_back(to);
            return false;
        });

        EXPECT_EQ(visitedOrder, std::vector<int>({ 1, 2, 4, 7, 9, 10, 3, 6, 5, 8 }));
    }

    {
        graphlite::algorithm::GraphLiteDijkstra<decltype(graph)> dijkstra;

        auto distances = dijkstra.Dijkstra(graph, 1, [](int vertex) { return false; });

        EXPECT_EQ(distances[1], 0);
        EXPECT_EQ(distances[2], 2);
        EXPECT_EQ(distances[3], 3);
        EXPECT_EQ(distances[4], 3);
        EXPECT_EQ(distances[5], 6);
        EXPECT_EQ(distances[6], 4);
        EXPECT_EQ(distances[7], 6);
        EXPECT_EQ(distances[8], 11);
        EXPECT_EQ(distances[9], 6);
        EXPECT_EQ(distances[10], 8);

        auto path = dijkstra.getPath(10);
        EXPECT_EQ(path, std::vector<int>({ 1, 2, 9, 10 }));
    }
}

TEST(CppGraphLiteTest, RandomGraphTest) 
{
    checkRandomGraph(5, 1, 10);
    checkRandomGraph(20, 5, 30);
    checkRandomGraph(50, 1, 10);
    checkRandomGraph(100, 1, 50);
}

