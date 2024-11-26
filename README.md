# CppGraphLite

CppGraphLite is a lightweight and easy-to-use C++20 graph library designed to support a wide variety of graph structures, including directed and undirected graphs, with optional edge data. The focus is on a lightweight interface which is easy to pick up and use in your project.

---

## Features

- **Graph Types**: Supports both directed and undirected graphs.
- **Edge Data**: Optional support for storing and accessing data associated with edges.
- **Graph Algorithms**:
  - Breadth-First Search (BFS)
  - Depth-First Search (DFS)
  - Dijkstra's Shortest Path Algorithm
  - More to come...
- **Modular Design**: Easily extendable for custom algorithms and use cases.

---

## Installation

### Requirements

- **C++ Compiler**: C++20 compliant
- **CMake**: Version 3.15 or higher

### Build Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/thisDotLucas/CppGraphLite.git
   cd CppGraphLite
   ```

2. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```

3. Configure and build the project:
   ```bash
   cmake ..
   cmake --build .
   ```

---

## Usage

### Basic Example

```cpp
#include "CppGraphLite.h"
#include <iostream>

int main() {
    using namespace graphlite;

    Graph<int, EdgeType::Directed> graph;

    // Add vertices
    graph.insert(1);
    graph.insert(2);
    graph.insert(3);

    // Add edges
    graph.insert(1, 2);
    graph.insert(2, 3);

    // Traverse and print edges
    for (const auto& vertex : graph.vertices()) {
        std::cout << "Vertex " << vertex << " is connected to: ";
        for (const auto& edge : graph.edges(vertex)) {
            std::cout << edge << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
```

---

### Key API Functions

#### Graph Creation
```cpp
Graph<int, EdgeType::Directed> graph;
graph.insert(1);          // Add vertex
graph.insert(1, 2);       // Add directed edge

Graph<int, EdgeType::Directed, double> graphWithEdgeData;
graph.insert(1, 2, 5.0);  // Add directed edge with data
```

#### Traversal
```cpp
auto vertices = graph.vertices(); // Get all vertices
auto edges = graph.edges(1);      // Get edges of a vertex
```

#### Algorithms

- **BFS**:
  ```cpp
  graphlite::algorithm::GraphLiteBFS<decltype(graph)> bfs;
  bfs.BFS(graph, startVertex, [](auto parent, auto vertex) {
      std::cout << "Visited: " << vertex << std::endl;
      return false; // Return true to stop
  });
  ```

- **Dijkstra**:
  ```cpp
  graphlite::algorithm::GraphLiteDijkstra<decltype(graph)> dijkstra;
  auto distances = dijkstra.Dijkstra(graph, startVertex, [](auto vertex) {
      return false; // Return true to stop
  });
  ```
