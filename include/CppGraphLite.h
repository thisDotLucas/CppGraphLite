﻿#pragma once
#include "CppGraphLiteTypeTraits.h"
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <type_traits>
#include <optional>
#include <tuple>
#include <ranges>
#include <algorithm>
#include <queue>
#include <functional>

namespace graphlite
{
	enum class EdgeType
	{
		Directed,
		Undirected
	};

	template <typename _VertexType, EdgeType edgeType = EdgeType::Directed, typename EdgeData = std::nullopt_t>
	class Graph
	{
	public:
		using VertexType = _VertexType;
		using EdgeDataType = EdgeData;

		using EdgeListType = std::conditional_t<details::Hashable<VertexType>,
			std::unordered_set<VertexType>,
			std::set<VertexType>>;

		using AdjacencyListType = std::conditional_t<details::Hashable<VertexType>,
			std::unordered_map<VertexType, EdgeListType>,
			std::map<VertexType, EdgeListType>>;

		using DegreeStructureType = std::conditional_t<details::Hashable<VertexType>,
			std::unordered_map<VertexType, size_t>,
			std::map<VertexType, size_t>>;

		using EdgeDataStructureType = std::conditional_t<std::same_as<EdgeData, std::nullopt_t>, std::tuple<>, 
			std::conditional_t<details::Hashable<std::pair<VertexType, VertexType>>,
			std::unordered_map<std::pair<VertexType, VertexType>, EdgeData>,
			std::map<std::pair<VertexType, VertexType>, EdgeData>>>;

		template <typename _VertexType> // Universal/forwarding reference
		void insert(_VertexType&& vertex)
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			if (m_adjacencyList.insert({ vertex, {} }).second)
			{
				m_inDegree.insert({ vertex, 0 });
				m_outDegree.insert({ std::forward<_VertexType>(vertex), 0 });
			}
		}

		template <typename _VertexType>
		requires (std::same_as<EdgeData, std::nullopt_t>)
		void insert(_VertexType&& from, _VertexType&& to)
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			if constexpr (edgeType == EdgeType::Directed)
			{
				_insert(std::forward<_VertexType>(from), std::forward<_VertexType>(to));
			}
			else
			{
				_insert(from, to);
				_insert(std::forward<_VertexType>(to), std::forward<_VertexType>(from));
			}
		}

		template <typename _VertexType, typename _EdgeData>
		requires (!std::same_as<EdgeData, std::nullopt_t>)
		void insert(_VertexType&& from, _VertexType&& to, _EdgeData&& edgeData)
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>
				&& details::is_compatible_v<_EdgeData, EdgeData>);

			if constexpr (edgeType == EdgeType::Directed)
			{
				_insert(std::forward<_VertexType>(from), std::forward<_VertexType>(to), std::forward<_EdgeData>(edgeData));
			}
			else
			{
				_insert(from, to, edgeData);
				_insert(std::forward<_VertexType>(to), std::forward<_VertexType>(from), std::forward<_EdgeData>(edgeData));
			}
		}


		template <typename _VertexType>
		void erase(_VertexType&& vertex)
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			if (!m_adjacencyList.contains(vertex))
				return;

			for (auto& [_vertex, edges] : m_adjacencyList)
			{
				if (edges.erase(vertex) != 0)
				{
					m_outDegree[_vertex]--;
				}
			}

			if constexpr (!std::same_as<EdgeData, std::nullopt_t>)
			{
				std::erase_if(m_edgeData, [&](const auto& edgeData)
				{
					const auto& [from, to] = edgeData.first;
					return from == vertex || to == vertex;
				});
			}

			m_inDegree.erase(vertex);
			m_outDegree.erase(vertex);

			for (auto& _vertex : m_adjacencyList[vertex])
			{
				m_inDegree[_vertex]--;
			}

			m_adjacencyList.erase(std::forward<_VertexType>(vertex));
		}

		[[nodiscard]] std::vector<VertexType> vertices() const
		{
			auto keys = std::views::keys(m_adjacencyList);
			return { std::begin(keys), std::end(keys) };
		}

		template <typename _VertexType>
		[[nodiscard]] EdgeListType edges(_VertexType&& vertex) const&
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			if (m_adjacencyList.contains(vertex))
			{
				return m_adjacencyList.at(std::forward<_VertexType>(vertex));
			}
			else
			{
				return EdgeListType{};
			}
		}

		template <typename _VertexType>
		[[nodiscard]] EdgeListType&& edges(_VertexType&& vertex) &&
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			if (m_adjacencyList.contains(vertex))
			{
				return std::move(m_adjacencyList[std::forward<_VertexType>(vertex)]);
			}
			else
			{
				return EdgeListType{};
			}
		}

		template <typename _VertexType>
		requires (!std::same_as<EdgeData, std::nullopt_t>)
		[[nodiscard]] std::optional<EdgeData> edgeData(_VertexType&& from, _VertexType&& to) const&
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			if (m_edgeData.contains({ from, to }))
				return m_edgeData.at({ std::forward<_VertexType>(from), std::forward<_VertexType>(to) });
			else
				return std::nullopt;
		}

		template <typename _VertexType>
		requires (!std::same_as<EdgeData, std::nullopt_t>)
		[[nodiscard]] std::optional<EdgeData> edgeData(_VertexType&& from, _VertexType&& to) &&
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			if (m_edgeData.contains({ from, to }))
				return std::move(m_edgeData[{ std::forward<_VertexType>(from), std::forward<_VertexType>(to) }]);
			else
				return std::nullopt;
		}

		[[nodiscard]] size_t size() const
		{
			return m_adjacencyList.size();
		}

		template <typename _VertexType>
		[[nodiscard]] size_t inDegree(_VertexType&& vertex)
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			return m_inDegree[std::forward<_VertexType>(vertex)];
		}

		template <typename _VertexType>
		[[nodiscard]] size_t outDegree(_VertexType&& vertex)
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			return m_outDegree[std::forward<_VertexType>(vertex)];
		}

	private:
		template <typename _VertexType>
		void _insert(_VertexType&& from, _VertexType&& to)
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>);

			if (!m_adjacencyList.contains(to))
			{
				if (!m_adjacencyList.insert({ to, {} }).second)
					return;
				else
				{
					m_inDegree.insert({ to, 0 });
					m_outDegree.insert({ to, 0 });
				}
			}

			if (m_adjacencyList.contains(from))
			{
				if (m_adjacencyList[from].insert(to).second)
				{
					m_outDegree[std::forward<_VertexType>(from)]++;
					m_inDegree[std::forward<_VertexType>(to)]++;
				}
			}
			else
			{
				if (m_adjacencyList.insert({ from, { to } }).second)
				{
					m_outDegree[std::forward<_VertexType>(from)]++;
					m_inDegree[std::forward<_VertexType>(to)]++;
				}
			}
		}

		template <typename _VertexType, typename _EdgeData>
		void _insert(_VertexType&& from, _VertexType&& to, _EdgeData&& edgeData)
		{
			static_assert(details::is_compatible_v<_VertexType, VertexType>
				&& details::is_compatible_v<_EdgeData, EdgeData>);

			_insert(from, to);
			m_edgeData[{ std::forward<_VertexType>(from), std::forward<_VertexType>(to) }] = std::forward<_EdgeData>(edgeData);
		}

		AdjacencyListType m_adjacencyList;
		DegreeStructureType m_inDegree;
		DegreeStructureType m_outDegree;
		EdgeDataStructureType m_edgeData;
	};

	static_assert(std::same_as<Graph<int, EdgeType::Directed>::AdjacencyListType,
		std::unordered_map<int, std::unordered_set<int>>>);

	static_assert(std::same_as<Graph<int, EdgeType::Directed>::EdgeListType,
		std::unordered_set<int>>);

	static_assert(std::same_as<Graph<int, EdgeType::Directed>::DegreeStructureType,
		std::unordered_map<int, size_t>>);
}

namespace graphlite::algorithm
{
	template<typename Graph>
	class GraphLiteBFS
	{		
		using VisitedType = std::conditional_t<details::Hashable<typename Graph::VertexType>,
			std::unordered_set<typename Graph::VertexType>,
			std::set<typename Graph::VertexType>>;

	public:
		using ProcessFunction = std::function<bool(const std::optional<typename Graph::VertexType>, const typename Graph::VertexType&)>;
		void BFS(const Graph& graph, const typename Graph::VertexType& start, ProcessFunction processFunction)
		{
			VisitedType visited{ start };

			std::queue<typename Graph::VertexType> toProcess;
			toProcess.push(start);

			bool stop = processFunction(std::nullopt, start);

			while (!stop && toProcess.size())
			{
				auto vertex = toProcess.front();

				auto isUnprocessed = [&](const typename Graph::VertexType& vertex) { return !visited.contains(vertex); };
				for (auto&& connectedVertex : graph.edges(vertex)
					| std::views::filter(isUnprocessed))
				{
					visited.insert(connectedVertex);
					toProcess.push(connectedVertex);
					stop = processFunction(vertex, connectedVertex);
				}

				toProcess.pop();
			}
		}
	};

	template<typename Graph>
	class GraphLiteDFS
	{		
		using VisitedType = std::conditional_t<details::Hashable<typename Graph::VertexType>,
			std::unordered_set<typename Graph::VertexType>,
			std::set<typename Graph::VertexType>>;

	public:
		using ProcessFunction = std::function<bool(const std::optional<typename Graph::VertexType>, const typename Graph::VertexType&)>;
		void DFS(const Graph& graph, const typename Graph::VertexType& vertex, ProcessFunction processFunction)
		{
			m_processing.insert(vertex);

			if (m_first)
			{
				m_stop = processFunction(std::nullopt, vertex);
				m_first = false;
			}

			for (auto&& connectedVertex : graph.edges(vertex))
			{
				if (m_stop)
				{
					m_first = true;
					break;
				}

				if (!m_processing.contains(connectedVertex) && !m_processed.contains(connectedVertex))
				{
					m_stop = processFunction(vertex, connectedVertex);
					DFS(graph, connectedVertex, processFunction);
				}
			}

			m_processing.erase(vertex);
			m_processed.insert(vertex);

			if (m_processed.size() == graph.size())
				m_first = true;
		}

	private:
		bool m_first{ true };
		bool m_stop{ false };
		std::unordered_set<typename Graph::VertexType> m_processing;
		std::unordered_set<typename Graph::VertexType> m_processed;
	};

	template<typename Graph, typename WeightRetrieverFunction = std::function<typename Graph::EdgeDataType(const typename Graph::EdgeDataType&)>>
	class GraphLiteDijkstra
	{
	public:
		using ProcessFunction = std::function<bool(const typename Graph::VertexType&)>;

		using VisitedType = std::conditional_t<details::Hashable<typename Graph::VertexType>,
			std::unordered_set<typename Graph::VertexType>,
			std::set<typename Graph::VertexType>>;

		using WeightType = std::invoke_result_t<WeightRetrieverFunction, typename Graph::EdgeDataType>;

		using DistanceMapType = std::conditional_t<details::Hashable<typename Graph::VertexType>,
			std::unordered_map<typename Graph::VertexType, WeightType>,
			std::map<typename Graph::VertexType, WeightType>>;

		using PredecessorMapType = std::conditional_t<details::Hashable<typename Graph::VertexType>,
			std::unordered_map<typename Graph::VertexType, std::optional<typename Graph::VertexType>>,
			std::map<typename Graph::VertexType, std::optional<typename Graph::VertexType>>>;

		using PriorityQueueType = std::priority_queue<
			std::pair<typename Graph::VertexType, WeightType>,
			std::vector<std::pair<typename Graph::VertexType, WeightType>>,
			std::function<bool(const std::pair<typename Graph::VertexType, WeightType>&, const std::pair<typename Graph::VertexType, WeightType>&)>>;

		DistanceMapType Dijkstra(const Graph& graph, const typename Graph::VertexType& start, ProcessFunction processFunction, WeightRetrieverFunction getWeight = [](const typename Graph::EdgeDataType& edge) { return edge; })
		{
			DistanceMapType distances{};
			for (const auto& vertex : graph.vertices()) 
			{
				distances[vertex] = std::numeric_limits<WeightType>::max();
				m_predecessors[vertex] = std::nullopt;
			}

			distances[start] = WeightType{};

			m_vertexSet.clear();
			m_edgesCrossingSet = PriorityQueueType([&](const auto& l, const auto& r) { return l.second > r.second; });

			m_edgesCrossingSet.emplace(start, WeightType{});

			while (!m_edgesCrossingSet.empty())
			{
				const auto [currentVertex, currentDistance] = m_edgesCrossingSet.top();
				m_edgesCrossingSet.pop();

				if (m_vertexSet.contains(currentVertex))
					continue;

				m_vertexSet.insert(currentVertex);

				if (processFunction(currentVertex))
					break;

				for (auto&& toVertex : graph.edges(currentVertex))
				{
					auto edge = graph.edgeData(currentVertex, toVertex);
					auto newDistance = currentDistance + getWeight(edge.value());

					if (newDistance < distances[toVertex])
					{
						distances[toVertex] = newDistance;
						m_predecessors[toVertex] = currentVertex;
						m_edgesCrossingSet.emplace(toVertex, newDistance);
					}
				}
			}

			return distances;
		}

		std::vector<typename Graph::VertexType> getPath(const typename Graph::VertexType& to) const
		{
			std::vector<typename Graph::VertexType> path;

			std::optional<typename Graph::VertexType> current = to;
			while (current.has_value())
			{
				path.push_back(current.value());
				current = m_predecessors.at(current.value());
			}

			std::reverse(std::begin(path), std::end(path));

			return path;
		}

	private:
		VisitedType m_vertexSet;
		PriorityQueueType m_edgesCrossingSet;
		PredecessorMapType m_predecessors;
	};
}

