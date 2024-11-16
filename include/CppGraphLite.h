#pragma once
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
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

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
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

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
			static_assert(details::is_equivalent_v<_VertexType, VertexType>
				&& details::is_equivalent_v<_EdgeData, EdgeData>);

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
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

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

		template <typename _VertexType>
		[[nodiscard]] EdgeListType edges(_VertexType&& vertex) const&
		{
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

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
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

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
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

			if (m_edgeData.contains({ from, to }))
				return m_edgeData.at({ std::forward<_VertexType>(from), std::forward<_VertexType>(to) });
			else
				return std::nullopt;
		}

		template <typename _VertexType>
		requires (!std::same_as<EdgeData, std::nullopt_t>)
		[[nodiscard]] std::optional<EdgeData> edgeData(_VertexType&& from, _VertexType&& to) &&
		{
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

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
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

			return m_inDegree[std::forward<_VertexType>(vertex)];
		}

		template <typename _VertexType>
		[[nodiscard]] size_t outDegree(_VertexType&& vertex)
		{
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

			return m_outDegree[std::forward<_VertexType>(vertex)];
		}

	private:
		template <typename _VertexType>
		void _insert(_VertexType&& from, _VertexType&& to)
		{
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

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
			static_assert(details::is_equivalent_v<_VertexType, VertexType>
				&& details::is_equivalent_v<_EdgeData, EdgeData>);

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

			for (auto&& connectedVertex : graph.edges(vertex))
			{
				if (m_stop)
					break;

				if (!m_processing.contains(connectedVertex) && !m_processed.contains(connectedVertex))
				{
					m_stop = processFunction(vertex, connectedVertex);
					DFS(graph, connectedVertex, processFunction);
				}
			}

			m_processing.erase(vertex);
			m_processed.insert(vertex);
		}

		bool m_stop{ false };
		std::unordered_set<typename Graph::VertexType> m_processing;
		std::unordered_set<typename Graph::VertexType> m_processed;
	};
}

