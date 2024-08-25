#pragma once
#include "CppGraphLiteTypeTraits.h"
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <type_traits>

namespace graphlite
{
	enum class EdgeType
	{
		Directed,
		Undirected
	};

	template <typename VertexType, EdgeType edgeType>
	class Graph
	{
		using EdgeListType = std::conditional_t<details::Hashable<VertexType>,
			std::unordered_set<VertexType>,
			std::set<VertexType>>;

		using AdjacencyListType = std::conditional_t<details::Hashable<VertexType>,
			std::unordered_map<VertexType, EdgeListType>,
			std::map<VertexType, EdgeListType>>;

		using DegreeStructureType = std::conditional_t<details::Hashable<VertexType>,
			std::unordered_map<VertexType, size_t>,
			std::map<VertexType, size_t>>;

	public:
		void insert(VertexType&& vertex)
		{
			if (m_adjacencyList.insert({ vertex, {} }).second)
			{
				m_inDegree.insert({ vertex, 0 });
				m_outDegree.insert({ std::forward<VertexType>(vertex), 0 });
			}
		}

		void insert(VertexType&& from, VertexType&& to)
		{
			if constexpr (edgeType == EdgeType::Directed)
			{
				_insert(std::forward<VertexType>(from), std::forward<VertexType>(to));
			}
			else
			{
				_insert(from, to);
				_insert(std::forward<VertexType>(to), std::forward<VertexType>(from));
			}
		}

		void erase(VertexType&& vertex)
		{
			if (!m_adjacencyList.contains(vertex))
				return;

			for (auto& [vertex, edges] : m_adjacencyList)
			{
				if (edges.erase(vertex) != std::end(edges))
					m_outDegree[vertex]--;
			}

			m_inDegree.erase(vertex);
			m_outDegree.erase(vertex);
			m_adjacencyList.erase(std::forward<VertexType>(vertex));
		}

		[[nodiscard]] EdgeListType edges(VertexType&& vertex) const const&
		{
			if (m_adjacencyList.contains(vertex))
			{
				return m_adjacencyList[std::forward<VertexType>(vertex)];
			}
			else
			{
				return EdgeListType{};
			}
		}

		[[nodiscard]] EdgeListType&& edges(VertexType&& vertex) &&
		{
			if (m_adjacencyList.contains(vertex))
			{
				return std::move(m_adjacencyList[std::forward<VertexType>(vertex)]);
			}
			else
			{
				return EdgeListType{};
			}
		}

		[[nodiscard]] size_t size() const
		{
			return m_adjacencyList.size();
		}

		[[nodiscard]] size_t inDegree(VertexType&& vertex)
		{
			return m_inDegree[std::forward<VertexType>(vertex)];
		}

		[[nodiscard]] size_t outDegree(VertexType&& vertex)
		{
			return m_outDegree[std::forward<VertexType>(vertex)];
		}

	private:

		void _insert(VertexType&& from, VertexType&& to)
		{
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
					m_outDegree[std::forward<VertexType>(from)]++;
					m_inDegree[std::forward<VertexType>(to)]++;
				}
			}
			else
			{
				if (m_adjacencyList.insert({ from, { to } }).second)
				{
					m_outDegree[std::forward<VertexType>(from)]++;
					m_inDegree[std::forward<VertexType>(to)]++;
				}
			}
		}

		AdjacencyListType m_adjacencyList;
		DegreeStructureType m_inDegree;
		DegreeStructureType m_outDegree;
	};
}

