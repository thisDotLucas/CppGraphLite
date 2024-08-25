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

		template <typename _VertexType>
		void erase(_VertexType&& vertex)
		{
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

			if (!m_adjacencyList.contains(vertex))
				return;

			for (auto& [vertex, edges] : m_adjacencyList)
			{
				if (edges.erase(vertex) != std::end(edges))
					m_outDegree[vertex]--;
			}

			m_inDegree.erase(vertex);
			m_outDegree.erase(vertex);
			m_adjacencyList.erase(std::forward<_VertexType>(vertex));
		}

		template <typename _VertexType>
		[[nodiscard]] EdgeListType edges(_VertexType&& vertex) const const&
		{
			static_assert(details::is_equivalent_v<_VertexType, VertexType>);

			if (m_adjacencyList.contains(vertex))
			{
				return m_adjacencyList[std::forward<_VertexType>(vertex)];
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

		AdjacencyListType m_adjacencyList;
		DegreeStructureType m_inDegree;
		DegreeStructureType m_outDegree;
	};
}

