#pragma once
#include <unordered_map>
#include <map>
#include <set>
#include <unordered_set>
#include <type_traits>

namespace graphlite
{
	enum class GraphType
	{
		Directed,
		Undirected
	};

	template <typename VertexType, GraphType graphType>
	class Graph
	{
		using EdgeListType = std::unordered_set<VertexType>;
		using AdjacencyListType = std::unordered_map<VertexType, EdgeListType>;

	public:

		void insert(VertexType&& vertex)
		{
			m_adjacencyList.insert(std::forward<VertexType>(vertex), {});
		}

		void insert(VertexType&& from, VertexType&& to)
		{
			if constexpr (graphType == GraphType::Directed)
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

			for (auto& [_, edges] : m_adjacencyList)
			{
				edges.erase(vertex);
			}

			m_adjacencyList.erase(std::forward<VertexType>(vertex));
		}

		EdgeListType edges(VertexType&& vertex) const&
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

		EdgeListType&& edges(VertexType&& vertex) &&
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

		size_t size() const
		{
			return m_adjacencyList.size();
		}

	private:

		void _insert(VertexType&& from, VertexType&& to)
		{
			if (m_adjacencyList.contains(from))
			{
				m_adjacencyList[std::forward<VertexType>(from)].push_back(std::forward<VertexType>(to));
			}
			else
			{
				m_adjacencyList.insert(std::forward<VertexType>(from), { std::forward<VertexType>(to) });
			}
		}

		AdjacencyListType m_adjacencyList;
	};
}

