#pragma once
#include <type_traits>

namespace details
{
	template<typename T>
	concept Hashable = requires(T a) {
		{ std::hash<T>{}(a) } -> std::convertible_to<std::size_t>;
	};

	template <typename T, typename U>
	using is_equivalent = std::is_same<std::remove_cvref_t<T>, std::remove_cvref_t<U>>;

	template <typename T, typename U>
	constexpr bool is_equivalent_v = is_equivalent<T, U>::value;
}