#pragma once
#include <type_traits>

namespace details
{
	template<typename T>
	concept Hashable = requires(T a) {
		{ std::hash<T>{}(a) } -> std::convertible_to<std::size_t>;
	};

	template <typename T>
	using remove_cv_ref = std::remove_cv_t<std::remove_reference_t<T>>;

	template <typename T, typename U>
	using is_equivalent = std::is_same<remove_cv_ref<T>, remove_cv_ref<U>>;

	template <typename T, typename U>
	constexpr bool is_equivalent_v = is_equivalent<T, U>::value;
}