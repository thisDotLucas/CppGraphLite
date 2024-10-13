find_package(GTest CONFIG QUIET)

if (GTest_FOUND)
	add_executable(Test CppGraphLiteTest.cpp)
	target_link_libraries (Test ${PROJECT_NAME} GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
else()
    message(STATUS "Package GTest was not found.")
endif()
