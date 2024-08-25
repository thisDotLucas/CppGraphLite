find_package(GTest CONFIG REQUIRED)

add_executable(Test CppGraphLiteTest.cpp)

target_link_libraries (Test ${PROJECT_NAME} GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
