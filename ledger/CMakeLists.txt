cmake_minimum_required(VERSION 3.20)
project(clasp_ledger VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SQLite3 REQUIRED)

add_library(clasp_ledger STATIC
    src/ledger.cpp
)
target_include_directories(clasp_ledger
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(clasp_ledger
    PUBLIC
        SQLite::SQLite3
)
target_compile_options(clasp_ledger
    PRIVATE
        -Wall -Wextra -Wpedantic -Werror
)

add_executable(clasp_ledger_cli
    src/main.cpp
)
target_link_libraries(clasp_ledger_cli
    PRIVATE
        clasp_ledger
)
target_compile_options(clasp_ledger_cli
    PRIVATE
        -Wall -Wextra -Wpedantic -Werror
)

enable_testing()
add_executable(clasp_ledger_tests
    tests/test_ledger.cpp
)
target_link_libraries(clasp_ledger_tests
    PRIVATE
        clasp_ledger
)
target_compile_options(clasp_ledger_tests
    PRIVATE
        -Wall -Wextra -Wpedantic -Werror
)
add_test(NAME clasp_ledger_tests COMMAND clasp_ledger_tests)

