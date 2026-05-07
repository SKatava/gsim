cmake_minimum_required(VERSION 3.22)
 
# ── GoogleTest ────────────────────────────────────────────────────────────────
include(FetchContent)
 
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
)
 
# Prevent GoogleTest from hijacking our install target and CRT settings (MSVC).
set(INSTALL_GTEST      OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON  CACHE BOOL "" FORCE)
 
FetchContent_MakeAvailable(googletest)
 
# ── Test executable ───────────────────────────────────────────────────────────
add_executable(gsim-tests
    GravityEngineTest.cpp
)
 
target_link_libraries(gsim-tests PRIVATE
    gsim-simulation     # the library under test
    GTest::gtest_main   # brings in gtest + a default main()
)
 
# Let GoogleTest's cmake helpers register each TEST / TEST_F with CTest.
include(GoogleTest)
gtest_discover_tests(gsim-tests)

