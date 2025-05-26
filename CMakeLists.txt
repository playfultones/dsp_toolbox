cmake_minimum_required(VERSION 3.14)
project(dsp_toolbox VERSION 1.0.0 LANGUAGES CXX)

# Enable testing
enable_testing()

# Set C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set warning flags
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Werror -pedantic)
endif()

# Find all test files
file(GLOB TEST_FILES "tests/test_*.cpp")

# Add test executables
foreach(TEST_FILE ${TEST_FILES})
    # Get the filename without extension to use as the test name
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    
    # Create executable
    add_executable(${TEST_NAME} ${TEST_FILE})
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    
    # Add test
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()

# Find all header files
file(GLOB_RECURSE HEADER_FILES "**/*.h")

# Create compilation test executable for each header
foreach(HEADER ${HEADER_FILES})
    # Get the filename without extension
    get_filename_component(TEST_NAME ${HEADER} NAME_WE)
    
    # Create a test source file that includes the header
    set(TEST_SOURCE "${CMAKE_BINARY_DIR}/compilationtest_${TEST_NAME}.cpp")
    file(WRITE ${TEST_SOURCE} "#include \"${HEADER}\"\nint main() { return 0; }\n")
    
    # Add executable
    add_executable(compilationtest_${TEST_NAME} ${TEST_SOURCE})
    target_include_directories(compilationtest_${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    
    # Add test
    add_test(NAME ${TEST_NAME}_compiles COMMAND compilationtest_${TEST_NAME})
endforeach()
