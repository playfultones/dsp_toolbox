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

# Find all header files
file(GLOB_RECURSE HEADER_FILES "**/*.h")

# Create test executable for each header
foreach(HEADER ${HEADER_FILES})
    # Get the filename without extension
    get_filename_component(TEST_NAME ${HEADER} NAME_WE)
    
    # Create a test source file that includes the header
    set(TEST_SOURCE "${CMAKE_BINARY_DIR}/test_${TEST_NAME}.cpp")
    file(WRITE ${TEST_SOURCE} "#include \"${HEADER}\"\nint main() { return 0; }\n")
    
    # Add executable
    add_executable(test_${TEST_NAME} ${TEST_SOURCE})
    target_include_directories(test_${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    
    # Add test
    add_test(NAME ${TEST_NAME}_compiles COMMAND test_${TEST_NAME})
endforeach()