#!/usr/bin/env bash
## Template source: https://sharats.me/posts/shell-script-best-practices/

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then
    set -o xtrace
fi

if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
    echo 'Usage: ./build.sh [config]

Handles build validation and executes the unit tests.
Dependencies: cmake and a C++ compiler on all platforms.
              Ninja on macOS and Linux.
              Visual Studio 2022 on Windows.

Arguments:
    config: Build configuration (Debug/Release). Default is Release.
'
    exit
fi

cd "$(dirname "$0")"

main() {
    # Get build configuration
    local config="${1:-Release}"

    # Create build directory
    mkdir -p build
    cd build

    # Detect OS and set appropriate generator
    local generator="Ninja"
    local build_args=()
    local test_args=()
    if [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]] || [[ "$OSTYPE" == "win32" ]]; then
        # Windows - use Visual Studio 2022
        generator="Visual Studio 17 2022"
        build_args+=("--config" "$config")
        test_args+=("-C" "$config")
    fi

    # Configure and build
    cmake -G "$generator" ..
    if [ ${#build_args[@]} -eq 0 ]; then
        cmake --build .
    else
        cmake --build . "${build_args[@]}"
    fi

    # Run tests
    if [ ${#test_args[@]} -eq 0 ]; then
        ctest --output-on-failure
    else
        ctest --output-on-failure "${test_args[@]}"
    fi
}

main "$@"