#!/usr/bin/env bash
## Template source: https://sharats.me/posts/shell-script-best-practices/

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then
    set -o xtrace
fi

if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
    echo 'Usage: ./build.sh

Handles build validation and executes the unit tests.
Dependencies: cmake and a C++ compiler on all platforms.
              Ninja on macOS and Linux.
              Visual Studio 2022 on Windows.
'
    exit
fi

cd "$(dirname "$0")"

main() {
    # Create build directory
    mkdir -p build
    cd build

    # Detect OS and set appropriate generator
    local generator="Ninja"
    if [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]] || [[ "$OSTYPE" == "win32" ]]; then
        # Windows - use Visual Studio 2022
        generator="Visual Studio 17 2022"
    fi

    # Configure and build
    cmake -G "$generator" ..
    cmake --build .

    # Run tests
    ctest --output-on-failure
}

main "$@"