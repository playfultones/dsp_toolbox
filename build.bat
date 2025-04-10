@echo off
setlocal

:: Create build directory
if not exist build mkdir build
cd build

:: Configure and build
cmake ..
cmake --build .

:: Run tests
ctest --output-on-failure

if errorlevel 1 exit /b 1