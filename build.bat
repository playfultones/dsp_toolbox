@echo off
setlocal

:: Create build directory
if not exist build mkdir build
cd build

:: Configure and build
cmake ..
cmake --build . --config Debug

:: Run tests with Debug configuration
ctest --output-on-failure -C Debug

if errorlevel 1 exit /b 1