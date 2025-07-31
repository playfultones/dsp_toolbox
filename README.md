# High-Performance Audio DSP Toolbox

## Core Design Principles

These are the fundamental design principles taken into consideration for this library.

### Memory & Performance

1. **Fixed Block Sizes at Compile Time**
   - Template parameters for buffer sizes enable cache-efficient processing
   - Eliminates runtime overhead and improves compiler optimizations

2. **Template-based Configuration**
   - Sample rate, block size, and precision as template parameters
   - Enables compile-time optimizations and type safety

3. **Memory-Safe Buffer Access**
   - Use `std::span` or similar constructs for bounds-checked access
   - Prevents buffer overruns while maintaining performance

4. **SIMD/Vectorization Support**
   - Design APIs to process multiple samples simultaneously
   - Ensure proper memory alignment (16/32/64 byte boundaries)
   - Consider explicit SIMD intrinsics or compiler-friendly patterns

### Real-time Safety

5. **RAII for Resource Management**
   - Automatic cleanup prevents resource leaks
   - Exception-safe design patterns

6. **Separate Control and Audio Rate Processing**
   - Decouple parameter updates from audio processing
   - Enables thread-safe parameter changes

7. **Lock-free/Wait-free Algorithms**
   - Use lock-free queues for inter-thread communication
   - Never use mutexes or locks in audio callbacks
   - Careful consideration of memory ordering

8. **Zero-allocation Audio Callbacks**
   - Pre-allocate all required memory
   - No dynamic allocations in `process()` calls
   - Custom allocators for non-realtime paths only

### Code Architecture

9. **Avoid Virtual Functions in Hot Paths**
   - Use CRTP (Curiously Recurring Template Pattern) for polymorphism
   - Enables inlining and reduces indirection overhead

10. **Branch Prediction Friendly Code**
    - Minimize conditional branches in inner loops
    - Prefer branchless algorithms
    - Profile and optimize hot paths

11. **Strong Type Safety**
    - Use strong types for units (e.g., `Frequency<float>`, `Decibels<float>`)
    - Compile-time parameter validation
    - Liberal use of `constexpr`

### Platform-Specific Considerations

12. **Denormal Handling**
    - Add small DC offset or noise to prevent denormals
    - Use platform-specific flush-to-zero flags
    - Consider explicit denormal detection and handling

13. **Cache-line Awareness**
    - Avoid false sharing between threads
    - Pack related data together
    - Consider data layout (Structure of Arrays vs Array of Structures)

14. **Exception Safety**
    - Mark all audio callback paths `noexcept`
    - Documents real-time safety requirements
    - Enables compiler optimizations
