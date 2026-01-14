/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <array>
#include <concepts>
#include <tuple>
#include <type_traits>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Compile-time constant value usable as template parameter.
     *
     * Enables automatic specialization and compile-time optimization.
     * When used with `if constexpr`, allows compiler to eliminate dead code.
     *
     * @tparam T Value type (integral or floating-point)
     * @tparam Value The compile-time constant value
     *
     * @code
     * constexpr auto unity = ConstantValue<float, 1.0f>{};
     * constexpr auto zero = ConstantValue<float, 0.0f>{};
     * @endcode
     */
    template <typename T, T Value>
        requires std::is_arithmetic_v<T>
    struct ConstantValue
    {
        using value_type = T;
        static constexpr T value = Value;

        [[nodiscard]] constexpr T get() const noexcept { return value; }
        [[nodiscard]] constexpr operator T() const noexcept { return value; }

        template <T Other>
        [[nodiscard]] constexpr bool operator== (ConstantValue<T, Other>) const noexcept
        {
            return Value == Other;
        }

        template <T Other>
        [[nodiscard]] constexpr auto operator<=> (ConstantValue<T, Other>) const noexcept
        {
            return Value <=> Other;
        }

        [[nodiscard]] constexpr bool operator== (T other) const noexcept
        {
            return Value == other;
        }

        [[nodiscard]] constexpr auto operator<=> (T other) const noexcept
        {
            return Value <=> other;
        }
    };

    /**
     * @brief Auto-deducing compile-time constant.
     * @code
     * auto gain = CV<1.0f>{};  // ConstantValue<float, 1.0f>
     * auto size = CV<128>{};   // ConstantValue<int, 128>
     * @endcode
     */
    template <auto V>
    using CV = ConstantValue<decltype (V), V>;

    template <int V>
    using CVInt = ConstantValue<int, V>;

    template <std::size_t V>
    using CVSize = ConstantValue<std::size_t, V>;

    /**
     * @brief Runtime value with same interface as ConstantValue.
     *
     * Provides symmetry with ConstantValue for generic code.
     *
     * @tparam T Value type (integral or floating-point)
     *
     * @code
     * float userParam = 0.75f;
     * auto rv = RV{userParam};
     * @endcode
     */
    template <typename T>
        requires std::is_arithmetic_v<T>
    struct RuntimeValue
    {
        using value_type = T;
        T value {};

        constexpr RuntimeValue() = default;
        constexpr explicit RuntimeValue (T v) noexcept : value (v) {}

        [[nodiscard]] constexpr T get() const noexcept { return value; }
        [[nodiscard]] constexpr operator T() const noexcept { return value; }

        [[nodiscard]] constexpr bool operator== (const RuntimeValue& other) const noexcept = default;
        [[nodiscard]] constexpr auto operator<=> (const RuntimeValue& other) const noexcept = default;

        [[nodiscard]] constexpr bool operator== (T other) const noexcept
        {
            return value == other;
        }

        [[nodiscard]] constexpr auto operator<=> (T other) const noexcept
        {
            return value <=> other;
        }
    };

    template <typename T>
    using RV = RuntimeValue<T>;

    template <typename T>
    RuntimeValue (T) -> RuntimeValue<T>;

    namespace detail
    {
        template <typename T>
        struct is_constant_value : std::false_type
        {
        };

        template <typename T, T V>
        struct is_constant_value<ConstantValue<T, V>> : std::true_type
        {
        };

        template <typename T>
        struct is_runtime_value : std::false_type
        {
        };

        template <typename T>
        struct is_runtime_value<RuntimeValue<T>> : std::true_type
        {
        };
    } // namespace detail

    /**
     * @brief Interface concept for compile-time/runtime value types.
     *
     * Defines the required interface for types usable in CRV-generic code.
     * Use this when a template calls methods on a CRV parameter.
     *
     * | Member | Type | Description |
     * |--------|------|-------------|
     * | `value_type` | type alias | The underlying arithmetic type |
     * | `get()` | `value_type` | Returns the held value |
     *
     * ## Example Implementation
     * ```cpp
     * template<typename T>
     * struct MyCRV {
     *     using value_type = T;
     *     T val;
     *     constexpr T get() const noexcept { return val; }
     * };
     * static_assert(CRVLike<MyCRV<float>>);
     * ```
     */
    template <typename T>
    concept CRVLike = requires (const T t) {
        typename T::value_type;
        { t.get() } noexcept -> std::convertible_to<typename T::value_type>;
    };

    template <typename T>
    concept IsAnyConstantValue = detail::is_constant_value<std::remove_cv_t<T>>::value;

    template <typename T, typename ValueType>
    concept IsConstantValue = IsAnyConstantValue<T> && std::is_same_v<typename std::remove_cv_t<T>::value_type, ValueType>;

    template <typename T>
    concept IsAnyRuntimeValue = detail::is_runtime_value<std::remove_cv_t<T>>::value;

    template <typename T, typename ValueType>
    concept IsRuntimeValue = IsAnyRuntimeValue<T> && std::is_same_v<typename std::remove_cv_t<T>::value_type, ValueType>;

    /**
     * @brief Constrains T to either ConstantValue or RuntimeValue with ValueType.
     *
     * Combines interface requirements (CRVLike) with type-membership checks
     * to enable compile-time optimization via `if constexpr`.
     *
     * @code
     * template<IsCRV<float> GainType>
     * void applyGain(BufferView& buffer, GainType gain) {
     *     if constexpr (IsConstantValue<GainType, float>) {
     *         if constexpr (GainType::value == 1.0f) return;
     *     }
     *     // ... apply gain
     * }
     * @endcode
     */
    template <typename T, typename ValueType>
    concept IsCRV = CRVLike<T> && std::same_as<typename T::value_type, ValueType> && (IsConstantValue<T, ValueType> || IsRuntimeValue<T, ValueType>);

    template <typename T>
    concept IsAnyCRV = CRVLike<T> && (IsAnyConstantValue<T> || IsAnyRuntimeValue<T>);

    static_assert (CRVLike<ConstantValue<float, 1.0f>>, "ConstantValue must satisfy CRVLike");
    static_assert (CRVLike<ConstantValue<double, 0.0>>, "ConstantValue<double> must satisfy CRVLike");
    static_assert (CRVLike<ConstantValue<int, 42>>, "ConstantValue<int> must satisfy CRVLike");
    static_assert (CRVLike<RuntimeValue<float>>, "RuntimeValue must satisfy CRVLike");
    static_assert (CRVLike<RuntimeValue<double>>, "RuntimeValue<double> must satisfy CRVLike");
    static_assert (CRVLike<RuntimeValue<int>>, "RuntimeValue<int> must satisfy CRVLike");

    template <typename T, typename ValueType, ValueType Expected>
    constexpr bool isConstantEqual = false;

    template <typename ValueType, ValueType Value, ValueType Expected>
    constexpr bool isConstantEqual<ConstantValue<ValueType, Value>, ValueType, Expected> = (Value == Expected);

    template <typename T>
    constexpr bool isConstantZero = false;

    template <typename ValueType, ValueType Value>
    constexpr bool isConstantZero<ConstantValue<ValueType, Value>> = (Value == ValueType {});

    template <typename T>
    constexpr bool isConstantOne = false;

    template <typename ValueType, ValueType Value>
    constexpr bool isConstantOne<ConstantValue<ValueType, Value>> = (Value == static_cast<ValueType> (1));

    /**
     * @brief Add two CRV values. Result is CV if both inputs are CV.
     */
    template <IsAnyCRV A, IsAnyCRV B>
        requires std::is_same_v<typename A::value_type, typename B::value_type>
    [[nodiscard]] constexpr auto crvAdd (A a, B b) noexcept
    {
        using T = typename A::value_type;
        if constexpr (IsAnyConstantValue<A> && IsAnyConstantValue<B>)
        {
            return CV<A::value + B::value> {};
        }
        else
        {
            return RV<T> { a.get() + b.get() };
        }
    }

    /**
     * @brief Multiply two CRV values. Result is CV if both inputs are CV.
     */
    template <IsAnyCRV A, IsAnyCRV B>
        requires std::is_same_v<typename A::value_type, typename B::value_type>
    [[nodiscard]] constexpr auto crvMul (A a, B b) noexcept
    {
        using T = typename A::value_type;
        if constexpr (IsAnyConstantValue<A> && IsAnyConstantValue<B>)
        {
            return CV<A::value * B::value> {};
        }
        else
        {
            return RV<T> { a.get() * b.get() };
        }
    }

    /**
     * @brief Specifies values that should be hoisted to compile-time constants.
     *
     * When a runtime value matches one in HoistSpec, hoisted() calls the function
     * with a ConstantValue instead of RuntimeValue, moving decisions to block start.
     *
     * @tparam T Value type to hoist
     * @tparam Values Special values to generate compile-time paths for
     *
     * @code
     * using BypassSpec = HoistSpec<bool, true, false>;
     * using GainSpec = HoistSpec<float, 0.0f, 1.0f>;
     * @endcode
     */
    template <typename T, T... Values>
        requires std::is_arithmetic_v<T>
    struct HoistSpec
    {
        using value_type = T;
        static constexpr std::size_t numValues = sizeof...(Values);
        static constexpr std::array<T, numValues> values { Values... };

        [[nodiscard]] static constexpr bool contains (T value) noexcept
        {
            return ((value == Values) || ...);
        }
    };

    using BoolHoistSpec = HoistSpec<bool, false, true>;

    template <std::floating_point T = float>
    using LinearGainHoistSpec = HoistSpec<T, T { 0 }, T { 1 }>;

    namespace detail
    {
        template <typename T, T FirstValue, T... RemainingValues, typename Func>
        constexpr decltype (auto) hoistedRecurse (T value, Func&& func)
        {
            if (value == FirstValue)
            {
                return func (CV<FirstValue> {});
            }

            if constexpr (sizeof...(RemainingValues) > 0)
            {
                return hoistedRecurse<T, RemainingValues...> (value, std::forward<Func> (func));
            }
            else
            {
                return func (RV<T> { value });
            }
        }

    } // namespace detail

    /**
     * @brief Dispatch a runtime value to a function with CV or RV.
     *
     * If the value matches any in HoistSpec, calls func with ConstantValue.
     * Otherwise calls with RuntimeValue.
     *
     * @tparam Spec HoistSpec defining special values
     * @param value The runtime value to hoist
     * @param func Function to call with either CV or RV
     *
     * @code
     * hoisted<BoolHoistSpec>(bypass, [&](auto bypassCRV) {
     *     if constexpr (isConstantEqual<decltype(bypassCRV), bool, true>) {
     *         return;
     *     }
     *     // ... process
     * });
     * @endcode
     */
    template <typename Spec, typename Func>
        requires std::is_arithmetic_v<typename Spec::value_type>
    constexpr decltype (auto) hoisted (typename Spec::value_type value, Func&& func)
    {
        return [&]<typename T, T... Values> (HoistSpec<T, Values...>) {
            if constexpr (sizeof...(Values) == 0)
            {
                return func (RV<T> { value });
            }
            else
            {
                return detail::hoistedRecurse<T, Values...> (value, std::forward<Func> (func));
            }
        }(Spec {});
    }

    /**
     * @brief Hoist multiple values and call function with all CV/RV parameters.
     *
     * @code
     * hoistedProcess<BoolHoistSpec, GainHoistSpec>(
     *     std::make_tuple(bypass, gain),
     *     [&](auto bypassCRV, auto gainCRV) {
     *         processImpl(buffer, bypassCRV, gainCRV);
     *     }
     * );
     * @endcode
     */
    template <typename... Specs, typename Func>
    constexpr decltype (auto) hoistedProcess (std::tuple<typename Specs::value_type...> values, Func&& func)
    {
        return std::apply (
            [&func] (auto... args) {
                return hoistedProcessImpl<Specs...> (std::forward<Func> (func), args...);
            },
            values);
    }

    namespace detail
    {
        template <typename Spec, typename Func, typename... AccumulatedArgs>
        constexpr decltype (auto) hoistedProcessHelper (
            Func&& func,
            typename Spec::value_type value,
            AccumulatedArgs... accArgs)
        {
            return hoisted<Spec> (value, [&] (auto crvArg) {
                return func (accArgs..., crvArg);
            });
        }

        template <typename FirstSpec, typename... RemainingSpecs, typename Func, typename... AccumulatedArgs>
        constexpr decltype (auto) hoistedProcessHelper (
            Func&& func,
            typename FirstSpec::value_type firstValue,
            typename RemainingSpecs::value_type... remainingValues,
            AccumulatedArgs... accArgs)
            requires (sizeof...(RemainingSpecs) > 0)
        {
            return hoisted<FirstSpec> (firstValue, [&] (auto crvArg) {
                return hoistedProcessHelper<RemainingSpecs...> (
                    std::forward<Func> (func),
                    remainingValues...,
                    accArgs...,
                    crvArg);
            });
        }

    } // namespace detail

    template <typename... Specs, typename Func, typename... Args>
    constexpr decltype (auto) hoistedProcessImpl (Func&& func, Args... args)
    {
        return detail::hoistedProcessHelper<Specs...> (std::forward<Func> (func), args...);
    }

    /**
     * @brief Specifies chunk sizes for range hoisting.
     *
     * Defines compile-time chunk sizes to divide a runtime range into fixed pieces.
     * Chunks are processed largest-first, enabling SIMD optimization.
     *
     * @tparam ChunkSizes Chunk sizes in descending order (e.g., 8, 4, 2, 1)
     *
     * @code
     * using SimdChunks = RangeHoistSpec<4, 2, 1>;
     * using AvxChunks = RangeHoistSpec<8, 4, 2, 1>;
     * @endcode
     */
    template <std::size_t... ChunkSizes>
        requires (sizeof...(ChunkSizes) > 0)
    struct RangeHoistSpec
    {
        static constexpr std::size_t numChunks = sizeof...(ChunkSizes);
        static constexpr std::array<std::size_t, numChunks> chunks { ChunkSizes... };

        static constexpr std::size_t largest = chunks[0];
        static constexpr std::size_t smallest = chunks[numChunks - 1];
    };

    using ScalarRangeSpec = RangeHoistSpec<1>;
    using SseFloatRangeSpec = RangeHoistSpec<4, 1>;
    using AvxFloatRangeSpec = RangeHoistSpec<8, 4, 1>;
    using Avx512FloatRangeSpec = RangeHoistSpec<16, 8, 4, 1>;
    using SseDoubleRangeSpec = RangeHoistSpec<2, 1>;
    using AvxDoubleRangeSpec = RangeHoistSpec<4, 2, 1>;

    namespace detail
    {
        template <std::size_t ChunkSize, std::size_t... RemainingChunks, typename Func>
        void rangeHoistRecurse (std::size_t& offset, std::size_t& remaining, Func&& func)
        {
            while (remaining >= ChunkSize)
            {
                func (offset, CV<ChunkSize> {});
                offset += ChunkSize;
                remaining -= ChunkSize;
            }

            if constexpr (sizeof...(RemainingChunks) > 0)
            {
                rangeHoistRecurse<RemainingChunks...> (offset, remaining, std::forward<Func> (func));
            }
        }

    } // namespace detail

    /**
     * @brief Divide a runtime range into compile-time constant chunks.
     *
     * Breaks a runtime-sized range into fixed-size chunks (largest-first),
     * calling func with a compile-time constant chunk size.
     *
     * @tparam Spec RangeHoistSpec defining chunk sizes
     * @param count Total elements to process
     * @param func Called as func(offset, chunkSizeCV)
     *
     * @code
     * rangeHoist<SseFloatRangeSpec>(numSamples, [&](std::size_t offset, auto chunkSize) {
     *     if constexpr (chunkSize == 4UL) {
     *         // SIMD path
     *     } else {
     *         // Scalar path
     *     }
     * });
     * @endcode
     */
    template <typename Spec, typename Func>
    void rangeHoist (std::size_t count, Func&& func)
    {
        [&]<std::size_t... ChunkSizes> (RangeHoistSpec<ChunkSizes...>) {
            std::size_t offset = 0;
            std::size_t remaining = count;
            if constexpr (sizeof...(ChunkSizes) > 0)
            {
                detail::rangeHoistRecurse<ChunkSizes...> (offset, remaining, std::forward<Func> (func));
            }
        }(Spec {});
    }

    template <typename Spec, typename Func>
    void rangeHoist (std::size_t startOffset, std::size_t count, Func&& func)
    {
        rangeHoist<Spec> (count, [&] (std::size_t localOffset, auto chunkSize) {
            func (startOffset + localOffset, chunkSize);
        });
    }

} // namespace PlayfulTones::DspToolbox
