/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/concepts.hpp"
#include "dsp_toolbox/core/decibels.hpp"
#include "dsp_toolbox/core/decibels_full_scale.hpp"
#include "dsp_toolbox/core/frequency.hpp"
#include "dsp_toolbox/core/linear_gain.hpp"
#include "dsp_toolbox/core/milliseconds.hpp"
#include "dsp_toolbox/core/seconds.hpp"
#include "dsp_toolbox/core/smoothed_value.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>
#include <string_view>
#include <type_traits>

namespace PlayfulTones::DspToolbox
{

    //--------------------------------------------------------------------------
    // FixedString - Constexpr string for use in NTTPs
    //--------------------------------------------------------------------------

    /**
     * @brief Fixed-size constexpr string for use in non-type template parameters.
     *
     * This is a structural type that can be used in template arguments while
     * still providing convenient string literal construction.
     *
     * @tparam MaxLen Maximum string length (including null terminator)
     */
    template <std::size_t MaxLen = 32>
    struct FixedString
    {
        std::array<char, MaxLen> data {};
        std::size_t len = 0;

        constexpr FixedString() = default;

        /**
         * @brief Construct from string literal.
         */
        template <std::size_t N>
        constexpr FixedString (const char (&str)[N]) : len (N - 1)
        {
            static_assert (N <= MaxLen, "String literal too long for FixedString");
            for (std::size_t i = 0; i < N; ++i)
            {
                data[i] = str[i];
            }
        }

        [[nodiscard]] constexpr operator std::string_view() const noexcept
        {
            return { data.data(), len };
        }

        [[nodiscard]] constexpr const char* c_str() const noexcept
        {
            return data.data();
        }

        [[nodiscard]] constexpr std::size_t size() const noexcept
        {
            return len;
        }

        [[nodiscard]] constexpr bool empty() const noexcept
        {
            return len == 0;
        }

        constexpr bool operator== (const FixedString& other) const noexcept = default;

        constexpr bool operator== (std::string_view sv) const noexcept
        {
            return std::string_view (*this) == sv;
        }
    };

    // Deduction guide
    template <std::size_t N>
    FixedString (const char (&)[N]) -> FixedString<N>;

    //--------------------------------------------------------------------------
    // ParamDescriptor - Compile-time parameter metadata
    //--------------------------------------------------------------------------

    /**
     * @brief Compile-time parameter descriptor.
     *
     * Defines metadata for a parameter: identifier, display name, range, default, and unit.
     * Uses FixedString for NTTP compatibility - can be used as template argument to ParamSet.
     *
     * All string fields have sensible max lengths:
     * - id: 32 chars (machine identifier like "frequency")
     * - name: 48 chars (human-readable like "Base Frequency")
     * - unit: 8 chars (short unit like "Hz", "dB", "%")
     */
    struct ParamDescriptor
    {
        FixedString<32> id {}; ///< Machine-readable identifier
        FixedString<48> name {}; ///< Human-readable display name
        float minValue = 0.0f; ///< Minimum allowed value
        float maxValue = 1.0f; ///< Maximum allowed value
        float defaultValue = 0.0f; ///< Default value
        FixedString<8> unit {}; ///< Unit string (e.g., "Hz", "dB")

        constexpr bool operator== (const ParamDescriptor&) const noexcept = default;
    };

    /**
     * @brief Generic parameter with built-in smoothing and min/max validation.
     *
     * Wraps SmoothedValue with clamping and strong type conversion utilities.
     *
     * @tparam T Numeric value type (integral or floating-point)
     */
    template <Numeric T = float>
    struct Param
    {
        SmoothedValue<T> smoothed {};
        T minValue = T (0);
        T maxValue = T (1);
        const char* name = "";

        /**
         * @brief Configure smoothing time (call from prepare()).
         *
         * @param sampleRate Current sample rate in Hz
         * @param smoothingTime Smoothing duration (default 10ms)
         */
        constexpr void prepare (double sampleRate, Milliseconds<double> smoothingTime = Milliseconds<double> { 10.0 }) noexcept
        {
            smoothed.reset (sampleRate, smoothingTime);
        }

        /**
         * @brief Set target value with clamping to min/max range.
         *
         * @param target Target value (will be clamped to [minValue, maxValue])
         */
        constexpr void set (T target) noexcept
        {
            smoothed.setTargetValue (std::clamp (target, minValue, maxValue));
        }

        /**
         * @brief Get current smoothed value without advancing state.
         */
        [[nodiscard]] constexpr T get() const noexcept
        {
            return smoothed.getCurrentValue();
        }

        /**
         * @brief Advance smoothing and return current value.
         *
         * Call once per sample in the processing loop.
         */
        [[nodiscard]] constexpr T tick() noexcept
        {
            return smoothed.getNextValue();
        }

        /**
         * @brief Check if smoothing is in progress.
         */
        [[nodiscard]] constexpr bool isSmoothing() const noexcept
        {
            return smoothed.isSmoothing();
        }

        /**
         * @brief Set both current and target value immediately (no smoothing).
         *
         * @param value New value (will be clamped to [minValue, maxValue])
         */
        constexpr void setCurrentAndTarget (T value) noexcept
        {
            smoothed.setCurrentAndTargetValue (std::clamp (value, minValue, maxValue));
        }

        /**
         * @brief Convert current value to Frequency strong type.
         */
        [[nodiscard]] constexpr Frequency<T> asFrequency() const noexcept
        {
            return Frequency<T> { get() };
        }

        /**
         * @brief Convert current value to Decibels strong type.
         */
        [[nodiscard]] constexpr Decibels<T> asDecibels() const noexcept
        {
            return Decibels<T> { get() };
        }

        /**
         * @brief Convert current value to DecibelsFullScale strong type.
         */
        [[nodiscard]] constexpr DecibelsFullScale<T> asDecibelsFullScale() const noexcept
        {
            return DecibelsFullScale<T> { get() };
        }

        /**
         * @brief Convert current value to LinearGain strong type.
         */
        [[nodiscard]] constexpr LinearGain<T> asLinearGain() const noexcept
        {
            return LinearGain<T> { get() };
        }

        /**
         * @brief Convert current value to Milliseconds strong type.
         */
        [[nodiscard]] constexpr Milliseconds<T> asMilliseconds() const noexcept
        {
            return Milliseconds<T> { get() };
        }

        /**
         * @brief Convert current value to Seconds strong type.
         */
        [[nodiscard]] constexpr Seconds<T> asSeconds() const noexcept
        {
            return Seconds<T> { get() };
        }
    };

    /**
     * @brief Fixed-size collection of parameters with compile-time descriptors.
     *
     * ParamSet is parameterized by a descriptor array, which defines:
     * - Number of parameters (deduced from array size)
     * - Metadata for each parameter (id, name, range, default, unit)
     *
     * @tparam Descriptors Compile-time array of ParamDescriptor
     * @tparam T Numeric value type for parameters (default: float)
     *
     * ## Example Usage
     * ```cpp
     * inline constexpr std::array<ParamDescriptor, 2> MyDescriptors {{
     *     {"frequency", "Frequency", 20.0f, 20000.0f, 440.0f, "Hz"},
     *     {"gain", "Gain", -60.0f, 12.0f, 0.0f, "dB"}
     * }};
     *
     * using MyParams = ParamSet<MyDescriptors>;
     *
     * MyParams params;
     * params.set<0>(1000.0f);  // Compile-time index
     * params.set(1, -6.0f);    // Runtime index
     * ```
     */
    template <auto Descriptors, Numeric T = float>
    struct ParamSet
    {
        /// Number of parameters (deduced from descriptor array size)
        static constexpr std::size_t NumParams = Descriptors.size();

        /// Compile-time descriptors embedded in type
        static constexpr auto descriptors = Descriptors;

        /// Runtime parameter storage
        std::array<Param<T>, NumParams> m_params {};

        /**
         * @brief Default constructor - initializes params from descriptors.
         */
        constexpr ParamSet() noexcept
        {
            for (std::size_t i = 0; i < NumParams; ++i)
            {
                m_params[i].minValue = static_cast<T> (descriptors[i].minValue);
                m_params[i].maxValue = static_cast<T> (descriptors[i].maxValue);
                m_params[i].name = descriptors[i].name.c_str();
                m_params[i].smoothed.setCurrentAndTargetValue (static_cast<T> (descriptors[i].defaultValue));
            }
        }

        /**
         * @brief Configure smoothing for all parameters.
         *
         * @param sampleRate Current sample rate in Hz
         * @param smoothingTime Smoothing duration (default 10ms)
         */
        constexpr void prepare (double sampleRate, Milliseconds<double> smoothingTime = Milliseconds<double> { 10.0 }) noexcept
        {
            for (auto& p : m_params)
            {
                p.prepare (sampleRate, smoothingTime);
            }
        }

        //----------------------------------------------------------------------
        // Compile-time index-based access (static bounds checking)
        //----------------------------------------------------------------------

        /**
         * @brief Set parameter value by compile-time index (immediate).
         *
         * Sets both current and target value for immediate effect.
         *
         * @tparam Index Parameter index (bounds checked at compile time)
         * @param value Target value
         */
        template <std::size_t Index>
        constexpr void set (T value) noexcept
        {
            static_assert (Index < NumParams, "Param index out of bounds");
            m_params[Index].setCurrentAndTarget (value);
        }

        /**
         * @brief Set target value by compile-time index (for smoothing).
         *
         * Only sets target, allowing smoothing via tick().
         *
         * @tparam Index Parameter index (bounds checked at compile time)
         * @param value Target value
         */
        template <std::size_t Index>
        constexpr void setTarget (T value) noexcept
        {
            static_assert (Index < NumParams, "Param index out of bounds");
            m_params[Index].set (value);
        }

        /**
         * @brief Get parameter value by compile-time index.
         *
         * @tparam Index Parameter index (bounds checked at compile time)
         * @return Current smoothed value
         */
        template <std::size_t Index>
        [[nodiscard]] constexpr T get() const noexcept
        {
            static_assert (Index < NumParams, "Param index out of bounds");
            return m_params[Index].get();
        }

        /**
         * @brief Tick parameter by compile-time index.
         *
         * @tparam Index Parameter index (bounds checked at compile time)
         * @return Current value after advancing smoothing
         */
        template <std::size_t Index>
        [[nodiscard]] constexpr T tick() noexcept
        {
            static_assert (Index < NumParams, "Param index out of bounds");
            return m_params[Index].tick();
        }

        //----------------------------------------------------------------------
        // Runtime index-based access (for GUI binding)
        //----------------------------------------------------------------------

        /**
         * @brief Set parameter value by runtime index (immediate).
         *
         * Sets both current and target value for immediate effect.
         *
         * @param index Parameter index
         * @param value Target value
         */
        constexpr void set (std::size_t index, T value) noexcept
        {
            m_params[index].setCurrentAndTarget (value);
        }

        /**
         * @brief Set target value by runtime index (for smoothing).
         *
         * Only sets target, allowing smoothing via tick().
         *
         * @param index Parameter index
         * @param value Target value
         */
        constexpr void setTarget (std::size_t index, T value) noexcept
        {
            m_params[index].set (value);
        }

        /**
         * @brief Get parameter value by runtime index.
         *
         * @param index Parameter index
         * @return Current smoothed value
         */
        [[nodiscard]] constexpr T get (std::size_t index) const noexcept
        {
            return m_params[index].get();
        }

        /**
         * @brief Tick parameter by runtime index.
         *
         * @param index Parameter index
         * @return Current value after advancing smoothing
         */
        [[nodiscard]] constexpr T tick (std::size_t index) noexcept
        {
            return m_params[index].tick();
        }

        //----------------------------------------------------------------------
        // Introspection
        //----------------------------------------------------------------------

        /**
         * @brief Get number of parameters.
         */
        [[nodiscard]] static constexpr std::size_t numParams() noexcept
        {
            return NumParams;
        }

        /**
         * @brief Get descriptor for a parameter by index.
         *
         * @param index Parameter index
         * @return Reference to parameter descriptor
         */
        [[nodiscard]] static constexpr const ParamDescriptor& descriptor (std::size_t index) noexcept
        {
            return descriptors[index];
        }

        /**
         * @brief Access underlying Param by index (const).
         */
        [[nodiscard]] constexpr const Param<T>& operator[] (std::size_t index) const noexcept
        {
            return m_params[index];
        }

        /**
         * @brief Access underlying Param by index.
         */
        [[nodiscard]] constexpr Param<T>& operator[] (std::size_t index) noexcept
        {
            return m_params[index];
        }

        /**
         * @brief Get number of parameters (alias for compatibility).
         */
        [[nodiscard]] static constexpr std::size_t size() noexcept
        {
            return NumParams;
        }
    };

    //--------------------------------------------------------------------------
    // Concepts for parameter introspection
    //--------------------------------------------------------------------------

    /**
     * @brief Concept to detect if a type is a ParamSet instantiation.
     *
     * A type satisfies ParamSetLike if it has:
     * - `set(index, value)` runtime setter
     * - `get(index)` runtime getter returning float-convertible
     * - `NumParams` static constexpr member
     * - `descriptors` static constexpr member
     */
    template <typename P>
    concept ParamSetLike = requires (P p, std::size_t i) {
        { p.set (i, float {}) };
        { p.get (i) } -> std::convertible_to<float>;
        { P::NumParams } -> std::convertible_to<std::size_t>;
        { P::descriptors };
    };

    /**
     * @brief Concept for State types that contain a params member.
     *
     * A type satisfies HasParams if:
     * - It has a `params` member
     * - That member satisfies ParamSetLike
     */
    template <typename S>
    concept HasParams = requires (S s) {
        { s.params };
    } && ParamSetLike<std::remove_cvref_t<decltype (std::declval<S>().params)>>;

} // namespace PlayfulTones::DspToolbox
