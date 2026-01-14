/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/concepts.hpp"
#include "dsp_toolbox/core/milliseconds.hpp"

#include <concepts>
#include <cstddef>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Click-free parameter smoothing via linear interpolation.
     *
     * Time-based API for consistent behavior across sample rates.
     * Call reset() from prepare() to configure smoothing duration.
     *
     * For floating-point types, accumulates directly in T.
     * For integral types, computes value from progress to avoid truncation errors.
     *
     * @tparam T Numeric value type (integral or floating-point)
     */
    template <Numeric T = float>
    class SmoothedValue
    {
        T currentValue_ {};
        T targetValue_ {};
        T startValue_ {};
        std::size_t smoothingSteps_ { 32 };
        std::size_t stepsRemaining_ { 0 };

        // Only used for floating-point accumulation
        T step_ {};

    public:
        constexpr SmoothedValue() = default;

        /**
         * @brief Configure smoothing time (call from prepare()).
         *
         * @param sampleRate Current sample rate in Hz
         * @param smoothingTime Smoothing duration
         */
        constexpr void reset (double sampleRate, Milliseconds<double> smoothingTime) noexcept
        {
            smoothingSteps_ = static_cast<std::size_t> (sampleRate * smoothingTime.value() / 1000.0);
            stepsRemaining_ = 0;
        }

        /**
         * @brief Set a new target value and begin smoothing.
         *
         * @param newValue Target value to smooth towards
         */
        constexpr void setTargetValue (T newValue) noexcept
        {
            if (newValue == targetValue_)
            {
                return;
            }

            targetValue_ = newValue;

            if (smoothingSteps_ > 0)
            {
                startValue_ = currentValue_;
                stepsRemaining_ = smoothingSteps_;

                if constexpr (std::floating_point<T>)
                {
                    step_ = (targetValue_ - currentValue_) / static_cast<T> (smoothingSteps_);
                }
            }
            else
            {
                currentValue_ = targetValue_;
            }
        }

        /**
         * @brief Get the next smoothed value and advance state.
         *
         * @return Current smoothed value
         */
        [[nodiscard]] constexpr T getNextValue() noexcept
        {
            if (stepsRemaining_ > 0)
            {
                --stepsRemaining_;

                if (stepsRemaining_ == 0)
                {
                    currentValue_ = targetValue_;
                }
                else if constexpr (std::floating_point<T>)
                {
                    currentValue_ += step_;
                }
                else
                {
                    // For integral: compute from progress to avoid truncation errors
                    auto stepsTaken = smoothingSteps_ - stepsRemaining_;
                    auto delta = static_cast<double> (targetValue_ - startValue_);
                    auto progress = static_cast<double> (stepsTaken) / static_cast<double> (smoothingSteps_);
                    currentValue_ = startValue_ + static_cast<T> (delta * progress);
                }
            }
            return currentValue_;
        }

        /**
         * @brief Get current value without advancing state.
         */
        [[nodiscard]] constexpr T getCurrentValue() const noexcept { return currentValue_; }

        /**
         * @brief Check if smoothing is in progress.
         */
        [[nodiscard]] constexpr bool isSmoothing() const noexcept { return stepsRemaining_ > 0; }

        /**
         * @brief Set both current and target value immediately (no smoothing).
         *
         * @param value New value to set
         */
        constexpr void setCurrentAndTargetValue (T value) noexcept
        {
            currentValue_ = value;
            targetValue_ = value;
            startValue_ = value;
            stepsRemaining_ = 0;
        }
    };

} // namespace PlayfulTones::DspToolbox
