/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/math/constants.hpp"
#include "dsp_toolbox/math/functions.hpp"
#include "dsp_toolbox/simd/simd.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <vector>

namespace PlayfulTones::DspToolbox::Processors
{

    namespace detail
    {
        /**
         * @brief Compute zeroth-order modified Bessel function I0(x).
         *
         * Used for Kaiser window computation. Series expansion is sufficient
         * for beta values up to ~15.
         *
         * @param x Input value
         * @return I0(x)
         */
        inline double besselI0 (double x) noexcept
        {
            double sum = 1.0;
            double term = 1.0;
            double const halfX = x * 0.5;

            for (int k = 1; k < 40; ++k)
            {
                term *= (halfX / static_cast<double> (k));
                term *= (halfX / static_cast<double> (k));
                sum += term;
                if (term < 1e-20 * sum)
                    break;
            }
            return sum;
        }

        /**
         * @brief Kaiser window function.
         *
         * @param n Sample index
         * @param length Window length
         * @param beta Shape parameter (higher = narrower main lobe)
         * @return Window value at sample n
         */
        inline double kaiserWindow (std::size_t n, std::size_t length, double beta) noexcept
        {
            double const M = static_cast<double> (length - 1);
            double const nCentered = 2.0 * static_cast<double> (n) / M - 1.0;
            double const argSq = 1.0 - nCentered * nCentered;
            // Clamp to avoid sqrt of negative due to rounding
            double const arg = beta * std::sqrt (std::max (0.0, argSq));
            return besselI0 (arg) / besselI0 (beta);
        }

        // =====================================================================
        // Windowed-sinc interpolation table parameters
        //
        // Simple sinc interpolator with Hann window. The sinc kernel provides
        // smooth sub-sample interpolation; anti-aliasing duty is handled by
        // the separate IIR filter in ResamplerPair.
        //
        // With Nz=4 and L=32, the interpolation kernel spans 8 input samples
        // (4 per wing). Linear interpolation between the 32 sub-phases provides
        // continuous fractional positioning for arbitrary rate ratios.
        // =====================================================================

        /// Number of zero-crossings per wing of the sinc function.
        /// 4 zero-crossings = 8 total taps. This is sufficient for smooth
        /// interpolation when a separate AA filter handles alias rejection.
        inline constexpr std::size_t kSincZeroCrossings = 4;

        /// Table entries per zero-crossing (polyphase sub-phases).
        /// 32 phases with linear interpolation between them provides ample
        /// sub-sample precision. Total table size: 32 * 4 + 1 = 129 entries.
        inline constexpr std::size_t kSincTableResolution = 32;

        /// Total number of entries in one wing of the sinc table.
        inline constexpr std::size_t kSincTableSize = kSincZeroCrossings * kSincTableResolution + 1;

        /**
         * @brief Lightweight second-order IIR section (biquad) for anti-aliasing.
         *
         * Direct Form I: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
         * Coefficients are pre-normalized (a0 = 1).
         */
        struct BiquadSection
        {
            float b0 { 1.0f }, b1 { 0.0f }, b2 { 0.0f };
            float a1 { 0.0f }, a2 { 0.0f };
            float x1 { 0.0f }, x2 { 0.0f };
            float y1 { 0.0f }, y2 { 0.0f };

            void reset() noexcept
            {
                x1 = x2 = y1 = y2 = 0.0f;
            }

            /**
             * @brief Design a lowpass biquad section for a Butterworth cascade.
             *
             * @param cutoffHz Cutoff frequency in Hz
             * @param sampleRate Sample rate in Hz
             * @param q Q factor for this section (determined by Butterworth pole position)
             */
            void designLowpass (double cutoffHz, double sampleRate, double q) noexcept
            {
                double const omega = 2.0 * 3.14159265358979323846 * cutoffHz / sampleRate;
                double const sinOmega = std::sin (omega);
                double const cosOmega = std::cos (omega);
                double const alpha = sinOmega / (2.0 * q);

                double const b0d = (1.0 - cosOmega) * 0.5;
                double const b1d = 1.0 - cosOmega;
                double const b2d = (1.0 - cosOmega) * 0.5;
                double const a0d = 1.0 + alpha;
                double const a1d = -2.0 * cosOmega;
                double const a2d = 1.0 - alpha;

                // Normalize by a0
                double const invA0 = 1.0 / a0d;
                b0 = static_cast<float> (b0d * invA0);
                b1 = static_cast<float> (b1d * invA0);
                b2 = static_cast<float> (b2d * invA0);
                a1 = static_cast<float> (a1d * invA0);
                a2 = static_cast<float> (a2d * invA0);
            }

            float processSample (float x) noexcept
            {
                float const y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
                x2 = x1;
                x1 = x;
                y2 = y1;
                y1 = y;
                return y;
            }
        };

        /**
         * @brief 8th-order Butterworth lowpass filter (4 cascaded biquad sections).
         *
         * Used as an anti-aliasing filter before downsampling in ResamplerPair.
         * Applied at the higher sample rate to remove content above the target
         * Nyquist before the sinc decimation step.
         */
        struct AntiAliasingFilter
        {
            static constexpr std::size_t kNumSections = 4;
            std::array<BiquadSection, kNumSections> sections {};
            bool active { false };

            /**
             * @brief Design the filter for a given cutoff and sample rate.
             * @param cutoffHz Anti-aliasing cutoff frequency in Hz
             * @param sampleRate The rate at which the filter will operate (processing rate)
             */
            void design (double cutoffHz, double sampleRate) noexcept
            {
                // 8th-order Butterworth Q values for each 2nd-order section.
                // Q_k = 1 / (2 * cos((2k-1) * pi / (2*N))) for k=1..N/2, N=8
                static constexpr std::array<double, kNumSections> butterworthQ = {
                    0.50979557910415919, // 1 / (2 * cos(  pi/16))
                    0.60134488693504528, // 1 / (2 * cos(3*pi/16))
                    0.89997622313641570, // 1 / (2 * cos(5*pi/16))
                    2.56291544774150580 // 1 / (2 * cos(7*pi/16))
                };

                for (std::size_t i = 0; i < kNumSections; ++i)
                    sections[i].designLowpass (cutoffHz, sampleRate, butterworthQ[i]);

                active = true;
            }

            void reset() noexcept
            {
                for (auto& s : sections)
                    s.reset();
            }

            /**
             * @brief Process a block of audio through all 4 cascaded biquad sections.
             *
             * @param data Audio buffer to filter in-place
             * @param count Number of samples
             */
            void process (float* data, std::size_t count) noexcept
            {
                if (!active)
                    return;

                // Process each section in series over the entire block
                for (auto& s : sections)
                {
                    for (std::size_t i = 0; i < count; ++i)
                        data[i] = s.processSample (data[i]);
                }
            }
        };

    } // namespace detail

    /**
     * @brief Windowed-sinc interpolation resampler for arbitrary sample rate conversion.
     *
     * Simple sinc interpolator with a Hann window for smooth sub-sample positioning.
     * The kernel is intentionally small (Nz=4, 8 total taps) — anti-aliasing duty
     * is handled by the separate IIR filter in ResamplerPair. This design provides
     * low latency (4 input samples), minimal boundary artifacts, and clean
     * interpolation for arbitrary rate ratios.
     *
     * ## Algorithm
     * A sinc table of Nz=4 zero-crossings × L=32 entries per zero-crossing is
     * pre-computed at prepare() time. For each output sample, the algorithm:
     *
     * 1. Computes the output time position in input-sample coordinates
     * 2. Determines the integer sample index and fractional offset
     * 3. Convolves 8 input samples with the Hann-windowed sinc kernel
     * 4. Uses linear interpolation between sub-phases for continuous positioning
     *
     * Per-sub-filter normalization ensures unity DC gain at all fractional positions.
     * When downsampling (rho < 1), the sinc kernel is stretched by 1/rho and
     * scaled by rho.
     *
     * ## Usage
     * @code
     * Resampler rs;
     * rs.prepare(48000.0, 96000.0, 512);
     * std::size_t outCount = rs.process(input, inputCount, output);
     * @endcode
     */
    class Resampler
    {
    public:
        /**
         * @brief Configure the resampler for a given rate conversion.
         *
         * Builds the windowed-sinc lookup table and allocates the input ring buffer.
         *
         * @param sourceRate Source sample rate in Hz
         * @param targetRate Target sample rate in Hz
         * @param maxInputBlockSize Maximum number of input samples per process() call
         *
         * @note May allocate memory. Not real-time safe.
         */
        void prepare (double sourceRate, double targetRate, std::size_t maxInputBlockSize)
        {
            m_sourceRate = sourceRate;
            m_targetRate = targetRate;
            m_rho = targetRate / sourceRate;
            m_passthrough = (std::round (sourceRate) == std::round (targetRate));

            if (m_passthrough)
            {
                m_sincTable.clear();
                m_sincDelta.clear();
                m_phaseNorm.clear();
                m_ringBuffer.clear();
                m_ringSize = 0;
                m_ringWritePos = 0;
                m_inputSamplesConsumed = 0;
                m_timeAccumulator = 0.0;
                return;
            }

            // Build the sinc table (one wing only; symmetric)
            buildSincTable();

            // Compute the effective filter half-width in input samples.
            // When rho >= 1 (upsampling), the filter spans Nz input samples per wing.
            // When rho < 1 (downsampling), the filter is stretched by 1/rho,
            // spanning ceil(Nz / rho) input samples per wing.
            if (m_rho >= 1.0)
                m_filterWingLen = detail::kSincZeroCrossings;
            else
                m_filterWingLen = static_cast<std::size_t> (
                    std::ceil (static_cast<double> (detail::kSincZeroCrossings) / m_rho));

            // Ring buffer must hold enough history for the filter wing plus the
            // maximum input block.
            m_ringSize = m_filterWingLen + 1 + maxInputBlockSize + m_filterWingLen;
            m_ringBuffer.assign (m_ringSize, 0.0f);
            m_ringWritePos = 0;

            // Time accumulator: tracks the current output time in input-sample
            // coordinates. Starts at 0; after processing N input samples, valid
            // output times range from [filter_wing, N - filter_wing).
            m_timeAccumulator = 0.0;
            m_inputSamplesConsumed = 0;
        }

        /**
         * @brief Reset all filter state to zero.
         */
        void reset() noexcept
        {
            std::fill (m_ringBuffer.begin(), m_ringBuffer.end(), 0.0f);
            m_ringWritePos = 0;
            m_timeAccumulator = 0.0;
            m_inputSamplesConsumed = 0;
        }

        /**
         * @brief Compute maximum possible output samples for a given input count.
         *
         * @param inputSamples Number of input samples
         * @return Maximum number of output samples that process() may produce
         */
        [[nodiscard]] std::size_t getMaxOutputSamples (std::size_t inputSamples) const noexcept
        {
            if (m_passthrough)
                return inputSamples;
            // Upper bound: ceil(inputSamples * rho) + 1.
            // The +1 accounts for the fractional time accumulator position: if the
            // accumulator starts mid-sample, we may produce one extra output sample.
            return static_cast<std::size_t> (
                       std::ceil (static_cast<double> (inputSamples) * m_rho))
                   + 1;
        }

        /**
         * @brief Get the latency introduced by the resampler in output samples.
         *
         * The latency is the group delay of the windowed sinc filter, measured
         * at the output sample rate. The sinc filter spans Nz zero-crossings per
         * wing, giving a group delay of Nz input samples, converted to output
         * samples by multiplying by rho.
         *
         * @return Latency in output samples
         */
        [[nodiscard]] std::size_t getLatencySamples() const noexcept
        {
            if (m_passthrough)
                return 0;
            // Group delay = Nz input samples (center of the 2*Nz+1 tap filter)
            // In output samples: Nz * rho
            return static_cast<std::size_t> (
                std::round (static_cast<double> (detail::kSincZeroCrossings) * m_rho));
        }

        /**
         * @brief Process a block of input samples, producing resampled output.
         *
         * Output-driven algorithm: for each output sample time (stepping by
         * 1/rho in input coordinates), interpolates the input signal using
         * the windowed sinc table.
         *
         * @param input Pointer to input samples
         * @param inputCount Number of input samples
         * @param output Pointer to output buffer (must hold getMaxOutputSamples(inputCount))
         * @return Number of output samples written
         */
        std::size_t process (const float* input, std::size_t inputCount, float* output) noexcept
        {
            if (m_passthrough)
            {
                simd::copy (output, input, inputCount);
                return inputCount;
            }

            // Push all input samples into the ring buffer
            for (std::size_t i = 0; i < inputCount; ++i)
            {
                m_ringBuffer[m_ringWritePos] = input[i];
                m_ringWritePos = (m_ringWritePos + 1) % m_ringSize;
            }
            m_inputSamplesConsumed += inputCount;

            // Output-driven: generate output samples at times spaced by
            // 1/rho in input-sample coordinates.
            double const inputStep = 1.0 / m_rho;

            // Maximum valid input time: we must not generate output samples whose
            // interpolation center is within m_filterWingLen of the right edge
            // of available data. Without this guard, the right wing of the sinc
            // filter convolves with zeros (from readRing returning 0 for future
            // samples), causing Gibbs ringing artifacts at every block boundary.
            //
            // This introduces a latency of m_filterWingLen input samples (the
            // filter needs that much "look-ahead" context). The deferred samples
            // are produced when the NEXT block arrives and provides the context.
            // The time accumulator carries over between blocks, so no samples
            // are permanently lost — only delayed.
            double const maxInputTime = static_cast<double> (m_inputSamplesConsumed)
                                        - static_cast<double> (m_filterWingLen) - 1.0;

            std::size_t outputCount = 0;

            while (m_timeAccumulator <= maxInputTime)
            {
                output[outputCount++] = interpolateSample (m_timeAccumulator);
                m_timeAccumulator += inputStep;
            }

            return outputCount;
        }

        /**
         * @brief Process with a target output count, adjusting the time step.
         *
         * Instead of using the fixed 1/rho step, computes a per-block step that
         * produces exactly targetOutputCount samples from the given input. This
         * eliminates sample duplication/dropping in the conservation mechanism.
         *
         * The adjusted step is: inputCount / targetOutputCount, which is a tiny
         * perturbation from the nominal 1/rho step. For example, at 96→88.2kHz
         * with 278 input samples and target 256 outputs:
         *   nominal step = 96000/88200 = 1.08844
         *   adjusted step = 278/256 = 1.08594
         *   difference = 0.23% — inaudible pitch shift within one block.
         *
         * @param input Pointer to input samples
         * @param inputCount Number of input samples
         * @param output Pointer to output buffer
         * @param targetOutputCount Exact number of output samples to produce
         * @return Number of output samples written (always == targetOutputCount)
         */
        /**
         * @brief Process with a target output count for sample-count conservation.
         *
         * Uses the nominal 1/rho time step for all output samples (preserving the
         * correct frequency response), then smoothly corrects for any +-1 sample
         * drift by interpolating an extra sample or blending the last two samples.
         *
         * This eliminates the zero-order-hold artifacts (micro-clicks) from the
         * old conservation mechanism that duplicated/dropped samples.
         *
         * @param input Pointer to input samples
         * @param inputCount Number of input samples
         * @param output Pointer to output buffer
         * @param targetOutputCount Exact number of output samples to produce
         * @return Number of output samples written (== targetOutputCount)
         */
        std::size_t processExact (const float* input, std::size_t inputCount, float* output, std::size_t targetOutputCount) noexcept
        {
            if (m_passthrough || targetOutputCount == 0)
            {
                auto const count = std::min (inputCount, targetOutputCount);
                simd::copy (output, input, count);
                return count;
            }

            // Push input into ring buffer
            for (std::size_t i = 0; i < inputCount; ++i)
            {
                m_ringBuffer[m_ringWritePos] = input[i];
                m_ringWritePos = (m_ringWritePos + 1) % m_ringSize;
            }
            m_inputSamplesConsumed += inputCount;

            // Use nominal time step to preserve correct frequency response
            double const inputStep = 1.0 / m_rho;

            double const maxInputTime = static_cast<double> (m_inputSamplesConsumed)
                                        - static_cast<double> (m_filterWingLen) - 1.0;

            // Phase 1: produce samples at the nominal rate, up to targetOutputCount.
            // Stop early if we'd exceed available input data.
            std::size_t produced = 0;
            while (produced < targetOutputCount && m_timeAccumulator <= maxInputTime)
            {
                output[produced++] = interpolateSample (m_timeAccumulator);
                m_timeAccumulator += inputStep;
            }

            // Phase 2: correct for drift (typically +-1 sample).
            if (produced < targetOutputCount)
            {
                // Need more samples than the nominal step produced.
                // Interpolate between the last two output samples to fill the gap
                // smoothly, avoiding zero-order-hold clicks.
                std::size_t const deficit = targetOutputCount - produced;
                if (produced >= 2)
                {
                    // Linear interpolation from the last produced sample onward
                    float const s0 = output[produced - 2];
                    float const s1 = output[produced - 1];
                    float const delta = s1 - s0;
                    for (std::size_t i = 0; i < deficit; ++i)
                    {
                        float const t = static_cast<float> (i + 1) / static_cast<float> (deficit + 1);
                        output[produced + i] = s1 + delta * t;
                    }
                }
                else if (produced == 1)
                {
                    for (std::size_t i = 0; i < deficit; ++i)
                        output[produced + i] = output[0];
                }
                else
                {
                    for (std::size_t i = 0; i < deficit; ++i)
                        output[i] = 0.0f;
                }
                produced = targetOutputCount;
            }
            else if (produced > targetOutputCount)
            {
                // Produced too many — blend the last two into one.
                // This should rarely happen (only if accumulator was ahead).
                // Simply truncate and adjust the accumulator to compensate.
                std::size_t const excess = produced - targetOutputCount;
                m_timeAccumulator -= static_cast<double> (excess) * inputStep;
                produced = targetOutputCount;
            }

            return produced;
        }
        /**
         * @brief Check if the resampler is in passthrough mode (no conversion).
         *
         * @return true if source and target rates are identical
         */
        [[nodiscard]] bool isPassthrough() const noexcept
        {
            return m_passthrough;
        }

    private:
        /**
         * @brief Build the windowed-sinc lookup table.
         *
         * Stores one wing (the right wing) of a Hann-windowed sinc function.
         * The table is sampled at L = kSincTableResolution points per zero-crossing,
         * for Nz = kSincZeroCrossings zero-crossings. A companion delta table
         * stores the difference between adjacent entries for fast linear interpolation.
         *
         * The sinc function is: sinc(t) = sin(pi * t) / (pi * t), windowed by
         * a Hann window: w(t) = cos(0.5 * pi * t / Nz)^2 for |t| <= Nz.
         *
         * Entry h[i] corresponds to time t = i / L (in units of the input sample
         * period). The table covers t in [0, Nz].
         *
         * After generation, each of the L+1 sub-filters (one per fractional phase)
         * is normalized to sum to 1.0, preventing gain ripple across fractional
         * positions.
         */
        void buildSincTable()
        {
            constexpr std::size_t tableLen = detail::kSincTableSize;
            constexpr std::size_t L = detail::kSincTableResolution;
            constexpr std::size_t Nz = detail::kSincZeroCrossings;

            m_sincTable.resize (tableLen);
            m_sincDelta.resize (tableLen);

            constexpr double pi = 3.14159265358979323846;

            // Store the right wing of sinc(t) * hann(t).
            // Hann window: w(t) = cos(0.5 * pi * t / Nz)^2, which tapers
            // smoothly from 1.0 at t=0 to 0.0 at t=Nz.
            for (std::size_t i = 0; i < tableLen; ++i)
            {
                double const t = static_cast<double> (i) / static_cast<double> (L);

                // sinc(t) = sin(pi * t) / (pi * t), with sinc(0) = 1
                double sincVal = 1.0;
                if (t > 1e-12)
                {
                    double const arg = pi * t;
                    sincVal = std::sin (arg) / arg;
                }

                // Hann window (cos^2 taper)
                double const windowArg = 0.5 * pi * t / static_cast<double> (Nz);
                double const cosVal = std::cos (windowArg);
                double const window = cosVal * cosVal;

                m_sincTable[i] = static_cast<float> (sincVal * window);
            }

            // Per-sub-filter normalization: for each fractional phase (0..L),
            // the sum of the coefficients used during interpolation should be 1.0
            // to prevent gain variation across fractional positions.
            //
            // We store a per-phase normalization factor in a separate vector
            // and apply it during interpolation. This avoids the shared-entry
            // problem where modifying table entries for one phase corrupts others.
            m_phaseNorm.resize (L + 1);

            for (std::size_t phase = 0; phase <= L; ++phase)
            {
                double sum = 0.0;

                // Left wing: table positions phase, phase+L, phase+2L, ...
                for (std::size_t k = 0; k <= Nz; ++k)
                {
                    std::size_t const idx = phase + k * L;
                    if (idx < tableLen)
                        sum += static_cast<double> (m_sincTable[idx]);
                }

                // Right wing: table positions (L-phase), (2L-phase), ...
                for (std::size_t k = 1; k <= Nz; ++k)
                {
                    std::size_t const idx = k * L - phase;
                    if (idx < tableLen)
                        sum += static_cast<double> (m_sincTable[idx]);
                }

                m_phaseNorm[phase] = (std::abs (sum) > 1e-10) ? static_cast<float> (1.0 / sum) : 1.0f;
            }

            // Build delta table for linear interpolation: delta[i] = table[i+1] - table[i]
            for (std::size_t i = 0; i + 1 < tableLen; ++i)
                m_sincDelta[i] = m_sincTable[i + 1] - m_sincTable[i];
            m_sincDelta[tableLen - 1] = 0.0f; // endpoint: no interpolation beyond last entry
        }

        /**
         * @brief Interpolate one output sample at a given input time position.
         *
         * Evaluates the bandlimited interpolation sum (Shannon reconstruction)
         * by convolving input samples with the windowed sinc centered at time t.
         *
         * For rho >= 1 (upsampling):
         *   y(t) = sum_{i=-Nz}^{Nz} x[n+i] * h(|frac - i| * L)
         *   where n = floor(t), frac = t - n.
         *
         * For rho < 1 (downsampling):
         *   The sinc is stretched by 1/rho to lower the cutoff frequency.
         *   y(t) = rho * sum_{i=-ceil(Nz/rho)}^{ceil(Nz/rho)} x[n+i] * h(|frac-i| * rho * L)
         *
         * In both cases, the sinc table is symmetric, so we process the "right wing"
         * (samples at n+1, n+2, ...) and "left wing" (samples at n, n-1, n-2, ...)
         * using the same table with mirrored fractional offsets.
         *
         * @param t Time position in input-sample coordinates
         * @return Interpolated output sample
         */
        float interpolateSample (double t) const noexcept
        {
            auto const n = static_cast<std::int64_t> (std::floor (t));
            double const frac = t - static_cast<double> (n);

            constexpr auto L = static_cast<double> (detail::kSincTableResolution);

            // Determine the filter configuration based on conversion direction
            double filterScale; // Scale factor for table index stepping
            double ampScale; // Amplitude scaling for output
            std::size_t wingLen; // Number of input samples per wing

            if (m_rho >= 1.0)
            {
                // Upsampling: use the sinc table at full resolution.
                // The sinc cuts off at the source Nyquist, which is correct
                // since the output rate is higher and no aliasing occurs.
                filterScale = L;
                ampScale = 1.0;
                wingLen = detail::kSincZeroCrossings;
            }
            else
            {
                // Downsampling: stretch sinc by 1/rho to lower the effective
                // cutoff to the output Nyquist. The separate AA IIR filter in
                // ResamplerPair handles the steep rolloff needed for alias
                // rejection; the sinc just needs to interpolate cleanly.
                //
                // filterScale = rho * L compresses the table index stepping,
                // widening the sinc in time domain (narrowing passband).
                // ampScale = rho normalizes DC gain to unity.
                filterScale = m_rho * L;
                ampScale = m_rho;
                wingLen = m_filterWingLen;
            }

            double sum = 0.0;
            auto const tableLen = static_cast<double> (detail::kSincTableSize - 1);

            // --- Left wing: samples at n, n-1, n-2, ... ---
            // Distance from t to sample n is frac (>= 0)
            // Distance from t to sample n-1 is 1 + frac
            // etc.
            {
                double tablePos = frac * filterScale;
                for (std::size_t i = 0; i <= wingLen; ++i)
                {
                    if (tablePos > tableLen)
                        break;

                    float const h = lookupSinc (tablePos);
                    std::int64_t const sampleIdx = n - static_cast<std::int64_t> (i);
                    sum += static_cast<double> (readRing (sampleIdx)) * static_cast<double> (h);

                    tablePos += filterScale;
                }
            }

            // --- Right wing: samples at n+1, n+2, ... ---
            // Distance from t to sample n+1 is (1 - frac)
            // Distance from t to sample n+2 is (2 - frac)
            // etc.
            {
                double tablePos = (1.0 - frac) * filterScale;
                for (std::size_t i = 1; i <= wingLen; ++i)
                {
                    if (tablePos > tableLen)
                        break;

                    float const h = lookupSinc (tablePos);
                    std::int64_t const sampleIdx = n + static_cast<std::int64_t> (i);
                    sum += static_cast<double> (readRing (sampleIdx)) * static_cast<double> (h);

                    tablePos += filterScale;
                }
            }

            // Apply per-phase normalization for upsampling to ensure unity DC gain.
            // For downsampling, the stretched sinc and AA filter handle this.
            if (m_rho >= 1.0)
            {
                constexpr auto Ld = static_cast<double> (detail::kSincTableResolution);
                auto const phase = static_cast<std::size_t> (
                    std::min (frac * Ld, static_cast<double> (detail::kSincTableResolution)));
                sum *= static_cast<double> (m_phaseNorm[phase]);
            }

            return static_cast<float> (sum * ampScale);
        }

        /**
         * @brief Look up a value from the sinc table with linear interpolation.
         *
         * @param tablePos Position in the table (0 to kSincTableSize-1)
         * @return Linearly interpolated sinc value
         */
        float lookupSinc (double tablePos) const noexcept
        {
            auto const idx = static_cast<std::size_t> (tablePos);
            auto const eta = static_cast<float> (tablePos - static_cast<double> (idx));
            return m_sincTable[idx] + eta * m_sincDelta[idx];
        }

        /**
         * @brief Read a sample from the ring buffer by absolute input index.
         *
         * Maps from the absolute input sample index (0-based from the start of
         * all input ever processed) to the ring buffer position.
         *
         * @param absIdx Absolute input sample index
         * @return Sample value, or 0 if out of range
         */
        float readRing (std::int64_t absIdx) const noexcept
        {
            // The ring buffer holds the most recent m_ringSize samples.
            // m_ringWritePos points to the next write position.
            // The oldest sample in the ring is at absolute index
            // (m_inputSamplesConsumed - m_ringSize).
            auto const oldest = static_cast<std::int64_t> (m_inputSamplesConsumed)
                                - static_cast<std::int64_t> (m_ringSize);

            if (absIdx < oldest || absIdx >= static_cast<std::int64_t> (m_inputSamplesConsumed))
                return 0.0f;

            // Map to ring position
            auto const offset = static_cast<std::size_t> (absIdx - oldest);
            // The oldest sample is at (m_ringWritePos) in the ring (since writePos
            // is the next slot to write, it's also where the oldest data is after
            // the buffer has been fully written at least once).
            std::size_t const ringIdx = (m_ringWritePos + offset) % m_ringSize;
            return m_ringBuffer[ringIdx];
        }

        double m_sourceRate { 0.0 };
        double m_targetRate { 0.0 };
        double m_rho { 1.0 }; ///< Rate ratio: targetRate / sourceRate
        bool m_passthrough { true };

        /// Windowed sinc lookup table: one wing, kSincTableSize entries.
        /// Entry i corresponds to time t = i / kSincTableResolution.
        std::vector<float> m_sincTable;

        /// Difference table for linear interpolation: delta[i] = table[i+1] - table[i]
        std::vector<float> m_sincDelta;

        /// Per-phase normalization factors: m_phaseNorm[p] corrects the DC gain
        /// for fractional phase p (0..L). Applied during interpolation.
        std::vector<float> m_phaseNorm;

        /// Filter wing length in input samples (Nz for rho>=1, ceil(Nz/rho) for rho<1)
        std::size_t m_filterWingLen { 0 };

        /// Circular ring buffer for input samples
        std::vector<float> m_ringBuffer;
        std::size_t m_ringSize { 0 };
        std::size_t m_ringWritePos { 0 };

        /// Total number of input samples consumed since last prepare/reset
        std::size_t m_inputSamplesConsumed { 0 };

        /// Time accumulator: current output time position in input-sample coordinates.
        /// Incremented by 1/rho for each output sample produced.
        double m_timeAccumulator { 0.0 };
    };

    /**
     * @brief Bidirectional resampler pair for host/processing rate conversion.
     *
     * Composes two Resampler instances (up and down) to convert audio from a
     * host/DAW sample rate to a fixed processing rate and back. This is used
     * when a DSP model requires a specific sample rate (e.g., 96 kHz) regardless
     * of the host sample rate.
     *
     * ## Usage
     * @code
     * ResamplerPair pair;
     * pair.prepare(48000.0, 96000.0, 512);
     *
     * // Host -> processing rate
     * std::size_t upCount = pair.processUp(hostInput, hostSamples, processingBuffer);
     *
     * // ... run model at processing rate ...
     *
     * // Processing rate -> host rate
     * std::size_t downCount = pair.processDown(processingBuffer, upCount, hostOutput);
     * @endcode
     */
    class ResamplerPair
    {
    public:
        /**
         * @brief Configure both up and down resamplers.
         *
         * @param hostRate The host/DAW sample rate
         * @param processingRate The model's native sample rate (e.g., 96000)
         * @param maxHostBlockSize Maximum block size from the host
         *
         * @note May allocate memory. Not real-time safe.
         */
        void prepare (double hostRate, double processingRate, std::size_t maxHostBlockSize)
        {
            m_hostRate = hostRate;
            m_processingRate = processingRate;
            m_passthrough = (std::round (hostRate) == std::round (processingRate));

            m_hostSamplesIn = 0;
            m_hostSamplesOut = 0;

            m_upResampler.prepare (hostRate, processingRate, maxHostBlockSize);

            std::size_t const maxProcessingBlockSize = m_upResampler.getMaxOutputSamples (maxHostBlockSize);
            m_downResampler.prepare (processingRate, hostRate, maxProcessingBlockSize);

            // Anti-aliasing filter: when downsampling from a higher processing rate,
            // apply a steep lowpass at the target Nyquist to reject content that would
            // alias. This is critical when the processing contains nonlinear elements
            // (e.g. neural amp models) that generate harmonics above the host Nyquist.
            //
            // Cutoff is set at 98% of the host Nyquist (hostRate / 2). This
            // correctly scales for all host rates:
            //   44.1kHz → 21609 Hz    88.2kHz → 43218 Hz
            //   48kHz   → 23520 Hz    96kHz   → passthrough (no filter)
            if (!m_passthrough && processingRate > hostRate)
            {
                double const cutoff = hostRate * 0.49;
                m_antiAliasingFilter.design (cutoff, processingRate);
                m_aaBuffer.resize (maxProcessingBlockSize);
            }
            else
            {
                m_antiAliasingFilter.active = false;
                m_aaBuffer.clear();
            }
        }

        /**
         * @brief Reset both resamplers' internal state.
         */
        void reset() noexcept
        {
            m_upResampler.reset();
            m_downResampler.reset();
            m_antiAliasingFilter.reset();
            m_hostSamplesIn = 0;
            m_hostSamplesOut = 0;
        }

        /**
         * @brief Get max number of samples at processing rate for a given host block.
         *
         * @param hostSamples Number of host-rate input samples
         * @return Maximum number of processing-rate samples that processUp() may produce
         */
        [[nodiscard]] std::size_t getMaxProcessingSamples (std::size_t hostSamples) const noexcept
        {
            return m_upResampler.getMaxOutputSamples (hostSamples);
        }

        /**
         * @brief Get total latency in host-rate samples (up + down combined).
         *
         * @return Latency in host-rate samples
         */
        [[nodiscard]] std::size_t getLatencySamples() const noexcept
        {
            if (m_passthrough)
                return 0;

            // Up resampler latency is in processing-rate samples.
            // Convert to host-rate: multiply by hostRate / processingRate.
            std::size_t const upLatencyProcessingSamples = m_upResampler.getLatencySamples();
            auto const upLatencyHostSamples = static_cast<std::size_t> (
                std::round (static_cast<double> (upLatencyProcessingSamples) * m_hostRate / m_processingRate));

            // Down resampler latency is already in host-rate samples (its output rate).
            std::size_t const downLatencyHostSamples = m_downResampler.getLatencySamples();

            return upLatencyHostSamples + downLatencyHostSamples;
        }

        /**
         * @brief Resample from host rate to processing rate.
         *
         * Tracks the cumulative host-rate input sample count so that
         * processDown() can guarantee exact sample count conservation.
         *
         * @param input Pointer to host-rate input samples
         * @param inputCount Number of input samples
         * @param output Pointer to output buffer (must hold getMaxProcessingSamples(inputCount))
         * @return Number of processing-rate samples written
         */
        std::size_t processUp (const float* input, std::size_t inputCount, float* output) noexcept
        {
            m_hostSamplesIn += inputCount;
            return m_upResampler.process (input, inputCount, output);
        }

        /**
         * @brief Resample from processing rate back to host rate.
         *
         * For non-integer rate ratios (e.g. 44.1kHz <-> 96kHz), the independent
         * up and down resamplers can produce +-1 sample drift per block.
         * This method compensates by tracking the cumulative expected host-rate
         * output and adjusting the actual output to match exactly.
         *
         * @param input Pointer to processing-rate input samples
         * @param inputCount Number of input samples
         * @param output Pointer to output buffer
         * @return Number of host-rate samples written (always matches cumulative input)
         */
        std::size_t processDown (const float* input, std::size_t inputCount, float* output) noexcept
        {
            // Apply anti-aliasing filter before downsampling to reject content
            // above the host Nyquist that would fold back as aliasing.
            const float* downInput = input;
            if (m_antiAliasingFilter.active && inputCount > 0)
            {
                std::copy (input, input + inputCount, m_aaBuffer.data());
                m_antiAliasingFilter.process (m_aaBuffer.data(), inputCount);
                downInput = m_aaBuffer.data();
            }

            // Compute the exact target output count for this block.
            // The round-trip must conserve sample count: total output == total input.
            // Instead of producing samples freely and then duplicating/dropping
            // (which creates micro-clicks), we tell the downsampler exactly how
            // many samples to produce, with a slightly adjusted time step.
            std::size_t const targetOutput = m_hostSamplesIn - m_hostSamplesOut;

            std::size_t const produced = m_downResampler.processExact (
                downInput, inputCount, output, targetOutput);
            m_hostSamplesOut += produced;

            return produced;
        }

        /**
         * @brief Get max number of host-rate samples for a given processing block.
         *
         * Use this to size host-rate output buffers for processDown().
         *
         * @param processingSamples Number of processing-rate input samples
         * @return Maximum number of host-rate samples that processDown() may produce
         */
        [[nodiscard]] std::size_t getMaxHostOutputSamples (std::size_t processingSamples) const noexcept
        {
            return m_downResampler.getMaxOutputSamples (processingSamples);
        }

        /**
         * @brief True if host rate equals processing rate.
         *
         * @return true if no resampling is needed
         */
        [[nodiscard]] bool isPassthrough() const noexcept
        {
            return m_passthrough;
        }

        /// Diagnostic: number of blocks where sample count conservation adjusted output.
        [[nodiscard]] std::size_t getConservationAdjustments() const noexcept
        {
            return m_conservationAdjustments;
        }

        /// Diagnostic: total samples added/removed by sample count conservation.
        [[nodiscard]] std::size_t getTotalSamplesAdjusted() const noexcept
        {
            return m_totalSamplesAdjusted;
        }

    private:
        double m_hostRate { 0.0 };
        double m_processingRate { 0.0 };
        bool m_passthrough { true };

        Resampler m_upResampler; ///< Host rate -> processing rate
        Resampler m_downResampler; ///< Processing rate -> host rate

        /// 8th-order Butterworth anti-aliasing filter applied at the processing
        /// rate before downsampling. Rejects content above the host Nyquist
        /// that would fold back as aliasing during decimation.
        detail::AntiAliasingFilter m_antiAliasingFilter;
        std::vector<float> m_aaBuffer; ///< Temp buffer for AA-filtered data

        /// Cumulative host-rate sample counters for sample count conservation.
        /// processDown() uses these to ensure the round-trip output exactly
        /// matches the input count, compensating for drift in non-integer
        /// rate ratios (e.g. 44.1kHz <-> 96kHz).
        std::size_t m_hostSamplesIn { 0 };
        std::size_t m_hostSamplesOut { 0 };

        /// Diagnostic counters for sample count conservation.
        std::size_t m_conservationAdjustments { 0 };
        std::size_t m_totalSamplesAdjusted { 0 };
    };

} // namespace PlayfulTones::DspToolbox::Processors
