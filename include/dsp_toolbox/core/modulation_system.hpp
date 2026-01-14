/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/param.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief CV polarity hint for UI visualization.
     *
     * Indicates the expected range of CV signals for a slot.
     * This is a hint for UI display, not a runtime constraint.
     */
    enum class CVPolarity : std::uint8_t {
        Unipolar, ///< 0 to 1 expected
        Bipolar, ///< -1 to 1 expected
        Any ///< No preference
    };

    /**
     * @brief Compile-time configuration for a CV slot.
     *
     * Defines the static mapping from a CV input to a parameter, including
     * optional meta-modulation (where another CV modulates the amount).
     *
     * All fields are const to enforce immutability - configuration
     * is set at compile time via aggregate initialization.
     */
    struct CVSlotConfig
    {
        /** @brief Index of the parameter this CV modulates */
        const std::size_t paramIndex = 0;

        /** @brief Expected polarity of CV signal (hint for UI) */
        const CVPolarity polarity = CVPolarity::Unipolar;

        /** @brief Index of slot that modulates amount (-1 = none) */
        const std::int8_t amountModSlot = -1;

        /** @brief Human-readable name for this CV slot */
        const char* const name = "";
    };

    /**
     * @brief Runtime state for a CV slot.
     *
     * Contains the attenuverter amount and meta-modulation depth
     * that can be adjusted dynamically at runtime.
     */
    struct CVSlotState
    {
        /** @brief Attenuverter amount (-1 to 1, where 1 = full positive) */
        float amount = 1.0f;

        /** @brief Meta-modulation depth (scales the modulating CV's effect on amount) */
        float amountModDepth = 0.0f;
    };

    /**
     * @brief CV-to-parameter modulation system with fixed-size arrays.
     *
     * ModulationSystem maps CV inputs to parameters within a processor,
     * providing sample-accurate modulation without dynamic allocation.
     *
     * ## Key Features
     * - Static configuration (immutable after construction)
     * - Runtime-adjustable state (amount, modulation depth)
     * - Per-slot attenuverter (amount) control
     * - Meta-modulation support (CV modulating another slot's amount)
     * - Direct buffer access for CV channels
     *
     * ## Buffer Layout
     * CV channels follow audio channels in the BufferView:
     * ```
     * [0, audioChannels)              → Audio I/O
     * [audioChannels, totalChannels)  → CV I/O
     * ```
     *
     * ## Example Usage
     * @code
     * class SVFilter : public ProcessorBase<SVFilter, IOConfig<1, 1, 2, 0>> {
     * public:
     *     enum ParamId : std::size_t { kCutoff, kResonance, kNumParams };
     *     enum CVSlot : std::size_t { kCutoffCV, kResonanceCV, kNumCVInputs };
     *
     *     // Static CV configuration (immutable)
     *     static constexpr std::array<CVSlotConfig, kNumCVInputs> kCVConfig {{
     *         {.paramIndex = kCutoff, .polarity = CVPolarity::Bipolar, .name = "cutoff"},
     *         {.paramIndex = kResonance, .polarity = CVPolarity::Unipolar, .name = "resonance"}
     *     }};
     *
     *     struct Params {
     *         ParamSet<kNumParams> params;
     *         ModulationSystem<kNumCVInputs> mod{kCVConfig};
     *     };
     *
     *     static void processImpl(BufferView<float>& buffer, Params& params,
     *                             State& state, std::size_t sampleCount) noexcept {
     *         constexpr std::size_t audioChannels = IOConfig::audioChannels;
     *
     *         for (std::size_t i = 0; i < sampleCount; ++i) {
     *             float cutoff = params.mod.getModulated(
     *                 kCutoffCV, params.params, buffer, audioChannels, i);
     *             // ... use modulated cutoff value
     *         }
     *     }
     * };
     * @endcode
     *
     * @tparam NumSlots Number of CV input slots
     */
    template <std::size_t NumSlots>
    class ModulationSystem
    {
        const std::array<CVSlotConfig, NumSlots> config_ {};
        std::array<CVSlotState, NumSlots> state_ {};

    public:
        /**
         * @brief Default constructor with empty configuration.
         *
         * All slots map to parameter 0 with default settings.
         */
        constexpr ModulationSystem() = default;

        /**
         * @brief Construct with static configuration.
         *
         * @param config Array of CVSlotConfig defining the static slot mappings
         */
        constexpr explicit ModulationSystem (std::array<CVSlotConfig, NumSlots> config) noexcept
            : config_ (config)
        {
        }

        /**
         * @brief Get modulated parameter value at a specific sample index.
         *
         * Calculates: base_param + (CV[sampleIndex] * amount)
         *
         * If meta-modulation is configured for this slot, the amount
         * is first modulated by another CV:
         * effective_amount = amount + (modCV[sampleIndex] * amountModDepth)
         *
         * @tparam PS ParamSet type (must satisfy ParamSetLike concept)
         * @param slotIndex CV slot index (0 to NumSlots-1)
         * @param params ParamSet containing base parameter values
         * @param buffer BufferView containing audio and CV channels
         * @param audioChannels Number of audio channels (CV starts after this)
         * @param sampleIndex Sample index within the buffer
         * @return Modulated parameter value
         */
        template <ParamSetLike PS, typename SampleType = float>
        [[nodiscard]] constexpr float getModulated (
            std::size_t slotIndex,
            const PS& params,
            const BufferView<SampleType>& buffer,
            std::size_t audioChannels,
            std::size_t sampleIndex) const noexcept
        {
            const auto& cfg = config_[slotIndex];
            const auto& st = state_[slotIndex];

            float base = params.get (cfg.paramIndex);
            float effectiveAmount = st.amount;

            // Meta-modulation: another CV modulates this slot's amount
            if (cfg.amountModSlot >= 0)
            {
                const auto modSlotIndex = static_cast<std::size_t> (cfg.amountModSlot);
                const SampleType* modCV = buffer.getReadPointer (audioChannels + modSlotIndex);
                effectiveAmount += static_cast<float> (modCV[sampleIndex]) * st.amountModDepth;
            }

            // Primary modulation
            const SampleType* primaryCV = buffer.getReadPointer (audioChannels + slotIndex);
            return base + (static_cast<float> (primaryCV[sampleIndex]) * effectiveAmount);
        }

        /**
         * @brief Set the attenuverter amount for a CV slot.
         *
         * @param slot Slot index (0 to NumSlots-1)
         * @param amt Amount value (-1 to 1, where 1 = full positive modulation)
         */
        constexpr void setAmount (std::size_t slot, float amt) noexcept
        {
            state_[slot].amount = amt;
        }

        /**
         * @brief Get the attenuverter amount for a CV slot.
         *
         * @param slot Slot index (0 to NumSlots-1)
         * @return Current amount value
         */
        [[nodiscard]] constexpr float getAmount (std::size_t slot) const noexcept
        {
            return state_[slot].amount;
        }

        /**
         * @brief Set the meta-modulation depth for a CV slot.
         *
         * Meta-modulation allows another CV to modulate this slot's amount.
         *
         * @param slot Slot index (0 to NumSlots-1)
         * @param depth Depth value (scales the modulating CV's effect)
         */
        constexpr void setAmountModDepth (std::size_t slot, float depth) noexcept
        {
            state_[slot].amountModDepth = depth;
        }

        /**
         * @brief Get the meta-modulation depth for a CV slot.
         *
         * @param slot Slot index (0 to NumSlots-1)
         * @return Current meta-modulation depth
         */
        [[nodiscard]] constexpr float getAmountModDepth (std::size_t slot) const noexcept
        {
            return state_[slot].amountModDepth;
        }

        /**
         * @brief Access slot configuration (read-only).
         *
         * Configuration is immutable after construction.
         *
         * @param slot Slot index (0 to NumSlots-1)
         * @return Const reference to slot configuration
         */
        [[nodiscard]] constexpr const CVSlotConfig& getConfig (std::size_t slot) const noexcept
        {
            return config_[slot];
        }

        /**
         * @brief Access slot state (const).
         *
         * @param slot Slot index (0 to NumSlots-1)
         * @return Reference to slot state
         */
        [[nodiscard]] constexpr const CVSlotState& getState (std::size_t slot) const noexcept
        {
            return state_[slot];
        }

        /**
         * @brief Access slot state for modification.
         *
         * @param slot Slot index (0 to NumSlots-1)
         * @return Reference to slot state
         */
        [[nodiscard]] constexpr CVSlotState& getState (std::size_t slot) noexcept
        {
            return state_[slot];
        }

        /**
         * @brief Get the number of CV slots.
         *
         * @return Number of slots (compile-time constant)
         */
        [[nodiscard]] static constexpr std::size_t size() noexcept
        {
            return NumSlots;
        }
    };

} // namespace PlayfulTones::DspToolbox
