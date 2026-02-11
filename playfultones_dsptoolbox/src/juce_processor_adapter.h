/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "juce_buffer_view.h"
#include "juce_parameter_helpers.h"

#include "../../include/dsp_toolbox/processors/core/iprocessor.hpp"

#include <atomic>
#include <type_traits>
#include <variant>
#include <vector>

namespace PlayfulTones::DspToolbox::Juce
{

    /**
 * @brief Ready-to-use JUCE AudioProcessor wrapping a DSP Toolbox effect processor.
 *
 * JuceProcessorAdapter inherits from juce::AudioProcessor and provides:
 * - Automatic APVTS creation from ParamDescriptor metadata
 * - Thread-safe parameter synchronization (APVTS -> processor)
 * - State persistence (get/setStateInformation)
 * - Zero-copy buffer processing
 * - IOConfig-based bus layout support (mono, stereo, mono-to-stereo)
 *
 * ## Template Parameters
 *
 * @tparam ProcessorTemplate A processor template taking ConstexprSpec (e.g., Biquad)
 * @tparam SpecSetT The spec set to use (default: DesktopSpecsWithFallback)
 *
 * ## Direct Usage (Zero Boilerplate)
 *
 * @code
 * // PluginProcessor.cpp
 * using BiquadProcessor = JuceProcessorAdapter<Biquad>;
 *
 * juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
 * {
 *     return new BiquadProcessor("My Biquad Filter");
 * }
 * @endcode
 *
 * ## Stereo Processing
 *
 * For stereo processing with mono processors, use StereoExpander at the graph level:
 * @code
 * // Create stereo graph with StereoExpander nodes
 * StereoExpander<MultibandEQ<8>::template Processor, StereoEffectConfig, Spec>{}
 * @endcode
 *
 * ## Bus Layout Support
 *
 * Bus layouts are statically derived from the processor's IOConfig:
 * - MonoEffectConfig (1→1): Supports mono only
 * - StereoEffectConfig (2→2): Supports stereo only
 * - MonoToStereoEffectConfig (1→2): Supports mono-in/stereo-out and stereo
 */
    template <template <ConstexprSpec> typename ProcessorTemplate,
        typename SpecSetT = DesktopSpecsWithFallback>
    class JuceProcessorAdapter : public juce::AudioProcessor
    {
    public:
        /// Reference processor type for static IOConfig extraction
        using RefProcessor = ProcessorTemplate<DefaultSpec>;

        /// The processor's IOConfig type
        using IOConfigType = typename RefProcessor::IOConfig;

        /// True if processor is mono (1 in, 1 out)
        static constexpr bool IsMono = (IOConfigType::inAudio == 1 && IOConfigType::outAudio == 1);

        /// True if processor is stereo (2 in, 2 out)
        static constexpr bool IsStereo = (IOConfigType::inAudio == 2 && IOConfigType::outAudio == 2);

        /// True if processor is mono-to-stereo (1 in, 2 out)
        static constexpr bool IsMonoToStereo = (IOConfigType::inAudio == 1 && IOConfigType::outAudio == 2);

        /// ProcessorWrapper handles runtime spec selection
        using ProcessorStorage = ::PlayfulTones::DspToolbox::ProcessorWrapper<ProcessorTemplate, SpecSetT>;

        /// State type from the processor
        using StateType = typename ProcessorStorage::StateType;

        /// True if the underlying processor has a ParamSet in its State
        static constexpr bool HasParameters = HasParams<StateType>;

    private:
        juce::String name_;
        ProcessorStorage processor_;

        using APVTSStorage = std::conditional_t<HasParameters,
            juce::AudioProcessorValueTreeState,
            std::monostate>;
        using AtomicPtrArray = std::conditional_t<HasParameters,
            std::vector<std::atomic<float>*>,
            std::monostate>;

        [[no_unique_address]] APVTSStorage apvts_;
        [[no_unique_address]] AtomicPtrArray atomicPtrs_;

    public:
        /**
     * @brief Construct the adapter with the given processor name.
     *
     * @param name The name returned by getName() and shown in the DAW
     *
     * If the processor has parameters, APVTS is created and atomic pointers cached.
     */
        explicit JuceProcessorAdapter (const juce::String& name)
            : AudioProcessor (makeBusesProperties()),
              name_ (name),
              apvts_ (initializeAPVTS())
        {
            if constexpr (HasParameters)
            {
                cacheAtomicPointers();
            }
        }

        void prepareToPlay (double sampleRate, int samplesPerBlock) override
        {
            ProcessSpec spec {
                .sampleRate = SampleRate { sampleRate },
                .maxBlockSize = Samples<std::uint32_t> { static_cast<std::uint32_t> (samplesPerBlock) },
                .numChannels = static_cast<std::uint32_t> (getTotalNumOutputChannels()),
                .allowDynamicAllocation = true,
                .preferSIMD = true
            };
            processor_.prepare (spec);
        }

        void processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& /*midi*/) override
        {
            if constexpr (HasParameters)
            {
                syncParametersFromAPVTS();
            }

            auto view = makeBufferView (buffer);
            processor_.process (view);
        }

        void releaseResources() override
        {
            processor_.reset();
        }

        void getStateInformation (juce::MemoryBlock& destData) override
        {
            if constexpr (HasParameters)
            {
                auto state = apvts_.copyState();
                std::unique_ptr<juce::XmlElement> xml (state.createXml());
                copyXmlToBinary (*xml, destData);
            }
        }

        void setStateInformation (const void* data, int sizeInBytes) override
        {
            if constexpr (HasParameters)
            {
                std::unique_ptr<juce::XmlElement> xml (getXmlFromBinary (data, sizeInBytes));
                if (xml != nullptr && xml->hasTagName (apvts_.state.getType()))
                {
                    apvts_.replaceState (juce::ValueTree::fromXml (*xml));
                }
            }
        }

        const juce::String getName() const override { return name_; }
        bool acceptsMidi() const override { return false; }
        bool producesMidi() const override { return false; }
        bool isMidiEffect() const override { return false; }
        double getTailLengthSeconds() const override { return 0.0; }
        int getNumPrograms() override { return 1; }
        int getCurrentProgram() override { return 0; }
        void setCurrentProgram (int /*index*/) override {}
        const juce::String getProgramName (int /*index*/) override { return {}; }
        void changeProgramName (int /*index*/, const juce::String& /*newName*/) override {}
        bool hasEditor() const override { return true; }
        juce::AudioProcessorEditor* createEditor() override { return new juce::GenericAudioProcessorEditor (*this); }

        /**
     * @brief Get APVTS for editor binding and state access.
     *
     * Only available when processor has parameters.
     */
        [[nodiscard]] juce::AudioProcessorValueTreeState& getAPVTS() noexcept
            requires HasParameters
        {
            return apvts_;
        }

        /**
     * @brief Get APVTS (const).
     */
        [[nodiscard]] const juce::AudioProcessorValueTreeState& getAPVTS() const noexcept
            requires HasParameters
        {
            return apvts_;
        }

        /**
     * @brief Access the underlying ProcessorWrapper.
     *
     * Use visit() pattern for type-safe processor access.
     */
        [[nodiscard]] ProcessorStorage& getProcessor() noexcept
        {
            return processor_;
        }

        /**
     * @brief Access the underlying ProcessorWrapper (const).
     */
        [[nodiscard]] const ProcessorStorage& getProcessor() const noexcept
        {
            return processor_;
        }

        /**
     * @brief Check if processor is supported for current configuration.
     *
     * With DesktopSpecsWithFallback, this always returns true.
     */
        [[nodiscard]] bool isSupported() const noexcept
        {
            return processor_.isSupported();
        }

        /**
     * @brief Get processing latency in samples.
     */
        [[nodiscard]] std::size_t getLatencySamples() const noexcept
        {
            return processor_.getLatencySamples();
        }

    private:
        /**
     * @brief Get parameter descriptors from the processor's State type.
     */
        static constexpr auto getDescriptors()
            requires HasParameters
        {
            using ParamsType = std::remove_cvref_t<decltype (std::declval<StateType>().params)>;
            return ParamsType::descriptors;
        }

        /**
     * @brief Initialize APVTS storage.
     *
     * For processors with parameters, creates APVTS with parameter layout.
     * For parameterless processors, returns monostate.
     */
        APVTSStorage initializeAPVTS()
        {
            if constexpr (HasParameters)
            {
                return juce::AudioProcessorValueTreeState (
                    *this,
                    nullptr,
                    juce::Identifier ("Parameters"),
                    createParameterLayout<getDescriptors()>());
            }
            else
            {
                return std::monostate {};
            }
        }

        /**
     * @brief Cache raw atomic pointers for efficient parameter sync.
     *
     * Called once during construction. Stores pointers to APVTS parameter
     * atomics for O(1) access during processBlock().
     */
        void cacheAtomicPointers()
            requires HasParameters
        {
            constexpr auto descriptors = getDescriptors();
            atomicPtrs_.reserve (descriptors.size());

            for (std::size_t i = 0; i < descriptors.size(); ++i)
            {
                auto* rawParam = apvts_.getRawParameterValue (std::string (descriptors[i].id.c_str()));
                atomicPtrs_.push_back (rawParam);
            }
        }

        /**
     * @brief Sync parameters from APVTS to processor's ParamSet.
     *
     * Called at the start of each processBlock() on the audio thread.
     * Uses atomic load -> setTarget() for thread-safe, smoothed updates.
     */
        void syncParametersFromAPVTS()
            requires HasParameters
        {
            processor_.visit ([this] (auto& proc) {
                using ProcType = std::decay_t<decltype (proc)>;

                if constexpr (!std::is_same_v<ProcType, std::monostate>)
                {
                    if constexpr (HasParams<typename ProcType::State>)
                    {
                        auto& params = proc.getState().params;

                        for (std::size_t i = 0; i < atomicPtrs_.size(); ++i)
                        {
                            params.setTarget (i, atomicPtrs_[i]->load (std::memory_order_relaxed));
                        }
                    }
                }
            });
        }

        /**
     * @brief Check if a given bus layout is supported.
     *
     * Supported layouts are derived from the processor's IOConfig:
     * - Mono (1→1): mono only
     * - Stereo (2→2): stereo only
     * - MonoToStereo (1→2): mono→stereo or stereo→stereo
     */
        bool isBusesLayoutSupported (const BusesLayout& layouts) const override
        {
            const auto& mainIn = layouts.getMainInputChannelSet();
            const auto& mainOut = layouts.getMainOutputChannelSet();

            if constexpr (IsMono)
            {
                return mainIn == juce::AudioChannelSet::mono() && mainOut == juce::AudioChannelSet::mono();
            }
            else if constexpr (IsStereo)
            {
                return mainIn == juce::AudioChannelSet::stereo() && mainOut == juce::AudioChannelSet::stereo();
            }
            else if constexpr (IsMonoToStereo)
            {
                if (mainIn == juce::AudioChannelSet::mono() && mainOut == juce::AudioChannelSet::stereo())
                    return true;

                if (mainIn == juce::AudioChannelSet::stereo() && mainOut == juce::AudioChannelSet::stereo())
                    return true;

                return false;
            }
            else
            {
                return false;
            }
        }

        /**
     * @brief Create default bus configuration based on IOConfig.
     */
        static BusesProperties makeBusesProperties()
        {
            if constexpr (IsMono)
            {
                return BusesProperties()
                    .withInput ("Input", juce::AudioChannelSet::mono(), true)
                    .withOutput ("Output", juce::AudioChannelSet::mono(), true);
            }
            else if constexpr (IsStereo)
            {
                return BusesProperties()
                    .withInput ("Input", juce::AudioChannelSet::stereo(), true)
                    .withOutput ("Output", juce::AudioChannelSet::stereo(), true);
            }
            else if constexpr (IsMonoToStereo)
            {
                return BusesProperties()
                    .withInput ("Input", juce::AudioChannelSet::mono(), true)
                    .withOutput ("Output", juce::AudioChannelSet::stereo(), true);
            }
            else
            {
                // Fallback - should not reach here with valid IOConfig
                return BusesProperties()
                    .withInput ("Input", juce::AudioChannelSet::stereo(), true)
                    .withOutput ("Output", juce::AudioChannelSet::stereo(), true);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Juce
