#pragma once

#include <Engine/System/TimedSystem.hpp>

namespace Ra {
namespace Engine {
class SequenceComponent;

class RA_ENGINE_API SequenceSystem : public AbstractTimedSystem {
  public:
    /// Create a new animation system
    SequenceSystem();

    SequenceSystem( const SequenceSystem& ) = delete;
    SequenceSystem& operator=( const SequenceSystem& ) = delete;

    /// Create a task for each animation component to advance the current animation.
    void generateTasks( Ra::Core::TaskQueue* taskQueue,
                        const Ra::Engine::FrameInfo& frameInfo ) override;

    /// Load the animation from FileData
    /// Each element of the animation is loaded using a GeometrySystem
    void handleAssetLoading( Ra::Engine::Entity* entity,
                             const Ra::Core::Asset::FileData* fileData ) override;

    /// Toggle on/off playing of animations.
    void play( bool isPlaying ) override;

    /// Advance the animation next frame, then pauses.
    void step() override;

    /// Resets the skeleton to its rest pose.
    void reset() override;

    /// Saves all the state data related to the given frame into a cache file.
    /// Does nothing as all the data is stored in the Sequence (cache files are not expected to be
    /// used out of the application)
    inline void cacheFrame( const std::string& /*dir*/, uint /*frameId*/ ) const override {}

    /// Restores the state data related to the \p frameID -th frame from the cache file.
    /// \returns true if the frame has been successfully restored, false otherwise.
    bool restoreFrame( const std::string& dir, uint frameId ) override;

  private:
    /// See if animation is playing or paused
    /// \FIXME move to AbstractTimedSystem
    bool m_isPlaying{true};

    /// True if one step has been required to play
    /// \FIXME move to AbstractTimedSystem
    bool m_oneStep{false};
};

} // namespace Engine
} // namespace Ra
