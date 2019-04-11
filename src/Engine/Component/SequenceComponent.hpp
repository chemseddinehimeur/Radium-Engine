#pragma once

#include <Engine/Component/Component.hpp>
#include <Engine/Renderer/Displayable/DisplayableSequence.hpp>

namespace Ra {
namespace Engine {
class TriangleMeshComponent;
class VolumeComponent;
/*!
 *
 */
class RA_ENGINE_API SequenceComponent : public Component {
  public:
    SequenceComponent( const std::string& name, Entity* entity );
    ~SequenceComponent() override;

    // initialization
    void initialize() override;
    void addFrameComponent( TriangleMeshComponent* tmc );
    void addFrameComponent( VolumeComponent* vlc );
    void postprocess();

    // play
    void nextFrame();
    void reset( int id = 0 );

  private:
    std::shared_ptr<Engine::DisplayableSequence> _sequence{nullptr};
    std::vector<Ra::Core::Utils::Index> _roIds;
    Ra::Core::Utils::Index _sequenceIndex{};
};

} // namespace Engine
} // namespace Ra
