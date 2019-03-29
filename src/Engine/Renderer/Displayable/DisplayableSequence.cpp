#include <Engine/Renderer/Displayable/DisplayableSequence.hpp>

namespace Ra {
namespace Engine {

void DisplayableSequence::add( Displayable* object ) {
    _sequence.emplace_back( object );
}

void DisplayableSequence::activate( Core::Utils::Index id ) {
    if ( id.isValid() && id < _sequence.size() )
    {
        _current = id;
    } else
    { _current.setInvalid(); }
}

Core::Utils::optionnal<const std::unique_ptr<Displayable>&> DisplayableSequence::current() const {
    if ( _current.isValid() )
        return _sequence[_current];
    return {};
}

Displayable::PickingRenderMode DisplayableSequence::pickingRenderMode() const {
    if ( _current.isValid() )
        return _sequence[_current]->pickingRenderMode();
    return NO_PICKING;
}

const Core::Geometry::AbstractGeometry& DisplayableSequence::getGeometry() const {
    const auto& c = current();
    CORE_ASSERT( c != nullptr, "[Displayable Sequence] Invalid object" );
    return c->getGeometry();
}

Core::Geometry::AbstractGeometry& DisplayableSequence::getGeometry() {
    const auto& c = current();
    CORE_ASSERT( c != nullptr, "[Displayable Sequence] Invalid object" );
    return c->getGeometry();
}

void DisplayableSequence::updateGL() {
    if ( _current.isValid() )
        _sequence[_current]->updateGL();
}

void DisplayableSequence::render() {
    if ( _current.isValid() )
        _sequence[_current]->render();
}

size_t DisplayableSequence::getNumFaces() const {
    if ( _current.isValid() )
        return _sequence[_current]->getNumFaces();
    return 0;
}

size_t DisplayableSequence::getNumVertices() const {
    if ( _current.isValid() )
        return _sequence[_current]->getNumVertices();
    return 0;
}

} // namespace Engine
} // namespace Ra
