#pragma once
#include <Core/Utils/Index.hpp>
#define RA_REQUIRE_OPTIONAL
#include <Core/Utils/StdOptional.hpp> // trigger an error if optional is not found
#undef RA_REQUIRE_OPTIONAL

#include <Engine/Renderer/Displayable/DisplayableObject.hpp>

namespace Ra {
namespace Engine {

/**
 * Sequence of Displayable objects.
 *
 * Contains an ordered set of n objects (invalid objects are set to nullptr), and delegates the
 * Displayable methods to one activated object ( see activate(Index id), currentIndex() ).
 */
class RA_ENGINE_API DisplayableSequence : public Displayable {
  public:
    DisplayableSequence( const std::string& name );

    ~DisplayableSequence() override = default;

    /// add an object to the sequence
    /// \warning takes the pointer ownership
    void add( Displayable* object );

    /// Size of the sequence, i.e. the number of objects
    size_t size() const { return _sequence.size(); }

    /// select activated object
    void activate( Core::Utils::Index id );

    /// returns index of the activated object
    inline Core::Utils::Index currentIndex() const { return _current; }

    /// returns the activated object, or null if currentIndex is invalid
    Core::Utils::optionnal<const std::unique_ptr<Displayable>&> current() const;

    /// Returns the picking rendermode of the current Displayable
    PickingRenderMode pickingRenderMode() const override;

    /// Returns the underlying AbstractGeometry of the current Displayable
    /// \throws CORE_ASSERT if current() is nullptr
    const Core::Geometry::AbstractGeometry& getGeometry() const override;
    /// \throws CORE_ASSERT if current() is nullptr
    Core::Geometry::AbstractGeometry& getGeometry() override;

    void updateGL() override;
    void render() override;

    size_t getNumFaces() const override;
    size_t getNumVertices() const override;

  private:
    Core::Utils::Index _current;
    std::vector<std::unique_ptr<Displayable>> _sequence;
};

} // namespace Engine
} // namespace Ra
