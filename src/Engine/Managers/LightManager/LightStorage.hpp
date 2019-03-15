#ifndef RADIUMENGINE_LIGHTSTORAGE_HPP
#define RADIUMENGINE_LIGHTSTORAGE_HPP

#include "../../RaEngine.hpp"
#include "../../Renderer/Light/Light.hpp"

namespace Ra {
namespace Engine {
class RenderParameters;
}
} // namespace Ra

namespace Ra {
namespace Engine {

/**
 * Interface providing functions to access and store Lights in
 * an implementation-defined way.
 */
class RA_ENGINE_API LightStorage {
    // Radium-V2 : make light storage compatible with range for ...
  public:
    LightStorage() = default;

    virtual ~LightStorage() = default;

    /**
     * Upload Lights data to the GPU.
     */
    virtual void upload() const = 0;

    /**
     * Returns the container size.
     */
    virtual size_t size() const = 0;

    /**
     * Add a Light to the container.
     */
    virtual void add( const Light* li ) = 0;

    /**
     * Remove a Light from the container.
     */
    virtual void remove( const Light* li ) = 0;

    /**
     * Clear the container.
     */
    virtual void clear() = 0;

    /**
     * Access the container.
     */
    virtual const Light* operator[]( unsigned int n ) = 0;
};

} // namespace Engine
} // namespace Ra

#endif // RADIUMENGINE_LIGHTSTORAGE_HPP
