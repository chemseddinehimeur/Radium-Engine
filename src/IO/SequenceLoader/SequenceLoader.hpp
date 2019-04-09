#pragma once

#include <Core/Asset/FileLoaderInterface.hpp>
#include <IO/RaIO.hpp>

namespace Ra {
namespace IO {

/*!
 * \brief The SequenceFileLoader implements a hack to load sequences of Displayable using FileData.
 *
 * \FIXME FileData put hard constraints on the loading mechanism and prevent to easily load
 * sequences.
 * \TODO Remove FileData
 */
class RA_IO_API SequenceFileLoader : public Core::Asset::FileLoaderInterface {
  public:
    SequenceFileLoader();

    ~SequenceFileLoader() override;

    std::vector<std::string> getFileExtensions() const override;
    bool handleFileExtension( const std::string& extension ) const override;
    Core::Asset::FileData* loadFile( const std::string& filename ) const override;
    std::string name() const override;
};

} // namespace IO
} // namespace Ra
