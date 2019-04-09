#include <IO/SequenceLoader/SequenceLoader.hpp>

#include <Core/Asset/FileData.hpp>
#include <Core/Utils/StringUtils.hpp>

// \FIXME This inclusion is needed to access the file loaders and load the meshes in the sequence.
// This is ugly, but needed as long as we keep the filedata and the current (one file per entity)
// loading mechanism
#include <Engine/RadiumEngine.hpp>

#include <fstream>
#include <iostream>
#include <string>

const std::string seqExt( "seq" );

namespace Ra {
namespace IO {

using namespace Core::Utils; // log
using namespace Core::Asset; // Filedata

SequenceFileLoader::SequenceFileLoader() {}

SequenceFileLoader::~SequenceFileLoader() {}

std::vector<std::string> SequenceFileLoader::getFileExtensions() const {
    return std::vector<std::string>( {"*." + seqExt} );
}

bool SequenceFileLoader::handleFileExtension( const std::string& extension ) const {
    return extension.compare( seqExt ) == 0;
}

FileData* SequenceFileLoader::loadFile( const std::string& filename ) const {

    std::ifstream ss( filename );
    Engine::RadiumEngine* engine = Engine::RadiumEngine::getInstance();
    CORE_ASSERT( engine != nullptr, "The engine should be accessible here " );
    auto& loaders = engine->getFileLoaders();

    if ( loaders.empty() )
    {
        LOG( logERROR ) << "[Sequence] No fileloader found";
        return nullptr;
    }

    FileData* fileData = new FileData( filename );

    if ( !fileData->isInitialized() )
    {
        delete fileData;
        LOG( logERROR ) << "[Sequence] Filedata cannot be initialized...";
        return nullptr;
    }

    if ( fileData->isVerbose() )
    {
        LOG( logINFO ) << "[Sequence] File Loading begin...";
    }

    std::string line;
    while ( getline( ss, line ) )
    {
        for ( const auto& loader : loaders )
        {
            std::string extension = Core::Utils::getFileExt( line );
            if ( loader->handleFileExtension( extension ) )
            {
                FileData* loaded = loader->loadFile( line );
                if ( loaded != nullptr )
                {
                    if ( fileData->m_sequence == nullptr )
                        fileData->m_sequence = std::unique_ptr<FileData>( loaded );
                    else
                        fileData->m_sequence->moveIn( loaded );

                    break; // jump to next sequence element
                }
            }
        }
    }

    if ( fileData->m_sequence == nullptr )
    {
        delete fileData;
        LOG( logERROR ) << "[Sequence] No element found";
        return nullptr;
    }

    if ( fileData->isVerbose() )
    {
        LOG( logINFO ) << "[Sequence] File Loading end.";

        fileData->displayInfo();
    }

    fileData->m_processed = true;
    fileData->m_sequence->m_processed = true;

    return fileData;
}

std::string SequenceFileLoader::name() const {
    return "Sequence";
}
} // namespace IO
} // namespace Ra
