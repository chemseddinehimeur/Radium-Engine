#include <Engine/RadiumEngine.hpp>

#include <thread>
#include <chrono>
#include <mutex>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <streambuf>

#include <Core/Log/Log.hpp>
#include <Core/String/StringUtils.hpp>
#include <Core/Event/EventEnums.hpp>
#include <Core/Event/KeyEvent.hpp>
#include <Core/Event/MouseEvent.hpp>
#include <Engine/Entity/FrameInfo.hpp>
#include <Engine/Entity/System.hpp>
#include <Engine/Entity/Component.hpp>
#include <Engine/Entity/Entity.hpp>
#include <Engine/Entity/EntityManager.hpp>

#include <Engine/Assets/FileData.hpp>

#include <Engine/Parser/Parser.hpp>
#include <Engine/Parser/LoadedData.hpp>

#include <Engine/SystemDisplay/SystemDisplay.hpp>

namespace Ra
{
    namespace Engine
    {

        RadiumEngine::RadiumEngine()
        {
            LOG(logINFO) << "*** Radium Engine ***";
        }

        RadiumEngine::~RadiumEngine()
        {
        }

        void RadiumEngine::initialize()
        {
            m_renderObjectManager.reset( new RenderObjectManager );
            m_entityManager.reset( new EntityManager );

            for (std::pair<const std::string, std::shared_ptr<Ra::Engine::System>> systemPair : m_systems)
                systemPair.second->initialize();
        }

        void RadiumEngine::cleanup()
        {
            m_entityManager.reset();
            m_renderObjectManager.reset();

            for ( auto& system : m_systems )
            {
                system.second.reset();
            }
        }

        void RadiumEngine::endFrameSync()
        {
            m_entityManager->swapBuffers();
            m_renderObjectManager->swapBuffers();
        }

        void RadiumEngine::getTasks( Core::TaskQueue* taskQueue,  Scalar dt )
        {
            static uint frameCounter = 0;
            FrameInfo frameInfo;
            frameInfo.m_dt = dt;
            frameInfo.m_numFrame = frameCounter++;
            for ( auto& syst : m_systems )
            {
                syst.second->generateTasks( taskQueue, frameInfo );
            }
        }

        void RadiumEngine::registerSystem( const std::string& name, System* system )
        {
            CORE_ASSERT( m_systems.find( name ) == m_systems.end(),
                         "Same system added multiple times." );

            m_systems[name] = std::shared_ptr<System> ( system );
            LOG(logINFO) << "Loaded : " << name;
        }

        System* RadiumEngine::getSystem( const std::string& system ) const
        {
            System* sys = nullptr;
            auto it = m_systems.find( system );

            if ( it != m_systems.end() )
            {
                sys = it->second.get();
            }

            return sys;
        }

        bool RadiumEngine::loadFile( const std::string& filename )
        {
            Asset::FileData fileData( filename, true );

            std::string entityName = Core::StringUtils::getBaseName( filename, false );

            Entity* entity = m_entityManager->getOrCreateEntity( entityName );

            for( auto& system : m_systems ) {
                system.second->handleAssetLoading( entity, &fileData );
            }


            return true;
        }

        bool RadiumEngine::handleKeyEvent( const Core::KeyEvent& event )
        {
            for ( const auto& system : m_systems )
            {
                if ( system.second->handleKeyEvent( event ) )
                {
                    return true;
                }
            }

            return false;
        }

        bool RadiumEngine::handleMouseEvent( const Core::MouseEvent& event )
        {
            for ( const auto& system : m_systems )
            {
                if ( system.second->handleMouseEvent( event ) )
                {
                    return true;
                }
            }

            return false;
        }

        RenderObjectManager* RadiumEngine::getRenderObjectManager() const
        {
            return m_renderObjectManager.get();
        }

        EntityManager* RadiumEngine::getEntityManager() const
        {
            return m_entityManager.get();
        }

        RA_SINGLETON_IMPLEMENTATION(RadiumEngine)
    } // Namespace engine
} // namespace Ra
