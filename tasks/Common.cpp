/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Common.hpp"
#include <envire/Orocos.hpp>

using namespace traversability;

Common::Common(std::string const& name, TaskCore::TaskState initial_state)
    : CommonBase(name, initial_state), nextObjectId(0), mEnv(0)
{
}

Common::Common(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : CommonBase(name, engine, initial_state), nextObjectId(0), mEnv(0)
{
}

Common::~Common()
{
}

boost::int32_t Common::addCircle(::base::Vector3d const & positionMap, double radius, double traversability)
{
    CircleDescriptor *c = new CircleDescriptor();
    c->radius = radius;
    c->position = positionMap;
    c->traversability = traversability;
    
    int id = nextObjectId;
    nextObjectId++;
    
    objects.insert(std::make_pair(id, c));
    
    //inform activity, that update hook should be called
    trigger();
    
    return id;
}

void Common::removeObject(int32_t objectId)
{
    std::map<int, ObjectDescriptor *>::iterator it = objects.find(objectId);
    if(it == objects.end())
    {
        std::cout << "Error, no object with id " << objectId << " known " << std::endl;
        return;
    }
    
    delete it->second;
    objects.erase(it);

    //inform activity, that update hook should be called
    trigger();
}

void Common::addObjectsToMap(const envire::TraversabilityGrid &original, envire::TraversabilityGrid &grid)
{
    //copy grid
    grid = original;
    
    std::cout << "addObjectsToMap begin" << std::endl;
    for(std::map<int, ObjectDescriptor *>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        size_t x, y;
        if(!grid.toGrid(it->second->position, x, y, grid.getEnvironment()->getRootNode()))
            continue;

        envire::TraversabilityClass klass(it->second->traversability);
        int klassNr = grid.getTraversabilityClasses().size() + 1;
        std::cout << "New Klass Nr is " << klassNr << std::endl;
        grid.setTraversabilityClass(klassNr, klass);
        
        CircleDescriptor *c = dynamic_cast<CircleDescriptor *>(it->second);
        if(c)
        {
            //TODO move into envire baseGrid
            int steps = c->radius / grid.getScaleX() + 1;
            for(int xi = x - steps; xi < (int)(x + steps); xi++)
            {
                for(int yi = y - steps; yi < (int)(y + steps); yi++)
                {
                    if(!grid.inGrid(xi, yi))
                        continue;
                    
                    std::cout << "In Grid " << xi << " " << yi << std::endl;
                    
                    if(Eigen::Vector2i(xi - x, yi - y).norm() < steps)
                    {
                        std::cout << "In Radius " << xi << " " << yi << std::endl;
                        
                        if(grid.getTraversability(xi, yi).getDrivability() > klass.getDrivability())
                        {
                            std::cout << "Reduced traversability" << std::endl;
                            grid.setTraversabilityAndProbability(klassNr, 1.0, xi, yi);
                        }
                           
                    }
                }
            }
        }
        
        std::cout << "Added Object at " << it->second->position.transpose() << std::endl;
    }
    std::cout << "addObjectsToMap end" << std::endl;
}

bool Common::receiveMap()
{
    std::cout << "Common::receiveMap()" << std::endl;
    bool gotNewMap = false;
    
    // Read map data. Don't do anything until we get a new map
    RTT::extras::ReadOnlyPointer<envire::BinaryEvents> binary_events;
    while (_mls_map.read(binary_events) == RTT::NewData) {
        mEnv->applyEvents(*binary_events);
        if(binary_events->size())
            lastUpdate = (*binary_events)[0].time;
        gotNewMap = true;
        RTT::log(RTT::Info) << "Received new binary event" << RTT::endlog();
    }

    std::cout << "Common::receiveMap() ..." << std::endl;

    if(!gotNewMap) {
        RTT::log(RTT::Warning) << "Update hook is triggered but no new data available" << RTT::endlog();
        return false;
    }
    
    std::vector<envire::MLSGrid*> mls_maps = mEnv->getItems<envire::MLSGrid>();
    if(!mls_maps.size()) {
        RTT::log(RTT::Warning) << "Environment does not contain any MLS grids" << RTT::endlog();
        return false;
    }
    
    if(mls_maps.size()) {
        std::stringstream ss;
        ss << "Received MLS map(s): " << std::endl;
        std::vector<envire::MLSGrid*>::iterator it = mls_maps.begin();
        for(int i=0; it != mls_maps.end(); ++it, ++i)
        {
            ss << i << ": "<< (*it)->getUniqueId() << std::endl;
        }
        RTT::log(RTT::Info) << ss.str() << RTT::endlog();
    } else {
        RTT::log(RTT::Warning) << "Environment does not contain any MLS grids" << RTT::endlog();
    }

    if(mls_maps.size() != 1) {
        RTT::log(RTT::Warning) << "Environment does contain to much MLS grids (should be only one)" << RTT::endlog();
        envire::MLSGrid* mls_in = mEnv->getItem< envire::MLSGrid >(_mls_id.get()).get();
        if(!mls_in)
            return false;
    }
    else
    {
        mls_in = mls_maps.front();
    }

    return true;
}

void Common::flushMap()
{
    std::cout << "Flushing map " << std::endl;
    
    envire::Transform transform =
        mEnv->relativeTransform(mls_in->getFrameNode(), mEnv->getRootNode());

    envire::Environment out;
    out.setEnvironmentPrefix(_env_name.get());
    envire::FrameNode* frame_node = new envire::FrameNode(transform);
    out.attachItem(frame_node);
    out.getRootNode()->addChild(frame_node);
    
    envire::TraversabilityGrid *modifiedGrid = new envire::TraversabilityGrid();
    out.attachItem(modifiedGrid, frame_node);
    
    addObjectsToMap(*originalGrid, *modifiedGrid);
    
    if (!_env_save_path.get().empty())
    {
        std::string path = _env_save_path.get();
        path += "/reduced";
        out.serialize(path);
    }
    
    envire::OrocosEmitter emitter(&out, _traversability_map);
    emitter.setTime(lastUpdate);
    emitter.flush();

    std::cout << "Flushing map DONE" << std::endl;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Common.hpp for more detailed
// documentation about them.

bool Common::configureHook()
{
    if (! CommonBase::configureHook())
        return false;
    return true;
}
bool Common::startHook()
{
    if (! CommonBase::startHook())
        return false;
    return true;
}
void Common::updateHook()
{
    CommonBase::updateHook();
}
void Common::errorHook()
{
    CommonBase::errorHook();
}
void Common::stopHook()
{
    CommonBase::stopHook();
}
void Common::cleanupHook()
{
    CommonBase::cleanupHook();
}
