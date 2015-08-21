/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Common.hpp"

using namespace traversability;

Common::Common(std::string const& name, TaskCore::TaskState initial_state)
    : CommonBase(name, initial_state), nextObjectId(0)
{
}

Common::Common(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : CommonBase(name, engine, initial_state), nextObjectId(0)
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
}

void Common::addObjectsToMap(envire::TraversabilityGrid& grid)
{
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
