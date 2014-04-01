/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Grassfire.hpp"
#include <envire/Orocos.hpp>

using namespace traversability;

Grassfire::Grassfire(std::string const& name)
    : GrassfireBase(name), env(0)
{
}

Grassfire::Grassfire(std::string const& name, RTT::ExecutionEngine* engine)
    : GrassfireBase(name, engine), env(0) 
{
}

Grassfire::~Grassfire()
{
}


void Grassfire::body2MapCallback(const base::Time &time)
{
    _body_center2mls_map.get(time, body2Map);
    gotBody2Map = true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Grassfire.hpp for more detailed
// documentation about them.

bool Grassfire::configureHook()
{
    if (! GrassfireBase::configureHook())
        return false;
    
    _body_center2mls_map.registerUpdateCallback(boost::bind(&Grassfire::body2MapCallback, this, _1));
    
    return true;
}
bool Grassfire::startHook()
{
    if (! GrassfireBase::startHook())
        return false;
    
    if(!env)
        env = new envire::Environment();
    
    return true;
}
void Grassfire::updateHook()
{
    GrassfireBase::updateHook();

    // Read map data. Don't do anything until we get a new map
    envire::OrocosEmitter::Ptr binary_events;
    if (_mls_map.read(binary_events) == RTT::NewData) {
        env->applyEvents(*binary_events);
        RTT::log(RTT::Info) << "Received new binary event" << RTT::endlog();
    } else {
        RTT::log(RTT::Warning) << "Update hook is triggered but no new data available" << RTT::endlog();
        return;  
    }

    // Tries to extract the MLS with the specified ID. If that is not possible, the first
    // MLS will be used (if it is the only contained MLS).
    // Fails if no MLS is available or the ID does not match and there are more than one MLS.
    std::vector<envire::MLSGrid*> mls_maps = env->getItems<envire::MLSGrid>();
    if(!mls_maps.size()) {
        RTT::log(RTT::Warning) << "Environment does not contain any MLS grids" << RTT::endlog();
        return;
    }

    if(mls_maps.size() != 1) {
        RTT::log(RTT::Warning) << "Environment does contain to much MLS grids (should be only one)" << RTT::endlog();
        return;
    }

    envire::MLSGrid* mls_in = mls_maps.front();

    RTT::log(RTT::Info) << "Got a new mls map with id " << mls_in->getUniqueId() << RTT::endlog();

    envire::FrameNode* frame_node = mls_in->getFrameNode();
    
    
    // And convert to traversability
    envire::TraversabilityGrid* traversability =
        new envire::TraversabilityGrid(mls_in->getCellSizeX(), mls_in->getCellSizeY(), mls_in->getScaleX(), mls_in->getScaleY(),
                mls_in->getOffsetX(), mls_in->getOffsetY());
    env->attachItem(traversability, frame_node);
    envire::TraversabilityGrassfire *tr_op = new envire::TraversabilityGrassfire();
    tr_op->setConfig(_config.get());
    env->attachItem(tr_op);
    tr_op->setInput(mls_in);
    tr_op->setOutput(traversability);

    if(gotBody2Map)
    {
        tr_op->setStartPosition(body2Map.translation());
        tr_op->updateAll();

        env->detachItem(tr_op);
        env->detachItem(mls_in);
        
        envire::OrocosEmitter emitter(env, _traversability_map);
        assert(!binary_events->empty());
        emitter.setTime((*binary_events)[0].time);
        emitter.flush();
    }

    RTT::log(RTT::Debug) << "Traversability: cleanup for next update" << RTT::endlog();
    // Finally, reinitialise the environment for the next update
    delete env;
    env = new envire::Environment;
}
void Grassfire::errorHook()
{
    GrassfireBase::errorHook();
}
void Grassfire::stopHook()
{
    GrassfireBase::stopHook();
}
void Grassfire::cleanupHook()
{
    delete env;
    env = 0;
    GrassfireBase::cleanupHook();
}
