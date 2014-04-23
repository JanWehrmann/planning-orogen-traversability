/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Grassfire.hpp"
#include <envire/Orocos.hpp>
#include <envire/operators/TraversabilityGrowClasses.hpp>

using namespace traversability;
using namespace envire;

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
    _body_center2mls_map.registerUpdateCallback(boost::bind(&Grassfire::body2MapCallback, this, _1));
    growthRadius = _growTerrainRadius.get();
    gotBody2Map = false;

    if (! GrassfireBase::configureHook())
        return false;
    
    return true;
}
bool Grassfire::startHook()
{
    if (! GrassfireBase::startHook())
        return false;
    
    if(!env)
        env = new envire::Environment();
    
    gotMap = false;
    
    return true;
}

void Grassfire::generateTraversability()
{
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

    
    tr_op->setStartPosition(body2Map.translation());
    tr_op->updateAll();

    if(growthRadius > 0.0)
    {
        TraversabilityGrowClasses *op = new TraversabilityGrowClasses();
        op->setRadius(growthRadius);
        env->attachItem(op);
        
        envire::TraversabilityGrid* traversabilityGr =
            new envire::TraversabilityGrid(mls_in->getCellSizeX(), mls_in->getCellSizeY(), mls_in->getScaleX(), mls_in->getScaleY(),
                    mls_in->getOffsetX(), mls_in->getOffsetY());
        
        env->attachItem(traversabilityGr, frame_node);
            
        op->setInput(traversability);
        op->setOutput(traversabilityGr);

        op->updateAll();
        
        env->detachItem(op);
        env->detachItem(traversability);
    }
    
    env->detachItem(tr_op);
    env->detachItem(mls_in);
    
    envire::OrocosEmitter emitter(env, _traversability_map);
    emitter.setTime(mapTime);
    emitter.flush();
}


void Grassfire::updateHook()
{
    GrassfireBase::updateHook();

    // Read map data. Don't do anything until we get a new map
    envire::OrocosEmitter::Ptr binary_events;
    if (_mls_map.readNewest(binary_events) == RTT::NewData) {
        RTT::log(RTT::Info) << "Received new binary event" << RTT::endlog();
        
        //delete old map
        delete env;
        env = new envire::Environment;
        
        env->applyEvents(*binary_events);
        mapTime = (*binary_events)[0].time;
        gotMap = true;
        
        std::vector<envire::MLSGrid*> mls_maps = env->getItems<envire::MLSGrid>();
        if(!mls_maps.size()) {
            RTT::log(RTT::Warning) << "Environment does not contain any MLS grids" << RTT::endlog();
            gotMap = false;
            return;
        }

        if(mls_maps.size() != 1) {
            RTT::log(RTT::Warning) << "Environment does contain to much MLS grids (should be only one)" << RTT::endlog();
            gotMap = false;
            return;
        }

        mls_in = mls_maps.front();
    }

    if(gotMap && gotBody2Map)
    {
        generateTraversability();
        gotMap = false;
    }
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
