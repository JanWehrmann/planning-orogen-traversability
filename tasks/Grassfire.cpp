/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Grassfire.hpp"
#include <maps/tools/TraversabilityGrassfire.hpp>

using namespace traversability;

Grassfire::Grassfire(std::string const& name)
    : GrassfireBase(name)
{
}

Grassfire::Grassfire(std::string const& name, RTT::ExecutionEngine* engine)
    : GrassfireBase(name, engine)
{
}

Grassfire::~Grassfire()
{
}

void Grassfire::body2MapCallback(const base::Time &time)
{
    if(_body_center2map.get(time, body2Map))
    {
        gotBody2Map = true;
        LOG_INFO("Got body2map transformation!");
        LOG_DEBUG("body2map: Translation: %f %f %f",
                body2Map.translation().x(), body2Map.translation().y(), body2Map.translation().z());
    }
    else
    {
        LOG_WARN("Failed to retrieve body2map transformation!");
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Grassfire.hpp for more detailed
// documentation about them.

bool Grassfire::configureHook()
{
    if (! GrassfireBase::configureHook())
        return false;

    _body_center2map.registerUpdateCallback(boost::bind(&Grassfire::body2MapCallback, this, _1));
    gotBody2Map = false;

    return true;
}
bool Grassfire::startHook()
{
    if (! GrassfireBase::startHook())
        return false;
    return true;
}

void Grassfire::updateHook()
{
    GrassfireBase::updateHook();


    if (!receiveMap() || !gotBody2Map)
        return;

    maps::grid::TraversabilityGrid traversabilityGrid = maps::grid::TraversabilityGrid();
    maps::tools::TraversabilityGrassfire traversabilityGrassfire = maps::tools::TraversabilityGrassfire(_config.get());

    Eigen::Vector3d startPosition = Eigen::Vector3d(body2Map.translation().x(),
                                                    body2Map.translation().y(),
                                                    body2Map.translation().z());

    LOG_DEBUG("Grassfire startposition: %f %f %f", startPosition.x(), startPosition.y(), startPosition.z());
    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mls_in.getData(), startPosition);

    flushMap(traversabilityGrid);
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
    GrassfireBase::cleanupHook();
}
