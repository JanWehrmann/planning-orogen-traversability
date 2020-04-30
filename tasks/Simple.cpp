/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Simple.hpp"
#include <envire_core/items/SpatioTemporal.hpp>
#include <maps/tools/MLSToSlopes.hpp>
#include <maps/grid/Index.hpp>

using namespace traversability;

Simple::Simple(std::string const& name)
    : SimpleBase(name)
{
}

Simple::Simple(std::string const& name, RTT::ExecutionEngine* engine)
    : SimpleBase(name, engine)
{
}

Simple::~Simple()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Simple.hpp for more detailed
// documentation about them.

bool Simple::configureHook()
{
    if (! SimpleBase::configureHook())
        return false;

    return true;
}
bool Simple::startHook()
{
    if (! SimpleBase::startHook())
        return false;

    return true;
}
void Simple::updateHook()
{
    SimpleBase::updateHook();

    if (!receiveMap())
        return;

    RTT::log(RTT::Debug) << "Traversability: create the slope and max step grids" << RTT::endlog();

    // Create the slope and max step grids
    maps::grid::GridMapF mls_slopes = maps::grid::GridMapF();
    maps::grid::GridMapF mls_maxSteps = maps::grid::GridMapF();

    maps::tools::MLSToSlopes::computeSlopes(mls_in.getData(), mls_slopes, _window_size.get());
    maps::tools::MLSToSlopes::computeMaxSteps(mls_in.getData(), mls_maxSteps);

    RTT::log(RTT::Debug) << "Traversability: convert map to traversability map" << RTT::endlog();
    // And convert to traversability
    maps::grid::TraversabilityGrid traversabilityGrid = maps::grid::TraversabilityGrid();
    maps::tools::SimpleTraversability simpleTraversability = maps::tools::SimpleTraversability(_traversability_conf);
    simpleTraversability.calculateTraversability(traversabilityGrid, mls_slopes, mls_maxSteps);

    flushMap(traversabilityGrid);
}

void Simple::errorHook()
{
    SimpleBase::errorHook();
}
void Simple::stopHook()
{
    SimpleBase::stopHook();
}
void Simple::cleanupHook()
{
    SimpleBase::cleanupHook();
}
