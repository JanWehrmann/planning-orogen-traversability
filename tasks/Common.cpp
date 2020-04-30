/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Common.hpp"

using namespace traversability;

Common::Common(std::string const& name, TaskCore::TaskState initial_state)
    : CommonBase(name, initial_state)
{
}

Common::Common(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : CommonBase(name, engine, initial_state)
{
}

Common::~Common()
{
}

bool Common::receiveMap()
{
    bool gotNewMap = false;
    if (_mls_map.readNewest(mls_in) == RTT::NewData) {
        RTT::log(RTT::Info) << "Received new MLSMap" << RTT::endlog();
        gotNewMap = true;
    }

    if(!gotNewMap) {
        RTT::log(RTT::Warning) << "Update hook is triggered but no new data available" << RTT::endlog();
        return false;
    }

    return true;
}

void Common::flushMap(const maps::grid::TraversabilityGrid& map)
{
    envire::core::SpatioTemporal<maps::grid::TraversabilityGrid> trav_out_spatio_temporal(map);
    trav_out_spatio_temporal.data.getLocalFrame() = mls_in.data.getLocalFrame();
    trav_out_spatio_temporal.setFrameID(mls_in.getFrameID());
    trav_out_spatio_temporal.setTime(mls_in.getTime());
    trav_out_spatio_temporal.setUUID(mls_in.getUUID());

    _traversability_map.write(trav_out_spatio_temporal);
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
