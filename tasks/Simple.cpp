/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Simple.hpp"

using namespace traversability;

Simple::Simple(std::string const& name)
    : SimpleBase(name), seq_number(0)
{
}

Simple::Simple(std::string const& name, RTT::ExecutionEngine* engine)
    : SimpleBase(name, engine), seq_number(0)
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
    
    mMaxExtent = _map_max_extent.get();

    return true;

    return true;
}
bool Simple::startHook()
{
    if (! SimpleBase::startHook())
        return false;
    
    originalGrid = NULL;
    
    delete mEnv;
    mEnv = new envire::Environment;
    mEnv->setEnvironmentPrefix(_env_name.get());
    
    return true;
}
void Simple::updateHook()
{
    SimpleBase::updateHook();
    
    bool gotNewMap = receiveMap();
    
    if(!gotNewMap)
    {
        if(originalGrid)
            flushMap();
        
        return;
    }
    
    envire::FrameNode* frame_node = mls_in->getFrameNode();
    envire::Transform transform =
            mEnv->relativeTransform(frame_node, mEnv->getRootNode());

    envire::EnvironmentItem::Ptr mlsPtr = mEnv->detachItem(mls_in);

    delete mEnv;
    mEnv = new envire::Environment;
    mEnv->setEnvironmentPrefix(_env_name.get());
    
    frame_node = new envire::FrameNode(transform);
    mEnv->getRootNode()->addChild(frame_node);
    
    mEnv->attachItem(dynamic_cast<envire::CartesianMap *>(mlsPtr.get()), frame_node);
    mlsPtr.reset();

    frame_node = mls_in->getFrameNode();
    
    RTT::log(RTT::Info) << "Got a new mls map with id " << mls_in->getUniqueId() << RTT::endlog();


    // get the extents from the map, and extend it with the extents provided by
    // the parameters (if any)
    Eigen::AlignedBox<double, 2> extents = mls_in->getExtents();
    Eigen::Affine3d world2grid = mEnv->getRootNode()->relativeTransform( frame_node );
    for( std::vector<base::Vector2d>::iterator it = _map_extents.value().begin(); it != _map_extents.value().end(); it++ )
    {
        Eigen::Vector3d p;
        p << *it, 0;
        extents.extend( (world2grid * p).head<2>() );
    }
    double xScale = mls_in->getScaleX(), yScale = mls_in->getScaleY();
    double xCellSize = mls_in->getCellSizeX(), yCellSize = mls_in->getCellSizeX();
    // Bounding the size of the mls_in map on the number of patches in each dimension
    size_t xSize = std::min(mMaxExtent, (size_t) (extents.sizes().x() / xScale));
    size_t ySize = std::min(mMaxExtent, (size_t) (extents.sizes().y() / yScale));
    
    // No rescaling if extents.sizes() is not set / if it is set to 0.
    if(xSize <= 0) {
      xSize = xCellSize;
    }
    if(ySize <= 0) {
      ySize = yCellSize;
    }
    double xOffset = extents.min().x(), yOffset = extents.min().y();


    RTT::log(RTT::Debug) << "Traversability: input MLS: xScale: '" << xScale << "' yScale: '" << yScale << "'" << RTT::endlog();
    RTT::log(RTT::Debug) << "Traversability: input MLS: xSize (max_extent: '" << mMaxExtent << "'): '" << xSize << "' ySize: '" << ySize << "'" << RTT::endlog();
    RTT::log(RTT::Debug) << "Traversability: input MLS: xCellSize: '" << xCellSize << "' yCellSize: '" << yCellSize << "'" << RTT::endlog();

    // see if we need to resize the input mls 
    envire::MLSGrid* mls = mls_in;
    if( xSize != mls_in->getCellSizeX() || ySize != mls_in->getCellSizeY() )
    {
        mls = new envire::MLSGrid(xSize, ySize, xScale, yScale,
                xOffset, yOffset);
        mEnv->setFrameNode( mls, mls_in->getFrameNode() );
        envire::MergeMLS* op_mls_merge = new envire::MergeMLS;
        mEnv->attachItem(op_mls_merge);
        op_mls_merge->setInput( mls_in );
        op_mls_merge->setOutput( mls );
        op_mls_merge->updateAll();
    }

    RTT::log(RTT::Debug) << "Traversability: create the slope and max step grids" << RTT::endlog();

    // Create the slope and max step grids
    envire::Grid<float>* mls_geometry =
        new envire::Grid<float>(xSize, ySize, xScale, yScale,
                xOffset, yOffset, "mls_geometry");
    mEnv->attachItem(mls_geometry, frame_node);
    envire::MLSSlope* op_mls_slope = new envire::MLSSlope;
    op_mls_slope->setRequiredMeasurementsPerPatch(_required_measurements_per_patch.get());
    mEnv->attachItem(op_mls_slope);
    op_mls_slope->setInput(mls);
    op_mls_slope->setOutput(mls_geometry);

    RTT::log(RTT::Debug) << "Traversability: convert map to traversability map" << RTT::endlog();
    // And convert to traversability
    envire::TraversabilityGrid* traversability =
        new envire::TraversabilityGrid(xSize, ySize, xScale, yScale,
                xOffset, yOffset, "map");
    mEnv->attachItem(traversability, frame_node);
    envire::SimpleTraversability* op_trav = new envire::SimpleTraversability(_traversability_conf);
    mEnv->attachItem(op_trav);
    op_trav->setSlope(mls_geometry, "mean_slope");
    op_trav->setMaxStep(mls_geometry, "corrected_max_step");
    op_trav->setOutput(traversability, envire::TraversabilityGrid::TRAVERSABILITY);

    mEnv->updateOperators();
    RTT::log(RTT::Debug) << "Traversability: use env save path for serialization" << RTT::endlog();
    if (!_env_save_path.get().empty())
    {
        std::string path = _env_save_path.get();
        path += "/complete";
        mEnv->serialize(path);
    }
    
    originalGrid = traversability;
    
    flushMap();
}

void Simple::errorHook()
{
    SimpleBase::errorHook();
}
void Simple::stopHook()
{
    SimpleBase::stopHook();
    delete mEnv;
    mEnv = 0;
}
void Simple::cleanupHook()
{
    SimpleBase::cleanupHook();
}
