#include "MapGenerator.hpp"
#include <Eigen/LU>
#include <iostream>
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/MLSPatch.hpp>
#include <numeric/PlaneFitting.hpp>
#include <envire/operators/MLSProjection.hpp>

using namespace Eigen;
using namespace envire;


MapGenerator::MapGenerator(double size, double resolution)
    : scaleFactor( 1.0 )
{
    lastHeight = 0.0;
    heightToGround = 0.0;
    size_t gridCells = ceil(size / resolution);
    mlsGrid = new MLSGrid(gridCells, gridCells, resolution, resolution, -size/2.0 , -size/2.0);
    std::cout << "Gridcells is " << gridCells << " size is " << size << " res is " << resolution << " map owon size " << mlsGrid->getSizeX() << std::endl;
    mlsGrid->getConfig().updateModel = MLSConfiguration::SLOPE;
    mlsGridPos = new FrameNode();
    projOp = new MLSProjection();
    projOp->useUncertainty( false );
    
    environment.attachItem(mlsGrid);
    environment.attachItem(mlsGridPos);
    environment.attachItem(projOp);
    environment.getRootNode()->addChild(mlsGridPos);
    
    environment.setFrameNode(mlsGrid, mlsGridPos);
    projOp->setOutput(mlsGrid);    
}

MapGenerator::~MapGenerator()
{
    environment.detachFrameNode(mlsGrid, mlsGridPos);
    environment.detachItem(mlsGrid);
    environment.detachItem(mlsGridPos);
    environment.detachItem(projOp);
}
 
void MapGenerator::setHistoryScaling( float factor )
{
    scaleFactor = factor;
}

void MapGenerator::setHeightToGround(double value)
{
    heightToGround = value;
}

double MapGenerator::getHeightToGround() const
{
    return heightToGround;
}


void MapGenerator::setBoundarySize(double size)
{
    boundarySize = size;
}

bool MapGenerator::getZCorrection(Eigen::Affine3d& body2Odo)
{
    //correct body2Odo z measurement
    envire::GridBase::Position pInGrid;
    
    if(!mlsGrid->toGrid(body2Odo.translation(), pInGrid.x, pInGrid.y, environment.getRootNode()))
    {
        return false;
    }

    return true;
    
    //search for best height.
    //TODO find good stddev
    SurfacePatch *closestPatch = mlsGrid->get(pInGrid, lastHeight, 0.1);
    
    double curHeight;
    if(!closestPatch)
    {
        curHeight = lastHeight;
    }
    else
    {
        curHeight = closestPatch->getMaxZ();
    }
        
    Vector3d vecToGround = body2Odo.rotation() * Vector3d(0,0, heightToGround);
    
    body2Odo.translation().z() = curHeight + vecToGround.z();
    
    lastHeight = curHeight;

    return true;
}

void MapGenerator::addPointCloud(Pointcloud *cloud)
{    
    environment.attachItem(cloud);
    environment.setFrameNode(cloud, environment.getRootNode());
    
    projOp->setInput(cloud);
    projOp->updateAll();

    mlsGrid->scalePatchWeights( scaleFactor );
    
    projOp->removeInput(cloud);
    environment.detachFrameNode(cloud, environment.getRootNode());
    //this should delte the pointcloud
    environment.detachItem(cloud);
}


void MapGenerator::addPointVector(const std::vector<Eigen::Vector3d> &rangePoints_map)
{
    Pointcloud *cloud = new Pointcloud();
    cloud->vertices = rangePoints_map;
    
    addPointCloud(cloud);
}


bool MapGenerator::addLaserScan(const base::samples::LaserScan& ls, const Eigen::Affine3d& body2Map2, const Eigen::Affine3d& laser2Body)
{
    Eigen::Affine3d body2Map(body2Map2);
//     std::cout << "MapGenerator: Got laserScan" << std::endl;
//     std::cout << "body2Odo " << std::endl << body2Map2.matrix() << " laser2Body " << std::endl << laser2Body.matrix() << std::endl;

    bool mapMoved = moveMapIfRobotNearBoundary(body2Map.translation());
    
    //correct body2Odo z measurement
    Vector2i pInGrid;
    if(!getZCorrection(body2Map))
    {
        std::cout << "Odometry position not in Grid" <<std::endl;
        throw std::runtime_error("Odometry position not in Grid");
    }
    
    Pointcloud *cloud = new Pointcloud();
    const Affine3d laser2Map(body2Map * laser2Body);
    
//     std::cout << "laser2Map " << std::endl << laser2Map.matrix() << std::endl << " ray (1,0,0) " << std::endl << (laser2Map * Vector3d(1,0,0)).transpose() << std::endl;

    
    ls.convertScanToPointCloud(cloud->vertices, laser2Map);
    
    addPointCloud(cloud);
    
    return mapMoved;
}

bool MapGenerator::moveMapIfRobotNearBoundary(const Eigen::Vector3d& robotPosition_world)
{
    const double widthHalf = mlsGrid->getSizeX() / 2.0;
    const double heightHalf = mlsGrid->getSizeY() / 2.0;

    GridBase::Position posInGrid;
    if(!mlsGrid->toGrid(robotPosition_world, posInGrid.x, posInGrid.y, environment.getRootNode()))
    {
        //robot is out of grid, move grid to center to robot positon
        clearMap();
        lastHeight = robotPosition_world.z();
        
        Eigen::Affine3d newTransform(Eigen::Affine3d::Identity());
        newTransform.pretranslate(robotPosition_world);
        mlsGridPos->setTransform(newTransform);
        return true;
    }
    
    
    //compute position of grid center
    Eigen::Vector3d gridCenter_w = mlsGrid->fromGrid(mlsGrid->getCellSizeX() / 2.0, mlsGrid->getCellSizeY() / 2.0, environment.getRootNode());

    Eigen::Vector3d diffCenterRobot_w(robotPosition_world - gridCenter_w);
    
//     std::cout << "Center pos is " << gridCenter_w.transpose() << " robot pos is " << robotPosition_world.transpose() << " diff is " << diffCenterRobot_w.transpose() << std::endl;
    
    if(fabs(diffCenterRobot_w.x()) > (widthHalf - boundarySize) || fabs(diffCenterRobot_w.y()) > (heightHalf - boundarySize)) 
    {
        //robot touched boundary, move grid
        //we assume the robot keeps moving into the same direction
        diffCenterRobot_w *= 2.0 / 3.0;
               
        //TODO I don't like this, it may drift in respect to the frame node
        mlsGrid->move(-diffCenterRobot_w.x() / mlsGrid->getScaleX(), -diffCenterRobot_w.y() / mlsGrid->getScaleY() );

        Eigen::Affine3d newTransform(Eigen::Affine3d::Identity());
        Vector3d newGridPos = diffCenterRobot_w + mlsGridPos->getTransform().translation();
        //we need to preserve Z as teh move operation does not change Z values
        newGridPos.z() = gridCenter_w.z();
        newTransform.pretranslate(newGridPos);
        mlsGridPos->setTransform(newTransform);

        std::cout << "Robot pos "  << robotPosition_world.transpose() <<  " In Map moving from " << gridCenter_w.transpose() << " to " << newTransform.translation().transpose() << std::endl;

        return true;
    }
    return false;
}

void MapGenerator::addKnowMap(envire::MLSGrid const *mls, const Affine3d &mls2LaserGrid)
{
    mlsGrid->merge(*mls, mls2LaserGrid, SurfacePatch());
}

void MapGenerator::computeNewMap()
{
//     //interpolate grid
//     smoothElevationGrid(laserGrid, interpolatedGrid);
//     
//     computeSmoothElevelationGrid(interpolatedGrid, smoothedGrid);
//     
//     updateTraversabilityGrid(interpolatedGrid, traversabilityGrid);    
}

void MapGenerator::clearMap()
{
    lastHeight = 0.0;
    mlsGridPos->setTransform(Eigen::Affine3d::Identity());
    mlsGrid->clear();
}

void MapGenerator::clearEnvForSending()
{
    projOpStore = environment.detachItem(projOp);
}

void MapGenerator::restoreEnvAfterSending()
{
    environment.attachItem(projOp);
    projOp->setOutput(mlsGrid);
}



// ConsistencyStats MapGenerator::checkMapConsistencyInArea(const base::Pose& pose, double width, double height)
// {
//     double heading = pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
//     AngleAxisd rot = AngleAxisd(heading, Vector3d::UnitZ());
//     
//     ConsistencyStats stats;
//     
//     stats.cellCnt = 0;
//     stats.noMeasurementCnt = 0;
//     stats.measurementCnt = 0;
//     stats.averageCertainty = 0.0;
//     
//     for(double x = -width / 2.0; x <= (width / 2.0); x += 0.03)
//     {
//         for(double y = -height / 2.0; y <= (height / 2.0); y += 0.03)
//         {
//             Vector2i p_g;
//             Vector3d p_w = pose.position + rot * Vector3d(x, y, 0);
//             
//             if(laserGrid.getGridPoint(p_w, p_g))
//             {
//                 vfh_star::ElevationEntry &entry(laserGrid.getEntry(p_g));
//                 stats.cellCnt++;
//                 
//                 if(entry.getMeasurementCount())
//                 {
//                     stats.measurementCnt++;
//                     //FIXME hard coded
//                     stats.averageCertainty += entry.getMeasurementCount() / 50.0;
//                 }
//                 else 
//                 {
//                     stats.noMeasurementCnt++;
//                 }
//             }
//             else 
//             {
//                 std::cout << "Error point not in grid " << std::endl;
//             }
//         }
//     }
//     
//     stats.averageCertainty /= stats.measurementCnt;
//     
// //     std::cout << "CellCnt " << stats.cellCnt << " Cells with Measurement " << stats.measurementCnt << " Cells without Measurement " << stats.noMeasurementCnt << " average certainty " << stats.averageCertainty << std::endl;    
//     return stats;
// }
// 
