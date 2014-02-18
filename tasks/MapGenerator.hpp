#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include <base/samples/LaserScan.hpp>
#include <Eigen/Geometry>
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/operators/MLSProjection.hpp>

class MapGenerator
{
public:
    MapGenerator(double size, double resolution);
    ~MapGenerator();
    
    /** Sets the distance between the body frame and the ground. It
    * is zero by default */
    void setHeightToGround(double value);
    /** Returns the distance between the body frame and the ground */
    double getHeightToGround() const;
                
    bool getZCorrection(Eigen::Affine3d& body2Map);
        
    /**
    * Adds a vector of range points to the map
    * the points are expected to be in odometry coordinates.
    * Also the need to be taken in one laser scan.
    * */
    void addPointVector(const std::vector<Eigen::Vector3d> &rangePoints_map);

    bool addLaserScan(const base::samples::LaserScan& ls, const Eigen::Affine3d& body2Map, const Eigen::Affine3d& laser2Body);

    void addPointCloud(envire::Pointcloud *cloud);
    
    /**
    * This function test if the robot is near the outer bound of
    * the map and moves the map if needed.
    **/
    bool moveMapIfRobotNearBoundary(const Eigen::Vector3d& robotPosition_world);

    /**
    * Deletes all information in the maps
    **/
    void clearMap();
        
    /**
    * Manual trigger for map generation
    * */
    void computeNewMap();
        
    /**
    * The map is moved if the robot position
    * is inside the outer boundary of the map.
    * */
    void setBoundarySize(double size);

    void setGridEntriesWindowSize(int window_size);
    void addKnowMap(envire::MLSGrid const *knowMap, const Eigen::Affine3d& knownMap2InternalMap);
        
    envire::Environment &getEnvironment()
    {
        return environment;
    }
    
private:
        
//     /**
//     * This function test if the robot is near the outer bound of
//     * the grid and if the grid needs to be moved.
//     **/
//     bool moveGridIfRobotNearBoundary(ElevationGrid& grid, const Eigen::Vector3d& robotPosition_world);
    envire::MLSGrid *mlsGrid;
    envire::FrameNode *mlsGridPos;
    envire::Environment environment;
    envire::MLSProjection *projOp;

    double boundarySize;
    double mapSize;
    double lastHeight;
    ///height from bodyFrame to ground
    double heightToGround;
};

#endif // MAPGENERATOR_H
