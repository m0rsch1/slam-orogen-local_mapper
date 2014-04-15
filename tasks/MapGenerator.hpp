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
     * @brief set how much old measurements are taken into account
     * A factor of 1.0 treats all values the same, anything below 1.0
     * will let the weight of old values diminish exponentially.
     */
    void setHistoryScaling( float factor );
        
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
    
    void clearEnvForSending();
    void restoreEnvAfterSending();
    
    
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

    envire::EnvironmentItem::Ptr projOpStore;
    
    double mapHeight;
    double boundarySize;
    double mapSize;
    double lastHeight;
    ///height from bodyFrame to ground
    double heightToGround;

    /// scale factor for patch history
    float scaleFactor;
};

#endif // MAPGENERATOR_H
