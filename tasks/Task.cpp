/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <orocos/envire/Orocos.hpp>


using namespace local_mapper;
using namespace Eigen;


Task::RangeDataInput::RangeDataInput(transformer::Transformation &rangeData2Body, Task *task) : rangeData2Body(rangeData2Body), lastRangeData2Body(Eigen::Affine3d::Identity()), task(task)
{
}

void Task::RangeDataInput::addLaserScan(const base::Time& ts, const base::samples::LaserScan& scan_reading)
{
    task->lastScanTime = ts;
    
    Eigen::Affine3d rangeData2BodyCenter;
    if(!rangeData2Body.get(ts, rangeData2BodyCenter, true)) {
        RTT::log(RTT::Info) << "Interpolated transformation laser2body_center not available" << RTT::endlog();
        return;
    }

    Eigen::Affine3d bodyCenter2Odo;
    if(!task->_body_center2odometry.get(ts, bodyCenter2Odo, true)) {
        RTT::log(RTT::Info) << "Interpolated transformation body_center2odometry not available" << RTT::endlog();
        return;
    }
    task->curBodyCenter2Odo = bodyCenter2Odo;

    if(task->mapGenerator->moveMapIfRobotNearBoundary(bodyCenter2Odo.translation())) {
        RTT::log(RTT::Info) << "RangeDataInput:: Local map has been moved, robot has reached the boundary" << RTT::endlog();    
        task->gotNewMap = true;
    }
    
    task->gotNewMap |= task->mapGenerator->addLaserScan(scan_reading, bodyCenter2Odo, rangeData2BodyCenter);
    
    //computes the angle between two scan lines
    //this is basically a sweep detection
    double laserAngleChange = acos((rangeData2BodyCenter.linear() * Vector3d::UnitX()).dot(lastRangeData2Body.linear() * Vector3d::UnitX()));

    if(laserAngleChange < task->scanAngleTrigger)
        return;
    
    lastRangeData2Body = rangeData2BodyCenter;
    task->gotNewMap = true;

}


Task::Task(std::string const& name)
    : TaskBase(name), frontInput(_laser2body_center, this), backInput(_laser_back2body_center, this), mapGenerator(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), frontInput(_laser2body_center, this), backInput(_laser_back2body_center, this), mapGenerator(NULL)
{
}

Task::~Task()
{
    
}

bool Task::markRectInMap(double width, double height, ::base::samples::RigidBodyState const & pose, double offset)
{
    if(!mapGenerator)
        return false;
    
    //generate fake point cloud
    envire::Pointcloud *pc = new envire::Pointcloud();
    double delta = _map_resolution.get() / 4;
    Affine3d pointToMap(pose.getTransform());
    
    for(double y = -width/2.0; y < width/2.0; y += delta)
    {
        for(double x = -height/2.0; x < height/2.0; x += delta)
        {
            Vector3d p = pointToMap * Vector3d(x, y, offset);
            pc->vertices.push_back(p);
        }
    }
    
    mapGenerator->addPointCloud(pc);
    gotNewMap = true;

    return true;
}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{
    frontInput.addLaserScan(ts, scan_samples_sample);
//     std::cout << "LaserScan front" << std::endl;
}

void Task::scan_samples_backTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_back_sample)
{
//     backInput.addLaserScan(ts, scan_samples_back_sample);
    std::cout << "LaserScan back" << std::endl;
}

void Task::velodyne_scansTransformerCallback(const base::Time &ts, const ::velodyne_lidar::MultilevelLaserScan &velodyne_scans_unfiltered)
{
//     std::cout << "Velodyne" << std::endl;
    lastScanTime = ts;
    
    Eigen::Affine3d velodyne2bodyCenter;
    if(!_velodyne2body_center.get(ts, velodyne2bodyCenter, true)) {
        RTT::log(RTT::Info) << "Interpolated transformation laser2body_center not available" << RTT::endlog();
        return;
    }

    Eigen::Affine3d bodyCenter2Odo;
    if(!_body_center2odometry.get(ts, bodyCenter2Odo, true)) {
        RTT::log(RTT::Info) << "Interpolated transformation body_center2odometry not available" << RTT::endlog();
        return;
    }
    curBodyCenter2Odo = bodyCenter2Odo;

    if(mapGenerator->moveMapIfRobotNearBoundary(bodyCenter2Odo.translation())) {
        RTT::log(RTT::Info) << "velodyne_scansTransformerCallback: Local map has been moved, robot has reached the boundary" << RTT::endlog();    
    }

    
    if(!mapGenerator->getZCorrection(bodyCenter2Odo))
    {
        std::cout << "Warning, could not get Z Correction " << std::endl;
    }
    
    velodyne_lidar::MultilevelLaserScan velodyne_scans_sample;
    velodyne_lidar::ConvertHelper::filterOutliers(velodyne_scans_unfiltered, velodyne_scans_sample, _velodyne_maximum_angle_to_neighbor.get(), _velodyne_minimum_valid_neighbors.get());

//     std::cout << "MapGenerator: Got VelodyneScan" << std::endl;
//     std::cout << "bodyCenter2Odo " << std::endl << bodyCenter2Odo.matrix() << std::endl << " velodyne2bodyCenter " << std::endl << velodyne2bodyCenter.matrix() << std::endl;

    const Affine3d velodyne2Odometry(bodyCenter2Odo * velodyne2bodyCenter);
//     std::cout << "velodyne2Odometry " << std::endl << velodyne2Odometry.matrix() << std::endl << " ray (1,0,0) " << std::endl << (velodyne2Odometry * Vector3d(1,0,0)).transpose() << std::endl;
    
    std::vector<Vector3d> points;
    
    std::vector<std::vector<Vector3d> > scanLines;
    scanLines.reserve(32);
    
    std::vector<velodyne_lidar::MultilevelLaserScan::VerticalMultilevelScan >::const_iterator vertIt = velodyne_scans_sample.horizontal_scans.begin();
    std::vector<velodyne_lidar::MultilevelLaserScan::SingleScan>::const_iterator horIt; 

    unsigned int horCnt = 0;
    unsigned int verCnt = 0;
    unsigned invalid = 0;
    for(; vertIt != velodyne_scans_sample.horizontal_scans.end(); vertIt++)
    {
        
        AngleAxisd horizontalRotation(vertIt->horizontal_angle.getRad(), Vector3d::UnitZ());
        horCnt = 0;
        for(horIt = vertIt->vertical_scans.begin(); horIt != vertIt->vertical_scans.end(); horIt++)
        {
            //ignore invalid readings
            //ignore everything over 12 meters
            if(!velodyne_scans_sample.isRangeValid(horIt->range) || horIt->range > 12000)
            {
                invalid++;
                horCnt++;
                continue;
            }

            base::Angle verticalAngle(vertIt->vertical_start_angle + base::Angle::fromRad(vertIt->vertical_angular_resolution * horCnt));

//             if(verticalAngle > base::Angle::fromDeg(0))
//                 break;

            if(scanLines.size() <= horCnt)
            {
                scanLines.resize(horCnt + 1);
            }

            AngleAxisd verticalRotation(verticalAngle.getRad(), Vector3d::UnitY());
            
            Vector3d point = horizontalRotation * verticalRotation * Vector3d(horIt->range/1000.0, 0, 0);
            
            point = velodyne2Odometry * point;
            
            scanLines[horCnt].push_back(point);
            horCnt++;
        }
        verCnt ++;
    }

//     std::cout << "Got " << horCnt << " horizontal scans with " << verCnt << " points " << " invalide " << invalid << std::endl;

    
    for(std::vector<std::vector<Vector3d> >::const_iterator it = scanLines.begin();
        it != scanLines.end(); it++)
    {
        mapGenerator->addPointVector(*it);
    }
//     gotNewMap = true;
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    //maximum distance of the horizon in the map
    double mapSize = _map_size.get();
    double mapResolution = _map_resolution.get();
    double boundarySize = _boundary_size.get();
    
    RTT::log(RTT::Info) << "Map size has been set to " << mapSize << " boundary size is " << boundarySize << RTT::endlog();
    
    robotAngleTrigger = _robot_angle_change_trigger.get();
    robotDistanceTrigger = _robot_translation_trigger.get();
    scanAngleTrigger = _scan_angle_change_trigger.get();

    if (_map_update_model.get() != envire::MLSConfiguration::SLOPE 
        && _map_update_model.get() != envire::MLSConfiguration::SUM 
        && _map_update_model.get() != envire::MLSConfiguration::KALMAN )
    {
        RTT::log(RTT::Error) << "Wrong update model of the map. It should be SLOPE, SUM or KALMAN." << RTT::endlog();
        return false;
    }    
    
    if(mapGenerator)
        delete mapGenerator;
    
    mapGenerator = new MapGenerator(mapSize, mapResolution, _map_update_model.get());
    mapGenerator->setBoundarySize(boundarySize);
    mapGenerator->setHistoryScaling(_history_scale_factor.get());
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    
    gotNewMap = false;
    
    mapGenerator->setHeightToGround(_height_to_ground.get());
    mapGenerator->clearMap();
    
    curBodyCenter2Odo = Affine3d::Identity();
    lastBodyCenter2Odo = Affine3d::Identity();

    return true;
}


bool Task::setNewMap(const RTT::extras::ReadOnlyPointer< std::vector< envire::BinaryEvent > >& map, const base::samples::RigidBodyState& newMap2Odometry)
{
    envire::Environment env;
    env.applyEvents(*(map.get()));
    
    std::vector<envire::MLSGrid*> mls_maps = env.getItems<envire::MLSGrid>();
    if(!mls_maps.size()) {
        RTT::log(RTT::Warning) << "Environment does not contain any MLS grids" << RTT::endlog();
        return false;
    }

    if(mls_maps.size() != 1) {
        RTT::log(RTT::Warning) << "Environment does contain to much MLS grids (should be only one)" << RTT::endlog();
        return false;
    }

    mapGenerator->clearMap();
    mapGenerator->addKnowMap(mls_maps.front(), newMap2Odometry.getTransform());
    gotNewMap = true;
    return true;
}

bool Task::dropMap()
{
    mapGenerator->clearMap();
    gotNewMap = true;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    
    
    
    double distanceBodyToLastBody = (curBodyCenter2Odo.translation() - lastBodyCenter2Odo.translation()).norm();

    //compute the angle the robot turned on any axis
    double robotAngleChange(acos((curBodyCenter2Odo.linear() * Vector3d::UnitX()).dot(lastBodyCenter2Odo.linear() * Vector3d::UnitX())));
    
    if(gotNewMap || distanceBodyToLastBody > robotDistanceTrigger || robotAngleChange > robotAngleTrigger)
    {
        mapGenerator->clearEnvForSending();
        envire::OrocosEmitter emitter(&(mapGenerator->getEnvironment()), _map);
        emitter.setTime(lastScanTime);
        emitter.flush();
        mapGenerator->restoreEnvAfterSending();
//         std::cout << "Robot moved, flushing map" << std::endl;
        
        lastMapWriteTime = lastScanTime;
        lastBodyCenter2Odo = curBodyCenter2Odo;
    }

    gotNewMap = false;  

}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    delete mapGenerator;
    mapGenerator = NULL;
    
    TaskBase::cleanupHook();
}
