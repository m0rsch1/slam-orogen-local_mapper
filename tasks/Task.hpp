/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LOCAL_MAPPER_TASK_TASK_HPP
#define LOCAL_MAPPER_TASK_TASK_HPP

#include "local_mapper/TaskBase.hpp"
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <tasks/MapGenerator.hpp>

namespace local_mapper {

   /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','odometry_slam::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
        friend class TaskBase;
    protected:


        
        class RangeDataInput {
        public:
            RangeDataInput(transformer::Transformation &rangeData2Body, Task *task);
            void addLaserScan(const base::Time& ts, const base::samples::LaserScan& scan_reading); 
        private:
            transformer::Transformation &rangeData2Body;
            Eigen::Affine3d lastRangeData2Body;
            Task *task;
        };
        
        RangeDataInput frontInput;
        RangeDataInput backInput;
        
        virtual void scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample);

        virtual void scan_samples_backTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_back_sample);

        virtual void velodyne_scansTransformerCallback(const base::Time &ts, const ::velodyne_lidar::MultilevelLaserScan &velodyne_scans_sample);

        ///Last transformation from body to odometry
        Eigen::Affine3d curBodyCenter2Odo;
        Eigen::Affine3d lastBodyCenter2Odo;

        base::Time lastScanTime;
        base::Time lastMapWriteTime;
        
        ///If true, map will be written to output port
        bool gotNewMap;
        
        double scanAngleTrigger;
        double robotAngleTrigger;
        double robotDistanceTrigger;


        MapGenerator *mapGenerator;
    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "odometry_slam::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
        ~Task();

        virtual bool markRectInMap(double width, double height, const base::samples::RigidBodyState& pose, double offset);

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

