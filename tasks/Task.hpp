/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LIDAR_OUSTER_TASK_TASK_HPP
#define LIDAR_OUSTER_TASK_TASK_HPP

#include <base/samples/DepthMap.hpp>
#include <lidar_ouster/TaskBase.hpp>
#include <ouster/client.h>
#include <ouster/impl/build.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <string>

namespace lidar_ouster {

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine
to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These
interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the
associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the lidar_ouster namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','lidar_ouster::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix
argument.
     */
    class Task : public TaskBase {
        friend class TaskBase;

    private:
        const size_t m_udp_buf_size = 65536;
        std::vector<uint8_t> m_packet_buffer;
        const std::string m_data_destination = "";
        std::shared_ptr<ouster::sensor::client> m_handle;
        ouster::sensor::sensor_info m_metadata;
        std::string m_sensor_hostname;
        std::unique_ptr<ouster::ScanBatcher> m_scan_batcher;
        std::unique_ptr<ouster::sensor::packet_format> m_packet_format;
        double m_vertical_fov = 0.0;
        base::samples::DepthMap m_depth_map;
        bool m_remission_enabled = false;
        base::Time m_first_scan_timeout;
        base::Time m_scan_timeout;

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable via nameservices. \param initial_state The initial TaskState of
         * the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "lidar_ouster::Task");

        /** Default deconstructor of Task
         */
        ~Task();

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

        bool configureLidar();
        ouster::sensor::sensor_info getMetadata();
        ouster::LidarScan acquireData(base::Time const& timeout);
        void convertDataAndWriteOutput(ouster::LidarScan& scan);
        void writeIMUSample(std::vector<uint8_t>& pkt_buffer);
        ouster::img_t<uint8_t> getReflectivity(ouster::LidarScan const& scan);
        void configureDepthMap();
    };
}

#endif
