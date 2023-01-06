/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace lidar_ouster;
using namespace ouster;
using namespace std;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
        return false;
    if (!initLidar()) {
        throw std::runtime_error("Failed to connect to Lidar!");
    }
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    auto metadata = getMetadata();
    auto data = acquireData(metadata);
    convertData(data, metadata);
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
    TaskBase::cleanupHook();
}

bool Task::initLidar()
{
    sensor_hostname = _ip_address.get();
    handle = sensor::init_client(sensor_hostname, data_destination);
    if (!handle)
        return false;
    return true;
}

sensor::sensor_info Task::getMetadata()
{
    auto metadata = sensor::get_metadata(*handle);
    return sensor::parse_metadata(metadata);
}

LidarScan Task::acquireData(sensor::sensor_info &info)
{
    size_t width = info.format.columns_per_frame;
    size_t heigth = info.format.pixels_per_column;

    // A LidarScan holds lidar data for an entire rotation of the device
    LidarScan scan{width, heigth, info.format.udp_profile_lidar};

    // A ScanBatcher can be used to batch packets into scans
    sensor::packet_format pkt_format = sensor::get_format(info);
    ScanBatcher batch_to_scan(info.format.columns_per_frame, pkt_format);

    // buffer to store raw packet data
    auto pkt_buffer = std::make_unique<uint8_t[]>(UDP_BUF_SIZE);

    while (!scan.complete(info.format.column_window)) {
        sensor::client_state cli_state = sensor::poll_client(*handle);
        // check for error status
        if (cli_state & sensor::CLIENT_ERROR) {
            throw std::runtime_error("Sensor client returned error state!");
        }

        // check for lidar data, read a packet and add it to the current batch
        if (cli_state & sensor::LIDAR_DATA) {
            if (!sensor::read_lidar_packet(*handle, pkt_buffer.get(), pkt_format)) {
                {
                    throw std::runtime_error(
                        "Failed to read a packet of the expected size!");
                }
            }
            batch_to_scan(pkt_buffer.get(), scan);
        }
    }
    return scan;
}

void Task::convertData(LidarScan& scan, ouster::sensor::sensor_info& info)
{
    base::samples::DepthMap depth_map;
    // TODO check when the time will be computed
    depth_map.time = base::Time::now();
    depth_map.vertical_projection = base::samples::DepthMap::PROJECTION_TYPE::PLANAR;
    depth_map.horizontal_projection = base::samples::DepthMap::PROJECTION_TYPE::PLANAR;

    depth_map.vertical_size = info.format.pixels_per_column;
    depth_map.horizontal_size = info.format.columns_per_frame;

    // for (auto point : scan)
    // {
    //     depth_map.vertical_interval.push_back(point-aaa);
    //     depth_map.horizontal_interval.push_back(point-aa);
    //     depth_map.distances.push_back(point-aaaa);
    // }

    _depth_map.write(depth_map);
}

bool Task::configureLidar()
{
    // ADD A CONFIGURATION MODULE FOR IT.
}