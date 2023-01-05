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
    sensor_hostname = _ip_address.get();
    handle = sensor::init_client(sensor_hostname, data_destination);
    if (!handle)
        return false;
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
    _teste.get();
    auto metadata = sensor::get_metadata(*handle);

    sensor::sensor_info info = sensor::parse_metadata(metadata);

    // size_t width = info.format.columns_per_frame;
    // size_t height = info.format.pixels_per_column;

    // ouster::sensor::ColumnWindow column_window = info.format.column_window;

    string a = "Firmware version:" + info.fw_rev;
    // a += "\n  Serial number: " + to_string(info.sn) "\n  Product line:" + to_string(info.prod_line);
    // a += "\n  Scan dimensions:   " + to_string(width) + " x " + to_string(height);
    // a += "\n  Column window: [" + column_window.first + ", " column_window.second + "]";

    _output.write("a");
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
