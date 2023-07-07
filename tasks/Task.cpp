/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base-logging/Logging.hpp>
#include <functional>
#include <iostream>
#include <utility>

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

    if (!configureLidar()) {
        return false;
    }
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    auto lidar_config = _lidar_config.get();
    m_handle = sensor::init_client(_ip_address.get(), m_data_destination);
    if (!m_handle) {
        throw std::runtime_error("Failed to connect to sensor!");
    }
    m_metadata = getMetadata();
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    auto data = acquireData();
    convertData(data);
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    m_handle.reset();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

sensor::sensor_info Task::getMetadata()
{
    auto metadata = sensor::get_metadata(*m_handle);
    return sensor::parse_metadata(metadata);
}

LidarScan Task::acquireData()
{
    size_t width = m_metadata.format.columns_per_frame;
    size_t heigth = m_metadata.format.pixels_per_column;

    // A LidarScan holds lidar data for an entire rotation of the device
    LidarScan scan{width, heigth, m_metadata.format.udp_profile_lidar};

    // A ScanBatcher can be used to batch packets into scans
    sensor::packet_format pkt_format = sensor::get_format(m_metadata);
    ScanBatcher batch_to_scan(m_metadata.format.columns_per_frame, pkt_format);

    // buffer to store raw packet data
    auto pkt_buffer = std::make_unique<uint8_t[]>(m_udp_buf_size);
    bool status = false;
    while (!status) {
        sensor::client_state cli_state = sensor::poll_client(*m_handle);
        // check for error status
        if (cli_state & sensor::CLIENT_ERROR) {
            throw std::runtime_error("Sensor client returned error state!");
        }

        // check for lidar data, read a packet and add it to the current batch
        if (cli_state & sensor::LIDAR_DATA) {
            if (!sensor::read_lidar_packet(*m_handle, pkt_buffer.get(), pkt_format)) {
                {
                    throw std::runtime_error(
                        "Failed to read a packet of the expected size!");
                }
            }
            if (batch_to_scan(pkt_buffer.get(), scan)) {
                if (scan.complete(m_metadata.format.column_window)) {
                    status = true;
                }
            }
        }
    }
    return scan;
}

void Task::convertData(LidarScan& scan)
{

    base::samples::DepthMap depth_map;
    // TODO check when the time will be computed
    auto time = base::Time::now();
    depth_map.time = time;
    depth_map.timestamps.push_back(time);

    depth_map.vertical_projection = base::samples::DepthMap::PROJECTION_TYPE::POLAR;
    depth_map.horizontal_projection = base::samples::DepthMap::PROJECTION_TYPE::POLAR;

    auto width = m_metadata.format.columns_per_frame;
    auto height = m_metadata.format.pixels_per_column;

    depth_map.horizontal_interval.push_back(M_PI * 2.0);
    depth_map.horizontal_interval.push_back(0);

    depth_map.vertical_interval.push_back(22.5 * M_PI / (180.0));
    depth_map.vertical_interval.push_back(-22.5 * M_PI / (180.0));

    depth_map.vertical_size = m_metadata.format.pixels_per_column;
    depth_map.horizontal_size = m_metadata.format.columns_per_frame;

    auto img = scan.field(sensor::ChanField::RANGE);
    auto data = destagger<uint32_t>(img, m_metadata.format.pixel_shift_by_row);

    depth_map.distances.resize(data.rows() * data.cols());
    for (unsigned int h = 0; h < height; h++) {
        for (unsigned int w = 0; w < width; w++) {
            depth_map.distances[h * width + w] = static_cast<double>(data(h, w)) / 1000.0;
        }
    }
    _depth_map.write(depth_map);
}

bool Task::configureLidar()
{
    uint8_t config_flags = 0;
    auto sensor_hostname = _ip_address.get();
    auto lidar_config = _lidar_config.get();

    ouster::sensor::sensor_config config;

    // you cannot set the udp_dest flag while simultaneously setting
    //  config.udp_dest Will throw an invalid_argument if you do
    // todo: check this
    if (!lidar_config.udp_dest.empty()) {
        ouster::sensor::sensor_config config;
        config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
        config.udp_dest = lidar_config.udp_dest;
        if (!sensor::set_config(sensor_hostname, config, config_flags)) {
            LOG_ERROR_S << "Failed to configure Lidar!" << std::endl;
            return false;
        }
        sensor::set_config(sensor_hostname, config, config_flags);
    }

    config.udp_port_lidar = lidar_config.udp_port_lidar;
    config.udp_port_imu = lidar_config.udp_port_imu;
    config.ts_mode = lidar_config.ts_mode;
    config.ld_mode = lidar_config.ld_mode;
    config.operating_mode = lidar_config.operating_mode;
    config.multipurpose_io_mode = lidar_config.multipurpose_io_mode;
    config.azimuth_window =
        std::make_pair(lidar_config.azimuth_window[0], lidar_config.azimuth_window[1]);
    config.signal_multiplier = lidar_config.signal_multiplier;
    config.nmea_in_polarity = lidar_config.nmea_in_polarity;
    config.nmea_ignore_valid_char = lidar_config.nmea_ignore_valid_char;
    config.nmea_baud_rate = lidar_config.nmea_baud_rate;
    config.nmea_leap_seconds = lidar_config.nmea_leap_seconds;
    config.sync_pulse_in_polarity = lidar_config.sync_pulse_in_polarity;
    config.sync_pulse_out_polarity = lidar_config.sync_pulse_out_polarity;
    config.sync_pulse_out_angle = lidar_config.sync_pulse_out_angle;
    config.sync_pulse_out_pulse_width = lidar_config.sync_pulse_out_pulse_width;
    config.sync_pulse_out_frequency = lidar_config.sync_pulse_out_frequency;
    config.phase_lock_enable = lidar_config.phase_lock_enable;
    config.phase_lock_offset = lidar_config.phase_lock_offset;
    config.columns_per_packet = lidar_config.columns_per_packet;
    config.udp_profile_lidar = lidar_config.udp_profile_lidar;
    config.udp_profile_imu = lidar_config.udp_profile_imu;

    if (!sensor::set_config(sensor_hostname, config, config_flags)) {
        LOG_ERROR_S << "Failed to configure Lidar!" << std::endl;
        return false;
    }

    return true;
}