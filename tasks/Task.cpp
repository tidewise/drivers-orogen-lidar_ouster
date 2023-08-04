/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base-logging/Logging.hpp>

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
    m_handle = sensor::init_client(_ip_address.get(), m_data_destination);
    if (!m_handle) {
        throw std::runtime_error("Failed to connect to sensor!");
    }
    m_packet_buffer.resize(m_udp_buf_size);
    m_vertical_fov = _vertical_fov.get();
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    m_metadata = getMetadata();
    m_packet_format.reset(new sensor::packet_format(sensor::get_format(m_metadata)));
    // A ScanBatcher can be used to batch packets into scans
    m_scan_batcher.reset(
        new ScanBatcher(m_metadata.format.columns_per_frame, *m_packet_format));
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    auto data = acquireData();
    convertDataAndWriteOutput(data);
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    ouster::sensor::sensor_config config;
    config.operating_mode = ouster::sensor::OperatingMode::OPERATING_STANDBY;
    if (!sensor::set_config(m_sensor_hostname, config, 0)) {
        LOG_ERROR_S << "Failed to configure Lidar!" << std::endl;
    }
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    m_handle.reset();
}

sensor::sensor_info Task::getMetadata()
{
    auto metadata = sensor::get_metadata(*m_handle);
    return sensor::parse_metadata(metadata);
}

LidarScan Task::acquireData()
{
    // A LidarScan holds lidar data for an entire rotation of the device
    LidarScan scan{m_metadata.format.columns_per_frame,
        m_metadata.format.pixels_per_column,
        m_metadata.format.udp_profile_lidar};

    while (true) {
        sensor::client_state cli_state = sensor::poll_client(*m_handle);
        // check for error status
        if (cli_state & sensor::CLIENT_ERROR) {
            throw std::runtime_error("Sensor client returned error state!");
        }

        // check for lidar data, read a packet and add it to the current batch
        if (cli_state & sensor::LIDAR_DATA) {
            if (!sensor::read_lidar_packet(*m_handle,
                    m_packet_buffer.data(),
                    *m_packet_format)) {
                throw std::runtime_error("Failed to read a packet of the expected size!");
            }
            if ((*m_scan_batcher)(m_packet_buffer.data(), scan)) {
                if (scan.complete(m_metadata.format.column_window)) {
                    return scan;
                }
            }
        }
        if (cli_state & sensor::IMU_DATA) {
            sensor::read_imu_packet(*m_handle, m_packet_buffer.data(), *m_packet_format);
            writeIMUSample(m_packet_buffer);
        }
    }
}

void Task::writeIMUSample(std::vector<uint8_t> const& m_packet_buffer)
{
    base::samples::IMUSensors imu_samples;
    float acceleration_x, acceleration_y, acceleration_z, angular_velocity_x,
        angular_velocity_y, angular_velocity_z;
    memcpy(&acceleration_x, m_packet_buffer.data() + 24, sizeof(float));
    memcpy(&acceleration_y, m_packet_buffer.data() + 28, sizeof(float));
    memcpy(&acceleration_z, m_packet_buffer.data() + 32, sizeof(float));
    memcpy(&angular_velocity_x, m_packet_buffer.data() + 36, sizeof(float));
    memcpy(&angular_velocity_y, m_packet_buffer.data() + 40, sizeof(float));
    memcpy(&angular_velocity_z, m_packet_buffer.data() + 44, sizeof(float));
    imu_samples.acc << acceleration_x * 9.8, acceleration_y * 9.8, acceleration_z * 9.8;
    imu_samples.gyro << angular_velocity_x * M_PI / 180, angular_velocity_y * M_PI / 180,
        angular_velocity_z * M_PI / 180;
    imu_samples.time = base::Time::now();
    _imu_samples.write(imu_samples);
}

void Task::convertDataAndWriteOutput(LidarScan& scan)
{

    base::samples::DepthMap depth_map;
    auto time = base::Time::now();
    depth_map.time = time;
    depth_map.timestamps.push_back(time);

    depth_map.vertical_projection = base::samples::DepthMap::PROJECTION_TYPE::POLAR;
    depth_map.horizontal_projection = base::samples::DepthMap::PROJECTION_TYPE::POLAR;

    depth_map.horizontal_interval.push_back(M_PI * 2.0);
    depth_map.horizontal_interval.push_back(0);

    depth_map.vertical_interval.push_back(-(m_vertical_fov / 2) * M_PI / (180.0));
    depth_map.vertical_interval.push_back((m_vertical_fov / 2) * M_PI / (180.0));

    depth_map.vertical_size = m_metadata.format.pixels_per_column;
    depth_map.horizontal_size = m_metadata.format.columns_per_frame;

    auto range = scan.field(sensor::ChanField::RANGE);
    auto range_destaggered =
        destagger<uint32_t>(range, m_metadata.format.pixel_shift_by_row);

    auto reflectivity_destaggered = getReflectivity(scan);

    auto width = m_metadata.format.columns_per_frame;
    auto height = m_metadata.format.pixels_per_column;

    depth_map.distances.resize(range_destaggered.rows() * range_destaggered.cols());
    depth_map.remissions.resize(range_destaggered.rows() * range_destaggered.cols());
    for (unsigned int h = 0; h < height; h++) {
        for (unsigned int w = 0; w < width; w++) {
            unsigned int index = h * width + w;
            depth_map.distances[index] =
                static_cast<double>(range_destaggered(h, w)) / 1000.0;
            depth_map.remissions[index] =
                static_cast<double>(reflectivity_destaggered(h, w));
        }
    }
    _depth_map.write(depth_map);
}

ouster::img_t<uint8_t> Task::getReflectivity(ouster::LidarScan const& scan)
{
    Eigen::Array<uint8_t, -1, -1, Eigen::RowMajor> reflectivity;
    if (m_metadata.format.udp_profile_lidar ==
        sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        reflectivity = scan.field(sensor::ChanField::REFLECTIVITY).cast<uint8_t>();
    }
    else if (m_metadata.format.udp_profile_lidar ==
             sensor::UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL) {
        reflectivity = scan.field<uint8_t>(sensor::ChanField::REFLECTIVITY);
    }
    else { // legacy or single return profile
        reflectivity =
            scan.field<uint16_t>(sensor::ChanField::REFLECTIVITY).cast<uint8_t>();
    }
    return destagger<uint8_t>(reflectivity, m_metadata.format.pixel_shift_by_row);
}

bool Task::configureLidar()
{
    uint8_t config_flags = 0;
    m_sensor_hostname = _ip_address.get();
    auto lidar_config = _lidar_config.get();

    ouster::sensor::sensor_config config;

    // you cannot set the udp_dest flag while simultaneously setting
    //  config.udp_dest Will throw an invalid_argument if you do
    if (!lidar_config.udp_dest.empty()) {
        ouster::sensor::sensor_config config;
        config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
        config.udp_dest = lidar_config.udp_dest;
        if (!sensor::set_config(m_sensor_hostname, config, config_flags)) {
            LOG_ERROR_S << "Failed to configure Lidar!" << std::endl;
            return false;
        }
    }
    if (lidar_config.udp_port_lidar) {
        config.udp_port_lidar = lidar_config.udp_port_lidar;
    }
    if (lidar_config.udp_port_imu) {
        config.udp_port_imu = lidar_config.udp_port_imu;
    }
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

    if (!sensor::set_config(m_sensor_hostname, config, config_flags)) {
        LOG_ERROR_S << "Failed to configure Lidar!" << std::endl;
        return false;
    }

    return true;
}