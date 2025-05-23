/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base-logging/Logging.hpp>

using namespace lidar_ouster;
using namespace ouster;
using namespace sensor;
using namespace std;
using namespace base;
using namespace samples;

Task::Task(string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

void Task::configureDepthMap()
{
    m_depth_map.vertical_projection = DepthMap::PROJECTION_TYPE::POLAR;
    m_depth_map.horizontal_projection = DepthMap::PROJECTION_TYPE::POLAR;

    m_depth_map.horizontal_interval.push_back(M_PI * 2.0);
    m_depth_map.horizontal_interval.push_back(0);

    m_depth_map.vertical_interval.push_back(-(m_vertical_fov / 2) * M_PI / (180.0));
    m_depth_map.vertical_interval.push_back((m_vertical_fov / 2) * M_PI / (180.0));

    m_depth_map.vertical_size = m_metadata.format.pixels_per_column;
    m_depth_map.horizontal_size = m_metadata.format.columns_per_frame;

    m_depth_map.distances.resize(
        m_metadata.format.pixels_per_column * m_metadata.format.pixels_per_column);
    m_depth_map.remissions.resize(
        m_metadata.format.pixels_per_column * m_metadata.format.pixels_per_column);
}

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
        return false;
    if (!configureLidar()) {
        return false;
    }
    m_handle = init_client(_ip_address.get(), m_data_destination);
    if (!m_handle) {
        throw runtime_error("Failed to connect to sensor!");
    }
    m_scan_timeout = _scan_timeout.get();
    m_first_scan_timeout = _first_scan_timeout.get();
    m_packet_buffer.resize(m_udp_buf_size);
    m_vertical_fov = _vertical_fov.get();
    m_remission_enabled = _remission_enabled.get();
    m_metadata = getMetadata();
    configureDepthMap();
    m_packet_format.reset(new packet_format(get_format(m_metadata)));
    // A ScanBatcher can be used to batch packets into scans
    m_scan_batcher.reset(
        new ScanBatcher(m_metadata.format.columns_per_frame, *m_packet_format));
    auto data = acquireData(m_first_scan_timeout);
    if (!data.complete(m_metadata.format.column_window)) {
        exception(TIMEOUT);
        return false;
    }
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    sensor_config config;
    config.operating_mode = OperatingMode::OPERATING_NORMAL;
    if (!set_config(m_sensor_hostname, config, 0)) {
        LOG_ERROR_S << "Failed to configure Lidar!" << endl;
    }
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    auto data = acquireData(m_scan_timeout);
    if (!data.complete(m_metadata.format.column_window)) {
        exception(TIMEOUT);
        return;
    }
    convertDataAndWriteOutput(data);
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    m_handle.reset();
    sensor_config config;
    config.operating_mode = OperatingMode::OPERATING_STANDBY;
    if (!set_config(m_sensor_hostname, config, 0)) {
        LOG_ERROR_S << "Failed to configure Lidar!" << endl;
    }
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

sensor_info Task::getMetadata()
{
    auto metadata = get_metadata(*m_handle);
    return parse_metadata(metadata);
}

LidarScan Task::acquireData(Time const& timeout)
{
    Time deadline = timeout + Time::now();
    // A LidarScan holds lidar data for an entire rotation of the device
    LidarScan scan{m_metadata.format.columns_per_frame,
        m_metadata.format.pixels_per_column,
        m_metadata.format.udp_profile_lidar};

    while (true) {
        if (Time::now() > deadline) {
            LOG_ERROR_S << "The scan acquisition has timed out!";
            return scan;
        }

        client_state cli_state = poll_client(*m_handle);
        // check for error status
        if (cli_state & CLIENT_ERROR) {
            throw runtime_error("Sensor client returned error state!");
        }

        // check for lidar data, read a packet and add it to the current batch
        if (cli_state & LIDAR_DATA) {
            if (!read_lidar_packet(*m_handle, m_packet_buffer.data(), *m_packet_format)) {
                throw runtime_error("Failed to read a packet of the expected size!");
            }
            if ((*m_scan_batcher)(m_packet_buffer.data(), scan)) {
                if (scan.complete(m_metadata.format.column_window)) {
                    return scan;
                }
            }
        }
        if (cli_state & IMU_DATA) {
            read_imu_packet(*m_handle, m_packet_buffer.data(), *m_packet_format);
            writeIMUSample(m_packet_buffer);
        }
    }
}

void Task::writeIMUSample(vector<uint8_t>& m_packet_buffer)
{
    IMUSensors imu_samples;
    float* data = reinterpret_cast<float*>(m_packet_buffer.data() + 24);
    float acceleration_x = data[0];
    float acceleration_y = data[1];
    float acceleration_z = data[2];
    float angular_velocity_x = data[3];
    float angular_velocity_y = data[4];
    float angular_velocity_z = data[5];

    imu_samples.acc << acceleration_x * 9.8, acceleration_y * 9.8, acceleration_z * 9.8;
    imu_samples.gyro << angular_velocity_x * M_PI / 180, angular_velocity_y * M_PI / 180,
        angular_velocity_z * M_PI / 180;
    imu_samples.time = base::Time::now();
    _imu_samples.write(imu_samples);
}

void Task::convertDataAndWriteOutput(LidarScan& scan)
{
    auto time = base::Time::now();
    m_depth_map.time = time;
    m_depth_map.timestamps = { time };

    auto range = scan.field(ChanField::RANGE);
    auto range_destaggered =
        destagger<uint32_t>(range, m_metadata.format.pixel_shift_by_row);
    img_t<uint8_t> reflectivity_destaggered;
    if (m_remission_enabled) {
        reflectivity_destaggered = getReflectivity(scan);
    }
    auto width = m_metadata.format.columns_per_frame;
    auto height = m_metadata.format.pixels_per_column;
    m_depth_map.distances.clear();
    m_depth_map.remissions.clear();
    m_depth_map.distances.resize(range_destaggered.rows() * range_destaggered.cols());
    m_depth_map.remissions.resize(range_destaggered.rows() * range_destaggered.cols());
    for (unsigned int h = 0; h < height; h++) {
        for (unsigned int w = 0; w < width; w++) {
            unsigned int index = h * width + w;
            m_depth_map.distances[index] = range_destaggered(h, w) * 1e-3;
            if (m_remission_enabled) {
                if (reflectivity_destaggered(h, w) <= 100) {
                    m_depth_map.remissions[index] = reflectivity_destaggered(h, w) * 1e-2;
                }
                else {
                    m_depth_map.remissions[index] = 1.0;
                }
            }
        }
    }
    _depth_map.write(m_depth_map);
}

img_t<uint8_t> Task::getReflectivity(LidarScan const& scan)
{
    Eigen::Array<uint8_t, -1, -1, Eigen::RowMajor> reflectivity;
    if (m_metadata.format.udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        reflectivity = scan.field(ChanField::REFLECTIVITY).cast<uint8_t>();
    }
    else if (m_metadata.format.udp_profile_lidar ==
             UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL) {
        reflectivity = scan.field<uint8_t>(ChanField::REFLECTIVITY);
    }
    else { // legacy or single return profile
        reflectivity = scan.field<uint16_t>(ChanField::REFLECTIVITY).cast<uint8_t>();
    }
    return destagger<uint8_t>(reflectivity, m_metadata.format.pixel_shift_by_row);
}

bool Task::configureLidar()
{
    uint8_t config_flags = 0;
    m_sensor_hostname = _ip_address.get();
    auto lidar_config = _lidar_config.get();

    sensor_config config;

    // you cannot set the udp_dest flag while simultaneously setting
    //  config.udp_dest Will throw an invalid_argument if you do
    if (!lidar_config.udp_dest.empty()) {
        sensor_config config;
        config_flags |= CONFIG_UDP_DEST_AUTO;
        config.udp_dest = lidar_config.udp_dest;
        if (!set_config(m_sensor_hostname, config, config_flags)) {
            LOG_ERROR_S << "Failed to configure Lidar!" << endl;
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
    config.operating_mode = OperatingMode::OPERATING_STANDBY;
    config.multipurpose_io_mode = lidar_config.multipurpose_io_mode;
    config.azimuth_window =
        make_pair(lidar_config.azimuth_window[0], lidar_config.azimuth_window[1]);
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

    if (!set_config(m_sensor_hostname, config, config_flags)) {
        LOG_ERROR_S << "Failed to configure Lidar!" << endl;
        return false;
    }

    return true;
}