#ifndef lidar_ouster_TYPES_HPP
#define lidar_ouster_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */
#include <ouster/types.h>

namespace lidar_ouster {
    /**
     * Struct for sensor configuration parameters. This struct was imported from
     * /ouster/types.h and refactored to be used in orogen
     */
    struct SensorConfig {
        /**
         * The destination address for the lidar and imu data to be sent to
         */
        std::string udp_dest;

        /**
         * The destination port for the lidar data to be sent to
         * Set 0 to detect automatically
         */
        int udp_port_lidar = 0;

        /**
         * The destination port for the imu data to be sent to
         * Set 0 to detect automatically
         */
        int udp_port_imu = 0;

        /**
         * The timestamp mode for the sensor to use.
         * Refer to timestamp_mode for more details.
         */
        ouster::sensor::timestamp_mode ts_mode = ouster::sensor::timestamp_mode::TIME_FROM_INTERNAL_OSC;

        /**
         * The lidar mode for the sensor to use.
         * Refer to lidar_mode for more details.
         */
        ouster::sensor::lidar_mode ld_mode = ouster::sensor::lidar_mode::MODE_1024x20;

        /**
         * The multipurpose io mode for the sensor to use.
         * Refer to MultipurposeIOMode for more details.
         */
        ouster::sensor::MultipurposeIOMode multipurpose_io_mode = ouster::sensor::MultipurposeIOMode::MULTIPURPOSE_OFF;

        /**
         * The azimuth window for the sensor to use.
         * Refer to AzimuthWindow for more details.
         * Originally the Azimuth_window is ouster::sensor::AzimuthWindow type. 
         * However it depens on std::pair witch is not availabe at orogen.       
         */
        int azimuth_window[2];

        /**
         * Multiplier for signal strength of sensor. See the sensor docs for more
         * details on usage.
         */
        double signal_multiplier = 1;

        /**
         * The nmea polarity for the sensor to use.
         * Refer to Polarity for more details.
         */
        ouster::sensor::Polarity nmea_in_polarity = ouster::sensor::Polarity::POLARITY_ACTIVE_HIGH;

        /**
         * Whether NMEA UART input $GPRMC messages should be ignored.
         * Refer to the sensor docs for more details.
         */
        bool nmea_ignore_valid_char = false;

        /**
         * The nmea baud rate for the sensor to use.
         * Refer to Polarity> for more details.
         */
        ouster::sensor::NMEABaudRate nmea_baud_rate = ouster::sensor::NMEABaudRate::BAUD_9600;

        /**
         * Number of leap seconds added to UDP timestamp.
         * See the sensor docs for more details.
         */
        int nmea_leap_seconds = 0;

        /**
         * Polarity of SYNC_PULSE_IN input.
         * See Polarity for more details.
         */
        ouster::sensor::Polarity sync_pulse_in_polarity = ouster::sensor::Polarity::POLARITY_ACTIVE_HIGH;

        /**
         * Polarity of SYNC_PULSE_OUT output.
         * See Polarity for more details.
         */
        ouster::sensor::Polarity sync_pulse_out_polarity = ouster::sensor::Polarity::POLARITY_ACTIVE_HIGH;

        /**
         * Angle in degrees that sensor traverses between each SYNC_PULSE_OUT pulse.
         * See senor docs for more details.
         */
        int sync_pulse_out_angle = 360;

        /**
         * Width of SYNC_PULSE_OUT pulse in ms.
         * See sensor docs for more details.
         */
        int sync_pulse_out_pulse_width = 0;

        /**
         * Frequency of SYNC_PULSE_OUT pulse in Hz.
         * See sensor docs for more details.
         */
        int sync_pulse_out_frequency = 1;

        /**
         * Whether phase locking is enabled.
         * See sensor docs for more details.
         */
        bool phase_lock_enable = false;

        /**
         * Angle that sensors are locked to in millidegrees.
         * See sensor docs for more details.
         */
        int phase_lock_offset = 0;

        /**
         * Columns per packet.
         * See sensor docs for more details.
         */
        int columns_per_packet = 15;

        /**
         * The lidar profile for the sensor to use.
         * Refer to UDPProfileLidar for more details.
         */
        ouster::sensor::UDPProfileLidar udp_profile_lidar = ouster::sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY;

        /**
         * The imu profile for the sensor to use.
         * Refer to UDPProfileIMU for more details.
         */
        ouster::sensor::UDPProfileIMU udp_profile_imu = ouster::sensor::UDPProfileIMU::PROFILE_IMU_LEGACY;
    };

}

#endif
