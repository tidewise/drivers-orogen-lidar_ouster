# frozen_string_literal: true

using_task_library "lidar_ouster"
import_types_from "lidar_ouster"
import_types_from "base"

describe OroGen.lidar_ouster.Task do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.lidar_ouster
                  .Task
                  .deployed_as("lidar_ouster_task")
        )
        @task.properties.ip_address = "os-992203000508.local"
        config = Types.lidar_ouster.SensorConfig.new
        config.signal_multiplier = 1
        config.ld_mode = 4
        config.ts_mode = 1
        config.operating_mode = 1
        config.multipurpose_io_mode = 1
        config.nmea_in_polarity = 2
        config.nmea_baud_rate = 1
        config.sync_pulse_in_polarity = 2
        config.sync_pulse_out_polarity = 2
        config.sync_pulse_out_angle = 360
        config.sync_pulse_out_pulse_width = 0
        config.sync_pulse_out_frequency = 1
        config.udp_profile_lidar = 1
        config.udp_profile_imu = 1
        config.udp_port_lidar = 36_034
        config.udp_port_imu = 40_371
        config.udp_profile_lidar = 1
        config.columns_per_packet = 16
        config.azimuth_window = [0, 360_000]
        config.nmea_ignore_valid_char = 0
        config.nmea_leap_seconds = 0
        config.phase_lock_enable = false
        config.phase_lock_offset = 0
        @task.properties.lidar_config = config
    end

    it "displays information" do
        syskit_configure(@task)
        expect_execution { task.start! }.timeout(100).to do
            emit task.start_event
        end
        output = expect_execution do
        end.timeout(50).to { have_one_new_sample task.depth_map_port }
        pp "resultado"
        pp output
        ask_ok
    end

    def ask_ok
        puts "OK ? (y/n)"
        value = STDIN.readline.chomp
        raise "test failed" unless value == "y"
    end
end
