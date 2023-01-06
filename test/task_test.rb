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
        @task.properties.ip_address = "os-992234000654.local"
    end

    it "displays information" do
        syskit_configure_and_start(@task)
        output = expect_execution do
        end.to { have_one_new_sample task.depth_map_port}
        pp "resultado"
        pp output
        assert_equal(output.vertical_size, 0)
    end

end
