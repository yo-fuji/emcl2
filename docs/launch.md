# launchサンプル

[navigation2パッケージ](https://github.com/ros-planning/navigation2.git)の
[localization_launch.py](https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/localization_launch.py)において、
amclをemcl2に置換する手順を示します。

1. lifecycle_nodesのamclをemcl2に置換します。
    * 修正前
        ```
        lifecycle_nodes = ['map_server', 'amcl']
        ```
    * 修正後
        ```
        lifecycle_nodes = ['map_server', 'emcl2']
        ```

2. load_nodesのamclをemcl2で置換します。
    * 修正前
        ```
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        ```
    * 修正後
        ```
        Node(
            package='emcl2',
            executable='emcl2_node',
            name='emcl2',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        ```

3. load_composable_nodesのamclをemcl2に置換します。
    * 修正前
        ```
        ComposableNode(
            package='nav2_amcl',
            plugin='nav2_amcl::AmclNode',
            name='amcl',
            parameters=[configured_params],
            remappings=remappings),
        ```
    * 修正後
        ```
        ComposableNode(
            package='emcl2',
            plugin='emcl2::EMcl2Node',
            name='emcl2',
            parameters=[configured_params],
            remappings=remappings),
        ```

4. [nav2_params.yaml](https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml)のamcl設定をemcl2に置換します。
    * 修正前
        ```
        amcl:
            ros__parameters:
                alpha1: 0.2
                alpha2: 0.2
                alpha3: 0.2
                alpha4: 0.2
                alpha5: 0.2
                (以下省略)
        ```
    * 修正後
        ```
        emcl2:
            ros__parameters:
                odom_freq: 20
                num_particles: 500
                odom_frame_id: "odom"
                footprint_frame_id: "base_footprint"
                base_frame_id: "base_link"
                odom_fw_dev_per_fw: 0.19
                odom_fw_dev_per_rot: 0.0001
                odom_rot_dev_per_fw: 0.13
                odom_rot_dev_per_rot: 0.2
                laser_likelihood_max_dist: 0.2
                alpha_threshold: 0.5
                open_space_threshold: 0.05
                expansion_radius_position: 0.1
                expansion_radius_orientation: 0.2
                range_threshold: 0.3
                laser_min_range: 0.0
                laser_max_range: 100000000.0
                scan_increment: 1
        ```

5. [package.xml](https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/package.xml)の依存パッケージにemcl2を追加します。
    ```
    <exec_depend>emcl2</exec_depend>
    ```
