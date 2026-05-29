<!--
********************************************************************************
* Copyright (C) 2017-2025 German Aerospace Center (DLR).
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0
*
* Contributors:
*   Matthias Nichting
********************************************************************************
-->
# sumo_bridge

ROS 2 package that bridges [SUMO](https://sumo.dlr.de) (v1.22.0+) traffic simulation to the ADORe stack via libsumo. SUMO vehicles are published as `TrafficParticipantSet` messages and the ego vehicle state is injected back into SUMO each simulation step.

## Requirements

- SUMO 1.22.0 or later
- ROS 2 (Jazzy or later)
- ADORe ROS 2 workspace built and sourced

## Getting Started

Run the demo scenario using the provided script, which starts the node with the bundled `demo_sumo_bridge.sumocfg`:

```bash
bash sumo_configs/start_sumo_bridge.sh
```

Or run the node directly with a custom config:

```bash
ros2 run sumo_bridge sumo_bridge --ros-args \
    --param "sumo_config_file:=<absolute_path_to_config>.sumocfg" \
    --param "ego_start_position:=<lat>,<lon>,<psi>"
```

## ROS Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `sumo_config_file` | `string` | `""` | Absolute path to the `.sumocfg` file. Required. |
| `sumo_step_length` | `double` | `0.05` | Simulation step length in seconds |
| `use_gui` | `bool` | `false` | Launch `sumo-gui` instead of headless `sumo`. Simulation starts immediately without requiring the play button. |
| `gui_settings_file` | `string` | `""` | Path to a SUMO GUI settings XML file |
| `gui_zoom` | `double` | `5000.0` | Initial zoom level for `sumo-gui` (percentage; 5000 ≈ 10 m street level) |
| `gui_follow_ego` | `bool` | `true` | Whether `sumo-gui` should track and follow the ego vehicle |
| `ego_start_position` | `string` | `""` | Ego start position as `"lat,lon,psi"` (psi in radians CCW from east). Derives `utm_zone`, `utm_letter`, and SUMO coordinates automatically. Required for georeferenced scenarios; omit for local coordinate scenarios. |
| `ego_tracking_id` | `int` | `0` | Tracking ID of the ego vehicle used for GUI follow and color assignment |
| `ego_vehicle_color` | `string` | `"255,255,0"` | Ego vehicle color as `"R,G,B"` or `"R,G,B,A"` with values 0–255 |
| `utm_zone` | `int` | `32` | UTM zone number. Used only when `ego_start_position` is not set (local coordinate scenarios). |
| `utm_letter` | `string` | `"U"` | UTM zone letter. Used only when `ego_start_position` is not set. |
| `use_geo_conversion` | `bool` | `true` | When `false`, bypasses UTM/geo conversion and treats all coordinates as SUMO-local. Set to `false` for synthetic scenarios without real-world georeferencing. |
| `initial_traffic_count` | `int` | `0` | Number of SUMO traffic vehicles to spawn ahead of the ego at startup |
| `initial_traffic_spacing` | `double` | `20.0` | Distance in metres between each spawned traffic vehicle |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `traffic_participants` | `TrafficParticipantSet` | publish | SUMO vehicles converted to ADORe traffic participants |
| `simulated_traffic_participant` | `TrafficParticipant` | subscribe | Ego vehicle state injected into SUMO each step |

## Launch

Two example scenarios are provided.

### OSM scenario (georeferenced)

Uses a real OpenStreetMap network. The ego start and goal positions are specified as lat/lon and all coordinate conversion is handled automatically:

```python
EGO_START      = Position(lat_long=(52.314331, 10.53793), psi=3.14)
EGO_GOAL       = Position(lat_long=(52.31463, 10.55909), psi=0.0)
EGO_VEHICLE_ID = 111

ego_lat, ego_lon, ego_psi = EGO_START.get_lat_long_coordinates()

Node(
    package='sumo_bridge',
    executable='sumo_bridge',
    parameters=[
        {"sumo_config_file":      SUMO_CONFIG_PATH},
        {"use_gui":               True},
        {"gui_settings_file":     GUI_SETTINGS_PATH},
        {"gui_zoom":              5000.0},
        {"gui_follow_ego":        True},
        {"ego_tracking_id":       EGO_VEHICLE_ID},
        {"ego_vehicle_color":     "255,255,0"},
        {"ego_start_position":    f"{ego_lat},{ego_lon},{ego_psi}"},
        {"initial_traffic_count": 3},
        {"initial_traffic_spacing": 20.0},
    ],
)
```

### Synthetic scenario (local coordinates)

Uses a local coordinate network such as `circle50m`. Set `use_geo_conversion` to `false` and supply `utm_zone`/`utm_letter` directly:

```python
EGO_START      = Position(xy=(50.0, 0.0), psi=3.14/2)
EGO_VEHICLE_ID = 111
ego_utm        = EGO_START.get_utm_coordinates()

Node(
    package='sumo_bridge',
    executable='sumo_bridge',
    parameters=[
        {"sumo_config_file":   SUMO_CONFIG_PATH},
        {"use_gui":            True},
        {"gui_settings_file":  GUI_SETTINGS_PATH},
        {"gui_zoom":           100.0},
        {"gui_follow_ego":     False},
        {"ego_tracking_id":    EGO_VEHICLE_ID},
        {"ego_vehicle_color":  "255,0,0"},
        {"use_geo_conversion": False},
        {"utm_zone":           ego_utm[2]},
        {"utm_letter":         ego_utm[3]},
    ],
)
```

## Clock Synchronisation

The bridge synchronises SUMO simulation time to the ROS wall clock. Reference times (`tSUMO0`, `tROS0`) are captured after the first simulation step completes, ensuring both clocks are anchored at the same point. When `use_gui` is `true`, the `--start` flag is passed to `sumo-gui` so the simulation begins immediately without requiring the play button.

## Coordinate Conventions

| Convention | Details |
|---|---|
| ROS yaw (`yaw_angle`) | Radians, counter-clockwise from east |
| SUMO heading | Degrees, clockwise from north |
| SUMO `getPosition` | Returns front bumper position; the bridge offsets by half vehicle length to publish/consume centre positions |
| Geo conversion | `convertGeo(lon, lat, toGeo=true/false)` — note argument order is longitude first |

## Initial Traffic Spawning

When `initial_traffic_count > 0` the bridge spawns SUMO-controlled vehicles ahead of the ego start position at startup. Spawn positions are walked along the actual lane geometry using the lane shape polyline, so vehicles land on the road regardless of curvature. Each vehicle is given a unique randomised route built by walking the routing graph forward from its spawn edge, weighted toward higher speed-limit roads. Routes are deterministic per vehicle ID across runs.
