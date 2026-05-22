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
    --param "sumo_config_file:=<absolute_path_to_config>.sumocfg"
```

## ROS Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `sumo_config_file` | `string` | `""` | Absolute path to the `.sumocfg` file. Required. |
| `sumo_step_length` | `double` | `0.01` | Simulation step length in seconds |
| `use_gui` | `bool` | `false` | Launch `sumo-gui` instead of headless `sumo` |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `traffic_participants` | `TrafficParticipantSet` | publish | SUMO vehicles converted to ADORe traffic participants |
| `simulated_traffic_participant` | `TrafficParticipant` | subscribe | Ego vehicle state injected into SUMO |

## Launch

An example scenario including the ego vehicle stack and visualizer is available in [adore_simulation_scenarios](https://github.com/eclipse-adore/adore_simulation_scenarios/blob/main/sumo_test.launch.py):

```bash
ros2 launch sumo_bridge sumo_test.launch.py
```

To enable the SUMO GUI, set `use_gui` on the node:

```python
Node(
    package='sumo_bridge',
    executable='sumo_bridge',
    parameters=[
        {"sumo_config_file": "/path/to/config.sumocfg"},
        {"use_gui": True}
    ],
)
```
