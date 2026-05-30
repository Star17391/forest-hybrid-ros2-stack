# Third-party dependencies

## Nicla Vision (ADVRHumanoids) — Apache-2.0

| Submodule | Role |
|-----------|------|
| [nicla_vision_drivers](https://github.com/ADVRHumanoids/nicla_vision_drivers) | Firmware (`arduino/main`) — Wi-Fi stream, JPEG, IMU, ToF, mic |
| [nicla_vision_ros2](../src/external/nicla_vision_ros2) | ROS 2 receiver (`nicla_receiver` node) |

**Attribution:** Istituto Italiano di Tecnologia / ADVRHumanoids. See `LICENSE` in each repo and `docs/THIRD_PARTY.md`.

**Forest configuration (edit once):** [`config/forest_nicla_advr_config.h`](../config/forest_nicla_advr_config.h)

```bash
bash scripts/nicla_advr_init_submodules.sh
bash scripts/nicla_advr_apply_config.sh
```
