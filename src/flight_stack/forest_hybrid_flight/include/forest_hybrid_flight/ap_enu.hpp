#pragma once

// Conversões ArduPilot (NED, corpo FRD) → ROS (ENU, base_link FLU).
// Usado pelo hybrid_hop_executor, que publica /ardupilot/local_position_odom a partir
// da porta 5760 (stream sustentado).

#include <cmath>

namespace forest_hybrid_flight
{

// Quaternião (w,x,y,z) mínimo, sem dependência tf2.
struct Quat { double w{1}, x{0}, y{0}, z{0}; };

inline Quat quat_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  Quat q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

inline Quat quat_mul(const Quat & a, const Quat & b)
{
  return Quat{
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Quat quat_normalized(const Quat & q)
{
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n <= 0.0) { return Quat{}; }
  return Quat{q.w / n, q.x / n, q.y / n, q.z / n};
}

// Atitude do ArduPilot (corpo FRD relativo a NED) → orientação base_link (FLU) em ENU.
// Sandwich estilo mavros: q_enu_flu = NED_ENU_Q · q_ned · AIRCRAFT_BASELINK_Q,
// com NED_ENU_Q = euler(π,0,π/2) e AIRCRAFT_BASELINK_Q = euler(π,0,0).
inline Quat ned_attitude_to_enu_baselink(double roll, double pitch, double yaw)
{
  static const Quat kNedEnu = quat_from_rpy(M_PI, 0.0, M_PI_2);
  static const Quat kAircraftBaselink = quat_from_rpy(M_PI, 0.0, 0.0);
  const Quat q_ned = quat_from_rpy(roll, pitch, yaw);
  return quat_normalized(quat_mul(quat_mul(kNedEnu, q_ned), kAircraftBaselink));
}

}  // namespace forest_hybrid_flight
