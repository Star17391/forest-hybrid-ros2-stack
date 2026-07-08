#pragma once

#include <cmath>

#include <Eigen/Core>

namespace forest_tree_slam
{

// Intrínsecos pinhole + resolução da imagem (de sensor_msgs/CameraInfo).
struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};  // ponto principal x (px)
  double cy{0.0};  // ponto principal y (px)
  int width{0};
  int height{0};
  bool valid{false};
};

struct PixelProjection
{
  double u{0.0};      // px
  double v{0.0};      // px
  double depth{0.0};  // profundidade óptica (m), >0 = à frente
  bool valid{false};
};

// Projeta um ponto em `base_link` (x-frente, y-esquerda, z-cima) na imagem da
// câmara frontal. A câmara está em `base + extrinsic` com RPY=0 (frame "óptico"
// publicado body-oriented), por isso aplica-se a rotação body→óptico fixa:
//   óptico: z-frente, x-direita, y-baixo  ⇒  X_opt=-y_body, Y_opt=-z_body, Z_opt=x_body
// (igual ao `_R_OPT_BODY` de forest_vision_detection/geometry.py).
inline PixelProjection project_base_to_image(
  const Eigen::Vector3d & p_base, const Eigen::Vector3d & extrinsic,
  const CameraIntrinsics & k, double min_depth_m = 0.2)
{
  PixelProjection out;
  if (!k.valid) {
    return out;
  }
  const double xb = p_base.x() - extrinsic.x();  // body, à frente
  const double yb = p_base.y() - extrinsic.y();  // body, à esquerda
  const double zb = p_base.z() - extrinsic.z();  // body, acima
  const double depth = xb;                        // Z_opt = X_body
  if (depth <= min_depth_m) {
    return out;  // atrás da câmara / demasiado perto
  }
  const double u = k.fx * (-yb / depth) + k.cx;
  const double v = k.fy * (-zb / depth) + k.cy;
  if (u < 0.0 || v < 0.0 || u >= static_cast<double>(k.width) ||
    v >= static_cast<double>(k.height))
  {
    return out;  // fora da imagem
  }
  out.u = u;
  out.v = v;
  out.depth = depth;
  out.valid = true;
  return out;
}

// (u,v) cai dentro da caixa axis-aligned de centro (cx_px,cy_px) e tamanho (w,h)?
inline bool pixel_in_bbox(
  double u, double v, double box_cx, double box_cy, double box_w, double box_h)
{
  return std::abs(u - box_cx) <= 0.5 * box_w && std::abs(v - box_cy) <= 0.5 * box_h;
}

}  // namespace forest_tree_slam
