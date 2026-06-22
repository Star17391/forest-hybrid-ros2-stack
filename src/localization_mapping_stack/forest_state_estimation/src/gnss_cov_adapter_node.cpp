// Adapta a covariância do NavSatFix para valores realistas sob dossel florestal.
//
// Problema: os drivers GNSS frequentemente reportam covariâncias otimistas
// (centímetros a poucos metros), ignorando a degradação real por multipath e
// atenuação sob dossel. O EKF global funde estas medições com peso excessivo,
// causando falha de fusão.
//
// Solução: este nó lê /sensors/gnss/fix, infla a covariância para um mínimo
// de 25 m² horizontal (5 m std) e republica em /sensors/gnss/fix_adapted.
//
// Ref: docs/perception/references/2024_gnss_under_canopy_degradation.md
//   Recetores comuns sob dossel: 5–20 m de erro → cov mínima = 25 m²
//   Covariância otimista é a causa nº1 de falha de fusão.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GnssCovAdapterNode : public rclcpp::Node
{
public:
  GnssCovAdapterNode()
  : Node("gnss_cov_adapter_node")
  {
    input_topic_   = declare_parameter<std::string>("input_topic",  "/sensors/gnss/fix");
    output_topic_  = declare_parameter<std::string>("output_topic", "/sensors/gnss/fix_adapted");
    min_h_var_     = declare_parameter<double>("min_horizontal_var_m2", 25.0);
    min_v_var_     = declare_parameter<double>("min_vertical_var_m2",   49.0);
    hdop_base_var_ = declare_parameter<double>("hdop_base_var_m2",      4.0);
    hdop_scale_en_ = declare_parameter<bool>  ("hdop_scale_enabled",    true);
    trust_larger_  = declare_parameter<bool>  ("trust_driver_if_larger", true);

    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(output_topic_, 10);
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      input_topic_, 10,
      std::bind(&GnssCovAdapterNode::on_fix, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "gnss_cov_adapter: %s → %s  min_h=%.1f m²  min_v=%.1f m²",
      input_topic_.c_str(), output_topic_.c_str(), min_h_var_, min_v_var_);
  }

private:
  void on_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    auto out = *msg;  // cópia — só altera a covariância

    // Determinar covariância horizontal alvo
    double target_h = min_h_var_;
    if (hdop_scale_en_) {
      // NavSatFix não tem campo HDOP; se o driver pôs posição_covariance[0] como
      // hdop²*sigma², tentar inferir de lá. Caso geral: usar mínimo conservador.
      // TODO: se o driver publicar HDOP num campo custom, usar aqui.
      target_h = min_h_var_;
    }

    const double raw_h = msg->position_covariance[0];  // xx diagonal
    const double raw_y = msg->position_covariance[4];  // yy diagonal
    const double raw_v = msg->position_covariance[8];  // zz diagonal

    double final_h, final_y, final_v;
    if (trust_larger_) {
      final_h = std::max(raw_h, target_h);
      final_y = std::max(raw_y, target_h);
      final_v = std::max(raw_v, min_v_var_);
    } else {
      final_h = target_h;
      final_y = target_h;
      final_v = min_v_var_;
    }

    // NavSatFix position_covariance é row-major [xx,xy,xz; yx,yy,yz; zx,zy,zz]
    out.position_covariance[0] = final_h;
    out.position_covariance[4] = final_y;
    out.position_covariance[8] = final_v;
    // Termos cruzados a zero (diagonal conservador)
    out.position_covariance[1] = 0.0;
    out.position_covariance[2] = 0.0;
    out.position_covariance[3] = 0.0;
    out.position_covariance[5] = 0.0;
    out.position_covariance[6] = 0.0;
    out.position_covariance[7] = 0.0;
    out.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_h_var_{25.0};
  double min_v_var_{49.0};
  double hdop_base_var_{4.0};
  bool   hdop_scale_en_{true};
  bool   trust_larger_{true};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssCovAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
