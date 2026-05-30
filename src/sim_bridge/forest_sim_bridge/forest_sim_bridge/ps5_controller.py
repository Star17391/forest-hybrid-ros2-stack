import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class PS5MarbleController(Node):

    def __init__(self):
        super().__init__('ps5_marble_controller')

        # Publicadores para os tópicos exatos do ROS que a bridge está à espera
        self.pub_left = self.create_publisher(
            Float64, 
            '/model/marble_hd2/link/left_track/track_cmd_vel', 
            10)
        
        self.pub_right = self.create_publisher(
            Float64, 
            '/model/marble_hd2/link/right_track/track_cmd_vel', 
            10)

        self.create_subscription(
            Joy,
            '/joy',
            self.callback,
            10)
        
        self.get_logger().info("Controlo PS5 para Marble HD2 iniciado!")
        self.get_logger().info("Esquerdo (L) -> Track Esquerda | Direito (R) -> Track Direita")

    def callback(self, msg):
        # Eixos PS5 (Confirma se o analógico direito é o 3 ou 4 com ros2 topic echo /joy)
        left_input = msg.axes[1]   # Analógico Esquerdo
        right_input = msg.axes[4]  # Analógico Direito (tenta 3 se o 4 não responder)

        # O Marble é pesado. Precisamos de valores mais altos para ele girar.
        # Se 10.0 ainda for lento, experimenta 20.0
        multiplier = 15.0 

        # Aplicar uma "Deadzone" manual para evitar que ele ande sozinho 
        # se o comando estiver ligeiramente gasto
        def apply_deadzone(val):
            return val if abs(val) > 0.1 else 0.0

        msg_l = Float64()
        msg_l.data = float(apply_deadzone(left_input) * multiplier)

        msg_r = Float64()
        msg_r.data = float(apply_deadzone(right_input) * multiplier)

        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)
        
        # Debug para veres no terminal se os valores estão a sair independentes
        # self.get_logger().info(f"L: {msg_l.data:.2f} | R: {msg_r.data:.2f}")


def main():
    rclpy.init()
    node = PS5MarbleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()