import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicReader(Node):
    def __init__(self):
        super().__init__('ultrasonic_reader')
        self.get_logger().info('ultrasonic_reader node initialized.')

        '''
            Espera receber pelo menos 1 dos 4 topicos do albot em um tipo de mensagem
            Range, em que nele possui os atributos:
        
            header: Cabeçalho da mensagem que contém informações de tempo
            e identificação do quadro de referência.

            radiation_type: Tipo de radiação utilizada pelo sensor para medir 
            a distância.

            field_of_view: Ângulo de campo de visão do sensor em radianos. 
            Indica o alcance angular da leitura.

            min_range: Distância mínima que o sensor pode medir de forma confiável.

            max_range: Distância máxima que o sensor pode medir.

            range: Valor da distância medida pelo sensor em metros.
        
        
        '''


        self.sub_back_left = self.create_subscription(
            Range, '/distance/back_left_sonar', self.back_left_callback, 10)
        self.sub_back_right = self.create_subscription(
            Range, '/distance/back_right_sonar', self.back_right_callback, 10)
        self.sub_front_left = self.create_subscription(
            Range, '/distance/front_left_sonar', self.front_left_callback, 10)
        self.sub_front_right = self.create_subscription(
            Range, '/distance/front_right_sonar', self.front_right_callback, 10)

    def back_left_callback(self, msg):
        self.process_range(msg, 'Back Left')

    def back_right_callback(self, msg):
        self.process_range(msg, 'Back Right')

    def front_left_callback(self, msg):
        self.process_range(msg, 'Front Left')

    def front_right_callback(self, msg):
        self.process_range(msg, 'Front Right')

    def process_range(self, msg, sensor_name):
        distance = msg.range

        if distance < 1.0:
            print(sensor_name + ': Objeto Próximo')
        else:
            print(sensor_name + ': Nenhum Objeto Próximo')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()