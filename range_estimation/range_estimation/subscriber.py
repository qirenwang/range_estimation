import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from keras.models import load_model
import numpy as np
import array

class DataSubscriber(Node):

    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'features',
            self.listener_callback,
            10)
        self.subscription

        # Load the neural network model
        self.model = load_model('/home/qiren/ros2_ws/src/range_estimation/models/neural_network_model.h5')

    def listener_callback(self, msg):
        # 将收到的数据从 Float64MultiArray 转换为 NumPy 数组
        features_list = array.array('d', msg.data)  # 'd' 是双精度浮点数
        features = np.array(features_list).reshape(1, -1)  # 转换为二维 NumPy 数组

        # 使用模型进行预测
        prediction = self.model.predict(features)
        predicted_value = prediction[0][0]  # 提取预测值

        # 记录预测结果
        self.get_logger().info('Predicted Trip Distance: {:.2f} km'.format(predicted_value))

def main(args=None):
    rclpy.init(args=args)
    subscriber = DataSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
