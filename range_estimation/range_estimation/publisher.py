# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# import pandas as pd
# from sklearn.preprocessing import LabelEncoder, StandardScaler
# import numpy as np
# import os
# os.environ['CUDA_VISIBLE_DEVICES'] = '-1'  # 禁用所有 GPU

# class DataPublisher(Node):

#     def __init__(self):
#         super().__init__('data_publisher')
#         self.publisher_ = self.create_publisher(Float64MultiArray, 'features', 10)
#         self.timer = self.create_timer(1, self.timer_callback)
#         self.label_encoders = {}
#         self.scaler = StandardScaler()
#         self.current_index = 0  # 新增索引变量

#         # Load and preprocess the dataset
#         self.dataframe = pd.read_csv('/home/qiren/ros2_ws/src/range_estimation/data/data.csv')
#         # self.dataframe = self.dataframe.dropna(subset=['quantity(kWh)', 'avg_speed(km/h)'])
#         self.dataframe['trip_distance(km)'] = self.dataframe['trip_distance(km)'].apply(self.safe_float_convert)
#         self.dataframe = self.dataframe.dropna(subset=['trip_distance(km)'])

#         # Encode categorical variables
#         categorical_cols = ['manufacturer', 'model', 'version', 'fuel_type', 'tire_type', 'driving_style']
#         for col in categorical_cols:
#             if col in self.dataframe.columns:
#                 self.label_encoders[col] = LabelEncoder()
#                 self.dataframe[col] = self.label_encoders[col].fit_transform(self.dataframe[col])

#         # Extract features
#         self.features = self.dataframe[['power(kW)', 'quantity(kWh)', 'fuel_type', 'tire_type', 'city', 'motor_way',
#                                         'country_roads', 'driving_style', 'A/C', 'park_heating', 'avg_speed(km/h)']]
#         self.features = self.scaler.fit_transform(self.features)

#     def safe_float_convert(self, x):
#         try:
#             return float(x)
#         except ValueError:
#             parts = x.split('.')
#             if len(parts) > 2:
#                 return float(''.join(parts[:-1]) + '.' + parts[-1])
#             return np.nan

#     def timer_callback(self):
#         # Here we publish only one row of features for simplicity
#         # 获取要发布的特征行
#         features_row = self.features[self.row_index % len(self.features)]  # 使用模运算确保索引不超出范围
#         features_msg = Float64MultiArray()
#         features_msg.data = self.features[0].tolist()
#         self.publisher_.publish(features_msg)
#         self.get_logger().info('Publishing: "%s"' % features_msg.data)
#         self.row_index += 1  # 每次调用时递增计数器

# def main(args=None):
#     rclpy.init(args=args)
#     publisher = DataPublisher()
#     rclpy.spin(publisher)
#     publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pandas as pd
from sklearn.preprocessing import LabelEncoder, StandardScaler
import numpy as np
import os
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'  # 禁用所有 GPU

class DataPublisher(Node):

    def __init__(self):
        super().__init__('data_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'features', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.current_index = 0  # 新增索引变量
        self.label_encoders = {}
        self.scaler = StandardScaler()

        # Load and preprocess the dataset
        self.dataframe = pd.read_csv('/home/qiren/ros2_ws/src/range_estimation/data/data.csv')
        self.dataframe['trip_distance(km)'] = self.dataframe['trip_distance(km)'].apply(self.safe_float_convert)
        self.dataframe = self.dataframe.dropna(subset=['trip_distance(km)'])

        # Encode categorical variables
        categorical_cols = ['manufacturer', 'model', 'version', 'fuel_type', 'tire_type', 'driving_style']
        for col in categorical_cols:
            if col in self.dataframe.columns:
                self.label_encoders[col] = LabelEncoder()
                self.dataframe[col] = self.label_encoders[col].fit_transform(self.dataframe[col])

        # Extract features
        self.features = self.dataframe[['power(kW)', 'quantity(kWh)', 'fuel_type', 'tire_type', 'city', 'motor_way',
                                        'country_roads', 'driving_style', 'A/C', 'park_heating', 'avg_speed(km/h)']]
        self.features = self.scaler.fit_transform(self.features)

    def safe_float_convert(self, x):
        try:
            return float(x)
        except ValueError:
            parts = x.split('.')
            if len(parts) > 2:
                return float(''.join(parts[:-1]) + '.' + parts[-1])
            return np.nan

    def timer_callback(self):
        # 从特征数组中获取当前索引的行
        row = self.features[self.current_index % len(self.features)]
        self.current_index += 1  # 更新索引

        features_msg = Float64MultiArray()
        features_msg.data = row.tolist()
        self.publisher_.publish(features_msg)
        self.get_logger().info('Publishing: "%s"' % features_msg.data)

def main(args=None):
    rclpy.init(args=args)
    publisher = DataPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
