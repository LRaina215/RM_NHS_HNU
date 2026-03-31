#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import SerialReceiveData, GimbalCmd, Measurement
from sensor_msgs.msg import Imu
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import numpy as np
import random
import time
import subprocess
import os

# 定义QoS配置，提高数据接收可靠性
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class YawDataFusionNode(Node):
    def __init__(self):
        super().__init__('yaw_data_fusion_node')
        
        # 配置参数
        self.declare_parameter('imu_topics', ['/serial_receive_data', '/imu'])
        self.declare_parameter('gimbal_cmd_topic', 'armor_solver/cmd_gimbal')
        self.declare_parameter('measurement_topic', 'armor_solver/measurement')
        self.declare_parameter('fusion_rate', 5.0)  # Hz，降低频率从10Hz到5Hz
        self.declare_parameter('imu_weights', [0.6, 0.4])  # 各IMU的权重
        self.declare_parameter('enable_simulation', False)  # 是否启用模拟模式
        self.declare_parameter('simulation_amplitude', 10.0)  # 模拟数据的振幅
        self.declare_parameter('simulation_frequency', 0.5)  # 模拟数据的频率 (Hz)
        self.declare_parameter('enable_force_wakeup', True)  # 是否启用强制唤醒功能，默认改为True
        self.declare_parameter('imu_device_path', '/dev/ttyACM0')  # IMU设备路径
        self.declare_parameter('wakeup_time_threshold', 10.0)  # 唤醒时间阈值（秒）
        self.declare_parameter('wakeup_interval', 5.0)  # 唤醒尝试间隔（秒）
        
        # 获取参数值
        self.imu_topics = self.get_parameter('imu_topics').get_parameter_value().string_array_value
        self.gimbal_cmd_topic = self.get_parameter('gimbal_cmd_topic').get_parameter_value().string_value
        self.measurement_topic = self.get_parameter('measurement_topic').get_parameter_value().string_value
        fusion_rate = self.get_parameter('fusion_rate').get_parameter_value().double_value
        self.imu_weights = self.get_parameter('imu_weights').get_parameter_value().double_array_value
        self.enable_simulation = self.get_parameter('enable_simulation').get_parameter_value().bool_value
        self.simulation_amplitude = self.get_parameter('simulation_amplitude').get_parameter_value().double_value
        self.simulation_frequency = self.get_parameter('simulation_frequency').get_parameter_value().double_value
        self.enable_force_wakeup = self.get_parameter('enable_force_wakeup').get_parameter_value().bool_value
        self.imu_device_path = self.get_parameter('imu_device_path').get_parameter_value().string_value
        self.wakeup_time_threshold = self.get_parameter('wakeup_time_threshold').get_parameter_value().double_value
        self.wakeup_interval = self.get_parameter('wakeup_interval').get_parameter_value().double_value
        
        # 模拟模式相关变量
        self.simulation_time = 0.0
        self.last_simulation_update = self.get_clock().now()
        
        # 数据有效性检查阈值（秒）
        self.data_timeout = 0.5  # 数据超过0.5秒未更新则视为无效
        
        # 初始化数据存储结构
        self.imu_subscriptions = []
        self.topic_received_count = {}
        self.last_topic_receive_time = {}
        self.topic_connection_status = {}
        self.imu_data = {}
        
        # 存储最新的Gimbal和Measurement数据
        self.latest_gimbal_yaw = 0.0
        self.latest_measurement_yaw = 0.0
        self.gimbal_valid = False
        self.measurement_valid = False
        
        # 标记是否接收到过任何数据
        self.received_any_data = False
        
        # 数据存储历史，用于滤波
        self.yaw_history = {'imu': [], 'gimbal': [], 'fusion': []}
        self.history_length = 10  # 历史数据长度
        
        # 角速度积分相关变量
        self.last_integration_time = self.get_clock().now()  # 上次积分时间
        self.angular_velocity_integral = 0.0  # 角速度积分值
        self.integral_weight = 0.3  # 积分结果在最终融合中的权重
        
        # 唤醒相关变量初始化
        self.start_time = time.time()
        self.last_wakeup_attempt = 0.0
        self.wakeup_attempted = False
        
        # 发布者
        self.fusion_marker_publisher = self.create_publisher(
            MarkerArray,
            'yaw_fusion/markers',
            10)
        
        # 发布测试日志数据
        self.test_log_publisher = self.create_publisher(
            Point,
            'yaw_fusion/test_log',
            10)
        
        # 发布诊断信息
        self.diagnostics_publisher = self.create_publisher(
            Point,
            'yaw_fusion/diagnostics',
            10)
        
        # 创建一个定时器，定期发布融合结果
        self.timer = self.create_timer(1.0 / fusion_rate, self.timer_callback)
        
        # 创建一个定时器，定期检查话题连接状态
        self.topic_monitor_timer = self.create_timer(1.0, self.topic_monitor_callback)
        
        # 初始化订阅
        self.initialize_subscriptions()
        
        # 启动诊断
        self.get_logger().info(f'Yaw Data Fusion Node started with {len(self.imu_topics)} IMU topics')
        self.log_topics_status()
        
        if self.enable_simulation:
            self.get_logger().info(f'Simulation mode enabled: amplitude={self.simulation_amplitude}, frequency={self.simulation_frequency}Hz')
        else:
            self.get_logger().info('Simulation mode disabled. Waiting for actual IMU data...')
            # 提供故障排除建议
            self.provide_troubleshooting_advice()
            
            # 如果启用了强制唤醒功能，在启动时尝试唤醒一次
            if self.enable_force_wakeup:
                self.force_wakeup_imu()
    
    def force_wakeup_imu(self):
        """尝试强制唤醒IMU设备"""
        current_time = time.time()
        
        # 检查是否已经在指定时间间隔内尝试过唤醒
        if self.last_wakeup_attempt > 0 and current_time - self.last_wakeup_attempt < self.wakeup_interval:
            return
        
        self.last_wakeup_attempt = current_time
        self.wakeup_attempted = True
        
        try:
            self.get_logger().info(f'尝试强制唤醒IMU设备: {self.imu_device_path}')
            
            # 检查设备是否存在
            if not os.path.exists(self.imu_device_path):
                self.get_logger().warn(f'IMU设备不存在: {self.imu_device_path}')
                return
            
            # 发送唤醒命令：echo -ne '\x00' > /dev/ttyACM0
            subprocess.run(
                ['echo', '-ne', '\x00'],
                stdout=open(self.imu_device_path, 'w'),
                check=True,
                timeout=2
            )
            
            self.get_logger().info(f'成功发送唤醒命令到 {self.imu_device_path}')
            
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'发送唤醒命令超时: {self.imu_device_path}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'发送唤醒命令失败: {str(e)}')
        except PermissionError:
            self.get_logger().error(f'没有权限访问IMU设备: {self.imu_device_path}。请使用sudo运行或检查设备权限。')
        except Exception as e:
            self.get_logger().error(f'强制唤醒IMU时出错: {str(e)}')
    
    def timer_callback(self):
        current_time = self.get_clock().now()
        current_time_sec = time.time()
        
        # 如果启用了模拟模式，生成模拟数据
        if self.enable_simulation:
            self.generate_simulation_data()
        
        # 打印各话题的接收状态
        for topic, count in self.topic_received_count.items():
            last_receive_time_diff = (current_time - self.last_topic_receive_time[topic]).nanoseconds / 1e9
            self.get_logger().info(
                f'Topic {topic}: received {count} messages, ' \
                f'last received {last_receive_time_diff:.2f}s ago'
            )
        
        
        # 计算有效的IMU数据数量和加权平均值
        valid_imu_count = 0
        weighted_imu_sum = 0.0
        total_weight = 0.0
        
        # 收集所有有效的IMU yaw数据
        imu_yaw_values = []
        # 用于记录角速度突破阈值的信息
        gyr_z_threshold_exceeded = False
        # 收集有效的角速度数据用于积分
        valid_gyr_data = []
        
        for i, (topic, data_info) in enumerate(self.imu_data.items()):
            # 打印每个IMU的当前状态，以便调试
            time_diff = current_time - data_info['timestamp']
            
            # 检查是否有角速度信息（如果有）
            gyr_z_raw = data_info.get('gyr_z_raw', 0.0)
            gyr_z = data_info.get('gyr_z', 0.0)
            
            # 打印原始gyr_z_raw和转换后的gyr_z值
            self.get_logger().info(
                f'IMU[{i}] {topic}: yaw={data_info["yaw"]:.4f}, ' \
                f'gyr_z_raw={gyr_z_raw:.4f}, gyr_z={gyr_z:.4f}°/s, ' \
                f'time_diff={time_diff.nanoseconds/1e9:.4f}s, ' \
                f'valid={data_info["valid"]}'
            )
            
            # 检测角速度是否突破0.1°/s阈值
            if abs(gyr_z) > 0.1:
                gyr_z_threshold_exceeded = True
            
            # 放宽有效性检查，即使值为0也视为有效
            if self.is_data_valid(data_info):
                yaw_value = data_info['yaw']
                weight = self.imu_weights[i]
                
                imu_yaw_values.append(yaw_value)
                weighted_imu_sum += yaw_value * weight
                total_weight += weight
                valid_imu_count += 1
                
                # 收集有效的角速度数据
                if 'gyr_z' in data_info:
                    valid_gyr_data.append(data_info['gyr_z'])
        
        # 如果没有收到任何IMU数据且未启用模拟模式，提供故障排除建议
        if valid_imu_count == 0 and not self.enable_simulation:
            # 如果启用了强制唤醒功能，且满足唤醒条件，则尝试唤醒
            if self.enable_force_wakeup:
                # 条件1: 启动后超过指定时间未收到数据
                startup_time_passed = current_time_sec - self.start_time > self.wakeup_time_threshold
                # 条件2: 已经很久没有尝试唤醒了
                time_since_last_wakeup = current_time_sec - self.last_wakeup_attempt > self.wakeup_interval
                
                if startup_time_passed and time_since_last_wakeup:
                    self.force_wakeup_imu()
                    
            if not self.received_any_data:
                self.get_logger().warn('从未收到过任何IMU数据，请检查话题连接和数据发布')
            else:
                self.get_logger().warn(f'当前没有有效的IMU数据 (最后一次接收数据已超过{self.data_timeout}秒)')
            self.provide_troubleshooting_advice()
            return
        
        # 计算加权平均
        if total_weight > 0:
            weighted_avg_yaw = weighted_imu_sum / total_weight
        else:
            weighted_avg_yaw = 0.0
            self.get_logger().warn('没有有效的IMU数据用于融合')
            return
        
        # 应用低通滤波
        filtered_imu_yaw = self.apply_low_pass_filter('imu', weighted_avg_yaw)
        
        # 如果有Gimbal数据，进行进一步融合
        if self.gimbal_valid:
            # 应用低通滤波到Gimbal数据
            filtered_gimbal_yaw = self.apply_low_pass_filter('gimbal', self.latest_gimbal_yaw)
            
            # 计算IMU和Gimbal之间的差异
            imu_gimbal_diff = abs(filtered_imu_yaw - filtered_gimbal_yaw)
            
            # 根据差异动态调整权重
            # 如果差异小，给Gimbal更高权重；如果差异大，给IMU更高权重
            if imu_gimbal_diff < 5.0:  # 差异小于5度
                fusion_weight_imu = 0.4
                fusion_weight_gimbal = 0.6
            elif imu_gimbal_diff < 10.0:  # 差异在5-10度之间
                fusion_weight_imu = 0.6
                fusion_weight_gimbal = 0.4
            else:  # 差异大于10度，可能有一个传感器异常
                # 使用中位数作为鲁棒估计
                all_values = [filtered_imu_yaw, filtered_gimbal_yaw]
                if self.measurement_valid:
                    all_values.append(self.latest_measurement_yaw)
                fusion_yaw = np.median(all_values)
                fusion_weight_imu = 0.7
                fusion_weight_gimbal = 0.3
            
            # 计算最终融合结果
            fusion_yaw = filtered_imu_yaw * fusion_weight_imu + filtered_gimbal_yaw * fusion_weight_gimbal
            
            # 记录详细信息
            self.get_logger().info(
                f'IMU Yaw: {filtered_imu_yaw:.4f}, ' \
                f'Gimbal Yaw: {filtered_gimbal_yaw:.4f}, ' \
                f'Diff: {imu_gimbal_diff:.4f}, ' \
                f'Fusion Yaw: {fusion_yaw:.4f}, ' \
                f'Valid IMUs: {valid_imu_count}/{len(self.imu_topics)}'
            )
        else:
            # 没有Gimbal数据，仅使用IMU数据
            fusion_yaw = filtered_imu_yaw
            filtered_gimbal_yaw = 0.0
            imu_gimbal_diff = 0.0
            
            self.get_logger().info(
                f'IMU Yaw: {filtered_imu_yaw:.4f}, ' \
                f'Gimbal Yaw: N/A, ' \
                f'Fusion Yaw: {fusion_yaw:.4f}, ' \
                f'Valid IMUs: {valid_imu_count}/{len(self.imu_topics)}'
            )
        
        # 计算角速度积分值
        if valid_gyr_data:
            # 计算时间差（秒）
            time_diff_sec = (current_time - self.last_integration_time).nanoseconds / 1e9
            
            # 防止时间差过大（可能是系统暂停或其他原因）
            if time_diff_sec > 0.1:  # 如果时间差超过0.1秒，重置积分
                self.angular_velocity_integral = fusion_yaw
                self.last_integration_time = current_time
            else:
                # 计算平均角速度
                avg_gyr_z = sum(valid_gyr_data) / len(valid_gyr_data)
                
                # 更新积分值 (角速度 * 时间)
                self.angular_velocity_integral += avg_gyr_z * time_diff_sec
                
                # 更新积分时间
                self.last_integration_time = current_time
            
            # 记录积分信息
            self.get_logger().info(
                f'角速度积分: {self.angular_velocity_integral:.4f}°, ' \
                f'平均角速度: {sum(valid_gyr_data)/len(valid_gyr_data):.4f}°/s, ' \
                f'积分时间差: {time_diff_sec:.4f}s'
            )
            
            # 将积分结果与融合结果进行加权融合
            final_fusion_yaw = fusion_yaw * (1 - self.integral_weight) + self.angular_velocity_integral * self.integral_weight
        else:
            # 没有有效的角速度数据，仅使用原始融合结果
            final_fusion_yaw = fusion_yaw
        
        # 应用低通滤波到最终融合结果
        final_fusion_yaw = self.apply_low_pass_filter('fusion', final_fusion_yaw)
        
        # 发布测试日志数据，便于在PlotJuggler中查看
        log_msg = Point()
        log_msg.x = filtered_imu_yaw
        log_msg.y = filtered_gimbal_yaw if self.gimbal_valid else float('nan')
        log_msg.z = final_fusion_yaw
        self.test_log_publisher.publish(log_msg)
        
        # 创建可视化标记
        self.create_and_publish_markers(filtered_imu_yaw, filtered_gimbal_yaw, final_fusion_yaw)
    
    def provide_troubleshooting_advice(self):
        """提供IMU数据接收故障排除建议"""
        self.get_logger().info("=== IMU数据接收故障排除建议 ===")
        self.get_logger().info("1. 请确认IMU节点正在运行")
        self.get_logger().info(f"   - 检查命令: ros2 node list | grep -i imu")
        self.get_logger().info("2. 使用 'ros2 topic list' 命令检查可用的话题")
        self.get_logger().info("3. 使用 'ros2 topic echo <topic_name>' 命令验证话题是否有数据发布")
        self.get_logger().info("   - 例如: ros2 topic echo /serial_receive_data")
        self.get_logger().info("   - 或: ros2 topic echo /imu")
        self.get_logger().info("4. 确认IMU话题名称与配置的话题名称匹配")
        self.get_logger().info(f"   - 当前配置的IMU话题: {self.imu_topics}")
        self.get_logger().info("5. 检查IMU节点和本节点的QoS配置是否兼容")
        self.get_logger().info("   - 当前QoS: Reliable, KeepLast(10)")
        self.get_logger().info("6. 检查话题的消息类型是否匹配")
        self.get_logger().info("   - /imu 应使用 sensor_msgs/msg/Imu 类型")
        self.get_logger().info("   - /serial_receive_data 应使用自定义的SerialReceiveData类型")
        self.get_logger().info("7. 增加日志级别以获取更详细的调试信息")
        self.get_logger().info("   - 修改启动命令: ros2 run yaw_data_fusion yaw_fusion_node --ros-args --log-level debug")
        self.get_logger().info("8. 检查网络连接和防火墙设置")
        self.get_logger().info("9. 尝试使用RMW实现切换")
        self.get_logger().info("   - 例如: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
        self.get_logger().info("\n10. 检查IMU设备物理连接和权限")
        self.get_logger().info("    - 确认设备已正确连接: ls -l /dev/ttyACM*")
        self.get_logger().info("    - 检查设备权限: ls -la /dev/ttyACM0")
        self.get_logger().info("    - 若权限不足: sudo chmod 666 /dev/ttyACM0")
        self.get_logger().info("11. 直接读取IMU设备进行裸数据测试")
        self.get_logger().info("    - 杀死占用进程: sudo fuser -k /dev/ttyACM0")
        self.get_logger().info("    - 裸读验证广播: stty -F /dev/ttyACM0 115200 raw -echo && dd if=/dev/ttyACM0 bs=20 count=3 status=none | hexdump -C")
        self.get_logger().info("12. 执行IMU唤醒指令")
        self.get_logger().info("    - 一次性唤醒: echo -ne '\\x00' > /dev/ttyACM0")
        self.get_logger().info("    - 注意：仅上电后需要执行一次")
        self.get_logger().info("13. 使用专用调试工具进行实时解析测试")
        self.get_logger().info("    - 运行: python3 imu_debug.py")
        self.get_logger().info("    - 带零偏校准: python3 imu_debug.py --calibrate")
        self.get_logger().info("14. 验证IMU数据解析是否正确")
        self.get_logger().info("    - 帧格式: 20 B 固定，小端: 01 20 19 | counter(3) | reserved(3) | 0x44D4(2) | data1(3) | data2(3) | 0xFFFF(2)")
        self.get_logger().info("    - 比例系数: acc = (data1>>8)/4096.0 g, gyr = (data2>>8)/16.0 °/s")
        self.get_logger().info("15. 进行快速晃动测试")
        self.get_logger().info("    - 快速晃动IMU，观察角速度值是否突破0.1 °/s")
        self.get_logger().info("    - 若始终为0，检查解析算法或硬件量程")
        self.get_logger().info("    - 若有值但yaw仍为0，检查融合节点的零偏阈值")
        self.get_logger().info("===============================")
    
    def topic_monitor_callback(self):
        """定期检查话题连接状态"""
        # 打印话题状态概览
        self.print_topics_status_summary()
        
        # 检查是否需要强制唤醒IMU（如果启用了强制唤醒功能）
        if self.enable_force_wakeup:
            current_time = time.time()
            time_since_start = current_time - self.start_time
            
            # 检查是否满足唤醒条件：启动时间超过阈值且距离上次唤醒超过间隔
            if time_since_start > self.wakeup_time_threshold and \
               (not self.wakeup_attempted or current_time - self.last_wakeup_attempt > self.wakeup_interval):
                
                # 检查是否有IMU数据被成功接收
                has_received_imu_data = any(self.topic_received_count.get(topic, 0) > 0 for topic in self.imu_topics)
                
                # 如果没有接收到IMU数据，尝试唤醒
                if not has_received_imu_data:
                    self.force_wakeup_imu()
                    
    def initialize_subscriptions(self):
        """初始化所有话题订阅"""
        # 初始化IMU话题订阅
        for topic in self.imu_topics:
            # 初始化话题相关数据结构
            self.topic_received_count[topic] = 0
            self.last_topic_receive_time[topic] = self.get_clock().now()
            self.topic_connection_status[topic] = False
            
            # 初始化IMU数据存储
            self.imu_data[topic] = {
                'yaw': 0.0,
                'timestamp': self.get_clock().now(),
                'valid': False
            }
            
            # 根据话题名称判断消息类型并创建订阅
            if topic == '/imu':
                # /imu 话题使用 sensor_msgs/msg/Imu 类型
                self.imu_subscriptions.append(
                    self.create_subscription(
                        Imu,
                        topic,
                        self.create_imu_callback(topic),
                        qos_profile
                    )
                )
            else:
                # 其他IMU话题（如/serial_receive_data）使用自定义的SerialReceiveData类型
                self.imu_subscriptions.append(
                    self.create_subscription(
                        SerialReceiveData,
                        topic,
                        self.create_imu_callback(topic),
                        qos_profile
                    )
                )
        
        # 初始化Gimbal命令话题订阅
        self.create_subscription(
            GimbalCmd,
            self.gimbal_cmd_topic,
            self.gimbal_cmd_callback,
            qos_profile
        )
        
        # 初始化Measurement话题订阅
        self.create_subscription(
            Measurement,
            self.measurement_topic,
            self.measurement_callback,
            qos_profile
        )
        
        self.get_logger().info(f'已初始化 {len(self.imu_subscriptions)} 个话题订阅')
        
    def create_imu_callback(self, topic):
        """为指定话题创建IMU数据回调函数"""
        def callback(msg):
            current_time = self.get_clock().now()
            
            # 更新接收计数和最后接收时间
            self.topic_received_count[topic] += 1
            self.last_topic_receive_time[topic] = current_time
            self.topic_connection_status[topic] = True
            
            # 标记已接收到数据
            if not self.received_any_data:
                self.received_any_data = True
                self.get_logger().info(f'成功接收到来自 {topic} 的IMU数据')
            
            # 处理不同类型的IMU消息
            if isinstance(msg, Imu):
                # 处理sensor_msgs/msg/Imu类型的消息
                # 从四元数计算yaw角
                q = msg.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi
                
                # 获取角速度（绕z轴）
                gyr_z = msg.angular_velocity.z * 180.0 / math.pi  # 转换为度/秒
                
                self.imu_data[topic] = {
                    'yaw': yaw,
                    'gyr_z_raw': msg.angular_velocity.z,
                    'gyr_z': gyr_z,
                    'timestamp': current_time,
                    'valid': True
                }
            else:
                # 处理自定义的SerialReceiveData类型消息
                # 假设消息中包含yaw数据和角速度数据
                # 这里根据实际的消息结构进行调整
                try:
                    # 尝试从消息中提取yaw数据
                    if hasattr(msg, 'yaw'):
                        yaw = msg.yaw
                    else:
                        # 如果没有直接的yaw字段，尝试从其他字段计算
                        yaw = 0.0  # 默认值，实际应用中需要根据消息结构调整
                    
                    # 尝试提取角速度数据
                    gyr_z_raw = 0.0
                    gyr_z = 0.0
                    if hasattr(msg, 'gyr_z'):
                        gyr_z_raw = msg.gyr_z
                        gyr_z = gyr_z_raw  # 假设已经是度/秒单位
                    elif hasattr(msg, 'data2'):
                        # 假设data2字段包含角速度信息
                        # 根据之前的故障排除建议，这里假设比例系数为16.0
                        gyr_z_raw = msg.data2
                        gyr_z = (gyr_z_raw >> 8) / 16.0  # 转换为度/秒
                    
                    self.imu_data[topic] = {
                        'yaw': yaw,
                        'gyr_z_raw': gyr_z_raw,
                        'gyr_z': gyr_z,
                        'timestamp': current_time,
                        'valid': True
                    }
                except Exception as e:
                    self.get_logger().error(f'处理IMU数据时出错: {str(e)}')
                    self.imu_data[topic] = {
                        'yaw': 0.0,
                        'timestamp': current_time,
                        'valid': False
                    }
        
        return callback
        
    def gimbal_cmd_callback(self, msg):
        """处理Gimbal命令消息"""
        try:
            # 假设Gimbal命令中包含yaw数据
            if hasattr(msg, 'yaw'):
                self.latest_gimbal_yaw = msg.yaw
                self.gimbal_valid = True
                self.last_gimbal_time = self.get_clock().now()
                self.get_logger().debug(f'接收到Gimbal命令: yaw={msg.yaw}')
        except Exception as e:
            self.get_logger().error(f'处理Gimbal命令时出错: {str(e)}')
            self.gimbal_valid = False
            
    def measurement_callback(self, msg):
        """处理Measurement消息"""
        try:
            # 假设Measurement消息中包含yaw或角度数据
            # 这里根据实际的消息结构进行调整
            if hasattr(msg, 'yaw'):
                self.latest_measurement_yaw = msg.yaw
                self.measurement_valid = True
                self.last_measurement_time = self.get_clock().now()
                self.get_logger().debug(f'接收到测量数据: yaw={msg.yaw}')
        except Exception as e:
            self.get_logger().error(f'处理测量数据时出错: {str(e)}')
            self.measurement_valid = False
            
    def log_topics_status(self):
        """记录所有话题的状态"""
        self.get_logger().info("=== 初始化话题状态 ===")
        self.get_logger().info(f'IMU话题: {self.imu_topics}')
        self.get_logger().info(f'Gimbal命令话题: {self.gimbal_cmd_topic}')
        self.get_logger().info(f'测量话题: {self.measurement_topic}')
        self.get_logger().info(f'融合频率: {self.get_parameter("fusion_rate").get_parameter_value().double_value} Hz')
        self.get_logger().info(f'是否启用强制唤醒: {self.enable_force_wakeup}')
        if self.enable_force_wakeup:
            self.get_logger().info(f'IMU设备路径: {self.imu_device_path}')
            self.get_logger().info(f'唤醒时间阈值: {self.wakeup_time_threshold} 秒')
            self.get_logger().info(f'唤醒尝试间隔: {self.wakeup_interval} 秒')
        self.get_logger().info("====================")
        
    def generate_simulation_data(self):
        """生成模拟的IMU数据"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_simulation_update).nanoseconds / 1e9
        
        # 更新模拟时间
        self.simulation_time += time_diff
        self.last_simulation_update = current_time
        
        # 生成模拟的yaw数据（正弦波）
        simulation_yaw = self.simulation_amplitude * math.sin(2 * math.pi * self.simulation_frequency * self.simulation_time)
        
        # 生成模拟的角速度数据（yaw的导数）
        simulation_gyr_z = self.simulation_amplitude * 2 * math.pi * self.simulation_frequency * math.cos(2 * math.pi * self.simulation_frequency * self.simulation_time)
        
        # 为每个IMU话题生成模拟数据
        for i, topic in enumerate(self.imu_topics):
            # 添加一些随机噪声以区分不同的IMU
            noise = random.uniform(-0.5, 0.5)
            yaw_with_noise = simulation_yaw + noise
            
            # 更新IMU数据
            self.imu_data[topic] = {
                'yaw': yaw_with_noise,
                'gyr_z_raw': simulation_gyr_z / 180.0 * math.pi,  # 弧度/秒
                'gyr_z': simulation_gyr_z,  # 度/秒
                'timestamp': current_time,
                'valid': True
            }
            
            # 更新接收计数和连接状态
            self.topic_received_count[topic] += 1
            self.last_topic_receive_time[topic] = current_time
            self.topic_connection_status[topic] = True
        
        # 生成模拟的Gimbal数据
        if random.random() < 0.8:  # 80%的概率有有效的Gimbal数据
            self.latest_gimbal_yaw = simulation_yaw + random.uniform(-1.0, 1.0)
            self.gimbal_valid = True
        else:
            self.gimbal_valid = False
            
        # 生成模拟的测量数据
        if random.random() < 0.7:  # 70%的概率有有效的测量数据
            self.latest_measurement_yaw = simulation_yaw + random.uniform(-2.0, 2.0)
            self.measurement_valid = True
        else:
            self.measurement_valid = False
            
    def is_data_valid(self, data_info):
        """检查数据是否有效"""
        # 检查数据是否有效（时间差是否在超时阈值内）
        current_time = self.get_clock().now()
        time_diff = (current_time - data_info['timestamp']).nanoseconds / 1e9
        
        # 即使值为0也视为有效，只要时间差在阈值内
        return time_diff < self.data_timeout
        
    def apply_low_pass_filter(self, data_type, new_value):
        """应用低通滤波"""
        # 简单的移动平均滤波
        self.yaw_history[data_type].append(new_value)
        
        # 保持历史数据长度
        if len(self.yaw_history[data_type]) > self.history_length:
            self.yaw_history[data_type].pop(0)
        
        # 计算平均值作为滤波结果
        filtered_value = sum(self.yaw_history[data_type]) / len(self.yaw_history[data_type])
        
        return filtered_value
    
    def print_topics_status_summary(self):
        """打印话题状态概览"""
        current_time = self.get_clock().now()
        
        # 打印各话题的接收状态概览
        self.get_logger().info("=== 话题状态概览 ===")
        
        # IMU话题状态
        for topic, count in self.topic_received_count.items():
            last_receive_time_diff = (current_time - self.last_topic_receive_time[topic]).nanoseconds / 1e9
            status = "正常" if self.topic_connection_status.get(topic, False) else "异常"
            
            self.get_logger().info(
                f'IMU话题 {topic}: 接收 {count} 条消息, ' \
                f'最后接收 {last_receive_time_diff:.2f}s前, ' \
                f'状态: {status}'
            )
        
        # 其他话题状态
        self.get_logger().info(f'Gimbal命令: 最后有效数据: {self.gimbal_valid}')
        self.get_logger().info(f'测量数据: 最后有效数据: {self.measurement_valid}')
        
        self.get_logger().info("====================")
        
        # 打印IMU数据详情
        self.print_imu_data_details()
    
    def print_imu_data_details(self):
        """打印IMU数据详情"""
        current_time = self.get_clock().now()
        
        for i, (topic, data_info) in enumerate(self.imu_data.items()):
            # 计算时间差
            time_diff = (current_time - data_info['timestamp']).nanoseconds / 1e9
            
            # 只有在有变化或首次接收时才打印详细信息
            if time_diff < 1.0 or self.topic_received_count.get(topic, 0) <= 5:
                self.get_logger().debug(
                    f'IMU[{i}] {topic}: yaw={data_info["yaw"]:.4f}, ' \
                    f'time_diff={time_diff:.4f}s, ' \
                    f'valid={data_info["valid"]}'
                )
    
    def create_and_publish_markers(self, imu_yaw, gimbal_yaw, fusion_yaw):
        """创建并发布可视化标记"""
        marker_array = MarkerArray()
        
        # IMU Yaw 标记
        imu_marker = self.create_yaw_marker(
            'imu_yaw', 
            imu_yaw, 
            1.0, 0.0, 0.0  # 红色
        )
        
        # Gimbal Yaw 标记
        gimbal_marker = self.create_yaw_marker(
            'gimbal_yaw', 
            gimbal_yaw, 
            0.0, 1.0, 0.0  # 绿色
        )
        
        # 融合后的Yaw 标记
        fusion_marker = self.create_yaw_marker(
            'fusion_yaw', 
            fusion_yaw, 
            0.0, 0.0, 1.0  # 蓝色
        )
        
        # 添加标记到数组
        marker_array.markers.append(imu_marker)
        marker_array.markers.append(gimbal_marker)
        marker_array.markers.append(fusion_marker)
        
        # 发布标记数组
        self.fusion_marker_publisher.publish(marker_array)
    
    def create_yaw_marker(self, ns, yaw_value, r, g, b):
        marker = Marker()
        marker.header.frame_id = 'odom'  # 使用与自瞄系统相同的坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 设置箭头的起始点（原点）
        start_point = Point()
        start_point.x = 0.0
        start_point.y = 0.0
        start_point.z = 0.0
        
        # 设置箭头的终点（根据yaw角度计算）
        end_point = Point()
        end_point.x = math.cos(math.radians(yaw_value))  # 假设箭头长度为1米
        end_point.y = math.sin(math.radians(yaw_value))
        end_point.z = 0.0
        
        # 设置箭头的颜色和大小
        marker.points.append(start_point)
        marker.points.append(end_point)
        marker.scale.x = 0.02  # 箭头宽度
        marker.scale.y = 0.05  # 箭头头部宽度
        marker.scale.z = 0.01  # 箭头厚度
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        
        # 设置箭头的方向
        q = self.get_quaternion_from_euler(0, 0, math.radians(yaw_value))
        marker.pose.orientation = q
        
        # 设置标记的生命周期（1秒）
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        return marker
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        # 将欧拉角转换为四元数
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q

def main(args=None):
    rclpy.init(args=args)
    
    node = YawDataFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()