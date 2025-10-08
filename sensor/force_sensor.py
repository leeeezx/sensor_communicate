import os
import logging
from typing import Optional, List, Dict, Any, Generator
import serial
import time
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class AsciiSendModel:
    """
    关于传感器ascii模式的类，包括参数设置和工具函数
    """

    def __init__(self,
                 port_name: Optional[str] = None,
                 baudrate: int = 115200,
                 bytesize: int = 8,
                 parity: str = 'N',
                 stopbits: int = 1,
                 timeout: float = 1,
                 minimum_packet_interval: Optional[float] = None,
                 byte_num_of_one_message: int = 7,
                 buffer_size: Optional[int] = None,
                 **kwargs: Any):
        """
        参数初始化

        :param port_name: 串口名称
        :param baudrate: 波特率
        :param bytesize: 数据位
        :param parity: 校验位 ('N', 'E', 'O', 'M', 'S')
        :param stopbits: 停止位
        :param timeout: 超时时间
        :param minimum_packet_interval: 最小包间隔。ascii模式只能保证实际包间隔时间大于仪表的设定值
        :param byte_num_of_one_message: 一个报文的字节数
        :param buffer_size: 串口缓冲区大小
        :param kwargs: 其它参数
        """
        self.port_name = port_name
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.minimum_packet_interval = minimum_packet_interval
        self.byte_num_of_one_message = byte_num_of_one_message

        # 检查额外的参数
        if kwargs:
            raise ValueError('猪头，压根没有这玩意！： {!r}'.format(kwargs))

        # 配置并打开串口
        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            stopbits=self.stopbits,
            timeout=self.timeout
        )

    def close(self) -> None:
        """显式关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")

    def read_sensor_data(self,
                        standard_message_length: int = 7,
                        report_count: int = 50,
                        chunk_size: int = 4096) -> Generator[str, None, None]:
        """
        在ascii通讯模式下读取串口数据：
        1. 首先找到第一个回车符(0D)作为数据同步点
        2. 之后的数据才开始正式接收和解码处理

        :param standard_message_length: 标准通讯模式下，符合标准的默认报文长度（以字节为单位）
        :param report_count: 一次抛出的报文数量限制（已经转换为仪表数值，kg为单位）
        :param chunk_size: 缓冲区大小

        :return: 解码后的报文列表
        """
        if not self.ser.is_open:
            logger.error("串口未打开")
            raise serial.SerialException("串口未打开")

        buffer = bytearray()  # 创建空的字节串
        reports = []  # 创建报文空列表

        while True:  # 进入数据处理循环
            # 读取新数据并添加到buffer
            if self.ser.in_waiting:  # 如果串口中有数据等待
                chunk = self.ser.read(min(self.ser.in_waiting, chunk_size))  
                buffer.extend(chunk)  # 添加到buffer中

            while len(buffer) >= standard_message_length:  # 当buffer超过默认报文长度7
                cr_index = buffer.find(b'\r')  # cr_index作为空格符的索引
                if cr_index == -1 or cr_index < standard_message_length - 1:  
                    break  # 没有完整报文，退出循环

                valid_data = buffer[:cr_index + 1]
                buffer = buffer[cr_index + 1:]  # 更新buffer

                try:
                    # 将采集到的数据按照ascii编码进行解码，转换为str类型，然后去除首尾的空白符
                    decoded_data = valid_data.decode('ascii').strip()
                    yield decoded_data  # 生成器，每次返回一个报文
                except UnicodeDecodeError as e:
                    logging.error(f"解码错误：{e}，丢弃无效数据")

            # 如果buffer过大，可能表示数据积压，清理旧数据
            if len(buffer) > chunk_size * 2:
                buffer = buffer[-chunk_size:]
                logger.warning("数据积压，清理旧数据")

            # 如果没有足够的报文，短暂等待更多数据到达
            if not reports:
                time.sleep(0.02)  # 等待时间，可根据需要调整


class ThreeAxisForceSensor:
    """
    三维力传感器类。如果需要更多维度传感器，可以基于此类进行修改。
    """
    def __init__(self, port_x=None, port_y=None, port_z=None):
        """
        初始化三维力传感器参数

        :param port_x: x轴串口名称
        :param port_y: y轴串口名称
        :param port_z: z轴串口名称
        """
        self.sensors = {
            'x': AsciiSendModel(port_name=port_x),
            'y': AsciiSendModel(port_name=port_y),
            'z': AsciiSendModel(port_name=port_z)
        }
        
    def close(self):
        """
        关闭所有串口
        """
        for sensor in self.sensors.values():
            sensor.close()

class ForceSensorPublisher(Node):
    def __init__(self, three_axis_sensor):
        """
        初始化ROS2发布者节点名称

        :param three_axis_sensor: 三维力传感器对象
        """
        super().__init__('force_sensor_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, 'force_sensor_data', 10)
        self.sensor = three_axis_sensor
        self.force_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
    def read_and_publish(self):
        """读取并发布三轴力数据"""
        while rclpy.ok():
            try:
                # 读取三个方向的数据
                for axis, sensor in self.sensor.sensors.items():
                    for report in sensor.read_sensor_data():
                        try:
                            self.force_data[axis] = float(report)
                            break  # 获取到一个有效数据就继续下一个轴
                        except ValueError as e:
                            self.get_logger().warn(f'{axis}轴数据转换错误: {e}')
                
                # 发布三轴数据
                msg = Float32MultiArray()
                msg.data = [self.force_data['x'], self.force_data['y'], self.force_data['z']]
                self.publisher.publish(msg)

                # # 添加处理间隔
                # time.sleep(0.005)
                
            except Exception as e:
                self.get_logger().error(f'数据读取或发布错误: {e}')


class SingleAxisPublisher(Node):
    def __init__(self, name, sensor):
        super().__init__(f'force_sensor_{name}_publisher')
        # self.publisher = self.create_publisher(Float32, f'force_sensor_{name}', 10)
        self.axis_name = name
        self.publisher = self.create_publisher(WrenchStamped, f'force_sensor_{name}', 10)

        self.sensor = sensor
        
    def read_and_publish(self):
        while rclpy.ok():
            try:
                for data in self.sensor.read_sensor_data():
                    try:
                        # msg = Float32()
                        # msg.data = float(data) * 9.8
                        msg = WrenchStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        setattr(msg.wrench.force, self.axis_name, float(data) * 9.8)
                        self.publisher.publish(msg)
                    except ValueError as e:
                        self.get_logger().warn(f'数据转换错误: {e}')
            except Exception as e:
                self.get_logger().error(f'数据读取错误: {e}')
