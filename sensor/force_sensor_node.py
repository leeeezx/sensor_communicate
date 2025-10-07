import rclpy
import threading
from .force_sensor import ThreeAxisForceSensor, ForceSensorPublisher, AsciiSendModel, SingleAxisPublisher

def run_publisher(publisher):
    try:
        publisher.read_and_publish()
    except KeyboardInterrupt:
        pass

def main():
    rclpy.init()
    
    # 创建一个临时节点来获取参数
    temp_node = rclpy.create_node('temp_param_node')
    
    # 声明参数
    temp_node.declare_parameter('axis', '')
    temp_node.declare_parameter('port', '')
    
    # 获取参数
    axis = temp_node.get_parameter('axis').value
    port = temp_node.get_parameter('port').value
    
    # 销毁临时节点
    temp_node.destroy_node()
    
    if axis and port:
        # 单轴模式：基于启动参数创建单个传感器
        sensor = AsciiSendModel(port_name=port)
        publisher = SingleAxisPublisher(axis, sensor)
        print(f"{axis} 轴传感器连接成功")

        
        try:
            publisher.read_and_publish()
        except KeyboardInterrupt:
            pass
        finally:
            sensor.close()
            rclpy.shutdown()
    else:
        # 默认模式：三轴分开运行（保持原有逻辑）
        """***************三轴分开********************"""
        # 创建三个独立的传感器实例
        sensor_x = AsciiSendModel(port_name='/dev/ttyUSB0')
        sensor_y = AsciiSendModel(port_name='/dev/ttyUSB1')
        sensor_z = AsciiSendModel(port_name='/dev/ttyUSB2')
        
        # 为每个轴创建独立的发布者
        publisher_x = SingleAxisPublisher('x', sensor_x)
        publisher_y = SingleAxisPublisher('y', sensor_y)
        publisher_z = SingleAxisPublisher('z', sensor_z)
        
        # 创建三个线程运行发布者
        threads = []
        for pub in [publisher_x, publisher_y, publisher_z]:
            thread = threading.Thread(target=run_publisher, args=(pub,))
            thread.daemon = True
            thread.start()
            threads.append(thread)
        
        try:
            # 主线程等待
            for thread in threads:
                thread.join()
        except KeyboardInterrupt:
            pass
        finally:
            sensor_x.close()
            sensor_y.close()
            sensor_z.close()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
