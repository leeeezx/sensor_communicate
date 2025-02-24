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
    
    """***************三轴一起********************"""
    # # 创建三轴力传感器实例
    # sensor = ThreeAxisForceSensor(
    #     port_x='/dev/ttyUSB0',
    #     port_y='/dev/ttyUSB1',
    #     port_z='/dev/ttyUSB2'
    # )
    
    # # 创建发布者节点
    # publisher = ForceSensorPublisher(sensor)
    
    # try:
    #     publisher.read_and_publish()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     sensor.close()
    #     rclpy.shutdown()

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
