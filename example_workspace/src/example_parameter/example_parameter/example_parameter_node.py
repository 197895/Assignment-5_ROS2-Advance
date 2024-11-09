

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import ##节点
import rclpy.parameter                    # ROS2 节点类
from rcl_interfaces.msg import SetParametersResult
class ExampleParameter(Node):
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.timer = self.create_timer(5, ##callback)    # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.declare_parameter##声明参数         # 创建一个参数，并设置参数的默认值
        self.add_on_set_parameters_callback(self.parameters_callback)#添加参数设置回调函数
        
        
    def parameters_callback(self, params)->SetParametersResult:#参数设置回调函数
        result = SetParametersResult(successful=True)#创建参数设置结果对象
        for param in params:
            if param.name ##name怎么样?
                if param.value <0 or param.value >10:
                    result.successful ##如果不满足要求successful应该怎么设置？
                    result.reason = '速度不在0-10之间'
                    self.get_logger().info(f'速度不在0-10之间,参数设置失败')
                    break
                # 参数设置成功，可以在这里添加额外的逻辑
                self.get_logger().info(f'param {param.name} set to {param.value}')
                self.timer.cancel()#取消之前的定时器
                self.timer = self.create_timer(##周期是？, self.timer_callback)#更改定时器的周期
        return result
    def timer_callback(self):                                      # 创建定时器周期执行的回调函数
        
        speed = self.get_parameter('publish_speed').get_parameter_value()##什么类型？##
        self.get_logger().info('现在的发布速度是: %d' % speed)     # 输出日志信息，打印读取到的参数值

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ExampleParameter("my_param")            # 创建ROS2节点对象并进行初始化
    rclpy.spin(##什么)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
if __name__=="__main__":
    main()