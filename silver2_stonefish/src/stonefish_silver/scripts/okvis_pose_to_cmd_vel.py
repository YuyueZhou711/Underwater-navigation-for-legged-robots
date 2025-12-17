#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion

class DynamicOkvisController(Node):
    def __init__(self):
        super().__init__('okvis_odometry_to_cmd_vel')

        # ===========================
        # 1. 参数设置 (可动态调整)
        # ===========================
        # PID 增益
        self.kp_linear_x = 0.5   # 前进速度增益
        self.kp_linear_z = 0.5   # 垂直速度增益 (3D扩展)
        self.kp_angular  = 1.5   # 转向增益

        # 最大速度限制 (Stonefish Hexapod 物理限制)
        self.max_linear_x = 0.4  # m/s
        self.max_linear_z = 0.2  # m/s (下潜/上浮)
        self.max_angular  = 0.8  # rad/s

        # 加速度限制 (平滑控制关键: m/s^2)
        self.accel_linear_x = 0.2 
        self.accel_linear_z = 0.1
        self.accel_angular  = 0.5

        # 到达阈值
        self.pos_tolerance = 0.10   # 10cm
        self.yaw_tolerance = 0.05   # ~3度

        # ===========================
        # 2. 状态变量
        # ===========================
        self.current_pose = None # [x, y, z, roll, pitch, yaw]
        self.target_pose  = None # [x, y, z] (目前只存位置，Yaw朝向目标)
        
        # 当前命令缓存 (用于计算平滑加速度)
        self.last_cmd = Twist()
        self.last_time = self.get_clock().now()

        # ===========================
        # 3. ROS 通信接口
        # ===========================
        # 订阅 OKVIS2 里程计 (反馈)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/okvis_odometry',
            self.odom_callback,
            10
        )

        # 订阅 Rviz 导航目标 (动态目标)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Rviz "2D Nav Goal" 发送的话题
            self.goal_callback,
            10
        )

        # 发布速度指令给 Stonefish
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel', 
            10
        )

        # 控制循环 (20Hz)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("🚀 Dynamic Controller Started! Waiting for OKVIS odometry...")
        self.get_logger().info("👉 Use '2D Nav Goal' in Rviz to set a target.")

    def odom_callback(self, msg):
        """ 解析 OKVIS 里程计 """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        
        # 四元数转欧拉角
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.current_pose = {
            'x': p.x, 'y': p.y, 'z': p.z,
            'yaw': yaw
        }

        # 如果还没有目标，初始化目标为当前位置 (保持原地不动)
        if self.target_pose is None:
            self.target_pose = {'x': p.x, 'y': p.y, 'z': p.z}
            self.get_logger().info(f"📍 Initialized target to current position: {p.x:.2f}, {p.y:.2f}, {p.z:.2f}")

    def goal_callback(self, msg):
        """ 接收 Rviz 的动态目标 """
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        # Rviz 2D Goal 通常 z=0，我们保持当前 Z 或者是预设深度
        tz = self.current_pose['z'] if self.current_pose else 0.5 

        self.target_pose = {'x': tx, 'y': ty, 'z': tz}
        self.get_logger().info(f"🎯 New Target Received: X={tx:.2f}, Y={ty:.2f}")

    def control_loop(self):
        """ 主控制循环：计算误差 -> PID -> 平滑 -> 发布 """
        if self.current_pose is None or self.target_pose is None:
            return

        # 1. 计算误差
        dx = self.target_pose['x'] - self.current_pose['x']
        dy = self.target_pose['y'] - self.current_pose['y']
        dz = self.target_pose['z'] - self.current_pose['z']
        
        distance_xy = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_heading - self.current_pose['yaw'])

        # 2. 计算目标速度 (纯 PID)
        target_vel = Twist()

        # 逻辑：只有当角度对齐得差不多时，才全速前进，否则优先旋转 (Point-and-Shoot)
        if distance_xy > self.pos_tolerance:
            # 角度控制
            target_vel.angular.z = self.clamp(self.kp_angular * yaw_error, -self.max_angular, self.max_angular)
            
            # 距离控制 (如果在朝向范围内，则允许前进)
            if abs(yaw_error) < 0.5: # 约30度以内允许前进
                v_x = self.kp_linear_x * distance_xy
                # 距离越近速度越慢，但加一个最小速度防止蠕动，或者是简单的 P 控制
                target_vel.linear.x = self.clamp(v_x, -self.max_linear_x, self.max_linear_x)
            else:
                target_vel.linear.x = 0.0 # 原地旋转
        else:
            # 已到达 XY 目标
            target_vel.linear.x = 0.0
            target_vel.angular.z = 0.0
        
        # Z 轴控制 (3D 扩展: 简单的 P 控制保持深度)
        # 注意：Stonefish 的 hexcopter 或 submarines 通常 linear.z 对应垂直速度
        target_vel.linear.z = self.clamp(self.kp_linear_z * dz, -self.max_linear_z, self.max_linear_z)

        # 3. 速度平滑 (Ramp Control)
        # 避免电机指令突变，保护舵机并使步态更自然
        final_cmd = Twist()
        final_cmd.linear.x = self.ramp_value(self.last_cmd.linear.x, target_vel.linear.x, self.accel_linear_x)
        final_cmd.linear.z = self.ramp_value(self.last_cmd.linear.z, target_vel.linear.z, self.accel_linear_z)
        final_cmd.angular.z = self.ramp_value(self.last_cmd.angular.z, target_vel.angular.z, self.accel_angular)

        # 4. 发布
        self.cmd_pub.publish(final_cmd)
        
        # 更新缓存
        self.last_cmd = final_cmd

        # 调试日志 (可选，每秒打印一次防止刷屏)
        # self.get_logger().info(f"Dist: {distance_xy:.2f}m, YawErr: {yaw_error:.2f}, CmdX: {final_cmd.linear.x:.2f}")

    def ramp_value(self, current, target, max_accel):
        """ 速度平滑函数：限制每帧最大变化量 """
        step = max_accel * self.dt
        if target > current:
            return min(current + step, target)
        elif target < current:
            return max(current - step, target)
        return target

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def clamp(value, min_val, max_val):
        return max(min(value, max_val), min_val)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicOkvisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 发送停止指令
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()