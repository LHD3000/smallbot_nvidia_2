#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import os

# 文件路径定义
base_path = "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/"

# 角度相关文件
angle_files = {
    'raw': base_path + "angle_data.txt", 
    'data': base_path + "angle_data.txt",
    'target': base_path + "target_angle_data.txt",
    'timestamp': base_path + "timestamp_angle.txt",
    'target_timestamp': base_path + "timestamp_target_angle.txt"
}

# 速度相关文件
speed_files = {
    'raw': base_path + "speed_raw_data.txt",   
    'data': base_path + "speed_data.txt",
    'target': base_path + "target_speed_data.txt", 
    'timestamp': base_path + "timestamp_speed.txt",
    'target_timestamp': base_path + "timestamp_target_speed.txt"
}

# 角速度相关文件
gyro_files = {
    'raw': base_path + "gyro_data.txt", 
    'data': base_path + "gyro_data.txt",
    'target': base_path + "target_gyro_data.txt",
    'timestamp': base_path + "timestamp_gyro.txt",
    'target_timestamp': base_path + "timestamp_target_gyro.txt"
}
# RPM相关文件
rpm_files = {
    'left': base_path+"left_rpm_data.txt",
    'right': base_path+"right_rpm_data.txt",
    'timestamp': base_path+"timestamp_rpm.txt"
}
# 检查所有必需文件是否存在
def check_files_exist(files_dict, data_type):
    """检查指定类型的所有文件是否存在"""
    required_files = [
        (files_dict['data'], f"{data_type}_data"),
        (files_dict['target'], f"target_{data_type}_data"),
        (files_dict['timestamp'], f"timestamp_{data_type}"),
        (files_dict['target_timestamp'], f"timestamp_target_{data_type}")
    ]
    
    missing_files = []
    for file_path, name in required_files:
        if not os.path.exists(file_path):
            missing_files.append(f"{name}文件: {file_path}")
    
    if missing_files:
        print(f"错误：找不到{data_type}相关的文件：")
        for missing in missing_files:
            print(f"  {missing}")
        return False
    return True

# 读取数据文件
def read_data(file_path, data_name):
    """读取数据文件并返回数据列表"""
    try:
        with open(file_path, 'r') as f:
            data = [float(line.strip()) for line in f if line.strip()]
        print(f"  {data_name}: 已读取 {len(data)} 个数据点")
        return data
    except Exception as e:
        print(f"  读取{data_name}文件失败: {e}")
        return []
def process_rpm():
    left = read_data(rpm_files['left'],"left rpm")
    right = read_data(rpm_files['right'],"right rpm")
    timestamp = read_data(rpm_files['timestamp'],"rpm timestamp")
    min_len=min(len(left),len(right),len(timestamp))
    left=np.array(left[:min_len])
    right=np.array(right[:min_len])
    timestamp=np.array(timestamp[:min_len])
    return {"left":left,"right":right,"time":timestamp-timestamp[0]}
# 处理单个数据集的通用函数
def process_dataset(files_dict, data_type, label):
    """处理单个数据集，读取数据并计算相对时间"""
    print(f"处理{label}数据...")
    
    # 读取数据
    data = read_data(files_dict['data'], f"{label}数据")
    target_data = read_data(files_dict['target'], f"目标{label}数据")
    timestamps = read_data(files_dict['timestamp'], f"{label}时间戳")
    target_timestamps = read_data(files_dict['target_timestamp'], f"目标{label}时间戳")
    
    # 检查数据长度
    min_len = min(len(data), len(target_data), len(timestamps), len(target_timestamps))
    
    if min_len == 0:
        print(f"  {label}数据为空，跳过处理")
        return None
    
    # 裁剪到最小长度
    data = np.array(data[:min_len])
    target_data = np.array(target_data[:min_len])
    timestamps = np.array(timestamps[:min_len])
    target_timestamps = np.array(target_timestamps[:min_len])
    
    # 计算相对时间
    relative_time = timestamps - timestamps[0]
    target_relative_time = target_timestamps - target_timestamps[0]
    
    print(f"  {label}时间范围: {timestamps[0]:.3f} ~ {timestamps[-1]:.3f} 秒")
    print(f"  目标{label}时间范围: {target_timestamps[0]:.3f} ~ {target_timestamps[-1]:.3f} 秒")
    
    return {
        'data': data,
        'target_data': target_data,
        'relative_time': relative_time,
        'target_relative_time': target_relative_time,
        'label': label,
        'unit': '度' if '角度' in label else 'm/s' if '速度' in label else '度/秒'
    }

# 主程序
def main():
    print("=" * 60)
    print("角度、速度、角速度对比分析程序")
    print("=" * 60)
    
    # 检查所有文件是否存在
    all_files_ok = True
    all_files_ok &= check_files_exist(angle_files, "角度")
    all_files_ok &= check_files_exist(speed_files, "速度")
    all_files_ok &= check_files_exist(gyro_files, "角速度")
    
    if not all_files_ok:
        print("\n请确保所有数据文件都存在，然后重新运行程序。")
        exit(1)
    
    # 处理所有数据集
    angle_data = process_dataset(angle_files, "角度", "角度")
    speed_data = process_dataset(speed_files, "速度", "速度")
    gyro_data = process_dataset(gyro_files, "角速度", "角速度")
    
    if not angle_data or not speed_data or not gyro_data:
        print("数据处理失败，请检查数据文件内容。")
        exit(1)
    
    print("\n" + "=" * 60)
    print("创建对比图表...")
    print("=" * 60)
    
    # 创建包含三个子图的图表
    fig, axes = plt.subplots(4, 1, figsize=(16, 16), sharex=True)
    
    # 绘制角度对比图
    ax1 = axes[0]
    ax1.plot(angle_data['relative_time'], angle_data['data'], 
            'b-', linewidth=2, alpha=0.8, label='current_angle')
    ax1.plot(angle_data['target_relative_time'], angle_data['target_data'], 
            'r-', linewidth=2, alpha=0.8, label='target_angle')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_ylabel('degree', fontsize=12)
    ax1.set_title('angle', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10)
    
    # 添加角度统计信息
    angle_error = angle_data['data'] - angle_data['target_data']
    angle_rmse = np.sqrt(np.mean(angle_error**2))
    angle_mae = np.mean(np.abs(angle_error))
    
    stats_text = f'angle error:\n'
    stats_text += f'  RMSE: {angle_rmse:.3f}°\n'
    stats_text += f'  MAE: {angle_mae:.3f}°\n'
    stats_text += f'  number: {len(angle_data["data"])}'
    
    ax1.text(0.98, 0.98, stats_text, transform=ax1.transAxes,
             fontsize=9, verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 绘制速度对比图
    ax2 = axes[1]
    ax2.plot(speed_data['relative_time'], speed_data['data'], 
            'g-', linewidth=2, alpha=0.8, label='current speed')
    ax2.plot(speed_data['target_relative_time'], speed_data['target_data'], 
            'm-', linewidth=2, alpha=0.8, label='target_speed')
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_ylabel('m/s', fontsize=12)
    ax2.set_title('speed', fontsize=14, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=10)
    
    # 添加速度统计信息
    speed_error = speed_data['data'] - speed_data['target_data']
    speed_rmse = np.sqrt(np.mean(speed_error**2))
    speed_mae = np.mean(np.abs(speed_error))
    
    stats_text = f'speed error:\n'
    stats_text += f'  RMSE: {speed_rmse:.3f} m/s\n'
    stats_text += f'  MAE: {speed_mae:.3f} m/s\n'
    stats_text += f'  number: {len(speed_data["data"])}'
    
    ax2.text(0.98, 0.98, stats_text, transform=ax2.transAxes,
             fontsize=9, verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 绘制角速度对比图
    ax3 = axes[2]
    ax3.plot(gyro_data['relative_time'], gyro_data['data'], 
            'c-', linewidth=2, alpha=0.8, label='current_gyro')
    ax3.plot(gyro_data['target_relative_time'], gyro_data['target_data'], 
            'y-', linewidth=2, alpha=0.8, label='target_gyro')
    ax3.grid(True, alpha=0.3, linestyle='--')
    ax3.set_xlabel('s', fontsize=12)
    ax3.set_ylabel('degree/s)', fontsize=12)
    ax3.set_title('gyro', fontsize=14, fontweight='bold')
    ax3.legend(loc='upper right', fontsize=10)
    # RPM曲线
    rpm_data = process_rpm()
    ax4 = axes[3]
    ax4.plot( rpm_data['time'], rpm_data['left'], linewidth=2, label="left rpm" )
    ax4.plot(rpm_data['time'], rpm_data['right'],linewidth=2,label="right rpm")
    ax4.grid(True,alpha=0.3,linestyle='--')
    ax4.set_xlabel("s")
    ax4.set_ylabel("rpm")
    ax4.set_title( "motor rpm", fontsize=14, fontweight='bold')
    ax4.legend()
    # 添加角速度统计信息
    gyro_error = gyro_data['data'] - gyro_data['target_data']
    gyro_rmse = np.sqrt(np.mean(gyro_error**2))
    gyro_mae = np.mean(np.abs(gyro_error))
    
    stats_text = f'gyro error:\n'
    stats_text += f'  RMSE: {gyro_rmse:.3f} °/s\n'
    stats_text += f'  MAE: {gyro_mae:.3f} °/s\n'
    stats_text += f'  number: {len(gyro_data["data"])}'
    
    ax3.text(0.98, 0.98, stats_text, transform=ax3.transAxes,
             fontsize=9, verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    # 保存综合图表
    plt.savefig("all_metrics_comparison.png", dpi=300, bbox_inches='tight')
    print("综合对比图已保存到: all_metrics_comparison.png")
    
    # 显示图表
    plt.show()
    
    # 打印详细统计信息
    print("\n" + "=" * 60)
    print("详细统计信息汇总")
    print("=" * 60)
    
    print(f"\n角度对比统计:")
    print(f"  实际角度范围: {np.min(angle_data['data']):.3f} ~ {np.max(angle_data['data']):.3f} 度")
    print(f"  目标角度范围: {np.min(angle_data['target_data']):.3f} ~ {np.max(angle_data['target_data']):.3f} 度")
    print(f"  RMSE: {angle_rmse:.4f} 度")
    print(f"  MAE: {angle_mae:.4f} 度")
    print(f"  最大绝对误差: {np.max(np.abs(angle_error)):.4f} 度")
    
    print(f"\n速度对比统计:")
    print(f"  实际速度范围: {np.min(speed_data['data']):.3f} ~ {np.max(speed_data['data']):.3f} m/s")
    print(f"  目标速度范围: {np.min(speed_data['target_data']):.3f} ~ {np.max(speed_data['target_data']):.3f} m/s")
    print(f"  RMSE: {speed_rmse:.4f} m/s")
    print(f"  MAE: {speed_mae:.4f} m/s")
    print(f"  最大绝对误差: {np.max(np.abs(speed_error)):.4f} m/s")
    
    print(f"\n角速度对比统计:")
    print(f"  实际角速度范围: {np.min(gyro_data['data']):.3f} ~ {np.max(gyro_data['data']):.3f} 度/秒")
    print(f"  目标角速度范围: {np.min(gyro_data['target_data']):.3f} ~ {np.max(gyro_data['target_data']):.3f} 度/秒")
    print(f"  RMSE: {gyro_rmse:.4f} 度/秒")
    print(f"  MAE: {gyro_mae:.4f} 度/秒")
    print(f"  最大绝对误差: {np.max(np.abs(gyro_error)):.4f} 度/秒")
    

    # 显示图表
    plt.show()
    
    print("\n" + "=" * 60)
    print("数据处理完成！")
    print("=" * 60)

if __name__ == "__main__":
    main()
