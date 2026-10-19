#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
import os

# ==================== 绘图自定义配置 ====================
# 横坐标范围：None=自动显示全部数据；格式：(起始秒, 结束秒)，例如 (10, 50)
X_LIMIT =None

# 主网格精度：设为None则使用系统自动网格
# 数值对应坐标轴单位：x轴为秒，角度/角速度为度，速度为m/s
X_GRID_STEP = None   # X轴主网格间隔，例如 5 代表每5秒一条主网格线
Y_GRID_STEP = None   # Y轴主网格间隔，例如 2 代表每2度/2m/s一条主网格线
# 纵坐标自适应边距：数据范围上下预留比例，0.05=上下各留5%空白，避免曲线贴边
Y_MARGIN = 0.05
# 是否显示次网格（更细密的辅助网格）
SHOW_MINOR_GRID = True
# =====================================================

# 文件路径定义
base_path = "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/"

# 角度相关文件（原始+滤波后+目标）
angle_files = {
    'raw': base_path + "angle_data.txt",
    'filtered': base_path + "angle_filtered_data.txt",
    'target': base_path + "target_angle_data.txt",
    'raw_timestamp': base_path + "timestamp_angle.txt",
    'filtered_timestamp': base_path + "timestamp_angle_filtered.txt",
    'target_timestamp': base_path + "timestamp_target_angle.txt"
}
# 速度相关文件（原始+滤波后+目标）
speed_files = {
    'raw': base_path + "speed_raw_data.txt",
    'filtered': base_path + "speed_data.txt",
    'target': base_path + "target_speed_data.txt", 
    'raw_timestamp': base_path + "timestamp_speed_raw.txt",
    'filtered_timestamp': base_path + "timestamp_speed.txt",
    'target_timestamp': base_path + "timestamp_target_speed.txt"
}
# 角速度相关文件（原始+滤波后+目标）
gyro_files = {
    'raw': base_path + "gyro_data.txt",
    'filtered': base_path + "gyro_filtered_data.txt",
    'target': base_path + "target_gyro_data.txt",
    'raw_timestamp': base_path + "timestamp_gyro.txt",
    'filtered_timestamp': base_path + "timestamp_gyro_filtered.txt",
    'target_timestamp': base_path + "timestamp_target_gyro.txt"
}
# 电机电流相关文件
current_files = {
    'left': base_path + "left_current_data.txt",
    'right': base_path + "right_current_data.txt",
    'left_timestamp': base_path + "timestamp_left_current.txt",
    'right_timestamp': base_path + "timestamp_right_current.txt"
}
# 转向环（偏航角速度）相关文件（原始+滤波后+目标）
yaw_rate_files = {
    'raw': base_path + "gyro_z_data.txt",
    'filtered': base_path + "gyro_z_filtered_data.txt",
    'target': base_path + "target_yaw_rate_data.txt",
    'raw_timestamp': base_path + "timestamp_gyro_z.txt",
    'filtered_timestamp': base_path + "timestamp_gyro_z_filtered.txt",
    'target_timestamp': base_path + "timestamp_target_yaw_rate.txt"
}

# ========== 【新增】电机转速相关文件 ==========
rpm_files = {
    'left': base_path + "left_rpm_data.txt",
    'right': base_path + "right_rpm_data.txt",
    'timestamp': base_path + "timestamp_rpm.txt"
}


def setup_axes_limits(ax, x_lim=None, y_margin=0.05, 
                      x_grid_step=None, y_grid_step=None, show_minor=True):
    """
    统一设置坐标轴范围、网格精度
    - x_lim: x轴范围，None为自动
    - y_margin: y轴自适应上下边距比例
    - x_grid_step/y_grid_step: 主网格间隔，None为自动
    - show_minor: 是否显示次网格
    """
    # 1. 设置横坐标范围
    if x_lim is not None:
        ax.set_xlim(x_lim)
    
    # 2. 纵坐标自适应 + 边距优化
    ax.autoscale(axis='y', tight=True)
    y_min, y_max = ax.get_ylim()
    y_range = y_max - y_min
    ax.set_ylim(y_min - y_range * y_margin, y_max + y_range * y_margin)
    
    # 3. 设置X轴网格
    if x_grid_step is not None:
        ax.xaxis.set_major_locator(MultipleLocator(x_grid_step))
    if show_minor:
        ax.xaxis.set_minor_locator(AutoMinorLocator())
    
    # 4. 设置Y轴网格
    if y_grid_step is not None:
        ax.yaxis.set_major_locator(MultipleLocator(y_grid_step))
    if show_minor:
        ax.yaxis.set_minor_locator(AutoMinorLocator())
    
    # 5. 网格显示样式
    ax.grid(True, which='major', alpha=0.4, linestyle='-', linewidth=0.8)
    if show_minor:
        ax.grid(True, which='minor', alpha=0.15, linestyle='--', linewidth=0.5)


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


# 处理单个数据集的通用函数（支持原始+滤波后+目标）
def process_dataset(files_dict, label):
    """处理单个数据集，读取数据并计算相对时间"""
    print(f"处理{label}数据...")
    
    # 读取原始数据
    raw_data = read_data(files_dict['raw'], f"{label}原始")
    raw_timestamps = read_data(files_dict['raw_timestamp'], f"{label}原始时间戳")
    
    # 读取滤波后数据
    filtered_data = read_data(files_dict['filtered'], f"{label}滤波后")
    filtered_timestamps = read_data(files_dict['filtered_timestamp'], f"{label}滤波后时间戳")
    
    # 读取目标数据
    target_data = read_data(files_dict['target'], f"{label}目标")
    target_timestamps = read_data(files_dict['target_timestamp'], f"{label}目标时间戳")
    
    # 对齐最小长度
    min_len = min(len(raw_data), len(filtered_data), len(target_data),
                len(raw_timestamps), len(filtered_timestamps), len(target_timestamps))
    
    if min_len == 0:
        print(f"  {label}数据为空，跳过处理")
        return None
    
    # 裁剪并转数组
    raw_data = np.array(raw_data[:min_len])
    filtered_data = np.array(filtered_data[:min_len])
    target_data = np.array(target_data[:min_len])
    
    raw_timestamps = np.array(raw_timestamps[:min_len])
    filtered_timestamps = np.array(filtered_timestamps[:min_len])
    target_timestamps = np.array(target_timestamps[:min_len])
    
    # 计算相对时间（以原始数据起点为基准）
    base_time = raw_timestamps[0]
    raw_time = raw_timestamps - base_time
    filtered_time = filtered_timestamps - base_time
    target_time = target_timestamps - base_time
    
    return {
        'raw_data': raw_data,
        'filtered_data': filtered_data,
        'target_data': target_data,
        'raw_time': raw_time,
        'filtered_time': filtered_time,
        'target_time': target_time,
        'label': label,
        'unit': '度' if '角度' in label else 'm/s' if '速度' in label else '度/秒',
        'base_time': base_time  # 保存基准时间用于对齐其他数据
    }


# 主程序
def main():
    print("=" * 60)
    print("角度、速度、角速度、电机电流、偏航角速度、电机转速 对比分析")
    print("=" * 60)
    
    # 处理所有数据集
    angle_data = process_dataset(angle_files, "角度")
    speed_data = process_dataset(speed_files, "速度")
    gyro_data = process_dataset(gyro_files, "角速度")
    yaw_rate_data = process_dataset(yaw_rate_files, "偏航角速度")
    
    if not angle_data or not speed_data or not gyro_data:
        print("核心数据处理失败，请检查数据文件内容。")
        exit(1)

    # ========== 处理电机电流数据 ==========
    print("处理电机电流数据...")
    left_current = read_data(current_files['left'], "左电机电流")
    left_current_ts = read_data(current_files['left_timestamp'], "左电机电流时间戳")
    right_current = read_data(current_files['right'], "右电机电流")
    right_current_ts = read_data(current_files['right_timestamp'], "右电机电流时间戳")
    
    min_current_len = min(len(left_current), len(left_current_ts), 
                        len(right_current), len(right_current_ts))
    has_current = min_current_len > 0
    
    if has_current:
        left_current = np.array(left_current[:min_current_len])
        left_current_ts = np.array(left_current_ts[:min_current_len])
        right_current = np.array(right_current[:min_current_len])
        right_current_ts = np.array(right_current_ts[:min_current_len])
        
        # 与角度数据对齐时间基准
        base_time = angle_data['base_time']
        left_current_time = left_current_ts - base_time
        right_current_time = right_current_ts - base_time
    else:
        print("  电流数据为空，跳过电流绘图")

    # ========== 【新增】处理电机转速数据 ==========
    print("处理电机转速数据...")
    left_rpm = read_data(rpm_files['left'], "左电机转速")
    right_rpm = read_data(rpm_files['right'], "右电机转速")
    rpm_ts = read_data(rpm_files['timestamp'], "电机转速时间戳")
    
    min_rpm_len = min(len(left_rpm), len(right_rpm), len(rpm_ts))
    has_rpm = min_rpm_len > 0
    
    if has_rpm:
        left_rpm = np.array(left_rpm[:min_rpm_len])
        right_rpm = np.array(right_rpm[:min_rpm_len])
        rpm_ts = np.array(rpm_ts[:min_rpm_len])
        
        # 与角度数据对齐时间基准
        base_time = angle_data['base_time']
        left_rpm_time = rpm_ts - base_time
        right_rpm_time = rpm_ts - base_time
    else:
        print("  转速数据为空，跳过转速绘图")
    
    print("\n" + "=" * 60)
    print("创建对比图表...")
    print("=" * 60)
    
    # ========== 【修改】从5张子图改为6张子图，高度适配 ==========
    fig, axes = plt.subplots(6, 1, figsize=(16, 26), sharex=True)
    
    # ========== 1. 角度对比图 ==========
    ax1 = axes[0]
    ax1.plot(angle_data['raw_time'], angle_data['raw_data'], 
            'c--', linewidth=1.2, alpha=0.7, label='raw_angle')
    ax1.plot(angle_data['filtered_time'], angle_data['filtered_data'], 
            'b-', linewidth=2, alpha=0.9, label='filtered_angle')
    ax1.plot(angle_data['target_time'], angle_data['target_data'], 
            'r-', linewidth=2, alpha=0.9, label='target_angle')
    
    setup_axes_limits(ax1, 
                    x_lim=X_LIMIT, 
                    y_margin=Y_MARGIN,
                    x_grid_step=X_GRID_STEP, 
                    y_grid_step=Y_GRID_STEP, 
                    show_minor=SHOW_MINOR_GRID)
    
    ax1.set_ylabel('degree ', fontsize=12)
    ax1.set_title('angle', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10)
    
    # 角度统计信息
    angle_error = angle_data['filtered_data'] - angle_data['target_data']
    angle_rmse = np.sqrt(np.mean(angle_error**2))
    angle_mae = np.mean(np.abs(angle_error))
    
    stats_text = f'滤波后跟踪误差:\n'
    stats_text += f'  RMSE: {angle_rmse:.3f}°\n'
    stats_text += f'  MAE: {angle_mae:.3f}°\n'
    stats_text += f'  数据点数: {len(angle_data["filtered_data"])}'
    
    ax1.text(0.98, 0.98, stats_text, transform=ax1.transAxes,
            fontsize=9, verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # ========== 2. 速度对比图 ==========
    ax2 = axes[1]
    ax2.plot(speed_data['raw_time'], speed_data['raw_data'], 
            'y--', linewidth=1.2, alpha=0.7, label='raw_speed')
    ax2.plot(speed_data['filtered_time'], speed_data['filtered_data'], 
            'g-', linewidth=2, alpha=0.9, label='filtered_speed')
    ax2.plot(speed_data['target_time'], speed_data['target_data'], 
            'm-', linewidth=2, alpha=0.9, label='target_speed')
    
    setup_axes_limits(ax2, 
                    x_lim=X_LIMIT, 
                    y_margin=Y_MARGIN,
                    x_grid_step=X_GRID_STEP, 
                    y_grid_step=Y_GRID_STEP, 
                    show_minor=SHOW_MINOR_GRID)
    
    ax2.set_ylabel('speed (m/s)', fontsize=12)
    ax2.set_title('speed', fontsize=14, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=10)
    
    # 速度统计信息
    speed_error = speed_data['filtered_data'] - speed_data['target_data']
    speed_rmse = np.sqrt(np.mean(speed_error**2))
    speed_mae = np.mean(np.abs(speed_error))
    
    stats_text = f'滤波后跟踪误差:\n'
    stats_text += f'  RMSE: {speed_rmse:.3f} m/s\n'
    stats_text += f'  MAE: {speed_mae:.3f} m/s\n'
    stats_text += f'  数据点数: {len(speed_data["filtered_data"])}'
    
    ax2.text(0.98, 0.98, stats_text, transform=ax2.transAxes,
            fontsize=9, verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # ========== 3. 角速度对比图 ==========
    ax3 = axes[2]
    ax3.plot(gyro_data['raw_time'], gyro_data['raw_data'], 
            'c--', linewidth=1.2, alpha=0.7, label='raw_gyro')
    ax3.plot(gyro_data['filtered_time'], gyro_data['filtered_data'], 
            'b-', linewidth=2, alpha=0.9, label='filtered_gyro')
    ax3.plot(gyro_data['target_time'], gyro_data['target_data'], 
            'r-', linewidth=2, alpha=0.9, label='target_gyro')
    
    setup_axes_limits(ax3, 
                    x_lim=X_LIMIT, 
                    y_margin=Y_MARGIN,
                    x_grid_step=X_GRID_STEP, 
                    y_grid_step=Y_GRID_STEP, 
                    show_minor=SHOW_MINOR_GRID)
    
    ax3.set_ylabel('gyro (degree/s)', fontsize=12)
    ax3.set_title('gyro', fontsize=14, fontweight='bold')
    ax3.legend(loc='upper right', fontsize=10)
    
    # 角速度统计信息
    gyro_error = gyro_data['filtered_data'] - gyro_data['target_data']
    gyro_rmse = np.sqrt(np.mean(gyro_error**2))
    gyro_mae = np.mean(np.abs(gyro_error))
    
    stats_text = f'滤波后跟踪误差:\n'
    stats_text += f'  RMSE: {gyro_rmse:.3f} °/s\n'
    stats_text += f'  MAE: {gyro_mae:.3f} °/s\n'
    stats_text += f'  数据点数: {len(gyro_data["filtered_data"])}'
    
    ax3.text(0.98, 0.98, stats_text, transform=ax3.transAxes,
            fontsize=9, verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # ========== 4. 电机电流对比图 ==========
    ax4 = axes[3]
    if has_current:
        ax4.plot(left_current_time, left_current, 
                'b-', linewidth=1.5, alpha=0.9, label='left_motor_current')
        ax4.plot(right_current_time, right_current, 
                'r-', linewidth=1.5, alpha=0.9, label='right_motor_current')
        
        setup_axes_limits(ax4, 
                        x_lim=X_LIMIT, 
                        y_margin=Y_MARGIN,
                        x_grid_step=X_GRID_STEP, 
                        y_grid_step=Y_GRID_STEP, 
                        show_minor=SHOW_MINOR_GRID)
        
        # 电流统计信息
        stats_text = f'电机电流统计:\n'
        stats_text += f'  左电机范围: {np.min(left_current):.1f} ~ {np.max(left_current):.1f}\n'
        stats_text += f'  右电机范围: {np.min(right_current):.1f} ~ {np.max(right_current):.1f}\n'
        stats_text += f'  数据点数: {min_current_len}'
        
        ax4.text(0.98, 0.98, stats_text, transform=ax4.transAxes,
                fontsize=9, verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    else:
        ax4.text(0.5, 0.5, '无电流数据', transform=ax4.transAxes,
                ha='center', va='center', fontsize=12, color='gray')
    
    ax4.set_ylabel('current (unit)', fontsize=12)
    ax4.set_title('motor control current', fontsize=14, fontweight='bold')
    ax4.legend(loc='upper right', fontsize=10)
    
    # ========== 5. 转向环（偏航角速度）对比图 ==========
    ax5 = axes[4]
   
    if yaw_rate_data:
        ax5.plot(yaw_rate_data['raw_time'], yaw_rate_data['raw_data'], 
                'c--', linewidth=1.2, alpha=0.7, label='raw_yaw_rate')
        ax5.plot(yaw_rate_data['filtered_time'], yaw_rate_data['filtered_data'], 
                'b-', linewidth=2, alpha=0.9, label='filtered_yaw_rate')
        ax5.plot(yaw_rate_data['target_time'], yaw_rate_data['target_data'], 
                'r-', linewidth=2, alpha=0.9, label='target_yaw_rate')
        
        setup_axes_limits(ax5, 
                        x_lim=X_LIMIT, 
                        y_margin=Y_MARGIN,
                        x_grid_step=X_GRID_STEP, 
                        y_grid_step=Y_GRID_STEP, 
                        show_minor=SHOW_MINOR_GRID)
        
        ax5.set_ylabel('yaw rate (deg/s)', fontsize=12)
        ax5.set_title('Yaw Rate Control', fontsize=14, fontweight='bold')
        ax5.legend(loc='upper right', fontsize=10)
        
        # 统计信息
        yaw_error = yaw_rate_data['filtered_data'] - yaw_rate_data['target_data']
        yaw_rmse = np.sqrt(np.mean(yaw_error**2))
        yaw_mae = np.mean(np.abs(yaw_error))
        stats_text = f'跟踪误差:\n  RMSE: {yaw_rmse:.3f} °/s\n  MAE: {yaw_mae:.3f} °/s'
        ax5.text(0.98, 0.98, stats_text, transform=ax5.transAxes,
                fontsize=9, verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    else:
        ax5.text(0.5, 0.5, '无偏航角速度数据', transform=ax5.transAxes,
                ha='center', va='center', fontsize=12, color='gray')
        ax5.set_ylabel('yaw rate (deg/s)', fontsize=12)
        ax5.set_title('Yaw Rate Control', fontsize=14, fontweight='bold')

    # ========== 【新增】6. 电机转速对比图 ==========
    ax6 = axes[5]
    if has_rpm:
        ax6.plot(left_rpm_time, left_rpm, 
                'b-', linewidth=1.5, alpha=0.9, label='left_motor_rpm')
        ax6.plot(right_rpm_time, right_rpm, 
                'r-', linewidth=1.5, alpha=0.9, label='right_motor_rpm')
        
        setup_axes_limits(ax6, 
                        x_lim=X_LIMIT, 
                        y_margin=Y_MARGIN,
                        x_grid_step=X_GRID_STEP, 
                        y_grid_step=Y_GRID_STEP, 
                        show_minor=SHOW_MINOR_GRID)
        
        # 转速统计信息
        stats_text = f'电机转速统计:\n'
        stats_text += f'  左电机范围: {np.min(left_rpm):.1f} ~ {np.max(left_rpm):.1f} RPM\n'
        stats_text += f'  右电机范围: {np.min(right_rpm):.1f} ~ {np.max(right_rpm):.1f} RPM\n'
        stats_text += f'  数据点数: {min_rpm_len}'
        
        ax6.text(0.98, 0.98, stats_text, transform=ax6.transAxes,
                fontsize=9, verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    else:
        ax6.text(0.5, 0.5, '无转速数据', transform=ax6.transAxes,
                ha='center', va='center', fontsize=12, color='gray')
    
    ax6.set_xlabel('time (s)', fontsize=12)
    ax6.set_ylabel('motor speed (RPM)', fontsize=12)
    ax6.set_title('motor speed', fontsize=14, fontweight='bold')
    ax6.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    
    # 保存综合图表
    plt.savefig("all_metrics_with_current_rpm.png", dpi=300, bbox_inches='tight')
    print("综合对比图已保存到: all_metrics_with_current_rpm.png")
    
    # 打印详细统计信息
    print("\n" + "=" * 60)
    print("详细统计信息汇总")
    print("=" * 60)
    
    print(f"\n角度对比统计:")
    print(f"  原始角度范围: {np.min(angle_data['raw_data']):.3f} ~ {np.max(angle_data['raw_data']):.3f} 度")
    print(f"  滤波后角度范围: {np.min(angle_data['filtered_data']):.3f} ~ {np.max(angle_data['filtered_data']):.3f} 度")
    print(f"  目标角度范围: {np.min(angle_data['target_data']):.3f} ~ {np.max(angle_data['target_data']):.3f} 度")
    print(f"  滤波后跟踪 RMSE: {angle_rmse:.4f} 度")
    print(f"  滤波后跟踪 MAE: {angle_mae:.4f} 度")
    print(f"  最大绝对误差: {np.max(np.abs(angle_error)):.4f} 度")
    
    print(f"\n速度对比统计:")
    print(f"  原始车速范围: {np.min(speed_data['raw_data']):.3f} ~ {np.max(speed_data['raw_data']):.3f} m/s")
    print(f"  滤波后车速范围: {np.min(speed_data['filtered_data']):.3f} ~ {np.max(speed_data['filtered_data']):.3f} m/s")
    print(f"  目标速度范围: {np.min(speed_data['target_data']):.3f} ~ {np.max(speed_data['target_data']):.3f} m/s")
    print(f"  滤波后跟踪 RMSE: {speed_rmse:.4f} m/s")
    print(f"  滤波后跟踪 MAE: {speed_mae:.4f} m/s")
    print(f"  最大绝对误差: {np.max(np.abs(speed_error)):.4f} m/s")
    
    print(f"\n角速度对比统计:")
    print(f"  原始角速度范围: {np.min(gyro_data['raw_data']):.3f} ~ {np.max(gyro_data['raw_data']):.3f} 度/秒")
    print(f"  滤波后角速度范围: {np.min(gyro_data['filtered_data']):.3f} ~ {np.max(gyro_data['filtered_data']):.3f} 度/秒")
    print(f"  目标角速度范围: {np.min(gyro_data['target_data']):.3f} ~ {np.max(gyro_data['target_data']):.3f} 度/秒")
    print(f"  滤波后跟踪 RMSE: {gyro_rmse:.4f} 度/秒")
    print(f"  滤波后跟踪 MAE: {gyro_mae:.4f} 度/秒")
    print(f"  最大绝对误差: {np.max(np.abs(gyro_error)):.4f} 度/秒")
    
    # 电流统计
    if has_current:
        print(f"\n电机电流统计:")
        print(f"  左电机电流范围: {np.min(left_current):.3f} ~ {np.max(left_current):.3f}")
        print(f"  右电机电流范围: {np.min(right_current):.3f} ~ {np.max(right_current):.3f}")
        print(f"  电流差值均值: {np.mean(np.abs(left_current - right_current)):.3f}")
        print(f"  数据点数: {min_current_len}")

    # ========== 【新增】转速统计输出 ==========
    if has_rpm:
        print(f"\n电机转速统计:")
        print(f"  左电机转速范围: {np.min(left_rpm):.3f} ~ {np.max(left_rpm):.3f} RPM")
        print(f"  右电机转速范围: {np.min(right_rpm):.3f} ~ {np.max(right_rpm):.3f} RPM")
        print(f"  转速差值均值: {np.mean(np.abs(left_rpm - right_rpm)):.3f} RPM")
        print(f"  数据点数: {min_rpm_len}")
    
    # 显示图表
    plt.show()
    
    print("\n" + "=" * 60)
    print("数据处理完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
