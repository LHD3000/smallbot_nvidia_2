#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
import os

# ==================== 绘图自定义配置 ====================
X_LIMIT = None                 # None=全部；格式 (起始秒, 结束秒)
X_GRID_STEP = None
Y_GRID_STEP = None
Y_MARGIN = 0.05
SHOW_MINOR_GRID = True

# ==================== 文件路径 ====================
base_path = "/home/nvidia/jeston/smallbot_ws/src/aruco/src/"

def mk_files(sig):
    return {
        'raw': base_path + f"vision_raw_{sig}.txt",
        'filtered': base_path + f"vision_filtered_{sig}.txt",
        'raw_timestamp': base_path + f"vision_timestamp_raw_{sig}.txt",
        'filtered_timestamp': base_path + f"vision_timestamp_filtered_{sig}.txt",
    }

x_files   = mk_files("x")
y_files   = mk_files("y")
z_files   = mk_files("z")
yaw_files = mk_files("yaw")

# ==================== 坐标轴/网格设置 ====================
def setup_axes_limits(ax, x_lim=None, y_margin=0.05,
                      x_grid_step=None, y_grid_step=None, show_minor=True):
    if x_lim is not None:
        ax.set_xlim(x_lim)
    ax.autoscale(axis='y', tight=True)
    y_min, y_max = ax.get_ylim()
    y_range = y_max - y_min
    ax.set_ylim(y_min - y_range * y_margin, y_max + y_range * y_margin)
    if x_grid_step is not None:
        ax.xaxis.set_major_locator(MultipleLocator(x_grid_step))
    if show_minor:
        ax.xaxis.set_minor_locator(AutoMinorLocator())
    if y_grid_step is not None:
        ax.yaxis.set_major_locator(MultipleLocator(y_grid_step))
    if show_minor:
        ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.grid(True, which='major', alpha=0.4, linestyle='-', linewidth=0.8)
    if show_minor:
        ax.grid(True, which='minor', alpha=0.15, linestyle='--', linewidth=0.5)

# ==================== 读取数据 ====================
def read_data(file_path, data_name):
    try:
        with open(file_path, 'r') as f:
            data = [float(line.strip()) for line in f if line.strip()]
        print(f" {data_name}: 已读取 {len(data)} 个数据点")
        return data
    except Exception as e:
        print(f" 读取{data_name}文件失败: {e}")
        return []

def process_dataset(files_dict, label):
    print(f"处理{label}数据...")
    raw_data = read_data(files_dict['raw'], f"{label}原始")
    raw_ts   = read_data(files_dict['raw_timestamp'], f"{label}原始时间戳")
    filt_data = read_data(files_dict['filtered'], f"{label}滤波后")
    filt_ts  = read_data(files_dict['filtered_timestamp'], f"{label}滤波后时间戳")

    min_len = min(len(raw_data), len(filt_data), len(raw_ts), len(filt_ts))
    if min_len == 0:
        print(f" {label}数据为空，跳过处理")
        return None

    raw_data   = np.array(raw_data[:min_len])
    filt_data  = np.array(filt_data[:min_len])
    raw_ts     = np.array(raw_ts[:min_len])
    filt_ts    = np.array(filt_ts[:min_len])

    base_time = raw_ts[0]
    return {
        'raw_data': raw_data,
        'filtered_data': filt_data,
        'raw_time': raw_ts - base_time,
        'filtered_time': filt_ts - base_time,
        'label': label,
        'base_time': base_time,
    }

# ==================== 主程序 ====================
def main():
    print("=" * 60)
    print("视觉定位：位置 X/Y/Z 与航向角 AngleY 原始/滤波对比")
    print("=" * 60)

    x_data   = process_dataset(x_files, "位置X")
    y_data   = process_dataset(y_files, "位置Y")
    z_data   = process_dataset(z_files, "位置Z")
    yaw_data = process_dataset(yaw_files, "航向角")

    fig, axes = plt.subplots(4, 1, figsize=(16, 14), sharex=True)

    def plot_one(ax, data, title, ylabel, raw_color, filt_color):
        if data is None:
            ax.text(0.5, 0.5, f'无{title}数据', transform=ax.transAxes,
                    ha='center', va='center', fontsize=12, color='gray')
            ax.set_title(title, fontsize=14, fontweight='bold')
            return
        ax.plot(data['raw_time'], data['raw_data'], raw_color + '--',
                linewidth=1.2, alpha=0.7, label='raw')
        ax.plot(data['filtered_time'], data['filtered_data'], filt_color + '-',
                linewidth=2, alpha=0.9, label='filtered')
        setup_axes_limits(ax, x_lim=X_LIMIT, y_margin=Y_MARGIN,
                          x_grid_step=X_GRID_STEP, y_grid_step=Y_GRID_STEP,
                          show_minor=SHOW_MINOR_GRID)
        ax.set_ylabel(ylabel, fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)

        err = data['filtered_data'] - data['raw_data']
        rmse = np.sqrt(np.mean(err ** 2))
        mae = np.mean(np.abs(err))
        stats = (f'平滑偏差:\n'
                 f' RMSE: {rmse:.3f}\n'
                 f' MAE: {mae:.3f}\n'
                 f' 数据点数: {len(data["filtered_data"])}')
        ax.text(0.98, 0.98, stats, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plot_one(axes[0], x_data,   'Position X',   'X (m)',     'c', 'b')
    plot_one(axes[1], y_data,   'Position Y',   'Y (m)',     'y', 'g')
    plot_one(axes[2], z_data,   'Position Z',   'Z (m)',     'c', 'b')
    plot_one(axes[3], yaw_data, 'AngleY',       'AngleY (deg)', 'y', 'r')

    axes[-1].set_xlabel('time (s)', fontsize=12)
    plt.tight_layout()

    out = base_path + "vision_dock_plot.png"
    plt.savefig(out, dpi=300, bbox_inches='tight')
    print(f"\n对比图已保存: {out}")

if __name__ == '__main__':
    main()
