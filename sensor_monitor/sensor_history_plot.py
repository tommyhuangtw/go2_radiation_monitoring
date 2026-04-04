#!/usr/bin/env python3
"""
读取指定目录下所有 YYYYMMDD_HHMMSS_sensor_data.csv 文件，
将每个文件的 sensor_value 画成折线图（x 轴为 waypoint 序号），
不同时间的折线用不同颜色显示在同一张图内。
"""

import os
import re
import glob
import argparse
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import pandas as pd


def parse_datetime_from_filename(filename: str) -> str:
    """从文件名中提取日期时间，格式化为可读字符串用作图例标签。"""
    base = os.path.basename(filename)
    match = re.match(r"(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})_sensor_data\.csv", base)
    if match:
        y, mo, d, h, mi, s = match.groups()
        return f"{y}-{mo}-{d} {h}:{mi}:{s}"
    # 如果文件名不完全匹配，则直接用文件名（去掉扩展名）作为标签
    return os.path.splitext(base)[0]


def plot_sensor_data(directory: str, output: str = "sensor_plot.png"):
    # 查找所有匹配的 CSV 文件
    pattern = os.path.join(directory, "*sensor_data.csv")
    files = sorted(glob.glob(pattern))

    if not files:
        print(f"在目录 '{directory}' 下未找到匹配的 *sensor_data.csv 文件。")
        return

    print(f"找到 {len(files)} 个文件:")
    for f in files:
        print(f"  - {os.path.basename(f)}")

    # 为每条折线分配不同颜色
    colors = cm.get_cmap("tab10") if len(files) <= 10 else cm.get_cmap("tab20")

    fig, ax = plt.subplots(figsize=(12, 6))

    for i, filepath in enumerate(files):
        df = pd.read_csv(filepath)

        # 兼容不同列名：优先用 waypoint，否则用行号
        if "waypoint" in df.columns:
            x = df["waypoint"].values
        else:
            x = np.arange(1, len(df) + 1)

        y = df["sensor_value"].values
        label = parse_datetime_from_filename(filepath)
        color = colors(i % 20)

        ax.plot(x, y, marker="o", markersize=4, linewidth=1.5, color=color, label=label)

    ax.set_xlabel("Waypoint", fontsize=12)
    ax.set_ylabel("Sensor Value", fontsize=12)
    ax.set_title("Sensor Data Over Waypoints", fontsize=14)
    ax.legend(loc="best", fontsize=8, ncol=2 if len(files) > 8 else 1)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    output_path = os.path.join(directory, output)
    fig.savefig(output_path, dpi=150)
    print(f"\n图表已保存至: {output_path}")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="批量绘制 sensor_data CSV 折线图")
    parser.add_argument("directory", help="存放 CSV 文件的目录路径")
    parser.add_argument("-o", "--output", default="sensor_plot.png", help="输出图片文件名 (默认: sensor_plot.png)")
    args = parser.parse_args()

    plot_sensor_data(args.directory, args.output)