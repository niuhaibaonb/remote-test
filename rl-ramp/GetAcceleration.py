
import pandas as pd
from matplotlib import pyplot as plt
from sumolib import checkBinary  # Checks for the binary in environ vars
from traci import simulation, vehicle


def log_vehicle_acceleration_single_step(acceleration_data):
    # 获取当前仿真时间
    time = simulation.getTime()

    # 获取所有车辆的加速度
    for veh_id in vehicle.getIDList():
        acceleration = vehicle.getAcceleration(veh_id)
        acceleration_data.append([time, veh_id, acceleration])


def plot_acceleration_from_excel(excel_path):
    # 读取Excel文件
    df = pd.read_excel(excel_path)

    # 设置图像大小
    plt.figure(figsize=(10, 8))

    # 遍历每一列（除了'Time'列）来绘制加速度图
    for column in df.columns[1:]:  # 跳过第一列'Time'
        plt.plot(df['Time'], df[column], label=column)

    # 设置图例
    plt.legend()
    # 设置标题和坐标轴标签
    plt.title('Vehicle Acceleration Over Time')
    plt.xlabel('Time')
    plt.ylabel('Acceleration')

    # 显示图像
    plt.show()
