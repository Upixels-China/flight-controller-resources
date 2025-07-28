# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import detrend

# -----------------------------
# 1. 读取CSV文件
# -----------------------------
imu_file = 'imu.csv'
of_file = 'of.csv'

# 读取IMU数据
imu_df = pd.read_csv(imu_file)
imu_df['TimeSec'] = imu_df['TimeUS'] / 1e6

# 去掉重复时间戳（每对只留第一行）
imu_df = imu_df.groupby('TimeSec').first().reset_index()

# 读取OF数据（不做去重）
of_df = pd.read_csv(of_file)
of_df['TimeSec'] = of_df['TimeUS'] / 1e6

# -----------------------------
# 2. 插值光流数据到IMU时间轴
# -----------------------------
of_interp = np.interp(
    imu_df['TimeSec'],
    of_df['TimeSec'],
    of_df['FlowX']
)

# -----------------------------
# 3. 绘制原始波形
# -----------------------------
plt.figure(figsize=(12,6))
plt.plot(imu_df['TimeSec'], imu_df['GyrX'], label='IMU.GyrX')
plt.plot(imu_df['TimeSec'], of_interp, label='OF.FlowX (interpolated)')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.title('IMU vs OF (Interpolated)')
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# 4. 去趋势处理
# -----------------------------
imu_signal = detrend(imu_df['GyrX'].values)
of_signal = detrend(of_interp)

# -----------------------------
# 5. 互相关分析
# -----------------------------
correlation = np.correlate(of_signal, imu_signal, mode='full')
lag_index = np.argmax(correlation) - (len(imu_signal) - 1)

# 采样间隔
intervals = np.diff(imu_df['TimeSec'])
intervals = intervals[intervals > 0]
sampling_interval = np.median(intervals)

delay_sec = lag_index * sampling_interval

print()
print("Estimated delay: {:.1f} ms".format(delay_sec*1000))
print("Lag index:", lag_index)
print("Sampling interval (s):", sampling_interval)
print()

# -----------------------------
# 6. 绘制互相关曲线
# -----------------------------
lags = np.arange(-len(imu_signal)+1, len(imu_signal))
plt.figure(figsize=(12,6))
plt.plot(lags * sampling_interval * 1000, correlation)
plt.xlabel('Lag (ms)')
plt.ylabel('Correlation')
plt.title('Cross-correlation between IMU and OF')
plt.grid(True)
plt.show()

# -----------------------------
# 7. 平移光流再绘图
# -----------------------------
shifted_time = imu_df['TimeSec'] - delay_sec

plt.figure(figsize=(12,6))
plt.plot(imu_df['TimeSec'], imu_df['GyrX'], label='IMU.GyrX')
plt.plot(shifted_time, of_interp, label='OF.FlowX shifted ({:.1f} ms)'.format(delay_sec*1000))
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.title('Aligned IMU and OF after delay compensation')
plt.legend()
plt.grid(True)
plt.show()
