import matplotlib
matplotlib.use('Agg')  # Set non-interactive backend before importing pyplot
import matplotlib.pyplot as plt
import pandas as pd

# Load CSV file with commas as delimiter
df = pd.read_csv("plot/PID.csv", header=None, delimiter=',')

# Assign column names
df.columns = [
    'Timestamp', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Gyro_Integral_dt',
    'Accel_Timestamp_Relative', 'Acc_X', 'Acc_Y', 'Acc_Z', 'Accel_Integral_dt',
    'Accel_Clipping', 'Gyro_Clipping', 'Accel_Calibration_Count', 'Gyro_Calibration_Count'
]

# Convert microsecond timestamp to seconds for better readability
df['Timestamp'] = df['Timestamp'] / 1e6  # Convert from microseconds to seconds

# Shift time axis so it starts from zero at the first reading
time = df['Timestamp'] - df['Timestamp'].iloc[0]  

# Extract gyro data
gyro_x = df['Gyro_X']
gyro_y = df['Gyro_Y']
gyro_z = df['Gyro_Z']

# Plot gyroscope readings over time
plt.figure(figsize=(10, 5))
plt.plot(time, gyro_x, label="Gyro_X", color='r')
plt.plot(time, gyro_y, label="Gyro_Y", color='g')
plt.plot(time, gyro_z, label="Gyro_Z", color='b')
plt.xlabel("Elapsed Time (seconds)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Gyroscope Readings Over Time")
plt.legend()
plt.grid()

# Simply save in current directory
plt.savefig("gyro_readings.png")
plt.close()