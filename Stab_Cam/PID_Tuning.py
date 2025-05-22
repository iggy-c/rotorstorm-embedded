import pandas as pd
import matplotlib.pyplot as plt

file_path = "XXXXX.csv"

df = pd.read_csv(file_path, names=["log", "Time (ms)", "Heading", "Error", "Output", "PWM" ])
df = df.drop(columns="log")

df["Time (s)"] = df["Time (ms)"] / 1000.0

plt.figure(figsize=(12, 8))

#heading and setpoint
plt.subplot(4,1,1)
plt.plot(df["Time (s)"], df["Heading"], label="Heading")
plt.axhline(0,color='r', linestyle='--', label="Setpoint (0 degrees)")
plt.ylabel("Heading (deg)")
plt.legend()
plt.grid()

#error
plt.subplot(4, 1, 2)
plt.plot(df["Time (s)"], df["Error"], color='orange', label="Error")
plt.ylabel("Error (deg)")
plt.legend()
plt.grid()

#PID output
plt.subplot(4, 1, 3)
plt.plot(df["Time (s)"], df["Output"], color='green', label="PID Output")
plt.ylabel("PID Output")
plt.legend()
plt.grid()

#pwm
plt.subplot(4, 1, 4)
plt.plot(df["Time (s)"], df["PWM"], color='purple', label="PWM Signal")
plt.ylabel("PWM Value")
plt.xlabel("Time (s)")
plt.legend()
plt.grid()

#showgraphs
plt.tight_layout()
plt.show()