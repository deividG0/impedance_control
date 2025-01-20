import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Current torque correlation from the Dynamixel PRO website:
# https://emanual.robotis.com/assets/images/dxl/pro/h54-100-s500-r_performance_graph_2.jpg

# # Define a candidate function (e.g., polynomial, exponential)
# def func(x, a, b, c):
#     return a * x**2 + b * x + c

# Define a candidate function (e.g., polynomial, exponential)
def func(x, a, b, c, d):
    return a * x**3 + b * x**2 + c * x + d

# # Load your data points (x_data, y_data) after digitizing
# x_data = np.array([8.50, 17.00, 25.50, 34.00, 42.50, 51.00, 59.50, 68.00])  # Torque values
# y_data = np.array([2.55, 4.08, 5.44, 7.14, 8.84, 10.54, 12.25, 14.62])  # Current values

# Load your data points (x_data, y_data) after digitizing
x_data = np.array([4.25, 8.50, 17.00, 25.50, 34.00, 42.50, 51.00, 59.50, 68.00])  # Torque values
y_data = np.array([1.70, 2.55, 4.08, 5.44, 7.14, 8.84, 10.54, 12.25, 14.62])  # Current values

# Fit the function to the data
params, covariance = curve_fit(func, x_data, y_data)

print('params', params)

predicted = func(76.50, *params)
print(predicted)

# Plot the results
plt.scatter(x_data, y_data, label='Data Points', color='pink')
plt.plot(x_data, func(x_data, *params), label='Fitted Curve', color='red')
plt.legend()
plt.show()
