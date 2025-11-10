#!/bin/python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

# Read the CSV file
df = pd.read_csv("task7_data_collection.csv")

# Filter to sizes > 0 and distances > 0
df = df[(df["size"] > 0) & (df["distance"] > 0)]

# Start processing from this row to max_row (remove some outliers)
row = 100
max_row = -1 # Excluding some outliers, if needed.

x_data = df["size"].iloc[row:max_row].values
y_data = df["distance"].iloc[row:max_row].values

# x_data, y_data as before
N = len(x_data)
weights = np.ones(N)
weights[-100:] = 100

# curve_fit uses sigma = std deviation → smaller sigma = more weight
sigma = 1 / weights

# --- Power-law model ---
def model_f(x, a, c):
    return a * (x ** c)

# --- Fit power-law with curve_fit ---
param, param_cov = curve_fit(model_f, x_data, y_data, sigma=sigma, absolute_sigma=False, maxfev=10000)
a_power, c_power = param
print(f"Power-law fit: a={a_power:.3f}, c={c_power:.3f}")

# --- Generate model curves for plotting ---
x_model = np.linspace(min(x_data), max(x_data), 100)
y_power_model = model_f(x_model, a_power, c_power)

# --- Plot ---
plt.figure(figsize=(8,6))
plt.scatter(x_data[300:], y_data[300:], facecolors='none', edgecolors='r', label='data')
plt.plot(x_model, y_power_model, color='g', label='power-law fit')
plt.xlabel("Size")
plt.ylabel("Distance")
plt.legend()
plt.tight_layout()
plt.savefig("fitted-graph.png")
