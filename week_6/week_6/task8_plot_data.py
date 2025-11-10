#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv("task7_data_collection.csv")   # change to your filename

# Plot size vs. distance
plt.figure()
plt.scatter(df["size"], df["distance"])  # scatter plot
plt.xlabel("Size")
plt.ylabel("Distance")
plt.title("Size vs Distance")
plt.show()