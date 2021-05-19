import json 
import matplotlib.pyplot as plt
import numpy as np

def get_length():
    with open("/home/maciej/dev_ws/src/swiech_szmurlo/zadanie5/zadanie5/position_data.txt", "r") as read_file:
        data = json.load(read_file)

    x_coeffs = data["x"]
    y_coeffs = data["y"]
    z_coeffs = data["z"]


    return [x_coeffs, y_coeffs, z_coeffs]

def plot():

    [x_coeffs, y_coeffs, z_coeffs] = get_length()
    axis_points =  list(range(0, len(x_coeffs)))

    plt.plot(axis_points, x_coeffs)
    plt.plot(axis_points, x_coeffs, linestyle='dashed', color='r')

    plt.plot(axis_points, y_coeffs)
    plt.plot(axis_points, y_coeffs, linestyle='dashed', color='r')

    plt.plot(axis_points, z_coeffs)
    plt.plot(axis_points, z_coeffs, linestyle='dashed', color='r')

    plt.xlabel("Kroki")
    plt.ylabel("Położenie")
    plt.title("Wykresy położenia końcówki robota wyznaczonego przy pomocy kinematyki odwrotnej oraz zadanego położenia końcówki.")
    
    plt.show()

if __name__ == "__main__":
    plot()

    