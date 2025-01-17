import numpy as np
import matplotlib
# Wayland shenanigans
matplotlib.use("Qt5Agg")
from matplotlib.widgets import Slider, Button
from matplotlib import pyplot as plt
from math import pi


class Plot:
    def __init__(self, min=0, max=2, points=1000):
        self.min = min
        self.max = max
        self.points = points

        # Create the figure and axis
        self.fig, self.axis = plt.subplots(figsize=(12, 10))
        plt.subplots_adjust(left=0.15, bottom=0.3)

        # Initial plot setup
        self.x_values = np.linspace(min, max, points)  # Domain
        self.y_values = self.h_func(self.x_values)  # Compute y-values
        self.plot_line, = self.axis.plot(self.x_values, self.y_values)

    def lambda_func(self, x_values):
        return 5 * np.sin(2 * np.pi * 1 * x_values)

    def h_func(self, x_values):
        return 3 * np.power(pi, -self.lambda_func(x_values))

    def plot(self):
        plt.title("Zoomable Function Plot")
        plt.grid(True)
        plt.show()

class SliderPlot(Plot):
    def __init__(self, min=0, max=2, points=1000):
        super().__init__(min,max,points)
        # Create sliders
        self.x_slider = Slider(
            ax=plt.axes([0.2, 0.15, 0.65, 0.03]),
            label='Zoom X-axis',
            valmin=0.1,
            valmax=10,
            valinit=1,
        )

        self.y_slider = Slider(
            ax=plt.axes([0.05, 0.35, 0.015, 0.5]),  # Adjust to make vertical slider fit
            label='Zoom Y-axis',
            valmin=0.1,
            valmax=10,
            valinit=1,
            orientation="vertical"
        )

        # Add listeners for sliders
        self.x_slider.on_changed(self.update)
        self.y_slider.on_changed(self.update)

        # Create reset button
        self.reset_button = Button(
            ax=plt.axes([0.9, 0.9, 0.1, 0.05]),  # Adjust location for the button
            label="Reset"
        )
        self.reset_button.on_clicked(self.reset)

    def update(self, val):
        # Get the current slider values
        x_zoom = self.x_slider.val
        y_zoom = self.y_slider.val

        # Update the axis limits based on slider values
        new_min = self.min * x_zoom
        new_max = self.max * x_zoom

        # Adjusting the x-limits to avoid clipping of periodicity when zooming out
        self.axis.set_xlim(new_min, new_max)
        self.axis.set_ylim(np.min(self.y_values) * y_zoom, np.max(self.y_values) * y_zoom)

        # Recompute the y-values based on the new x-axis range
        self.x_values = np.linspace(new_min, new_max, self.points)  # Adjust x-values to match the zoom
        self.y_values = self.h_func(self.x_values)

        self.plot_line.set_xdata(self.x_values)
        self.plot_line.set_ydata(self.y_values)

        # Redraw the plot
        self.fig.canvas.draw_idle()

    def reset(self, event):
        # Reset the sliders to their initial values
        self.x_slider.reset()
        self.y_slider.reset()

    def plot(self):
        super().plot()

def main():
    graph = SliderPlot()
    graph.plot()

if __name__ == "__main__":
    main()
