import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

SPHERE_COUNT = 0
CAM_COUNT = 1
IMAGE_SIZE = 2
TOTAL_TIME = 4
SETUP_TIME = 5
RENDER_TIME = 6
READBACK_TIME = 7


DATA_LABELS = {SPHERE_COUNT: "Sphere Count",
               CAM_COUNT: "Camera Count",
               IMAGE_SIZE: "Image Size (pixels)",
               TOTAL_TIME: "Total Time",
               SETUP_TIME: "Setup Time",
               RENDER_TIME: "Render Time",
               READBACK_TIME: "Readback Time"}
IMAGE_LABELS = {320 * 240: "320x240",
                640 * 480: "640x480",
                1280 * 960: "1280x960",
                2560 * 1920: "2560x1920"}

def parse_data(file_path: Path):
    """A block of data looks like this:

    benchmark/engine_name/s/c/w/h/r
    All registered profiles average times:
    Time (s)   Samples   Total Time (s)  Label 
    total_time       160       0.01987654  RenderEngineGl::DoRenderColorImage
    setup_time       160       0.00123456  RenderEngineGl::DoRenderColorImage - setup  # noqa
    render_time       160       0.01543210  RenderEngineGl::DoRenderColorImage - render  # noqa
    readback_time       160       0.00019876  RenderEngineGl::DoRenderColorImage - readback  # noqa

    where
    s is the number of spheres,
    c is the number of cameras,
    w is the width of the image,
    h is the height of the image,
    r is whether readback is enabled (true/false).
    
    Creates a dictionary of 2D tables. Each entry based on the engine_name. In
    the table, each row is a data point and the columns are defined as:

    (spheres, cameras, width * height, readback_enabled, total_time,
     setup_time, render_time, readback_time)
    """
    benchmarks = {}
    with open(file_path, 'r') as f:
        lines = f.readlines()
        l = 0
        while not lines[l].startswith('RenderBenchmark'):
            l += 1  # Skip to the first benchmark line.

        while l < len(lines):
            header = lines[l].strip().replace('true', '1').replace('false', '0')
            tokens = header.split('/')
            name = tokens[1]  # Engine name
            s, c, w, h, r = map(int, tokens[2:])
            if name not in benchmarks:
                benchmarks[name] = []
            line_list = [s, c, w * h, r]
            l += 3  # Skip "All registered profiles ..." line and the header line.
            for _ in range(4):
                line_list.append(float(lines[l].split()[0]))
                l += 1
            benchmarks[name].append(line_list)
    return dict((name, np.array(data)) for name, data in benchmarks.items())


def reduce_plot_data(x, y):
    """Reduces the x and y data by averaging y-values for each unique x-value
    and ordering the x-values in monotonically increasing values."""
    unique_x = np.unique(x)
    reduced_y = []
    for ux in unique_x:
        mask = (x == ux)
        reduced_y.append(np.mean(y[mask]))
    sorted_indices = np.argsort(unique_x)
    unique_x = unique_x[sorted_indices]
    reduced_y = np.array(reduced_y)[sorted_indices]
    return unique_x, reduced_y


def render_time_vs_scene_complexity(data_dict):
    """Plots the render time vs scene complexity for each engine. Each subplot
    corresponds to a different engine, and the x-axis is the number of spheres.
    However, there's a separate line based on image size."""
    fig, axes = plt.subplots(1, len(data_dict), figsize=(10, 5))
    axes = axes.flatten()
    subplot = 0
    for engine_name, data in data_dict.items():
        ax = axes[subplot]
        ax.set_title(engine_name)
        ax.set_xlabel("Sphere Count")
        ax.set_ylabel("Render Time (ms)")
        ax.grid(True)
        render_times = data[:, TOTAL_TIME] * 1000  # Convert to milliseconds
        resolutions = np.unique(data[:, IMAGE_SIZE])
        for resolution in resolutions[::-1]:
            mask = data[:, IMAGE_SIZE] == resolution
            x, y = reduce_plot_data(data[mask, SPHERE_COUNT],
                                    render_times[mask])
            ax.plot(x, y, label=f"Image Size: {IMAGE_LABELS[int(resolution)]}")
        subplot += 1
        ax.legend()
    fig.suptitle("Total Render Time vs Scene Complexity")
    y_limits = [ax.get_ylim()[1] for ax in axes]

    # Force the vertical scale to match to facilitate comparison.
    for ax in axes:
        ax.set_ylim(0, max(y_limits) + 1)
    # plt.legend()
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout to fit title
    plt.subplots_adjust(wspace=0.3)  # Adjust space between subplots


def _add_stacked_time_bars(ax, data, title, blocks, mask, x_axis, tick_maker,
                           offset, name, add_bar_label):
    """Adds stacked bars to the given axis. The bar values are assumed to be
    time values (in seconds).

    Args:
        ax: The matplotlib axis to plot on.
        data: The data array containing the values to plot.
        title: The title of the plot.
        blocks: A list of columns in `data` to stack in the bars.
        mask: A boolean mask to select which rows in the data to use.
        x_axis: The column index for the x-axis values.
        tick_maker: A function that takes x values and returns labels for them.
        offset: Offset for the bar positions (this allows for creating grouped
                bars).
        name: Terse name of the engine; it will get applied to the top of each
              stacked bar.
        add_bar_label: Whether to label the data, leading to an entry in the
                       legend. This should only be true for *one* invocation of
                       this method, otherwise the legend will contain
                       duplicates.
    """
    # We know blocks will only include these two blocks.
    colors = {READBACK_TIME: "#849", RENDER_TIME: "#CB0"}
    width = 0.38
    ax.set_title(title)
    x_values = data[mask, x_axis]

    bottom = np.zeros_like(np.unique(x_values), dtype=float)
    ticks = np.arange(len(bottom))
    for col in blocks:
        label = DATA_LABELS[col] if not add_bar_label else None
        y = data[mask, col] * 1000
        x, y = reduce_plot_data(x_values, y)
        bars = ax.bar(ticks + offset, y, width, label=label,
                        color=colors[col], bottom=bottom)
        ax.set_xticks(ticks)
        ax.set_xticklabels(tick_maker(x))
        bottom = bottom + y
    for i, bar in enumerate(bars):
        ax.text(bar.get_x() + bar.get_width() / 2, bottom[i], name,
                ha='center', va='bottom', fontsize=9)

    ax.legend()
    ax.set_xlabel(DATA_LABELS[x_axis])
    ax.set_ylabel("Render Time (ms)")

def render_phases_vis_dimensions(data_dict):
    fig, axes = plt.subplots(1, len(data_dict), figsize=(10, 5))
    axes = axes.flatten()
    width = 0.38
    offset = -width * 0.5
    BLOCKS = [READBACK_TIME, RENDER_TIME]
    colors = {READBACK_TIME: "#849", RENDER_TIME: "#CB0"}
    first_done = True
    for engine_name, data in data_dict.items():
        name = 'VTK' if 'vtk' in engine_name.lower() else 'GL'
        # Left subplot is a grouped, stacked bar chart of rendering time
        # (broken down by setup, render, and readback) vs sphere count.
        _add_stacked_time_bars(ax=axes[0],
                  data=data,
                  title="Render Time vs Scene Complexity (640x480 image)",
                  blocks=BLOCKS,
                  # Only using the 640x480 image size for this plot.
                  mask=data[:, IMAGE_SIZE] == 640 * 480,
                  x_axis=SPHERE_COUNT,
                  tick_maker=lambda x: [f"{int(x_i)}" for x_i in x],
                  offset=offset,
                  name=name,
                  add_bar_label=first_done)

        # Right subplot is also a grouped, stacked bar chart of rendering time
        # vs image size.
        _add_stacked_time_bars(ax=axes[1],
                  data=data,
                  title="Render Time vs Image Size (1 sphere)",
                  blocks=BLOCKS,
                  # Only using single-sphere counts for this plot.
                  mask=data[:, SPHERE_COUNT] == 1,
                  x_axis=IMAGE_SIZE,
                  tick_maker=lambda x: [IMAGE_LABELS[int(x_i)] for x_i in x],
                  offset=offset,
                  name=name,
                  add_bar_label=first_done)
        offset += width * 1.05
        first_done = False
    fig.suptitle("Sensitivity to Scene Complexity and Image Size")
    plt.legend()
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout to fit title
    plt.subplots_adjust(wspace=0.3)  # Adjust space between subplots


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file_path', type=str, help='Path to the readback profile data file.')
    args = parser.parse_args()

    # Parse the data from the file.
    file_path = Path(args.file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"The file {file_path} does not exist.")

    # Parse the data from the file.
    if not file_path.is_file():
        raise ValueError(f"The path {file_path} is not a valid file.")

    # Parse the data from the file.
    data_dict = parse_data(file_path)

    render_time_vs_scene_complexity(data_dict)
    render_phases_vis_dimensions(data_dict=data_dict)

    plt.show()

if __name__ == "__main__":
    main() 
