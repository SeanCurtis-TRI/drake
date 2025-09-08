import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

SPHERE_COUNT = 0
CAM_COUNT = 1
IMAGE_SIZE = 2
LIGHT_COUNT = 3
SPHERE_TYPE = 4
TOTAL_TIME = 5
SETUP_TIME = 6
RENDER_TIME = 7
READBACK_TIME = 8


DATA_LABELS = {SPHERE_COUNT: "Sphere Count",
               CAM_COUNT: "Camera Count",
               IMAGE_SIZE: "Image Size (pixels)",
               LIGHT_COUNT: "Light Count",
               SPHERE_TYPE: "Sphere Type",
               TOTAL_TIME: "Total Time",
               SETUP_TIME: "Setup Time",
               RENDER_TIME: "Render Time",
               READBACK_TIME: "Readback Time",
               -1: "Render Engine"
               }
IMAGE_LABELS = {320 * 240: "320x240",
                640 * 480: "640x480",
                1280 * 960: "1280x960",
                2560 * 1920: "2560x1920"}

def parse_data(file_path: Path, prefix: str):
    """A block of data looks like this:

    prefixBenchmark/engine_name/s/c/w/h/n/st
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
    n is number of lights,
    st is the type of sphere (0-primitive, 1-textured obj, 2-texture gltf).
    
    Creates a dictionary of 2D tables. Each entry based on the engine_name. In
    the table, each row is a data point and the columns are defined as:

    (spheres, cameras, width * height, light_count, sphere_type, total_time,
     setup_time, render_time, readback_time)
    """
    benchmarks = {}
    with open(file_path, 'r') as f:
        lines = f.readlines()
        l = 0
        while not lines[l].startswith(prefix):
            l += 1  # Skip to the first benchmark line.

        while l < len(lines):
            header = lines[l].strip().replace('true', '1').replace('false', '0')
            tokens = header.split('/')
            name = tokens[1]  # Engine name
            s, c, w, h, lc, st = map(int, tokens[2:])
            if name not in benchmarks:
                benchmarks[name] = []
            line_list = [s, c, w * h, lc, st]
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
            mask = ((data[:, IMAGE_SIZE] == resolution) &
                    (data[:, CAM_COUNT] == 1))
            x, y = reduce_plot_data(data[mask, SPHERE_COUNT],
                                    render_times[mask])
            ax.plot(x, y, label=f"Image Size: {IMAGE_LABELS[int(resolution)]}")
        subplot += 1
        ax.legend()
    fig.suptitle("Total Render Time vs Scene Complexity")
    y_limits = [ax.get_ylim()[1] for ax in axes]

    # Force the vertical scale to match to facilitate comparison.
    for ax in axes:
        ax.set_ylim(0, max(y_limits) * 1.1)
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
        label = DATA_LABELS[col] if add_bar_label else None
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

    if add_bar_label:
        ax.legend()
    ax.set_xlabel(DATA_LABELS[x_axis])
    ax.set_ylabel("Render Time (ms)")

def _add_grouped_time_bars(ax, data, title, group_member, x_axis, mask,
                           tick_maker, label_maker=None):
    """Adds grouped bars to the given axis. The bar values are assumed to be
    time values (in seconds).

    Args:
        ax: The matplotlib axis to plot on.
        data: The data array containing the values to plot.
        title: The title of the plot.
        group_member: The axis in `data` that defines the bar group.
        x_axis: The column index for the x-axis values.
        mask: A boolean mask to select which rows in the data to use.
        tick_maker: A function that takes x values and returns labels for them.
    """
    if label_maker is None:
        label_maker = lambda x: f"{int(x)}"

    COLORS = ["#000", "#d00", "#00d", "#f80", "#b0b", "#dd0"]
    width = 0.1  # Revisit this.
    ax.set_title(title, fontsize=14)
    members = np.unique(data[:, group_member])
    ticks = np.arange(len(members))
    offset = -0.5 * width * (len(members) - 1)
    for i, member in enumerate(members):
        member_mask = data[:, group_member] == member
        x_values = data[member_mask & mask, x_axis]
        y_values = data[member_mask & mask, RENDER_TIME] * 1000
        x, y = reduce_plot_data(x_values, y_values)
        ticks = np.arange(len(x))
        ax.bar(ticks + offset, y, width, label=label_maker(member),
               color=COLORS[i % len(COLORS)])
        offset += width
    ax.set_xticks(ticks)
    ax.set_xticklabels(tick_maker(x))
    ax.legend(title=DATA_LABELS[group_member])
    ax.set_xlabel(DATA_LABELS[x_axis], fontsize=12)
    ax.set_ylabel("Render Time (ms)", fontsize=12)

def render_phases_vis_dimensions(data_dict):
    fig, axes = plt.subplots(1, len(data_dict), figsize=(10, 5))
    axes = axes.flatten()
    width = 0.38
    offset = -width * 0.5
    BLOCKS = [READBACK_TIME, RENDER_TIME]
    needs_labels = True
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
                  add_bar_label=needs_labels)

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
                  add_bar_label=needs_labels)
        offset += width * 1.05
        needs_labels = False
    fig.suptitle("Sensitivity to Scene Complexity and Image Size")
    plt.legend()
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout to fit title
    plt.subplots_adjust(wspace=0.3)  # Adjust space between subplots

def render_time_vs_lights(data_dict):
    """Plots the render time vs number of lights for each engine. Each subplot
    corresponds to a different engine, and the x-axis is the number of lights.
    However, there's a separate line based on image size."""
    fig, axes = plt.subplots(2, len(data_dict), figsize=(12, 11))
    axes = axes.flatten() if len(data_dict) > 1 else [axes]

    # Create four subplots.
    # The top row considers image size (with fixed number of spheres).
    # The bottom row considers sphere count (with fixed image size).
    # In both rows, there are two subplots: one for each render engine. Each
    # subplot has a number of bar groups (one for each unique x-axis value).
    # Each group has one bar for each number of lights.
    axis_index = 0
    for engine_name, data in data_dict.items():
        name = 'VTK' if 'vtk' in engine_name.lower() else 'GL'

        ax = axes[axis_index]
        axis_index += 1

        _add_grouped_time_bars(
            ax=ax,
            data=data,
            title=f"Varying Image Size with One Sphere ({name})",
            group_member=LIGHT_COUNT,
            x_axis=IMAGE_SIZE,
            mask=data[:, SPHERE_COUNT] == 1,
            tick_maker=lambda x: [IMAGE_LABELS[int(x_i)] for x_i in x],
        )
    for engine_name, data in data_dict.items():
        name = 'VTK' if 'vtk' in engine_name.lower() else 'GL'

        ax = axes[axis_index]
        axis_index += 1

        _add_grouped_time_bars(
            ax=ax,
            data=data,
            title=f"Varying Sphere Count in a 2560x1920 Image ({name})",
            group_member=LIGHT_COUNT,
            x_axis=SPHERE_COUNT,
            mask=data[:, IMAGE_SIZE] == 2560 * 1920,
            tick_maker=lambda x: [f"{int(x_i)}" for x_i in x],
        )
    
    # N.B. the vertical axis on the VTK plot with 960 spheres is so much larger
    # than the others that it makes it hard to compare the other plots.
    # Uncomment the following lines to force the vertical scale to match.
    # y_limits = [ax.get_ylim()[1] for ax in axes]
    # for ax in axes:
    #     ax.set_ylim(0, max(y_limits) * 1.1)

    fig.suptitle("The Effect of Lights", fontsize=24)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout to fit title
    plt.subplots_adjust(wspace=0.3)  # Adjust space between subplots


def render_time_vs_textures(data_dict):
    """Produces a number of plots exploring the cost of textures."""
    # Notes:
    #  Hypotheses:
    #    - There is a discernible cost in doing texture look ups over not.
    #      - Fix image size and sphere count (large image, single sphere).
    #      - Single plot with two paired bars.
    #        - The pair consists of a (primitive, textured) times.
    #        - One pair per render engine.
    #      - "Cost of texture lookup"
    #    - There is a per-fragment cost so, as image size grows, rendering
    #      cost increases.
    #      - Fix sphere count (1). Vary image size.
    #      - Single plot with two triples of bars.
    #        - Each triple contains the increasing image sizes.
    #        - One triple per render engine.
    #      - "Per fragment cost"
    #    - VTK incurs an additional cost per texture (i.e., the cost grows
    #      per sphere).
    #      -what plot?

    accum_data = []
    tick_labels = []
    for i, (engine_name, data) in enumerate(data_dict.items()):
        accum_data.append(np.column_stack((data, np.zeros(data.shape[0]) + i)))
        tick_labels.append('VTK' if 'vtk' in engine_name.lower() else 'GL')
    data = np.row_stack(accum_data)

    fig_size = (6, 4)

    _, axes = plt.subplots(1, 1, figsize=fig_size)
    _add_grouped_time_bars(
        ax=axes,
        data=data,
        title=f"Cost of Texture Lookups",
        group_member=SPHERE_TYPE,
        x_axis=-1,  # The added render engine column.
        mask=(data[:, SPHERE_COUNT] == 1) & (data[:, IMAGE_SIZE] == 2560 * 1920),
        tick_maker=lambda x: [tick_labels[int(x_i)] for x_i in x],
        label_maker=lambda x: "Textured" if x == 1 else "Primitive"
    )

    _, axes = plt.subplots(1, 1, figsize=fig_size)
    def plot_line(sphere_type, render_type):
        mask = ((data[:, IMAGE_SIZE] == 2560 * 1920) &
                (data[:, SPHERE_TYPE] == sphere_type) &
                (data[:, -1] == render_type))
        x_values = data[mask, SPHERE_COUNT]
        y_values = data[mask, RENDER_TIME] * 1000
        x, y = reduce_plot_data(x_values, y_values)
        sphere_label = "Textured" if sphere_type == 1 else "Primitive"
        axes.plot(
            x, y, label=f"{tick_labels[render_type]} - {sphere_label}",
            linewidth=2
        )
        axes.set_xticks(x)
        axes.set_xticklabels([f"{int(x_i)}" for x_i in x])

    for render_type in (0, 1):
        for sphere_type in (0, 1):
            plot_line(sphere_type=sphere_type, render_type=render_type)

    axes.legend()
    axes.set_xlabel("Sphere Count", fontsize=12)
    axes.set_ylabel("Render Time (ms)", fontsize=12)
    axes.set_title("VTK Texture Penalty", fontsize=14)


def _print_data(dict_data):
    """Prints the data dictionary in a human-readable format."""
    for engine_name, data in dict_data.items():
        print(engine_name)
        _print_table(data)


def _print_table(table):
    """Prints the table with one row per row; no numpy line wrapping, commas,
    or brackets.
    """
    for row in table:
        print(" ".join([str(x) for x in row]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--readback_path', type=str,
                        help='Path to the readback profile data file.')
    parser.add_argument("--light_path", type=str,
                        help="Path to the light profiling data file.")
    parser.add_argument("--texture_path", type=str,
                        help="Path to the texture profiling data file.")
    args = parser.parse_args()

    def parse_from_file(file_path: Path, prefix: str):
        if not file_path.exists():
            raise FileNotFoundError(f"The file {file_path} does not exist.")
        if not file_path.is_file():
            raise ValueError(f"The path {file_path} is not a valid file.")
        # Parse the data from the file.
        return parse_data(file_path, prefix)

    have_drawn = False
    if args.readback_path:
        data_dict = parse_from_file(Path(args.readback_path), "Readback")
        render_time_vs_scene_complexity(data_dict=data_dict)
        render_phases_vis_dimensions(data_dict=data_dict)
        have_drawn = True
    if args.light_path:
        data_dict = parse_from_file(Path(args.light_path), "Lighting")
        render_time_vs_lights(data_dict=data_dict)
        have_drawn = True
    if args.texture_path:
        data_dict = parse_from_file(Path(args.texture_path), "Texture")
        render_time_vs_textures(data_dict=data_dict)
        have_drawn = True

    if have_drawn:
        plt.show()
    else:
        print("No data to plot. Please provide valid file paths.")

if __name__ == "__main__":
    main() 
