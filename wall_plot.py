import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.animation import FuncAnimation, FFMpegWriter
from tqdm import tqdm
import os

# def parse_log_file(file_path, x_threshold, y_threshold, log_freq):
#     wall_points = []
#     current_positions = []
#     temp_wall_points = []
#     away_from_wall_bools = []
#     last_tick = 0
#     inVectorToWall = False
#     inReorientTowardsWall = False
#     madeWallContact = False

#     with open(file_path, 'r') as file:
#         lines = file.readlines()

#         for line in lines:
#             tick_difference = 0
#             if line.startswith("Tick:"):
#                 tick_line = line.split(":")[1].strip().split("\t")

#                 current_tick = int(tick_line[0])
#                 tick_difference = current_tick - last_tick if current_tick - last_tick < 50 else 0
                
#                 inVectorToWall = tick_line[1] == "In VectorToWall"
#                 inReorientTowardsWall = tick_line[1] == "In ReorientTowardsWall"
                
#             if inVectorToWall:
#                 if not madeWallContact: madeWallContact = True
#                 if line.startswith("\tAvg Wall Points:"):
#                     temp_wall_points = []
#                     points = line.split(":")[1].strip().split(" ")
#                     for point in points:
#                         x, y = map(float, point.split(","))
#                         temp_wall_points.append((x, y))
#                 elif line.startswith("\tCurrent Position:"):
#                     x, y = map(float, line.split(":")[1].strip().split(","))

#                     if x >= x_threshold and y <= y_threshold:
#                         current_positions.append((x, y))
#                         wall_points.extend(temp_wall_points[-log_freq:])

#             if len(current_positions) > len(away_from_wall_bools):
#                 if tick_difference == 0 and inReorientTowardsWall:
#                     away_from_wall_bools.append(True)

                


            

#             last_tick = current_tick
#             temp_wall_points = []

#     return wall_points, current_positions

def parse_log_file(file_path, x_threshold, y_threshold, log_freq):
    wall_points = []
    current_positions = []
    temp_wall_points = []

    with open(file_path, 'r') as file:
        lines = file.readlines()

        for line in lines:
            if line.startswith("\tAvg Wall Points:"):
                temp_wall_points = []
                points = line.split(":")[1].strip().split(" ")
                for point in points:
                    x, y = map(float, point.split(","))
                    temp_wall_points.append((x, y))
            elif line.startswith("\tCurrent Position:"):
                x, y = map(float, line.split(":")[1].strip().split(","))

                if x >= x_threshold and y <= y_threshold:
                    current_positions.append((x, y))
                    wall_points.append(temp_wall_points[-log_freq:])

                temp_wall_points = []

    return wall_points, current_positions


################################################################################################################################################

def plot_wall_points_and_positions(wall_points, current_positions):
    wall_x, wall_y = zip(*wall_points)
    current_x, current_y = zip(*current_positions)

    plt.figure(figsize=(10, 6))
    plt.scatter(wall_x, wall_y, c='blue', label='Wall Points')
    plt.scatter(current_x, current_y, c='red', label='Current Position', marker='x')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Wall Points and Current Position')
    plt.legend()
    plt.grid(True)
    plt.show()
    plt.savefig('wall_plot.png')



def plot_wall_points_and_positions_with_regression(wall_points, current_positions):
    plt.figure(figsize=(10, 6))

    # Initialize variables to track when the line is crossed
    crossed = False
    cumulative_wall_x = []
    cumulative_wall_y = []
    final_slope, final_intercept = None, None

    for i in range(len(current_positions)):
        current_x, current_y = current_positions[i]

        # Extract the corresponding wall points for the current position
        points = wall_points[i]
        if points:
            wall_x, wall_y = zip(*points)
            cumulative_wall_x.extend(wall_x)
            cumulative_wall_y.extend(wall_y)
        else:
            continue

        if not crossed:
            # Perform linear regression on the cumulative wall points
            slope, intercept = np.polyfit(cumulative_wall_x, cumulative_wall_y, 1)  # Fit a line (degree 1)
            final_slope, final_intercept = slope, intercept  # Store the last slope and intercept

            # Check if the current position crosses the regression line
            y_on_line = slope * current_x + intercept
            if current_y > y_on_line:
                crossed = True
        
        # Plot the wall points and current position
        plt.scatter(wall_x, wall_y, c='blue', label='Wall Points' if i == 0 else "")
        color = 'purple' if crossed else 'red'
        plt.scatter(current_x, current_y, c=color, marker='x')

    # After the loop, plot the final regression line only once
    if final_slope is not None and final_intercept is not None:
        # Ensure the line starts at the first wall point and extends beyond the last point
        extended_wall_x = np.linspace(min(cumulative_wall_x), max(cumulative_wall_x) + 2, 500)
        final_regression_y = final_slope * extended_wall_x + final_intercept
        plt.plot(extended_wall_x, final_regression_y, c='green', label='Final Regression Line')

    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Wall Points, Current Position, and Regression Line')
    plt.legend()
    plt.grid(True)
    plt.savefig('wall_plot_with_regression.png')
    plt.show()

################################################################################################################################################

# Custom FFMpegWriter with progress tracking
class ProgressWriter(FFMpegWriter):
    def __init__(self, *args, total_frames=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.total_frames = total_frames
        self.pbar = tqdm(total=total_frames, desc="Saving Animation")

    def grab_frame(self, **savefig_kwargs):
        super().grab_frame(**savefig_kwargs)
        self.pbar.update(1)

    def finish(self):
        super().finish()
        self.pbar.close()

def update(frame, wall_points, current_positions, scatter_wall, scatter_current, line):
    if frame > 0:
        wall_x, wall_y = zip(*wall_points[:frame])
        current_x, current_y = zip(*current_positions[:frame])

        scatter_wall.set_offsets(np.c_[wall_x, wall_y])
        scatter_current.set_offsets(np.c_[current_x, current_y])

        if len(wall_x) > 1:
            try:
                slope, intercept = np.polyfit(wall_x, wall_y, 1)
            except np.RankWarning:
                slope, intercept = 0, np.mean(wall_y)  # Default to a horizontal line at mean y

            regression_line_y = np.array(wall_x) * slope + intercept
            line.set_data(wall_x, regression_line_y)

    return scatter_wall, scatter_current, line  # Only return the elements that have changed

def create_animation(wall_points, current_positions):
    fig, ax = plt.subplots(figsize=(10, 6))
    scatter_wall = ax.scatter([], [], c='blue', label='Wall Points')
    scatter_current = ax.scatter([], [], c='red', label='Current Position', marker='x')
    line, = ax.plot([], [], c='green', label='Linear Regression Line')

    ax.set_xlim(min([x for x, y in wall_points]) - 1, max([x for x, y in wall_points]) + 1)
    ax.set_ylim(min([y for x, y in wall_points]) - 1, max([y for x, y in wall_points]) + 1)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Wall Points, Current Position, and Regression Line Over Time')
    ax.legend()
    ax.grid(True)

    # Manually control the animation and progress bar
    progress_bar = tqdm(total=len(wall_points), desc="Creating Animation")

    # Directory to save individual frames
    os.makedirs("frames", exist_ok=True)

    for frame in range(len(wall_points)):
        update(frame, wall_points, current_positions, scatter_wall, scatter_current, line)
        plt.savefig(f"frames/frame_{frame:04d}.png")  # Save each frame as an image
        progress_bar.update(1)

    progress_bar.close()

    # Combine frames into a video using ffmpeg
    os.system("ffmpeg -framerate 20 -i frames/frame_%04d.png -c:v libx264 -pix_fmt yuv420p wall_plot_linreg_animation.mp4")
    
    # Cleanup the frames directory
    for file in os.listdir("frames"):
        os.remove(os.path.join("frames", file))
    os.rmdir("frames")

################################################################################################################################################

# Usage
file_path = './wall_plot.txt'
wall_points, current_positions = parse_log_file(file_path, -10, 10, 10) # make sure to adjust last value for log_freq if it changes

# plot_wall_points_and_positions(wall_points, current_positions)
plot_wall_points_and_positions_with_regression(wall_points, current_positions)
# create_animation(wall_points, current_positions)
