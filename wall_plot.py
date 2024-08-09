import matplotlib.pyplot as plt

def parse_log_file(file_path, x_threshold):
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


                if x >= x_threshold:
                    current_positions.append((x, y))
                    wall_points.extend(temp_wall_points)
                temp_wall_points = []

    return wall_points, current_positions


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

# Usage
file_path = './wall_plot.txt'
wall_points, current_positions = parse_log_file(file_path, -0.5)

plot_wall_points_and_positions(wall_points, current_positions)
