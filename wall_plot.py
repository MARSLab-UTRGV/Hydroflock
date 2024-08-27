import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.animation import FuncAnimation, FFMpegWriter
from tqdm import tqdm
import os
import glob
import argparse
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import UnivariateSpline
from sklearn.linear_model import LinearRegression
from sklearn.neighbors import NearestNeighbors
import pickle
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
from sklearn.cluster import KMeans

LOGDIR = "./controllers/footbot_hydroflock/controller_logs/"

def parse_log_file(file_path, x_threshold, y_threshold, log_freq):

    wall_points = []
    current_positions = []
    temp_wall_points = []
    ds_wall_points = set()
    inVectorToWall = False
    inReorientTowardsWall = False
    inCalculateTangentialMovement = False
    inDoLinearRegression = False
    inWallPointsUsingDistanceScanner = False

    with open(file_path, 'r') as file:
        lines = file.readlines()

        for line in lines:
            if line.startswith("Tick:"):
                tick_line = line.split(":")[1].strip().split("\t")
                inVectorToWall = tick_line[1] == "In VectorToWall"
                inReorientTowardsWall = tick_line[1] == "In ReorientTowardsWall"
                inCalculateTangentialMovement = tick_line[1] == "In CalculateTangentialMovement"
                inDoLinearRegression = tick_line[1] == "In DoLinearRegression"
                inWallPointsUsingDistanceScanner = tick_line[1] == "In WallPointsUsingDistanceScanner"
            
            if inVectorToWall:
                if line.startswith("\tAvg Wall Points:"):
                    temp_wall_points = []
                    points = line.split(":")[1].strip().split(" ")
                    for point_a in points:
                        x, y = map(float, point_a.split(","))
                        temp_wall_points.append((x, y))
                elif line.startswith("\tCurrent Position:"):
                    x, y = map(float, line.split(":")[1].strip().split(","))

                    if x >= x_threshold and y <= y_threshold:
                        current_positions.append((x, y))
                        wall_points.append(temp_wall_points[-log_freq:])

                    temp_wall_points = []
            elif inReorientTowardsWall:
                pass
            elif inCalculateTangentialMovement:
                pass
            elif inDoLinearRegression:
                pass
            elif inWallPointsUsingDistanceScanner:
                if line.startswith("\tDS Wall Points:"):
                    points = line.split(":")[1].strip().split(" ")
                    for point_b in points:
                        if point_b == "": continue
                        x, y = map(float, point_b.split(","))
                        ds_wall_points.add((x, y)) if (x, y) not in ds_wall_points else None


    return wall_points, current_positions, ds_wall_points


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
    # plt.show()

def plot_ds_wall_points(ds_wall_points, current_positions):
    
    plt.figure(figsize=(10, 6))

    batch_size = 100

    ############################################################################################

    def gaussian_smoothing(points_list, sigma):
        points_array = np.array(points_list)
        smoothed_x = gaussian_filter1d(points_array[:, 0], sigma=sigma)
        smoothed_y = gaussian_filter1d(points_array[:, 1], sigma=sigma)
    
        return list(zip(smoothed_x, smoothed_y))
    
    ###############################################

    def moving_average(points_list, window_size):
        """Compute the moving average for a list of (x, y) points."""
        if len(points_list) < window_size:
            raise ValueError("The window size must be less than or equal to the length of the points list.")
        
        moving_averages = []
        for i in range(len(points_list) - window_size + 1):
            window = points_list[i:i + window_size]
            avg_x = sum(point[0] for point in window) / window_size
            avg_y = sum(point[1] for point in window) / window_size
            moving_averages.append((avg_x, avg_y))
        
        return moving_averages

    ###############################################

    def spline_fitting(points_list):
        # Sort points by x-values
        points_list.sort(key=lambda point: point[0])
        
        x, y = zip(*points_list)
        spline = UnivariateSpline(x, y)
        smoothed_x = np.linspace(min(x), max(x), len(x))
        smoothed_y = spline(smoothed_x)
        
        return list(zip(smoothed_x, smoothed_y))
    
    ###############################################
    
    def order_points_nearest_neighbor(points_list, filename='ordered_points_nn.pkl'):
        # Check if the file already exists
        if os.path.exists(filename):
            with open(filename, 'rb') as f:
                ordered_points = pickle.load(f)
            print(f"Loaded ordered points from {filename}")
            return ordered_points
        
        # Proceed with ordering if file doesn't exist
        prog_bar = tqdm(total=len(points_list), desc="Ordering Points Nearest Neighbor")
        ordered_points = [points_list[0]]  # Start with the first point
        points_list = points_list[1:]
        while points_list:
            last_point = ordered_points[-1]
            nearest_idx = np.argmin([np.linalg.norm(np.array(last_point) - np.array(point)) for point in points_list])
            ordered_points.append(points_list.pop(nearest_idx))
            prog_bar.update(1)
        
        prog_bar.close()

        # Save the ordered points to a file
        with open(filename, 'wb') as f:
            pickle.dump(ordered_points, f)
        print(f"Saved ordered points to {filename}")
        print(f"Use the --fresh flag to erase this file and regenerate the ordered points.")

        return ordered_points
    
    ###############################################
    
    def grid_based_clustering(points_list, grid_size=(0.05, 0.05)):
        points = np.array(points_list)

        # Define the grid based on the arena's dimensions
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        
        x_bins = np.arange(x_min, x_max, grid_size[0])
        y_bins = np.arange(y_min, y_max, grid_size[1])
        
        # Create the grid
        grid = np.zeros((len(x_bins), len(y_bins)))
        
        # Populate the grid with point densities
        for point in points:
            x_idx = np.searchsorted(x_bins, point[0]) - 1
            y_idx = np.searchsorted(y_bins, point[1]) - 1
            grid[x_idx, y_idx] += 1
        
        # Normalize grid by max value to classify densities
        max_density = np.max(grid)
        grid_classified = np.zeros_like(grid)
        
        # Define classification thresholds
        thresholds = {
            0: 0.2 * max_density,   # Light
            1: 0.4 * max_density,   # Light-Medium
            2: 0.6 * max_density,   # Medium
            3: 0.8 * max_density,   # Medium-Heavy
            4: 1.0 * max_density    # Full
        }

        for i in range(len(thresholds) - 1):
            grid_classified[(grid > thresholds[i]) & (grid <= thresholds[i + 1])] = i + 1

        # Cells with density higher than 80% of max_density are classified as 'full'
        grid_classified[grid > 0.8 * max_density] = 5
        
        return grid_classified, x_bins, y_bins
    
    ###############################################

    def plot_grid(grid_classified, x_bins, y_bins):
        plt.figure(figsize=(10, 5))
        cmap = plt.cm.get_cmap('Greys', 6)  # Use a colormap with 6 levels (0 for empty, 1-5 for densities)
        plt.imshow(grid_classified.T, origin='lower', cmap=cmap, extent=(x_bins[0], x_bins[-1], y_bins[0], y_bins[-1]))
        cbar = plt.colorbar(label='Density Classification')
        cbar.set_ticks([0, 1, 2, 3, 4, 5])
        cbar.set_ticklabels(['Empty', 'Light', 'Light-Medium', 'Medium', 'Medium-Heavy', 'Full'])
    
    ###############################################

    def grid_based_averaging(points_list, grid_size=(0.1, 0.1)):
        points = np.array(points_list)

        # Define the grid based on the arena's dimensions
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        
        x_bins = np.arange(x_min, x_max, grid_size[0])
        y_bins = np.arange(y_min, y_max, grid_size[1])
        
        # Create the grid and accumulator for points
        grid_x = np.zeros((len(x_bins), len(y_bins)))
        grid_y = np.zeros((len(x_bins), len(y_bins)))
        count_grid = np.zeros((len(x_bins), len(y_bins)))
        
        # Populate the grid with point densities and accumulate positions
        for point in points:
            x_idx = np.searchsorted(x_bins, point[0]) - 1
            y_idx = np.searchsorted(y_bins, point[1]) - 1
            grid_x[x_idx, y_idx] += point[0]
            grid_y[x_idx, y_idx] += point[1]
            count_grid[x_idx, y_idx] += 1
        
        # Calculate average (centroid) for each grid cell
        avg_x = np.divide(grid_x, count_grid, out=np.zeros_like(grid_x), where=count_grid != 0)
        avg_y = np.divide(grid_y, count_grid, out=np.zeros_like(grid_y), where=count_grid != 0)
        
        # Filter out cells without any points
        avg_points = [(avg_x[i, j], avg_y[i, j]) for i in range(len(x_bins)) for j in range(len(y_bins)) if count_grid[i, j] > 0]

        return avg_points

    def plot_grid_averaged_points(avg_points):
        avg_x, avg_y = zip(*avg_points)
        plt.figure(figsize=(10, 5))
        plt.scatter(avg_x, avg_y, c='blue', label='Averaged Points')
        plt.legend()
        plt.grid(True)

    ###############################################

    def plot_points(points_list):

        ds_x, ds_y = [], []

        progress_bar = tqdm(total=len(points_list), desc="Getting DS Wall Points")

        for i, (x, y) in enumerate(points_list):
            ds_x.append(x)
            ds_y.append(y)
            if i % batch_size == 0:
                plt.scatter(ds_x, ds_y, c='orange')
                ds_x, ds_y = [], []
            progress_bar.update(1)

        if ds_x and ds_y:
            plt.scatter(ds_x, ds_y, c='orange', label='DS Wall Points')

        progress_bar.close()

    def get_ds_points(last1k=True):
            
        ds_x, ds_y = [], []

        if last1k: progress_bar = tqdm(total=1000, desc="Getting DS Wall Points")
        else: progress_bar = tqdm(total=len(ds_wall_points), desc="Getting DS Wall Points")

        points_list = list(ds_wall_points)

        if last1k: points_list = points_list[-1000:]

        for i, (x, y) in enumerate(points_list):
            ds_x.append(x)
            ds_y.append(y)
            progress_bar.update(1)

        progress_bar.close()

        # return the list of points
        return list(zip(ds_x, ds_y))

    ###############################################    

    def apply_dbscan_to_averaged_points(avg_points, eps=0.05, min_samples = 3):
        points_array = np.array(avg_points)
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(points_array)
        return labels

    def plot_dbscan_clusters(avg_points, labels):
        plt.figure(figsize=(10, 6))

        # Define a list of visually distinct colors
        predefined_colors = [
            'red', 'blue', 'green', 'purple', 'orange', 'brown', 'pink', 'gray', 'olive', 'cyan'
        ]
        unique_labels = set(labels)

        for k in unique_labels:
            if k == -1:
                # Black used for noise.
                col = 'black'
                mark = 'x'
            else:
                # Use predefined colors, cycle if there are more clusters than colors
                col = predefined_colors[k % len(predefined_colors)]
                mark = 'o'

            class_member_mask = (labels == k)
            xy = np.array(avg_points)[class_member_mask]
            plt.scatter(xy[:, 0], xy[:, 1], c=col, marker=mark, label=f'Cluster {k}')

    ###############################################

    def dbscan_clustering(points, eps=0.1, min_samples=5):
        """
        Perform DBSCAN clustering on the provided points.

        Parameters:
        - points: A list or array of (x, y) coordinates representing the points to cluster.
        - eps: The maximum distance between two samples for them to be considered as in the same neighborhood.
        - min_samples: The number of samples in a neighborhood for a point to be considered as a core point.

        Returns:
        - db: The fitted DBSCAN model.
        - labels: Array of cluster labels assigned to each point (-1 indicates noise).
        """
        # Convert points to numpy array if they aren't already
        points_array = np.array(points)
        
        # Perform DBSCAN clustering
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points_array)
        
        # Extract labels (each point's cluster assignment)
        labels = db.labels_

        return db, labels

    ###############################################

    def fit_lines_to_clusters(points, labels):
        unique_labels = np.unique(labels)
        line_fits = []

        for label in unique_labels:
            if label == -1:
                continue  # Skip noise points

            # Extract the points for this cluster
            cluster_points = np.array([points[i] for i in range(len(points)) if labels[i] == label])
            
            # Fit a line using simple linear regression
            x = cluster_points[:, 0].reshape(-1, 1)
            y = cluster_points[:, 1].reshape(-1, 1)
            
            model = LinearRegression()
            model.fit(x, y)
            line_fits.append((model, label))

        return line_fits

    def plot_lines_with_clusters(points, labels, line_fits):
        plt.figure(figsize=(10, 6))
        
        # Convert points to numpy array if it's a list of tuples
        points = np.array(points)
        
        # Plot the clusters
        unique_labels = np.unique(labels)
        colors = plt.cm.jet(np.linspace(0, 1, len(unique_labels)))

        for k, col in zip(unique_labels, colors):
            if k == -1:
                # Black used for noise.
                col = [0, 0, 0, 1]

            class_member_mask = (labels == k)
            xy = points[class_member_mask]
            plt.scatter(xy[:, 0], xy[:, 1], c=[col], label=f'Cluster {k}' if k != -1 else 'Noise')

        # Plot the fitted lines within the range of each cluster's data
        for model, label in line_fits:
            cluster_mask = (labels == label)
            cluster_points = points[cluster_mask]
            
            if len(cluster_points) > 0:
                # Determine the x range within the current cluster's data
                line_x = np.linspace(min(cluster_points[:, 0]), max(cluster_points[:, 0]), 100)
                line_y = model.predict(line_x.reshape(-1, 1))
                plt.plot(line_x, line_y, label=f'Line Fit Cluster {label}', linewidth=2)


    ###############################################

    def fit_ransac_to_points(avg_points):
        points_array = np.array(avg_points)
        x = points_array[:, 0].reshape(-1, 1)  # Reshape x for sklearn
        y = points_array[:, 1]

        # Apply RANSAC for robust line fitting
        ransac = RANSACRegressor()
        ransac.fit(x, y)
        
        # Predict the y values for the fitted line
        line_x = np.linspace(x.min(), x.max(), 100)
        line_y = ransac.predict(line_x.reshape(-1, 1))
        
        return line_x, line_y, ransac

    def plot_ransac_results(avg_points, line_x, line_y, ransac):
        plt.figure(figsize=(10, 6))
    
        # Convert avg_points to array for indexing
        points_array = np.array(avg_points)
        x = points_array[:, 0].reshape(-1, 1)
        y = points_array[:, 1]

        # Predict the y values for the fitted line (using original x points)
        line_y = ransac.predict(x)
        
        # Plot the original averaged points
        plt.scatter(x, y, c='blue', label='Averaged Points')
        
        # Plot the RANSAC fitted line
        plt.plot(x, line_y, color='red', label='RANSAC Fitted Line', linewidth=2)
        
        # Highlight the inliers used by RANSAC
        inlier_mask = ransac.inlier_mask_
        
        plt.scatter(x[inlier_mask], y[inlier_mask], c='yellow', marker='o', label='Inliers')
        plt.scatter(x[~inlier_mask], y[~inlier_mask], c='green', marker='x', label='Outliers')
        
        plt.grid(True)

    ###############################################

    def multi_segment_ransac(avg_points, min_inliers=5, residual_threshold=0.05):
        points_array = np.array(avg_points)
        segments = []
        remaining_points = points_array

        while len(remaining_points) > min_inliers:
            x = remaining_points[:, 0].reshape(-1, 1)
            y = remaining_points[:, 1]

            # Apply RANSAC for robust line fitting
            ransac = RANSACRegressor(residual_threshold=residual_threshold)
            ransac.fit(x, y)

            inlier_mask = ransac.inlier_mask_
            inliers = remaining_points[inlier_mask]

            # If the number of inliers is sufficient, record the segment
            if len(inliers) >= min_inliers:
                segments.append((ransac.estimator_.coef_[0], ransac.estimator_.intercept_, inliers))
                # Remove inliers from the remaining points
                remaining_points = remaining_points[~inlier_mask]
            else:
                break

        return segments, remaining_points

    def plot_segments_with_ransac(segments, remaining_points):
        plt.figure(figsize=(10, 6))

        # Plot the remaining outliers
        if len(remaining_points) > 0:
            plt.scatter(remaining_points[:, 0], remaining_points[:, 1], c='green', marker='x', label='Remaining Points')

        # Plot each segment
        colors = plt.cm.jet(np.linspace(0, 1, len(segments)))
        for i, (slope, intercept, inliers) in enumerate(segments):
            x_vals = np.linspace(inliers[:, 0].min(), inliers[:, 0].max(), 100)
            y_vals = slope * x_vals + intercept
            plt.plot(x_vals, y_vals, color=colors[i], label=f'Segment {i+1}')
            plt.scatter(inliers[:, 0], inliers[:, 1], c=[colors[i]], marker='o', label=f'Inliers {i+1}')

    def fit_ransac_to_points_any_orientation(avg_points, residual_threshold=0.1):
        points_array = np.array(avg_points)
        x = points_array[:, 0].reshape(-1, 1)
        y = points_array[:, 1]
        
        # Fit using RANSAC for both (x, y) and (y, x) and choose the best
        ransac_xy = RANSACRegressor(residual_threshold=residual_threshold)
        ransac_xy.fit(x, y)
        residuals_xy = np.abs(y - ransac_xy.predict(x))

        ransac_yx = RANSACRegressor(residual_threshold=residual_threshold)
        ransac_yx.fit(y.reshape(-1, 1), x)
        residuals_yx = np.abs(x - ransac_yx.predict(y.reshape(-1, 1)))

        if residuals_xy.mean() < residuals_yx.mean():
            return ransac_xy, x, y, ransac_xy.predict(x)
        else:
            return ransac_yx, y, x, ransac_yx.predict(y.reshape(-1, 1))

    def plot_ransac_results_any_orientation(ransac, x, y, line):
        plt.figure(figsize=(10, 6))
        plt.scatter(x, y, c='blue', label='Averaged Points')
        plt.plot(x, line, color='red', label='RANSAC Fitted Line', linewidth=2)
        
        inlier_mask = ransac.inlier_mask_
        plt.scatter(x[inlier_mask], y[inlier_mask], c='yellow', marker='o', label='Inliers')
        plt.scatter(x[~inlier_mask], y[~inlier_mask], c='green', marker='x', label='Outliers')

    ###############################################  

    def kmeans_clustering(points, n_clusters=3):
        points_array = np.array(points)
        kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(points_array)
        labels = kmeans.labels_
        return kmeans, labels

    def plot_kmeans_clusters(points, labels, kmeans):
        plt.figure(figsize=(10, 6))
        
        # Plot each cluster with a different color
        unique_labels = np.unique(labels)
        colors = plt.cm.jet(np.linspace(0, 1, len(unique_labels)))

        for i, label in enumerate(unique_labels):
            cluster_points = np.array(points)[labels == label]
            plt.scatter(cluster_points[:, 0], cluster_points[:, 1], c=[colors[i]], label=f'Cluster {label+1}')

        # Plot cluster centers
        plt.scatter(kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1], c='black', marker='x', s=100, label='Centroids')

    ############################################### 

    def piecewise_linear_approximation(points, tolerance=0.05):
        points = np.array(points)
        start_index = 0
        segments = []

        while start_index < len(points) - 1:
            end_index = start_index + 1
            while end_index < len(points):
                segment_points = points[start_index:end_index + 1]
                x = segment_points[:, 0]
                y = segment_points[:, 1]
                
                # Fit a line to the points
                coefficients = np.polyfit(x, y, 1)
                fitted_y = np.polyval(coefficients, x)
                
                # Calculate the deviation from the line
                residuals = np.abs(fitted_y - y)
                
                if np.max(residuals) > tolerance:
                    break
                end_index += 1

            # Append the current segment
            segments.append(points[start_index:end_index])
            start_index = end_index

        return segments

    def plot_piecewise_segments(segments):
        plt.figure(figsize=(10, 6))

        # Plot each segment with a different color
        colors = plt.cm.jet(np.linspace(0, 1, len(segments)))
        for i, segment in enumerate(segments):
            x, y = segment[:, 0], segment[:, 1]
            plt.plot(x, y, color=colors[i], label=f'Segment {i+1}')
            plt.scatter(x, y, c=colors[i], marker='o')


    ############################################################################################

    # order_points_nearest_neighbor(list(ds_wall_points))

    # filtered_points = gaussian_smoothing(order_points_nearest_neighbor(list(ds_wall_points)), sigma=10)
    # filtered_points = moving_average(order_points_nearest_neighbor(list(ds_wall_points)), window_size=5)
    
    ### Apply grid-based clustering
    # grid_classified, x_bins, y_bins = grid_based_clustering(list(ds_wall_points), grid_size=(0.05, 0.05))

    # Plot density classification grid
    # plot_grid(grid_classified, x_bins, y_bins)

    ### Apply grid-based averaging
    # avg_points = grid_based_averaging(get_ds_points(last1k=True), grid_size=(0.125, 0.125))

    ### Trying DBSCAN with all the points
    # labels = apply_dbscan_to_averaged_points(get_ds_points(last1k=True), eps=0.15, min_samples=5)
    # plot_dbscan_clusters(get_ds_points(last1k=True), labels)

    ### Fitting lines to dbscan clusters
    db_model, db_labels = dbscan_clustering(get_ds_points(last1k=True), eps=0.15, min_samples=5)  # Adjust eps and min_samples as needed
    line_fits = fit_lines_to_clusters(get_ds_points(last1k=True), db_labels)
    plot_lines_with_clusters(get_ds_points(last1k=True), db_labels, line_fits)

    ### Trying DBSCAN with the average points
    # labels = apply_dbscan_to_averaged_points(avg_points, eps=0.15, min_samples=2)
    # plot_dbscan_clusters(avg_points, labels)

    ### plot a single ransac line
    # line_x, line_y, ransac_model = fit_ransac_to_points(avg_points)
    # plot_ransac_results(avg_points, line_x, line_y, ransac_model)

    ### trying segmenting using ransac
    # segments, remaining_points = multi_segment_ransac(avg_points)
    # plot_segments_with_ransac(segments, remaining_points)

    ### trying ransac with any orientation
    # ransac_model, x_used, y_used, line_y = fit_ransac_to_points_any_orientation(avg_points)
    # plot_ransac_results_any_orientation(ransac_model, x_used, y_used, line_y)

    ### Plot the averaged points
    # plot_grid_averaged_points(avg_points)

    ### KMeans clustering
    # kmeans_model, kmeans_labels = kmeans_clustering(avg_points, n_clusters=3)
    # plot_kmeans_clusters(avg_points, kmeans_labels, kmeans_model)

    ### Piecewise linear approximation
    # segments = piecewise_linear_approximation(avg_points, tolerance=0.05)
    # plot_piecewise_segments(segments)

    ### print out last 1k points
    # plot_points(get_ds_points(last1k=True))

    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title(f'Wall Points from Distance Scanner ({args.id})')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'ds_wall_plot_{args.id}.png')

def get_latest_directory(parent_directory):
    # Use glob to get all subdirectories in the parent directory
    all_subdirs = [d for d in glob.glob(parent_directory + "/*") if os.path.isdir(d)]

    # Sort directories by creation time
    latest_directory = max(all_subdirs, key=os.path.getmtime)

    return latest_directory

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

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Plot wall points and current positions from controller log file')

    parser.add_argument('-i', '--id', type=str, help='ID of the controller log file to plot (e.g. "fb2")', required=True)

    parser.add_argument('-d', '--directory', type=str, help='Specific directory containing controller log files (format example: Aug22_03-52PM) (default is the most recent)', default=get_latest_directory(LOGDIR))

    parser.add_argument('-f', '--fresh', action='store_true', help="Use this flag to erase old ordered_points_nn.pkl files and create a new one.")

    args = parser.parse_args()

    if args.fresh:
        for file in os.listdir("./"):
            if file.endswith(".pkl"):
                # prompt user to confirm deletion
                if input(f"Delete {file}? (y/n): ").lower() == 'y':
                    print(f"Removing {file}")
                    os.remove(os.path.join("./", file))
                else:
                    if input("Continue with the script? (y/n): ").lower() != 'y':
                        exit()

    file_path = f"{args.directory}/{args.id}_adr_dev.log"

    print(f'file_path = {file_path}')

    wall_points, current_positions, ds_wall_points = parse_log_file(file_path, -10, 10, 10) # make sure to adjust last value for log_freq if it changes

    print(f'ds_wall_points length = {len(ds_wall_points)}')

    # plot_wall_points_and_positions(wall_points, current_positions)
    # plot_wall_points_and_positions_with_regression(wall_points, current_positions)
    plot_ds_wall_points(ds_wall_points, current_positions)
    # create_animation(wall_points, current_positions)
