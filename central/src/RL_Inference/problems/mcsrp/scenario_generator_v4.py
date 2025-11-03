"""
Random scenario generator
- Generates separate UGV points, separate UAV points
- Any UAV point is not away > 6.5 km than any UGV point
- Any UAV point is not < 2 km than its closest UGV point
"""

import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import distance
import os
import pandas as pd

# Optional: Set seeds for reproducibility
# np.random.seed(0)
# random.seed(0)

# ---- Initialization -------
output_folder = 'generated_scenarios'
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

def load_ugv_road_network(filepath=r"C:\Users\mmonda4\Downloads\solver\RL_based_solver\Evaluation_ss\ugv_road2.csv"):
    """Load UGV points from CSV file"""
    try:
        df = pd.read_csv(filepath)
        # Fixed: Proper column access
        all_ugv_points = df[['UGV_X', 'UGV_Y']].values.tolist()
        return all_ugv_points
    except FileNotFoundError:
        print(f"Warning: {filepath} not found. Using random points.")
        # Fallback: Generate random road network
        return (np.random.rand(100, 2) * 20).tolist()

#all_ugv_points = load_ugv_road_network()

all_ugv_points = [[1.61, 1.38], [2.0, 1.84], [2.25, 2.19], [2.5, 2.56], [2.81, 2.99], [3.16, 3.44], [3.58, 3.9], [3.92, 4.28], [4.28, 4.67], [1.17, 4.69], [1.69, 4.77], [2.06, 4.78], [2.5, 4.85], [3.06, 4.72], [3.28, 4.96], [3.62, 4.83], [3.97, 5.0], [4.36, 5.06], [0.95, 6.6], [1.36, 6.6], [1.77, 6.36], [2.23, 6.14], [2.56, 6.11], [3.02, 5.74], [3.52, 5.96], [3.48, 5.54], [4.37, 5.38], [7.91, 8.52], [7.44, 8.16], [7.12, 7.74], [6.95, 7.41], [6.69, 7.22], [6.27, 6.81], [5.97, 6.49], [5.7, 6.26], [5.11, 5.83], [8.61, 4.66], [8.03, 4.63], [7.61, 4.89], [7.47, 5.17], [6.8, 5.15], [6.22, 5.42], [6.02, 5.12], [5.56, 5.2], [5.11, 5.19], [5.3, 4.7], [5.52, 4.35], [5.84, 4.1], [6.08, 3.71], [5.77, 3.38], [5.33, 3.14], [4.69, 3.3], [4.94, 3.71], [4.83, 4.58], [5.31, 8.76], [5.31, 8.23], [5.25, 7.82], [5.16, 7.31], [5.09, 7.08], [5.0, 6.75], [4.94, 6.4], [4.86, 6.09], [4.75, 5.72], [3.42, 9.21], [3.62, 8.7], [3.67, 8.36], [3.81, 8.03], [3.98, 7.73], [4.13, 7.28], [4.2, 6.97], [4.33, 6.56], [4.52, 5.93], [3.52, 1.19], [3.56, 1.58], [3.66, 1.98], [3.72, 2.39], [3.86, 2.78], [3.98, 3.16], [4.14, 3.57], [4.3, 3.96], [4.47, 4.43], [8.97, 6.77], [8.47, 6.47], [8.02, 6.29], [7.66, 6.19], [7.19, 6.05], [6.67, 6.02], [6.12, 5.83], [5.5, 5.61], [5.14, 5.5], [5.95, 0.56], [6.06, 0.98], [6.45, 1.47], [6.87, 1.74], [7.47, 2.1], [7.89, 2.37], [8.06, 2.82], [7.83, 3.14], [6.7, 4.02], [4.67, 5.28]]





def sample_in_annulus(center, min_radius, max_radius, num_samples, bounds=None):
    """
    Sample points in an annulus (ring) around center.
    Ensures points are between min_radius and max_radius from center.
    
    Parameters:
    -----------
    center : array-like
        Center point [x, y]
    min_radius : float
        Minimum distance from center
    max_radius : float
        Maximum distance from center
    num_samples : int
        Number of samples to generate
    bounds : tuple, optional
        (x_min, x_max, y_min, y_max) to clip points within boundaries
    """
    angles = np.random.uniform(0, 2 * np.pi, num_samples)
    # Use sqrt for uniform distribution in annulus
    radii = np.sqrt(np.random.uniform(min_radius**2, max_radius**2, num_samples))
    
    x_vals = center[0] + radii * np.cos(angles)
    y_vals = center[1] + radii * np.sin(angles)
    
    points = np.column_stack((x_vals, y_vals))
    
    # Clip to boundaries if specified
    if bounds is not None:
        x_min, x_max, y_min, y_max = bounds
        points[:, 0] = np.clip(points[:, 0], x_min, x_max)
        points[:, 1] = np.clip(points[:, 1], y_min, y_max)
    
    return points


def generate_uav_points(ugv_points, num_uav, min_distance=2.0, max_distance=6.5):
    """
    Generate UAV points with distance constraints:
    - At least min_distance km from closest UGV
    - At most max_distance km from closest UGV
    - All coordinates must be non-negative
    """
    ugv_points = np.array(ugv_points)
    ugv_points_set = set(map(tuple, ugv_points))
    
    # Determine bounds (non-negative only)
    x_min, y_min = 0.0, 0.0
    x_max = np.max(ugv_points[:, 0]) + max_distance
    y_max = np.max(ugv_points[:, 1]) + max_distance
    bounds = (x_min, x_max, y_min, y_max)
    
    # Oversample to account for duplicates and rejections
    oversample_factor = 10  # Increased for boundary rejections
    all_potential_points = []
    
    # Generate candidates around each UGV
    for ugv in ugv_points:
        candidates = sample_in_annulus(
            ugv, min_distance, max_distance, 
            num_uav * oversample_factor // len(ugv_points),
            bounds=bounds
        )
        all_potential_points.append(candidates)
    
    all_potential_points = np.vstack(all_potential_points)
    
    # Filter out negative coordinates (double-check)
    all_potential_points = all_potential_points[
        (all_potential_points[:, 0] >= 0) & (all_potential_points[:, 1] >= 0)
    ]
    
    # Remove duplicates with UGV points
    all_potential_points = np.array([
        point for point in all_potential_points 
        if tuple(np.round(point, 5)) not in ugv_points_set
    ])
    
    if len(all_potential_points) == 0:
        raise ValueError("No valid UAV points found. Adjust distance constraints.")
    
    # Verify distance constraints for all candidates
    valid_points = []
    for point in all_potential_points:
        dists = np.linalg.norm(ugv_points - point, axis=1)
        min_dist = np.min(dists)
        
        if min_distance <= min_dist <= max_distance:
            valid_points.append(point)
    
    valid_points = np.array(valid_points)
    
    if len(valid_points) < num_uav:
        print(f"Warning: Only {len(valid_points)} valid UAV points found (requested {num_uav})")
        return valid_points
    
    # Randomly select required number
    indices = np.random.choice(len(valid_points), num_uav, replace=False)
    uav_points = valid_points[indices]
    
    # Final assertion: all coordinates must be non-negative
    assert np.all(uav_points >= 0), "Generated UAV points contain negative coordinates!"
    
    return uav_points


def scenario_gen(ugv_graph_size=15, uav_graph_size=45, 
                min_uav_ugv_dist=2.0, max_uav_ugv_dist=6.5,
                mandatory_ugv_points=None):
    """
    Generate a complete scenario with UGV and UAV points.
    
    Parameters:
    -----------
    uav_graph_size : int
        Number of UAV points to generate
    ugv_graph_size : int
        Number of UGV points to select
    min_uav_ugv_dist : float
        Minimum distance (km) between UAV and closest UGV
    max_uav_ugv_dist : float
        Maximum distance (km) between UAV and any UGV
    mandatory_ugv_points : list
        Specific UGV points that must be included
    
    Returns:
    --------
    uav_points_miles : np.ndarray
        UAV coordinates in miles
    ugv_points_miles : np.ndarray
        UGV coordinates in miles
    """
    if mandatory_ugv_points is None:
        mandatory_ugv_points = []
    
    # Sample UGV points from road network
    remaining_points = [point for point in all_ugv_points 
                       if point not in mandatory_ugv_points]
    
    if len(remaining_points) < ugv_graph_size - len(mandatory_ugv_points):
        raise ValueError(f"Not enough UGV road points available. "
                        f"Need {ugv_graph_size}, have {len(all_ugv_points)}")
    
    sampled_points = random.sample(remaining_points, 
                                  ugv_graph_size - len(mandatory_ugv_points))
    ugv_points = mandatory_ugv_points + sampled_points
    ugv_points = np.array(ugv_points)
    
    # Convert to miles and check uniqueness
    ugv_points_miles = np.round(ugv_points * 0.62137, 2)
    assert len(ugv_points_miles) == len(set(map(tuple, ugv_points_miles))), \
        "Duplicates found in UGV points"
    
    # Generate UAV points with retries
    # max_attempts = 10
    # for attempt in range(max_attempts):
    while True:
        try:
            # Generate in km first
            uav_points = generate_uav_points(ugv_points, uav_graph_size, 
                                            min_uav_ugv_dist, max_uav_ugv_dist)
            
            # Convert to miles
            uav_points_miles = np.round(uav_points * 0.62137, 2)
            
            # Check UAV uniqueness
            if len(uav_points_miles) != len(set(map(tuple, uav_points_miles))):
                continue
            
            # Check UAV-UGV separation
            uav_set = set(map(tuple, uav_points_miles))
            ugv_set = set(map(tuple, ugv_points_miles))
            
            if uav_set.isdisjoint(ugv_set):
                #print(f"Scenario generated successfully)")
                #visualize_scenario(uav_points_miles, ugv_points_miles)
                return uav_points_miles, ugv_points_miles
        
        except ValueError as e:
            print(f"Attempt failed: {e}")
            continue
    
    raise ValueError(f"Failed to generate valid scenario after {max_attempts} attempts. "
                    f"Try adjusting distance constraints or graph sizes.")


def visualize_scenario(uav_points, ugv_points, save_path=None):
    """Visualize the generated scenario"""
    plt.figure(figsize=(10, 8))
    
    plt.scatter(ugv_points[:, 0], ugv_points[:, 1], 
               c='blue', s=100, marker='s', label='UGV Points', 
               edgecolors='black', linewidths=1.5)
    
    plt.scatter(uav_points[:, 0], uav_points[:, 1], 
               c='red', s=80, marker='^', label='UAV Points', 
               edgecolors='black', linewidths=1.5, alpha=0.7)
    
    plt.xlabel('X Position (miles)', fontsize=12)
    plt.ylabel('Y Position (miles)', fontsize=12)
    plt.title(f'Generated Scenario: {len(ugv_points)} UGVs, {len(uav_points)} UAVs', 
             fontsize=14)
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved visualization to {save_path}")
    
    plt.show()


# # Example usage
# if __name__ == "__main__":
#     # Generate scenario
#     uav_pts, ugv_pts = scenario_gen(
#         uav_graph_size=45, 
#         ugv_graph_size=15,
#         min_uav_ugv_dist=2.0,
#         max_uav_ugv_dist=6.5
#     )
    
#     print(f"\nGenerated:")
#     print(f"  UGV points: {len(ugv_pts)}")
#     print(f"  UAV points: {len(uav_pts)}")
    
#     # Visualize
#     visualize_scenario(uav_pts, ugv_pts, 
#                       save_path=f'{output_folder}/scenario_example.png')
    
#     # Save to CSV
#     pd.DataFrame(ugv_pts, columns=['X', 'Y']).to_csv(
#         f'{output_folder}/ugv_points.csv', index=False)
#     pd.DataFrame(uav_pts, columns=['X', 'Y']).to_csv(
#         f'{output_folder}/uav_points.csv', index=False)