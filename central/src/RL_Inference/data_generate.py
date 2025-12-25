# -*- coding: utf-8 -*-
"""
@author: safwan
Generate dataset for UAV-UGV cooperative mission (MCSRP)
"""

import os
import torch
from utils.data_utils import save_dataset, check_extension
from RL_scene_helper import scene_to_dataset

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")



# ======================================================
#  Core Generator
# ======================================================
def generate_mcsrp_data(scene_path, num_samples, ref_lat, ref_lon):
    """
    Generate dataset entries for UAV-UGV missions from a scene YAML file.
    Args:
        scene_path (str): Path to the YAML scene file.
        num_samples (int): Number of dataset samples.
        ref_lat (float): Reference latitude for ENU conversion.
        ref_lon (float): Reference longitude for ENU conversion.
    Returns:
        list of dicts containing mission data.
    """
    data = []

    for _ in range(num_samples):
        # Extract points and distance matrix
        ugv_loc, uav_loc, sp_matrix, depot_ix = scene_to_dataset(
            scene_path,
            ref_lat=ref_lat,
            ref_lon=ref_lon,
            device=device,
            visualize=False
        )

        uav_points = uav_loc.to(device)
        ugv_points = ugv_loc.to(device)
        
        unique_points = torch.cat([ugv_points, uav_points], dim=0)
        depot_ix = torch.tensor([depot_ix], device=device)

        

        # Build encoder input [x, y, agent_type]
        ones = torch.ones(uav_points.size(0), 1, device=device)
        zeros = torch.zeros(ugv_points.size(0), 1, device=device)
        encoder_space = torch.cat([
            torch.cat([ugv_points, zeros], dim=-1),  # UGV nodes → type 0
            torch.cat([uav_points, ones], dim=-1)    # UAV nodes → type 1
        ], dim=0)

        data.append({
            'depot': depot_ix,
            'uav loc': uav_points,
            'ugv loc': ugv_points,
            'encoder space': encoder_space,
            'unique loc': unique_points,
            'sp_matrix': sp_matrix
        })

    return data


# ======================================================
#  Public Function
# ======================================================
def generate_dataset_from_scene(
    scene_path,
    num_samples=256,
    ref_lat=41.869884,
    ref_lon=-87.650660,
    output_path='data/mcsrp/mcsrp_scene.pkl',
    overwrite=False
):
    """
    Generate and optionally save UAV-UGV dataset.
    Args:
        scene_path (str): YAML scene path.
        num_samples (int): Number of samples to generate.
        ref_lat, ref_lon (float): ENU reference coordinates.
        output_path (str, optional): If provided, saves dataset to file.
        overwrite (bool): Overwrite file if exists.
    Returns:
        list: Generated dataset.
    """
    dataset = generate_mcsrp_data(scene_path, num_samples, ref_lat, ref_lon)

    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        filename = check_extension(output_path)
        if not overwrite and os.path.isfile(filename):
            raise FileExistsError(f"{filename} already exists. Use overwrite=True.")
        save_dataset(dataset, filename)
        print(f" Dataset saved to: {filename}")

    return dataset