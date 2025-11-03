# -*- coding: utf-8 -*-
"""
@author: safwan
Evaluate trained DRL model for UAV-UGV cooperative routing (MCSRP)
Decoding: Sampling (default)
"""

import os
import math
import time
import numpy as np
import torch
import pandas as pd
from tqdm import tqdm
from termcolor import cprint
from datetime import timedelta
from torch.utils.data import DataLoader

from utils import load_model, move_to
from utils.data_utils import save_dataset
from Custom_environment_v1 import UAV_UGV_Env
from action_yaml_wrapper import get_yaml
from data_generate import generate_dataset_from_scene

env = UAV_UGV_Env()
torch.manual_seed(0)


# ======================================================
#  Utility Functions
# ======================================================
def remove_consecutive_duplicates(a_list):
    """Remove consecutive duplicate elements."""
    if not a_list:
        return []
    new_list = [a_list[0]]
    for i in range(1, len(a_list)):
        if a_list[i] != a_list[i - 1]:
            new_list.append(a_list[i])
    return new_list


def get_best(sequences, cost, agents, ids=None, batch_size=None):
    """Select the lowest-cost trajectory for each instance."""
    if ids is None:
        idx = cost.argmin()
        kkk
        return sequences[idx:idx+1, ...], cost[idx:idx+1, ...]
    splits = np.hstack([0, np.where(ids[:-1] != ids[1:])[0] + 1])
    mincosts = np.minimum.reduceat(cost, splits)
    group_lengths = np.diff(np.hstack([splits, len(ids)]))
    all_argmin = np.flatnonzero(np.repeat(mincosts, group_lengths) == cost)
    result = np.full(len(group_lengths) if batch_size is None else batch_size, -1, dtype=int)
    result[ids[all_argmin[::-1]]] = all_argmin[::-1]
    return [sequences[i] if i >= 0 else None for i in result], \
           [cost[i] if i >= 0 else math.inf for i in result], \
           [agents[i] if i >= 0 else None for i in result]


# ======================================================
#  Core Evaluation Function
# ======================================================
def eval_dataset(dataset_path, model_path, uav_graph_size=30, ugv_graph_size=10,
                 val_size=1, width=1024, batch_size = 256, decode_strategy="sample", softmax_temperature=1,
                 results_dir="results", save_csv=True, scenario_yaml="Inputs/scene.yaml"):
    """
    Evaluate DRL model on MCSRP dataset and return UAV/UGV YAML actions.
    """

    model, _ = load_model(model_path, 5)
    use_cuda = torch.cuda.is_available()
    device = torch.device("cuda:0" if use_cuda else "cpu")

    # Dataset creation
    dataset = model.problem.make_dataset(
        uav_graph_size=uav_graph_size,
        ugv_graph_size=ugv_graph_size,
        num_samples=val_size,
        filename=dataset_path
    )

    model.to(device)
    model.eval()
    model.set_decode_type("greedy" if decode_strategy == "greedy" else "sampling", temp=softmax_temperature)

    dataloader = DataLoader(dataset, batch_size=1, shuffle=False)
    results = []

    folder = "./Tours_output"
    os.makedirs(folder, exist_ok=True)
    full_path = os.path.join(folder, "actions_RL_sampling.csv")

    if os.path.isfile(full_path):
        os.remove(full_path)

    for batch_id, batch in enumerate(tqdm(dataloader, desc="Evaluating")):
        batch = move_to(batch, device)

        start = time.time()
        with torch.no_grad():
            batch_rep = batch_size
            iter_rep = width // batch_size
            sequences, costs, agents = model.sample_many(batch, batch_rep=batch_rep, iter_rep=iter_rep)
            cprint(f'Batch {batch_id + 1}: Completed sampling', 'yellow', attrs=['bold'])

        batch_size = len(costs)
        ids = torch.arange(batch_size, dtype=torch.int64, device=costs.device)

        sequences, costs, agents = get_best(
                sequences.cpu().numpy(), costs.cpu().numpy(), agents.cpu().numpy(),
                ids.cpu().numpy() if ids is not None else None,
                batch_size
            )
        duration = time.time() - start

        

        for seq, cost, agent in zip(sequences, costs, agents):
            seq = seq.tolist()
            agent = agent.tolist()
            actions = remove_consecutive_duplicates(list(zip(seq, agent)))
            results.append((cost, actions, duration))

    # Extract results
    costs, tours, durations = zip(*results)
    #print("\n--- Evaluation Summary ---")
    #print(f"Average cost: {np.mean(costs):.3f} ± {2*np.std(costs)/np.sqrt(len(costs)):.3f}")
    #print(f"Total duration: {timedelta(seconds=int(np.sum(durations)))}")

    # Convert actions to YAMLs
    yaml_result = get_yaml([pd.Series(tours).iloc[0]], scenario_yaml)[0]

    #print("\n--- UAV Action YAML ---")
    #print(yaml_result["uav_action_yaml"])
    #print("\n--- UGV Action YAML ---")
    #print(yaml_result["ugv_action_yaml"])
    #print(f"\nTotal mission time: {yaml_result['total_mission_time']} s")

    # Optionally save results
    if save_csv:
        df_out = pd.DataFrame({'route': pd.Series(tours), 'Costs': pd.Series(costs)})
        df_out.to_csv(full_path, index=False)
        print(f"\nSaved routes to {full_path}")

    return yaml_result["uav_action_yaml"], yaml_result["ugv_action_yaml"]


# ======================================================
#  High-Level Wrapper for External Use
# ======================================================
def run_inference(
    scene_path='Inputs/scene.yaml',
    model_path='model/DRL/epoch-99.pt',
    dataset_output='data/mcsrp/mcsrp_scene.pkl',
    ref_lat=41.869884,
    ref_lon=-87.650660,
    batch_size=256,
    overwrite=True,
    **kwargs
):
    """
    High-level wrapper for inference:
    1. Generate dataset from a scene YAML.
    2. Run DRL model inference.
    3. Return UAV and UGV action YAMLs.
    """
    print("\n[Step 1] Generating dataset from scene...")
    generate_dataset_from_scene(
        scene_path=scene_path,
        num_samples=1,  # single scenario for inference
        ref_lat=ref_lat,
        ref_lon=ref_lon,
        output_path=dataset_output,
        overwrite=overwrite
    )

    print("\n[Step 2] Running model inference...")
    uav_yaml, ugv_yaml = eval_dataset(
        dataset_path=dataset_output,
        model_path=model_path,
        scenario_yaml=scene_path,
        batch_size=256,
        **kwargs
    )

    print("\n✅ Inference complete.")
    return uav_yaml, ugv_yaml


# uav_yaml, ugv_yaml = run_inference(
#     scene_path='Inputs/scene.yaml',
#     model_path='model/DRL/epoch-99.pt',
#     dataset_output='data/mcsrp/mcsrp_scene.pkl',
#     width=256*4,
#     decode_strategy='sample',
#     val_size=1
# )