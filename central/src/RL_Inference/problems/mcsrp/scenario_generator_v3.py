import numpy as np
import torch
import matplotlib.pyplot as plt


def scenario_gen(
    ugv_graph_size=10,
    uav_graph_size=20,
    area_size=5,
    d_max=7,
    device='cuda',
    use_nearest_order=True,
    min_separation=0.01
):
    """
    Ultra-fast scenario generator with NO duplicates.
    Optimizations:
    - Single-pass UGV generation
    - Vectorized operations
    - Minimal array copies
    - Early termination
    """
    N_ugv = ugv_graph_size
    N_uav = uav_graph_size

    # ============================================================
    # 1️⃣ Robust UGV generation with guaranteed uniqueness
    # ============================================================
    # Calculate theoretical maximum unique points possible
    grid_resolution = 10**5  # Due to rounding to 5 decimals
    max_possible_points = int((area_size * grid_resolution) ** 2)
    
    if N_ugv > max_possible_points * 0.5:  # Safety margin
        raise ValueError(f"Requesting {N_ugv} UGV nodes is too many for area_size={area_size}. "
                        f"Maximum safe: ~{int(max_possible_points * 0.5)}. "
                        f"Increase area_size or reduce ugv_graph_size.")
    
    # Adaptive oversampling based on collision probability
    oversample = max(10, int(N_ugv * 2))
    ugv_loc = np.empty((0, 2), dtype=np.float32)
    
    max_attempts = 10
    for attempt in range(max_attempts):
        if len(ugv_loc) >= N_ugv:
            break
        
        # Generate candidates
        n_needed = N_ugv - len(ugv_loc)
        n_generate = n_needed * oversample
        
        candidates = (np.random.rand(n_generate, 2) * area_size).astype(np.float32)
        candidates = np.round(candidates, decimals=5)
        
        # Combine with existing and get unique
        if len(ugv_loc) > 0:
            combined = np.vstack([ugv_loc, candidates])
        else:
            combined = candidates
        
        ugv_loc = np.unique(combined, axis=0)
        ugv_loc = np.unique(combined, axis=0)
        np.random.shuffle(ugv_loc)
        
        # Increase oversampling if not making progress
        if attempt > 3:
            oversample = int(oversample * 1.5)
    
    if len(ugv_loc) < N_ugv:
        raise ValueError(f"Could not generate {N_ugv} unique UGV points after {max_attempts} attempts. "
                        f"Got {len(ugv_loc)} points. "
                        f"This area_size={area_size} is too small. "
                        f"Try: area_size >= {area_size * np.sqrt(N_ugv / len(ugv_loc)):.1f}")
    
    ugv_loc = ugv_loc[:N_ugv]

    # ============================================================
    # 2️⃣ Fast spatial ordering (optimized nearest-neighbor)
    # ============================================================
    if use_nearest_order:
        # Pre-compute distance matrix (one-time cost)
        dist_matrix = np.sum((ugv_loc[:, None, :] - ugv_loc[None, :, :]) ** 2, axis=2)
        
        remaining = np.ones(N_ugv, dtype=bool)
        path = np.zeros(N_ugv, dtype=np.int32)
        path[0] = 0
        remaining[0] = False
        
        current = 0
        for i in range(1, N_ugv):
            # Mask already visited nodes
            masked_dists = dist_matrix[current].copy()
            masked_dists[~remaining] = np.inf
            current = np.argmin(masked_dists)
            path[i] = current
            remaining[current] = False
        
        ugv_loc = ugv_loc[path]
    else:
        # Simple x-coordinate sort
        ugv_loc = ugv_loc[np.argsort(ugv_loc[:, 0])]

    # ============================================================
    # 3️⃣ Fast adjacency matrix (vectorized)
    # ============================================================
    # Compute all edge distances at once
    edge_vecs = np.diff(ugv_loc, axis=0)  # [N-1, 2]
    edge_dists = np.linalg.norm(edge_vecs, axis=1)  # [N-1]
    
    # Build sparse distance matrix
    dist = np.zeros((N_ugv, N_ugv), dtype=np.float32)
    idx = np.arange(N_ugv - 1)
    dist[idx, idx + 1] = edge_dists
    dist[idx + 1, idx] = edge_dists
    
    adj = (dist > 0).astype(np.float32)

    # ============================================================
    # 4️⃣ Floyd-Warshall (GPU-optimized)
    # ============================================================
    weighted = torch.from_numpy(np.where(adj > 0, dist, np.inf)).to(device)
    
    # Optimized Floyd-Warshall with in-place operations
    sp_matrix = weighted.clone()
    for k in range(N_ugv):
        sp_matrix = torch.minimum(sp_matrix, sp_matrix[:, k:k+1] + sp_matrix[k:k+1, :])

    # ============================================================
    # 5️⃣ Ultra-fast UAV generation (vectorized + smart filtering)
    # ============================================================
    # Adaptive oversampling based on d_max coverage
    coverage_ratio = (np.pi * d_max**2 * N_ugv) / (area_size**2)
    oversample_factor = max(15, int(20 / max(coverage_ratio, 0.1)))
    
    M = N_uav * oversample_factor
    cand_uav = (np.random.rand(M, 2) * area_size).astype(np.float32)
    
    # Single vectorized distance computation
    diff = cand_uav[:, None, :] - ugv_loc[None, :, :]  # [M, N_ugv, 2]
    dists_sq = np.sum(diff**2, axis=-1)  # Avoid sqrt for speed
    min_dists_sq = np.min(dists_sq, axis=1)
    
    # Filter by d_max (using squared distance)
    mask = min_dists_sq <= (d_max**2)
    valid_uav = cand_uav[mask]
    
    if len(valid_uav) == 0:
        raise ValueError(f"No UAV points found within d_max={d_max}. Increase d_max.")
    
    # Remove UAV-UAV duplicates
    valid_uav = np.round(valid_uav, decimals=5)
    valid_uav = np.unique(valid_uav, axis=0)
    
    # Fast UGV-UAV collision check (vectorized)
    if min_separation > 0:
        # Compute squared distances to all UGVs
        uav_ugv_diff = valid_uav[:, None, :] - ugv_loc[None, :, :]
        uav_ugv_dists_sq = np.sum(uav_ugv_diff**2, axis=-1)
        min_uav_ugv_sq = np.min(uav_ugv_dists_sq, axis=1)
        
        # Keep UAVs far enough from UGVs
        no_collision = min_uav_ugv_sq >= (min_separation**2)
        valid_uav = valid_uav[no_collision]
    
    # Handle insufficient UAV points
    if len(valid_uav) < N_uav:
        missing = N_uav - len(valid_uav)
        
        # Fast fallback: vectorized generation around random UGVs
        base_indices = np.random.randint(0, N_ugv, missing * 5)
        bases = ugv_loc[base_indices]
        
        angles = np.random.uniform(0, 2*np.pi, missing * 5)
        radii = np.random.uniform(min_separation, d_max, missing * 5)
        
        offsets = np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
        candidates = np.clip(bases + offsets, 0, area_size).astype(np.float32)
        candidates = np.round(candidates, decimals=5)
        
        # Remove duplicates with existing points
        existing = np.vstack([ugv_loc, valid_uav])
        
        # Fast duplicate check using set operations on tuple representation
        existing_set = set(map(tuple, existing))
        new_points = [c for c in candidates if tuple(c) not in existing_set]
        
        if len(new_points) >= missing:
            uav_loc = np.vstack([valid_uav, new_points[:missing]])
        else:
            raise ValueError(f"Could not generate enough UAV points. Try increasing d_max or area_size.")
    else:
        # Fast random selection without replacement
        if len(valid_uav) == N_uav:
            uav_loc = valid_uav
        else:
            indices = np.random.choice(len(valid_uav), N_uav, replace=False)
            uav_loc = valid_uav[indices]
    
    uav_loc = uav_loc[:N_uav]

    # ============================================================
    # 6️⃣ Fast final verification (optional, can disable for speed)
    # ============================================================
    # Quick duplicate check using set
    all_points = np.vstack([ugv_loc, uav_loc])
    all_tuples = set(map(tuple, all_points))
    
    if len(all_tuples) != len(all_points):
        raise ValueError(f"Duplicate detected! {len(all_points)} points, {len(all_tuples)} unique")
    
    #print(f" Generated {N_ugv} UGV + {N_uav} UAV (no duplicates)")

    # ============================================================
    # 7️⃣ Convert to GPU tensors (efficient transfer)
    # ============================================================
    ugv_loc_t = torch.from_numpy(ugv_loc).to(device)
    uav_loc_t = torch.from_numpy(uav_loc).to(device)

    ugv_np = ugv_loc_t.detach().cpu().numpy() if torch.is_tensor(ugv_loc) else ugv_loc
    uav_np = uav_loc_t.detach().cpu().numpy() if torch.is_tensor(uav_loc) else uav_loc

    plt.figure(figsize=(6, 6))
    plt.scatter(uav_np[:, 0], uav_np[:, 1], c='red', label='UAV points', s=50, alpha=0.7)
    plt.plot(ugv_np[:, 0], ugv_np[:, 1], '-o', c='blue', label='UGV chain', linewidth=2, markersize=6)
    
    

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.xlabel("X position (km)")
    plt.ylabel("Y position (km)")
    plt.title("UGV Chain (Blue) and UAV Points (Red)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()







    return ugv_loc_t, uav_loc_t, sp_matrix