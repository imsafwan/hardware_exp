import sys, os
import yaml



def to_serializable(obj):
    """Recursively convert tensors, numpy numbers, etc. to native Python types."""
    import numpy as np
    import torch
    if isinstance(obj, dict):
        return {k: to_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [to_serializable(v) for v in obj]
    elif isinstance(obj, (np.generic, np.ndarray)):
        return obj.tolist()
    elif torch.is_tensor(obj):
        return obj.detach().cpu().tolist()
    elif isinstance(obj, (float, int, str, bool)) or obj is None:
        return obj
    else:
        return str(obj)

# Add both central/src and RL_Inference to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))  # adds src/
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "RL_Inference"))
import RL_Inference

from RL_Inference.sampling_evaluation import run_inference

# Get the directory of the current script (where scene.yaml is located)
current_script_dir = os.path.dirname(os.path.abspath(__file__))

# Get the directory of the RL_Inference folder (where model and data are located)
module_dir = os.path.dirname(os.path.abspath(RL_Inference.__file__))

# Construct absolute paths
scene_path = os.path.join(current_script_dir, 'Inputs', 'scene.yaml')
model_path = os.path.join(module_dir, 'model', 'DRL', 'epoch-99.pt')
dataset_output = os.path.join(module_dir, 'data', 'mcsrp', 'mcsrp_scene.pkl')

# Optional: Print paths for debugging
print(f"Resolved Paths:")
print(f"  Current script:  {current_script_dir}")
print(f"  Module location: {module_dir}")
print(f"  Scene:           {scene_path}")
print(f"  Model:           {model_path}")
print(f"  Dataset output:  {dataset_output}")

# Verify critical files exist before running
if not os.path.exists(scene_path):
    raise FileNotFoundError(f"Scene file not found: {scene_path}")
if not os.path.exists(model_path):
    raise FileNotFoundError(f"Model file not found: {model_path}")

# Run inference with resolved paths
uav_yaml, ugv_yaml = run_inference(
    scene_path=scene_path,
    model_path=model_path,
    dataset_output=dataset_output,
    width=256 * 4,
    decode_strategy='sample',
    val_size=1
)



# --------------------------------------------------------------------
# ✅ Save the generated YAMLs into `output/` folder relative to current dir
# --------------------------------------------------------------------
output_dir = os.path.join(current_script_dir, "output")
os.makedirs(output_dir, exist_ok=True)

uav_yaml_path = os.path.join(output_dir, "uav_actions.yaml")
ugv_yaml_path = os.path.join(output_dir, "ugv_actions.yaml")

# Convert to serializable
uav_serial = to_serializable(uav_yaml)
ugv_serial = to_serializable(ugv_yaml)

with open(uav_yaml_path, "w") as f:
    if isinstance(uav_serial, str):
        f.write(uav_serial)
    else:
        yaml.safe_dump(uav_serial, f, sort_keys=False)

with open(ugv_yaml_path, "w") as f:
    if isinstance(ugv_serial, str):
        f.write(ugv_serial)
    else:
        yaml.safe_dump(ugv_serial, f, sort_keys=False)

print(f"\n✅ UAV YAML saved to: {uav_yaml_path}")
print(f"✅ UGV YAML saved to: {ugv_yaml_path}")