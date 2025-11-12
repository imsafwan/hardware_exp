import matplotlib.pyplot as plt

# ------------------------------
# Parameters
# ------------------------------
total_fuel = 100.0  # seconds
safety = 0.90
fuel_max = total_fuel * safety  # seconds

# ------------------------------
# Mission data
# ------------------------------
sim_plan = {
    "to_1": 12.0, "mov1": 23.93, "mov2": 38.66, "mov3": 45.46,
    "mov4": 53.3, "mov5": 68.36, "land1": 88.36
}

# === PHASE 1 ===
executed_1 = {"to_1": 12.2, "mov1": 31.7}
replanned_1 = {
    "mov2": 48.17, "mov3": 54.89, "mov4": 65.89,  "land1": 85.89
}
delay_1 = executed_1["mov1"] - sim_plan["mov1"]
if_not_replanned_1 = {k: v + delay_1 for k, v in sim_plan.items() if k not in executed_1.keys()}

# === PHASE 2 ===
executed_2 = {"mov2": 52.5}
replanned_2 = {"mov3": 65.4, "land1": 85.4}
delay_2 = executed_2["mov2"] - replanned_1["mov2"]
if_not_replanned_2 = {k: v + delay_2 for k, v in replanned_1.items() if k not in executed_2.keys()}


# Hardware executed example
other_executed = {"mov3": 68.1,  "land1": 89.1}
all_executed = {**executed_1, **executed_2, **other_executed}


# ------------------------------
# Utility
# ------------------------------
def calc_durations(plan):
    """Return (labels, starts, durations) from cumulative plan dictionary."""
    times = list(plan.values())
    starts = [0] + times[:-1]
    durs = [t - s for s, t in zip(starts, times)]
    return list(plan.keys()), starts, durs


# ------------------------------
# Plot setup
# ------------------------------
fig, ax = plt.subplots(figsize=(8, 4.5))
bar_height = 0.6

# ---- Row 1: Simulation baseline ----
labels, starts, durs = calc_durations(sim_plan)
y_sim = 4.5
for label, start, dur in zip(labels, starts, durs):
    ax.barh(y_sim, dur, left=start, height=bar_height,
            color='#1F77B4', alpha=0.6, edgecolor='#155A8A', linewidth=1)
#     ax.text(start + dur / 2, y_sim, label, ha='center', va='center',
#             fontsize=9, fontweight='bold', color='white')

# =========================
# === PHASE 1 EXECUTION ===
# =========================
labels, starts, durs = calc_durations(executed_1)
y_exec1 = 3.2
for label, start, dur in zip(labels, starts, durs):
    ax.barh(y_exec1, dur, left=start, height=bar_height,
            color='#2ECC71', alpha=0.7, edgecolor='#1E8449', linewidth=1)
#     ax.text(start + dur / 2, y_exec1, label, ha='center', va='center',
#             fontsize=9, fontweight='bold', color='white')
execution_stop_1 = starts[-1] + durs[-1]

# --- Phase 1: Replanned path ---
labels, starts, durs = calc_durations(replanned_1)
starts[0] = starts[0] + execution_stop_1
durs[0] = starts[1] - starts[0]
y_replan1 = y_exec1 - bar_height / 4
for label, start, dur in zip(labels, starts, durs):
    ax.barh(y_replan1, dur, left=start, height=bar_height / 2,
            color='#17A589', alpha=0.7, edgecolor='#117864', linewidth=1)
#     ax.text(start + dur / 2, y_replan1, label, ha='center', va='center',
#             fontsize=9, fontweight='bold', color='white')

# --- Phase 1: If not replanned ---
labels, starts, durs = calc_durations(if_not_replanned_1)
starts[0] = starts[0] + execution_stop_1
durs[0] = starts[1] - starts[0]
y_ifnot1 = y_exec1 + bar_height / 4
for label, start, dur in zip(labels, starts, durs):
    ax.barh(y_ifnot1, dur, left=start, height=bar_height / 2,
            color='#E67E22', alpha=0.7, edgecolor='#AF601A', linewidth=1)
#     ax.text(start + dur / 2, y_ifnot1, label, ha='center', va='center',
#             fontsize=9, fontweight='bold', color='white')

# =========================
# === PHASE 2 EXECUTION ===
# =========================
labels, starts, durs = calc_durations(executed_2)
starts[0] = starts[0] + execution_stop_1
durs[0] = durs[0] - starts[0]
y_exec2 = 1.7
for label, start, dur in zip(labels, starts, durs):
    ax.barh(y_exec2, dur, left=start, height=bar_height,
            color='#2ECC71', alpha=0.7, edgecolor='#1E8449', linewidth=1)
#     ax.text(start + dur / 2, y_exec2, label, ha='center', va='center',
#             fontsize=9, fontweight='bold', color='white')
execution_stop_2 = starts[-1] + durs[-1]

# --- Phase 2: Replanned path ---
labels, starts, durs = calc_durations(replanned_2)
starts[0] = starts[0] + execution_stop_2
durs[0] = starts[1] - starts[0]
y_replan2 = y_exec2 - bar_height / 4
for label, start, dur in zip(labels, starts, durs):
    ax.barh(y_replan2, dur, left=start, height=bar_height / 2,
            color='#17A589', alpha=0.7, edgecolor='#117864', linewidth=1)
#     ax.text(start + dur / 2, y_replan2, label, ha='center', va='center',
#             fontsize=9, fontweight='bold', color='white')

# --- Phase 2: If not replanned ---
labels, starts, durs = calc_durations(if_not_replanned_2)
starts[0] = starts[0] + execution_stop_2
durs[0] = starts[1] - starts[0]
y_ifnot2 = y_exec2 + bar_height / 4
for label, start, dur in zip(labels, starts, durs):
    ax.barh(y_ifnot2, dur, left=start, height=bar_height / 2,
            color='#E67E22', alpha=0.7, edgecolor='#AF601A', linewidth=1)
#     ax.text(start + dur / 2, y_ifnot2, label, ha='center', va='center',
#             fontsize=9, fontweight='bold', color='white')

# =========================
# === Total EXECUTION ===
# =========================
labels, starts, durs = calc_durations(all_executed)
y_total = 0.2
for label, start, dur in zip(labels, starts, durs):
    ax.barh(y_total, dur, left=start, height=bar_height,
            color='#2ECC71', alpha=0.7, edgecolor='#1E8449', linewidth=1)
#     ax.text(start + dur / 2, y_total, label, ha='center', va='center',
#             fontsize=9, fontweight='bold', color='white')

# ------------------------------
# Formatting and Markers
# ------------------------------

ax.axvspan(
    fuel_max, total_fuel,
    color="#F81F06", alpha=0.1, label='Safety Margin',
    edgecolor='#C0392B', linewidth=0.001, linestyle='--'
)



ax.axvline(total_fuel, color="#F81E05", linestyle='--', linewidth=1,
           alpha=0.7, zorder=10, label='Max Fuel Duration')

ax.set_xlabel("Mission Time (s)", fontsize=12)
ax.invert_yaxis()
ax.grid(True, axis='x', linestyle='--', alpha=0.4)
ax.legend()
ax.set_xlim(0, 110)
# plt.tight_layout()
for artist in ax.patches:
    artist.set_rasterized(True)
plt.savefig("gantt_chart.pdf", dpi=300)
plt.show()
