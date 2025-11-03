import matplotlib.pyplot as plt

# Mission data
sim_plan = {"to_1": 11.0, "mov1": 19.1, "mov2": 28.1, "mov3": 36.1, 
            "mov4": 53.6, "mov5": 69.2, "land1": 89.2}
hw_executed = {"to_1": 11.7, "mov1": 20.7}
hw_if_continued = {"mov2": 29.7, "mov3": 37.7, "mov4": 55.2, "mov5": 70.8, "land1": 90.8}
hw_replanned = {"mov2": 29.9, "mov3": 37.9, "mov4": 43.1, "land1": 63.1}

# Build time series
sim_times = [0] + list(sim_plan.values())
hw_exec_times = [0] + list(hw_executed.values())
replan_time = hw_exec_times[-1]

# Calculate fuel percentages (fuel depletes as time increases)
max_time = sim_times[-1]
sim_fuel = [100 - (t/max_time)*100 for t in sim_times]
hw_exec_fuel = [100 - (t/max_time)*100 for t in hw_exec_times]

# Continuation paths (from replan point onward)
original_cont_times = [replan_time] + list(hw_if_continued.values())
original_cont_fuel = [hw_exec_fuel[-1]] + [100 - (t/max_time)*100 for t in hw_if_continued.values()]

replanned_cont_times = [replan_time] + list(hw_replanned.values())
replanned_cont_fuel = [hw_exec_fuel[-1]] + [100 - (t/max_time)*100 for t in hw_replanned.values()]

# Create plot
fig, ax = plt.subplots(figsize=(14, 8))

# Plot all trajectories
ax.plot(sim_times, sim_fuel, color='#7B68EE', linestyle='--', linewidth=2.5, 
        marker='o', markersize=7, label='Simulation Baseline', alpha=0.6, zorder=2)

ax.plot(hw_exec_times, hw_exec_fuel, color='#2ECC71', linewidth=5, marker='o', 
        markersize=12, label='✓ Executed', zorder=5, markeredgecolor='darkgreen', 
        markeredgewidth=2)

ax.plot(original_cont_times, original_cont_fuel, color='#E74C3C', linestyle='--', 
        linewidth=3, marker='s', markersize=8, label='❌ Original Plan (cancelled)', 
        alpha=0.75, zorder=3, markeredgecolor='darkred', markeredgewidth=1.5)

ax.plot(replanned_cont_times, replanned_cont_fuel, color='#1ABC9C', linewidth=4, 
        marker='^', markersize=10, label='✓ Replanned Path', zorder=4,
        markeredgecolor='#148F77', markeredgewidth=2)

# Mark replan point
ax.axvline(x=replan_time, color='#E67E22', linestyle=':', linewidth=3, 
           alpha=0.8, label='⚠ Replan Decision', zorder=3)
ax.fill_betweenx([0, 100], 0, replan_time, alpha=0.08, color='green')

# Add replan annotation
ax.annotate('REPLAN', xy=(replan_time, 50), xytext=(replan_time + 5, 65),
           fontsize=11, fontweight='bold', color='#E67E22',
           bbox=dict(boxstyle='round,pad=0.5', facecolor='white', 
                    edgecolor='#E67E22', linewidth=2),
           arrowprops=dict(arrowstyle='->', color='#E67E22', lw=2))

# Annotate simulation milestones
for label, time in sim_plan.items():
    fuel = 100 - (time/max_time)*100
    ax.annotate(label, (time, fuel), textcoords="offset points", xytext=(0, 15),
               ha='center', fontsize=8, color='#5B4EAD', alpha=0.7)

# Annotate replanned milestones
for label, time in hw_replanned.items():
    fuel = 100 - (time/max_time)*100
    ax.annotate(label, (time, fuel), textcoords="offset points", xytext=(0, -22),
               ha='center', fontsize=10, color='#148F77', fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.4', facecolor='#E8F8F5', 
                        edgecolor='#1ABC9C', alpha=0.9, linewidth=2))

# Styling
ax.grid(True, alpha=0.25, linestyle='--', linewidth=0.8)
ax.set_xlabel('Mission Time (seconds)', fontsize=13, fontweight='bold')
ax.set_ylabel('Fuel Remaining (%)', fontsize=13, fontweight='bold')
ax.set_title('Mission Fuel Management: Replanning Impact Analysis', 
            fontsize=16, fontweight='bold', pad=20)
ax.legend(loc='upper right', fontsize=11, framealpha=0.98, shadow=True, 
         edgecolor='gray', fancybox=True)
ax.set_ylim(-10, 110)
ax.set_xlim(-2, max(sim_times[-1], replanned_cont_times[-1]) + 3)
ax.set_facecolor('#FAFAFA')

plt.tight_layout()
plt.show()

# # Bottom left - Timeline comparison
# ax2 = fig.add_subplot(gs[1, 0])
# plans = ['Simulation', 'Original\n(cancelled)', 'Replanned\n(executed)']
# durations = [sim_times[-1], all_original_times[-1], all_replanned_times[-1]]
# colors = ['#3498db', '#FF6B6B', '#4ECDC4']
# bars = ax2.barh(plans, durations, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)

# for i, (bar, duration) in enumerate(zip(bars, durations)):
#     ax2.text(duration + 1, bar.get_y() + bar.get_height()/2, 
#             f'{duration:.1f}s', va='center', fontweight='bold', fontsize=10)

# ax2.set_xlabel('Total Mission Duration (seconds)', fontsize=11, fontweight='bold')
# ax2.set_title('Mission Duration Comparison', fontsize=12, fontweight='bold')
# ax2.grid(axis='x', alpha=0.3)

# # Bottom right - Efficiency metrics
# ax3 = fig.add_subplot(gs[1, 1])
# ax3.axis('off')

# time_saved = all_original_times[-1] - all_replanned_times[-1]
# fuel_saved_pct = (time_saved / all_original_times[-1]) * 100
# efficiency_gain = (all_original_times[-1] / all_replanned_times[-1] - 1) * 100

# metrics_text = f"""
# REPLANNING BENEFITS

# Time Saved: {time_saved:.1f} seconds
# Efficiency Gain: {fuel_saved_pct:.1f}%

# Original Duration: {all_original_times[-1]:.1f}s
# Replanned Duration: {all_replanned_times[-1]:.1f}s

# Replan triggered at: {last_exec_time:.1f}s
# Milestones completed: {len(hardware_plan_executed)}/{len(sim_plan)}
# """

# ax3.text(0.1, 0.5, metrics_text, fontsize=11, verticalalignment='center',
#          family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', 
#          alpha=0.8, edgecolor='black', linewidth=2))

# # Bottom - Phase breakdown
# ax4 = fig.add_subplot(gs[2, :])

# phases = list(hardware_plan_executed.keys()) + list(hardware_plan_replanned.keys())
# phase_times = list(hardware_plan_executed.values()) + list(hardware_plan_replanned.values())
# phase_durations = [phase_times[0]] + [phase_times[i] - phase_times[i-1] for i in range(1, len(phase_times))]

# colors_phases = ['#2ecc71']*len(hardware_plan_executed) + ['#4ECDC4']*len(hardware_plan_replanned)
# bars = ax4.bar(phases, phase_durations, color=colors_phases, alpha=0.7, edgecolor='black', linewidth=1.5)

# for bar, duration in zip(bars, phase_durations):
#     height = bar.get_height()
#     ax4.text(bar.get_x() + bar.get_width()/2., height,
#             f'{duration:.1f}s', ha='center', va='bottom', fontweight='bold', fontsize=9)

# ax4.set_ylabel('Duration (seconds)', fontsize=11, fontweight='bold')
# ax4.set_xlabel('Mission Phase', fontsize=11, fontweight='bold')
# ax4.set_title('Phase-by-Phase Duration Breakdown (Executed + Replanned)', fontsize=12, fontweight='bold')
# ax4.grid(axis='y', alpha=0.3)
# ax4.axvline(x=len(hardware_plan_executed)-0.5, color='red', linestyle='--', 
#            linewidth=2, alpha=0.7, label='Replan Point')
# ax4.legend()

plt.suptitle('Flight Mission Analysis Dashboard', fontsize=16, fontweight='bold', y=0.995)
plt.tight_layout()
plt.show()