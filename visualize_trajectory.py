#!/usr/bin/env python3
"""
Trajectory Visualizer for Tello Visual Odometry

This script reads trajectory data from trajectory_data.txt and visualizes it
in 2D and 3D plots. It shows:
- 2D top-down view (X-Y plane)
- 2D side view (X-Z plane)
- 3D trajectory
- Statistics (total distance, drift, etc.)

Usage:
    python3 visualize_trajectory.py [trajectory_data.txt] [--save] [--no-display]
"""

import numpy as np
import matplotlib
# Try to use non-interactive backend if display is not available
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
except Exception as e:
    print(f"Warning: Could not import matplotlib with GUI backend: {e}")
    matplotlib.use('Agg')  # Use non-interactive backend
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

import sys
import os
import argparse
from datetime import datetime
import shutil
import csv

def _try_load_csv_trajectory(filename):
    """
    Support new CSV outputs produced by this repo:
    - trajectories/vio_trajectory_*.csv: columns x_m,y_m,z_m
    - telemetry/tello_state_*.csv: columns t_sec,vgx,vgy,vgz (cm/s) -> integrate to meters
    """
    try:
        with open(filename, "r", newline="") as f:
            # Peek header
            first = f.readline()
            if not first:
                return None
            f.seek(0)
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                return None

            fields = set([h.strip() for h in reader.fieldnames if h])

            # Case 1: VIO pose CSV
            if {"x_m", "y_m", "z_m"}.issubset(fields):
                pts = []
                for row in reader:
                    try:
                        x = float(row.get("x_m", ""))
                        y = float(row.get("y_m", ""))
                        z = float(row.get("z_m", ""))
                    except Exception:
                        continue
                    pts.append([x, y, z])
                return np.array(pts) if len(pts) else None

            # Case 2: Telemetry CSV (integrate velocity)
            if {"t_sec", "vgx", "vgy", "vgz"}.issubset(fields):
                # Collect samples
                samples = []
                for row in reader:
                    try:
                        t = float(row.get("t_sec", ""))
                        # cm/s -> m/s
                        vx = float(row.get("vgx", "0")) / 100.0
                        vy = float(row.get("vgy", "0")) / 100.0
                        vz = float(row.get("vgz", "0")) / 100.0
                    except Exception:
                        continue
                    samples.append((t, vx, vy, vz))
                if len(samples) < 2:
                    return None

                samples.sort(key=lambda s: s[0])
                x = y = z = 0.0
                pts = [[0.0, 0.0, 0.0]]
                last_t = samples[0][0]
                for (t, vx, vy, vz) in samples[1:]:
                    dt = t - last_t
                    # Skip crazy gaps/duplicates
                    if dt <= 0 or dt > 1.0:
                        last_t = t
                        continue
                    x += vx * dt
                    y += vy * dt
                    z += vz * dt
                    pts.append([x, y, z])
                    last_t = t
                return np.array(pts) if len(pts) else None

            return None
    except Exception:
        return None

def backup_and_reset_trajectory(filename, no_backup=False):
    """Backup existing trajectory file and reset it for new data.
    
    Args:
        filename: Default trajectory filename to search for
        no_backup: If True, skip backup and just reset the file
    
    Returns:
        tuple: (backup_filename, new_filename) - paths to backup and new files
    """
    # Find the actual trajectory file location
    trajectory_path = None
    alternatives = [
        filename,
        "../trajectory_data.txt",
        "build/trajectory_data.txt",
        "../build/trajectory_data.txt",
        "trajectory_data.txt"
    ]
    
    for alt_file in alternatives:
        if os.path.exists(alt_file):
            trajectory_path = alt_file
            break
    
    backup_path = None
    if trajectory_path and os.path.exists(trajectory_path):
        # Create backup with timestamp (unless --no-backup is specified)
        if not no_backup:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_path = f"{trajectory_path}.backup_{timestamp}"
            shutil.copy2(trajectory_path, backup_path)
            print(f"Backed up existing trajectory to: {backup_path}")
        else:
            print(f"Overwriting existing trajectory file (no backup): {trajectory_path}")
        
        # Clear the original file
        with open(trajectory_path, 'w') as f:
            f.write("# Trajectory data from Tello VIO\n")
            f.write("# Format: X Y Z (in meters)\n")
            f.write("# This file was reset - new trajectory will be written here\n")
        print(f"Reset trajectory file: {trajectory_path}")
    elif not trajectory_path:
        # File doesn't exist, use default location
        trajectory_path = "trajectory_data.txt"
        print(f"Trajectory file not found, will create new one at: {trajectory_path}")
    
    return backup_path, trajectory_path

def generate_demo_trajectory(filename):
    """Generate a demo trajectory for visualization demonstration.
    
    Creates a spiral trajectory pattern.
    """
    print("Generating demo trajectory...")
    
    # Create a spiral trajectory pattern
    num_points = 200
    trajectory_points = []
    
    for i in range(num_points):
        t = i / num_points * 4 * np.pi  # 2 full rotations
        radius = 0.5 + 0.3 * (i / num_points)  # Expanding spiral
        x = radius * np.cos(t)
        y = radius * np.sin(t)
        z = 0.1 * i / num_points  # Gradually rising
        trajectory_points.append([x, y, z])
    
    # Write to file
    with open(filename, 'w') as f:
        f.write("# Trajectory data from Tello VIO\n")
        f.write("# Format: X Y Z (in meters)\n")
        f.write(f"# Total points: {num_points}\n")
        f.write("# Demo trajectory - spiral pattern\n")
        for point in trajectory_points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    
    print(f"Demo trajectory written to: {filename}")
    return np.array(trajectory_points)

def load_trajectory(filename):
    """Load trajectory data from file.
    
    Expected format:
    # Trajectory data from Tello VIO
    # Format: X Y Z (in meters)
    # Total points: N
    x1 y1 z1
    x2 y2 z2
    ...
    """
    trajectory = []
    
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found.")
        print("Looking in current directory and parent directory...")
        
        # Try alternative locations
        alternatives = [
            "../trajectory_data.txt",
            "build/trajectory_data.txt",
            "../build/trajectory_data.txt",
            "trajectory_data.txt"
        ]
        
        for alt_file in alternatives:
            if os.path.exists(alt_file):
                print(f"Found trajectory file at: {alt_file}")
                filename = alt_file
                break
        else:
            print("Could not find trajectory_data.txt in any of the expected locations.")
            return None
    
    # First try CSV formats
    if filename.lower().endswith(".csv"):
        traj = _try_load_csv_trajectory(filename)
        if traj is not None and len(traj) > 0:
            print(f"Loaded {len(traj)} trajectory points from CSV {filename}")
            return traj

    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            
        # Skip header lines (starting with #)
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # Parse X Y Z coordinates
            parts = line.split()
            if len(parts) >= 3:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                    trajectory.append([x, y, z])
                except ValueError:
                    print(f"Warning: Skipping invalid line: {line}")
                    continue
        
        if len(trajectory) == 0:
            print(f"Error: No valid trajectory data found in {filename}")
            return None
        
        print(f"Loaded {len(trajectory)} trajectory points from {filename}")
        return np.array(trajectory)
        
    except Exception as e:
        print(f"Error reading trajectory file: {e}")
        return None

def compute_statistics(trajectory):
    """Compute trajectory statistics."""
    if trajectory is None or len(trajectory) == 0:
        return None
    
    stats = {}
    
    # Total distance traveled
    distances = np.diff(trajectory, axis=0)
    distances = np.linalg.norm(distances, axis=1)
    stats['total_distance'] = np.sum(distances)
    
    # Start and end positions
    stats['start_pos'] = trajectory[0]
    stats['end_pos'] = trajectory[-1]
    
    # Total displacement (straight-line distance from start to end)
    stats['displacement'] = np.linalg.norm(trajectory[-1] - trajectory[0])
    
    # Drift (how much the trajectory drifted from start)
    stats['drift'] = stats['displacement']
    
    # Average velocity (assuming ~30 FPS)
    fps = 30.0
    time_span = len(trajectory) / fps  # seconds
    if time_span > 0:
        stats['avg_speed'] = stats['total_distance'] / time_span
    else:
        stats['avg_speed'] = 0.0
    
    # Bounding box
    stats['min_bounds'] = np.min(trajectory, axis=0)
    stats['max_bounds'] = np.max(trajectory, axis=0)
    stats['size'] = stats['max_bounds'] - stats['min_bounds']
    
    return stats

def plot_trajectory(trajectory, stats, save_file=None, show_display=True):
    """Plot trajectory in 2D and 3D.
    
    Args:
        trajectory: Array of trajectory points
        stats: Dictionary with trajectory statistics
        save_file: Optional filename to save the plot (e.g., 'trajectory_plot.png')
        show_display: Whether to show the plot interactively
    """
    if trajectory is None or len(trajectory) == 0:
        print("Error: No trajectory data to plot")
        return
    
    fig = plt.figure(figsize=(16, 12))
    
    # Extract coordinates
    x = trajectory[:, 0]
    y = trajectory[:, 1]
    z = trajectory[:, 2]
    
    # 1. Top-down view (X-Y plane)
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(x, y, 'b-', linewidth=2, label='Trajectory')
    ax1.plot(x[0], y[0], 'go', markersize=10, label='Start')
    ax1.plot(x[-1], y[-1], 'ro', markersize=10, label='End')
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('Top-Down View (X-Y Plane)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # 2. Side view (X-Z plane)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(x, z, 'b-', linewidth=2, label='Trajectory')
    ax2.plot(x[0], z[0], 'go', markersize=10, label='Start')
    ax2.plot(x[-1], z[-1], 'ro', markersize=10, label='End')
    ax2.set_xlabel('X (m)', fontsize=12)
    ax2.set_ylabel('Z (m)', fontsize=12)
    ax2.set_title('Side View (X-Z Plane)', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.axis('equal')
    
    # 3. Front view (Y-Z plane)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(y, z, 'b-', linewidth=2, label='Trajectory')
    ax3.plot(y[0], z[0], 'go', markersize=10, label='Start')
    ax3.plot(y[-1], z[-1], 'ro', markersize=10, label='End')
    ax3.set_xlabel('Y (m)', fontsize=12)
    ax3.set_ylabel('Z (m)', fontsize=12)
    ax3.set_title('Front View (Y-Z Plane)', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    ax3.axis('equal')
    
    # 4. 3D trajectory
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    ax4.plot(x, y, z, 'b-', linewidth=2, label='Trajectory')
    ax4.plot([x[0]], [y[0]], [z[0]], 'go', markersize=10, label='Start')
    ax4.plot([x[-1]], [y[-1]], [z[-1]], 'ro', markersize=10, label='End')
    ax4.set_xlabel('X (m)', fontsize=12)
    ax4.set_ylabel('Y (m)', fontsize=12)
    ax4.set_zlabel('Z (m)', fontsize=12)
    ax4.set_title('3D Trajectory', fontsize=14, fontweight='bold')
    ax4.legend()
    
    # Set equal aspect ratio for 3D plot
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    mid_x = (x.max()+x.min()) * 0.5
    mid_y = (y.max()+y.min()) * 0.5
    mid_z = (z.max()+z.min()) * 0.5
    ax4.set_xlim(mid_x - max_range, mid_x + max_range)
    ax4.set_ylim(mid_y - max_range, mid_y + max_range)
    ax4.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Add statistics text
    stats_text = f"""
    Statistics:
    • Total Points: {len(trajectory)}
    • Total Distance: {stats['total_distance']:.3f} m
    • Displacement: {stats['displacement']:.3f} m
    • Average Speed: {stats['avg_speed']:.3f} m/s
    • Start Position: ({stats['start_pos'][0]:.3f}, {stats['start_pos'][1]:.3f}, {stats['start_pos'][2]:.3f})
    • End Position: ({stats['end_pos'][0]:.3f}, {stats['end_pos'][1]:.3f}, {stats['end_pos'][2]:.3f})
    • Bounds: X=[{stats['min_bounds'][0]:.3f}, {stats['max_bounds'][0]:.3f}]
              Y=[{stats['min_bounds'][1]:.3f}, {stats['max_bounds'][1]:.3f}]
              Z=[{stats['min_bounds'][2]:.3f}, {stats['max_bounds'][2]:.3f}]
    """
    
    fig.suptitle('Tello Visual Odometry - Trajectory Visualization', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    # Add text box with statistics
    fig.text(0.5, 0.02, stats_text, ha='center', va='bottom', 
             fontsize=10, family='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout(rect=[0, 0.15, 1, 0.98])
    
    # Save plot if requested
    if save_file:
        plt.savefig(save_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {save_file}")
    
    # Show display if requested and available
    if show_display:
        try:
            plt.show()
        except Exception as e:
            print(f"Warning: Could not display plot: {e}")
            if not save_file:
                # Auto-save if display fails and no save file specified
                auto_save = "trajectory_plot.png"
                plt.savefig(auto_save, dpi=150, bbox_inches='tight')
                print(f"Plot automatically saved to: {auto_save}")
    else:
        # If not showing, always save
        if not save_file:
            save_file = "trajectory_plot.png"
        plt.savefig(save_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {save_file}")
    
    plt.close()  # Close figure to free memory

def main():
    parser = argparse.ArgumentParser(
        description='Visualize Tello Visual Odometry trajectory',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 visualize_trajectory.py                      # Visualize latest trajectory_data.txt
  python3 visualize_trajectory.py build/trajectory_data.txt
  python3 visualize_trajectory.py --save trajectory_plot.png
  python3 visualize_trajectory.py --no-display        # Save only, no GUI
  python3 visualize_trajectory.py --demo              # Generate and visualize demo trajectory
        """
    )
    parser.add_argument('trajectory_file', nargs='?', 
                       default='trajectory_data.txt',
                       help='Path to trajectory data file (default: trajectory_data.txt)')
    parser.add_argument('--save', '-s', type=str, metavar='FILE',
                       help='Save plot to file (e.g., trajectory_plot.png)')
    parser.add_argument('--no-display', action='store_true',
                       help='Do not display plot interactively (save only)')
    parser.add_argument('--demo', action='store_true',
                       help='Generate a demo trajectory instead of loading from file')
    parser.add_argument('--no-backup', action='store_true',
                       help='With --demo: overwrite existing file without creating a backup')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("  Tello Visual Odometry - Trajectory Visualizer")
    print("=" * 60)
    print()
    
    # Either generate a demo trajectory, or load from file
    if args.demo:
        # Backup/reset the target trajectory file, then write demo into it
        backup_path, trajectory_path = backup_and_reset_trajectory(args.trajectory_file, args.no_backup)
        
        # Determine where to write the demo trajectory
        if trajectory_path:
            demo_file = trajectory_path
        else:
            # Use first available location
            alternatives = [
                "trajectory_data.txt",
                "build/trajectory_data.txt",
                "../build/trajectory_data.txt"
            ]
            demo_file = alternatives[0]
            # Create directory if needed
            os.makedirs(os.path.dirname(demo_file) if os.path.dirname(demo_file) else '.', exist_ok=True)
        
        # Generate demo trajectory and visualize that
        trajectory = generate_demo_trajectory(demo_file)
        print(f"\nDemo trajectory created! Visualizing...\n")
    else:
        # Normal mode: just load the existing trajectory, do NOT modify files
        trajectory = load_trajectory(args.trajectory_file)
    
    if trajectory is None:
        print("Failed to load trajectory data.")
        sys.exit(1)
    
    # Compute statistics
    stats = compute_statistics(trajectory)
    if stats is None:
        print("Failed to compute statistics.")
        sys.exit(1)
    
    # Print statistics to console
    print("\nTrajectory Statistics:")
    print("-" * 60)
    print(f"  Total Points: {len(trajectory)}")
    print(f"  Total Distance: {stats['total_distance']:.3f} m")
    print(f"  Displacement: {stats['displacement']:.3f} m")
    print(f"  Average Speed: {stats['avg_speed']:.3f} m/s")
    print(f"  Start Position: ({stats['start_pos'][0]:.3f}, {stats['start_pos'][1]:.3f}, {stats['start_pos'][2]:.3f})")
    print(f"  End Position: ({stats['end_pos'][0]:.3f}, {stats['end_pos'][1]:.3f}, {stats['end_pos'][2]:.3f})")
    print(f"  Bounds:")
    print(f"    X: [{stats['min_bounds'][0]:.3f}, {stats['max_bounds'][0]:.3f}] (size: {stats['size'][0]:.3f} m)")
    print(f"    Y: [{stats['min_bounds'][1]:.3f}, {stats['max_bounds'][1]:.3f}] (size: {stats['size'][1]:.3f} m)")
    print(f"    Z: [{stats['min_bounds'][2]:.3f}, {stats['max_bounds'][2]:.3f}] (size: {stats['size'][2]:.3f} m)")
    print()
    
    # Plot trajectory
    if args.no_display:
        print("Generating trajectory plots (saving only, no display)...")
    else:
        print("Displaying trajectory plots...")
    
    plot_trajectory(trajectory, stats, save_file=args.save, show_display=not args.no_display)
    
    print("\nVisualization complete!")

if __name__ == "__main__":
    main()

