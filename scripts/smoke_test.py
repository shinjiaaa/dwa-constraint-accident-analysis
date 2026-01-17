"""
Smoke test: Minimal headless simulation test
Tests basic gym-pybullet-drones functionality without GUI
"""

import numpy as np
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import Physics

# Headless mode configuration
def run_smoke_test(steps=200):
    """
    Run a minimal headless simulation test
    
    Args:
        steps: Number of simulation steps to run
    """
    print("=" * 60)
    print("Smoke Test: Headless Drone Simulation")
    print("=" * 60)
    
    # Create environment in headless mode
    env = CtrlAviary(
        num_drones=1,
        gui=False,  # Headless mode
        physics=Physics.PYB,  # PyBullet physics
        pyb_freq=240,  # Physics frequency (Hz)
        ctrl_freq=240,  # Control frequency (Hz)
    )
    
    # Set random seed for reproducibility
    np.random.seed(42)
    
    # Reset environment
    obs, info = env.reset(seed=42)
    print(f"✓ Environment initialized")
    print(f"  Observation shape: {obs.shape}")
    print(f"  Observation space: {env.observation_space}")
    print(f"  Action space: {env.action_space}")
    
    # Get initial state
    initial_pos = obs[0][:3]  # x, y, z position
    initial_vel = obs[0][3:6]  # vx, vy, vz velocity
    print(f"\nInitial State:")
    print(f"  Position: [{initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f}]")
    print(f"  Velocity: [{initial_vel[0]:.3f}, {initial_vel[1]:.3f}, {initial_vel[2]:.3f}]")
    
    # Run simulation
    print(f"\nRunning {steps} simulation steps...")
    print("-" * 60)
    
    # Action: hover (neutral RPM)
    # Action space is typically RPM for each rotor (4 rotors)
    # Hover RPM is usually around 400-600
    hover_rpm = np.array([400.0, 400.0, 400.0, 400.0])
    
    positions = []
    velocities = []
    
    for step in range(steps):
        # Take action (hover)
        obs, reward, terminated, truncated, info = env.step(hover_rpm.reshape(1, -1))
        
        # Extract state
        pos = obs[0][:3]
        vel = obs[0][3:6]
        
        positions.append(pos.copy())
        velocities.append(vel.copy())
        
        # Print periodic updates
        if (step + 1) % 50 == 0 or step == 0:
            print(f"Step {step + 1:4d} | "
                  f"Pos: [{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}] | "
                  f"Vel: [{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}]")
        
        if terminated or truncated:
            print(f"\nEpisode terminated at step {step + 1}")
            break
    
    env.close()
    
    # Final statistics
    print("-" * 60)
    print("\nFinal Statistics:")
    positions = np.array(positions)
    velocities = np.array(velocities)
    
    print(f"  Position range:")
    print(f"    X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}]")
    print(f"    Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}]")
    print(f"    Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}]")
    print(f"  Velocity range:")
    print(f"    VX: [{velocities[:, 0].min():.3f}, {velocities[:, 0].max():.3f}]")
    print(f"    VY: [{velocities[:, 1].min():.3f}, {velocities[:, 1].max():.3f}]")
    print(f"    VZ: [{velocities[:, 2].min():.3f}, {velocities[:, 2].max():.3f}]")
    
    print("\n" + "=" * 60)
    print("✓ Smoke test passed!")
    print("=" * 60)
    
    return True

if __name__ == "__main__":
    try:
        run_smoke_test(steps=200)
    except Exception as e:
        print(f"\n✗ Smoke test failed with error:")
        print(f"  {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
