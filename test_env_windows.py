from mlagents_envs.environment import UnityEnvironment,ActionTuple
from mlagents_envs.base_env import ActionSpec, TerminalSteps
import numpy as np
from tqdm import trange
from PIDController import PIDController

steer_cmd = 0.1
throttle_cmd = 1.5
env = UnityEnvironment(file_name="ros2-env-windows-keyboard/uneven-terrain-driver", seed=1, side_channels=[],worker_id=0)# log_folder='logs/')#,no_graphics=True)
env.reset()

behavior_name = list(env.behavior_specs)[0]

# Initialize PID controller with appropriate parameters
pid_controller = PIDController(
    kp=0.000001,
    ki=0.0000,
    kd=0.00001,
    forward_distance_threshold=2.0,
    angle_clip=36.0,
    steer_angle_threshold=36.0
)

def interpolate_waypoints(waypoints, num_points_between=5):
    new_waypoints = []
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]
        # Generate linearly spaced points between the start and end waypoints
        interpolated = np.linspace(start, end, num=num_points_between, endpoint=False)
        new_waypoints.append(interpolated)
    
    new_waypoints.append([waypoints[-1]])  # Add the last waypoint
    return np.concatenate(new_waypoints)  # Concatenate all waypoint arrays into a single array

# Example waypoints
waypoints = np.array([
    [505, 34],
    [600, 45],
    [505, 34]
])

# Interpolate additional waypoints between the existing ones
interpolated_waypoints = interpolate_waypoints(waypoints, num_points_between=5)
print(interpolated_waypoints)
waypoints = interpolated_waypoints
current_waypoint_index = 0
desired_distance = 1.0

for ind in trange(1000):

    env_info = env.get_steps(behavior_name)
    if len(env_info[0].obs) == 0:
        continue
    
    # Extract necessary information from observations
    position = np.array([float(env_info[0].obs[-1][0][0]), float(env_info[0].obs[-1][0][2])])
    print(f"Position: {position}, ", ind)
    # Calculate error and forward distance
    error, current_waypoint_index = pid_controller.get_error(position, waypoints, current_waypoint_index, desired_distance)
    # print(f"Error: {error}, Current Waypoint Index: {current_waypoint_index}")
    print("desired waypoint: ", waypoints[current_waypoint_index])
    forward_distance = 1.0  # Replace with actual forward distance calculation if needed
    
    angle, speed = pid_controller.pid_control(forward_distance, error)
    # print(f"Position: {position}, Error: {error}, Angle: {angle}, Speed: {speed}")
    
    # Create actions based on PID control output
    actions = ActionTuple(np.array([[angle, speed]]), None)
    
    env.set_actions(behavior_name, actions)
    env.step()
env.close()
