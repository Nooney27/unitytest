from mlagents_envs.environment import UnityEnvironment,ActionTuple
from mlagents_envs.base_env import ActionSpec, TerminalSteps
import numpy as np
import tqdm

steer_cmd = 0.1
throttle_cmd = 1.5
env = UnityEnvironment(file_name="ros2-env-windows-keyboard/uneven-terrain-driver", seed=1, side_channels=[],worker_id=0)# log_folder='logs/')#,no_graphics=True)
env.reset()

behavior_name = list(env.behavior_specs)[0]

for i in tqdm.tqdm(range(100)):

    env_info = env.get_steps(behavior_name)
    print('xpos', float(env_info[0].obs[-1][0][0]))
    actions = ActionTuple(np.array([[steer_cmd,throttle_cmd]]),None)
    env.set_actions(behavior_name, actions)
    env.step()
env.close()
