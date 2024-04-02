# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to create a simple environment with a cartpole. It combines the concepts of
scene, action, observation and randomization managers to create an environment.
"""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.orbit.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on creating a cartpole base environment.")
parser.add_argument("--num_envs", type=int, default=4, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import torch
#import omni.isaac.orbit.envs.mdp as mdp
#from omni.isaac.orbit.envs import BaseEnv, BaseEnvCfg
#from omni.isaac.orbit.managers import ObservationGroupCfg as ObsGroup
#from omni.isaac.orbit.managers import ObservationTermCfg as ObsTerm
#from omni.isaac.orbit.managers import RandomizationTermCfg as RandTerm
#from omni.isaac.orbit.managers import SceneEntityCfg
#from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.envs import RLTaskEnv
#from omni.isaac.orbit_tasks.classic.cartpole.cartpole_env_cfg import CartpoleSceneCfg
from . import TaskBoardEnvCfg
#from omni.isaac.orbit.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
#from omni.isaac.orbit.sim.spawners.from_files.from_files_cfg import UsdFileCfg

from omni.isaac.orbit_assets import UR10_CFG  # isort:skip

import omni.isaac.contrib_tasks  # noqa: F401
import omni.isaac.orbit_tasks  # noqa: F401
from omni.isaac.orbit_tasks.utils import parse_env_cfg

###--------------------------------------------------------------------------------------------
import gymnasium as gym
import torch


def main():
    """Random actions agent with Orbit environment."""
    # create environment configuration
    env_cfg = parse_env_cfg(
        args_cli.task, use_gpu=not args_cli.cpu, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # print info (this is vectorized environment)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    # reset environment
    env.reset()
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # sample actions from -1 to 1
            actions = 2 * torch.rand(env.action_space.shape, device=env.unwrapped.device) - 1
            # apply actions
            env.step(actions)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()