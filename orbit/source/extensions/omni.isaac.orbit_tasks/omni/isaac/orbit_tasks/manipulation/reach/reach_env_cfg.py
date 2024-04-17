# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from dataclasses import MISSING

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.orbit.envs import RLTaskEnvCfg
from omni.isaac.orbit.managers import ActionTermCfg as ActionTerm
from omni.isaac.orbit.managers import CurriculumTermCfg as CurrTerm
from omni.isaac.orbit.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.orbit.managers import ObservationTermCfg as ObsTerm
from omni.isaac.orbit.managers import RandomizationTermCfg as RandTerm
from omni.isaac.orbit.managers import RewardTermCfg as RewTerm
from omni.isaac.orbit.managers import SceneEntityCfg
from omni.isaac.orbit.managers import TerminationTermCfg as DoneTerm
from omni.isaac.orbit.scene import InteractiveSceneCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.orbit.utils.noise import AdditiveUniformNoiseCfg as Unoise
from omni.isaac.orbit.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from omni.isaac.orbit.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from omni.isaac.orbit.assets import ArticulationCfg, AssetBaseCfg, RigidObject, RigidObjectCfg

import omni.isaac.orbit_tasks.manipulation.reach.mdp as mdp

##
# Scene definition
##


@configclass
class ReachSceneCfg(InteractiveSceneCfg):
    """Configuration for the scene with a robotic arm."""
    robot: ArticulationCfg = MISSING


    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # Cube for robot
    cube = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Cuboid",
        spawn=sim_utils.CuboidCfg(size=(0.4, 0.2, 1.0)),
    )

    #robot
    #UR10_CFG.init_state.pos = (0.0, 0.0, .5)
    #robot: ArticulationCfg = UR10_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
    
    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )

    taskBoard = AssetBaseCfg(
        prim_path = "{ENV_REGEX_NS}/Cuboid/TaskBoards",
        spawn = sim_utils.UsdFileCfg(usd_path ="/home/chris/Desktop/ws_sims/Models/taskBoards.usdc",
        scale=[0.001,0.001,0.001]
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-0.15, 0.25, 0.7))
        
        #init_state= InitialStateCfg()
    )

    # Object to move 
    object = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/Object",
                init_state=RigidObjectCfg.InitialStateCfg(pos=[0, 0.25, 0.6], rot=[1, 0, 0, 0]),
                spawn=UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                    scale=(0.8, 0.8, 0.8),
                    rigid_props=RigidBodyPropertiesCfg(
                        solver_position_iteration_count=16,
                        solver_velocity_iteration_count=1,
                        max_angular_velocity=1000.0,
                        max_linear_velocity=1000.0,
                        max_depenetration_velocity=5.0, 
                        disable_gravity=False,
                    ),
                ),
            )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-0.23, 0.1200, 0,], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(usd_path="/home/chris/Desktop/ws_sims/Models/TableMahog.usdc",scale=[0.0008,0.0008,0.0008]),
    )

##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

#    ee_pose = mdp.UniformPoseCommandCfg(
#        asset_name="robot",
#        body_name=MISSING,
#        resampling_time_range=(4.0, 4.0),
#        debug_vis=True,
#        ranges=mdp.UniformPoseCommandCfg.Ranges(
#            #pos_x=(0.2, 0.2),
#            #pos_y=(-0.2, 0.2),
#            #pos_z=(0.2, 0.25),
#            #roll=(0.0, 0.0),
#            #pitch=MISSING,  # depends on end-effector axis
#            #yaw=(-3.14, 3.14),
#            pos_x=(0, 0),
#            pos_y=(0.482, 0.482),
#            pos_z=(0.165, 0.165),
#            roll=(0,0),
#            pitch=(0,0),  # depends on end-effector
#            yaw=(0,0), 
#        ),
#    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action: ActionTerm = MISSING
    gripper_action: ActionTerm | None = None


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        #joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        #joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        #pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class RandomizationCfg:
    """Configuration for randomization."""

    reset_robot_joints = RandTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # task terms
    #end_effector_position_tracking = RewTerm(
    #    func=mdp.position_command_error,
    #    weight=-0.2,
    #    params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "command_name": "ee_pose"},
    #)
    #end_effector_orientation_tracking = RewTerm(
    #    func=mdp.orientation_command_error,
    #    weight=-0.05,
    #    params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "command_name": "ee_pose"},
    #)

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.0001)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0001,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -0.005, "num_steps": 4500}
    )


##
# Environment configuration
##

#new commit thing 
@configclass
class ReachEnvCfg(RLTaskEnvCfg):
    """Configuration for the reach end-effector pose tracking environment."""

    # Scene settings
    scene: ReachSceneCfg = ReachSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    randomization: RandomizationCfg = RandomizationCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 12.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0 / 60.0
