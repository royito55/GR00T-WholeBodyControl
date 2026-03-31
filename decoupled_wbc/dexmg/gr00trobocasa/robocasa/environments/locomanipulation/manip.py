"""
Static pick and place tasks for loco-manipulation.
These tasks require only upper body movement without navigation.
"""

import numpy as np
from robocasa.environments.locomanipulation.locomanip import LMSimpleEnv
from robocasa.utils.dexmg_utils import DexMGConfigHelper
from robocasa.utils.scene.configs import (
    ObjectConfig,
    SamplingConfig,
    SceneScaleConfig,
)
from robocasa.utils.scene.scene import SceneObject
from robocasa.utils.scene.success_criteria import (
    IsInContact,
    IsClose,
    SuccessCriteria,
)
from robocasa.utils.visuals_utls import Gradient, randomize_materials_rgba


class ManipAppleToPlate(LMSimpleEnv, DexMGConfigHelper):
    """
    Single table with apple and plate close together - no navigation required.
    Robot's lower body remains static throughout the task.
    """

    SCENE_SCALE = SceneScaleConfig(planar_scale=1.0)
    TABLE_GRADIENT: Gradient = Gradient(
        np.array([0.68, 0.34, 0.07, 1.0]), np.array([1.0, 1.0, 1.0, 1.0])
    )
    LIFT_OFFSET = 0.1

    def _get_objects(self) -> list[SceneObject]:
        # Single table in front of robot
        self.table = SceneObject(
            ObjectConfig(
                name="table",
                mjcf_path="objects/omniverse/locomanip/lab_table/model.xml",
                scale=1.0,
                static=True,
                sampler_config=SamplingConfig(
                    x_range=np.array([0.0, 0.0]),
                    y_range=np.array([0.0, 0.0]),
                    reference_pos=np.array([0.5, 0, 0]),  # Table in front of robot
                    rotation=np.array([np.pi * 0.5, np.pi * 0.5]),
                ),
            )
        )
        
        # Apple on the left side of table
        self.apple = SceneObject(
            ObjectConfig(
                name="apple",
                mjcf_path="objects/omniverse/locomanip/apple_0/model.xml",
                static=False,
                scale=1.0,
                sampler_config=SamplingConfig(
                    x_range=np.array([-0.10, -0.10]),
                    y_range=np.array([0.15, 0.15]),
                    # x_range=np.array([1.0, 1.0]),
                    # y_range=np.array([-1.0, -1.0]),
                    rotation=np.array([-np.pi, np.pi]),
                    reference_pos=np.array([0.4, 0, self.table.mj_obj.top_offset[2]]),
                ),
            )
        )
        
        # Plate on the right side of table (close to apple)
        self.plate = SceneObject(
            ObjectConfig(
                name="plate",
                mjcf_path="objects/omniverse/locomanip/plate_1/model.xml",
                scale=1.0,
                static=True,  # Plate is fixed on the table
                sampler_config=SamplingConfig(
                    x_range=np.array([-0.10, -0.10]),
                    y_range=np.array([0.0, 0.0]),
                    # x_range=np.array([1.0, 1.0]),
                    # y_range=np.array([1.0, 1.0]),
                    rotation=np.array([-np.pi, np.pi]),
                    reference_pos=np.array([0.4, 0, self.table.mj_obj.top_offset[2]]),
                ),
            )
        )
        
        return [self.table, self.apple, self.plate]

    def _get_success_criteria(self) -> SuccessCriteria:
        # Success when apple is in contact with plate
        return IsInContact(self.apple, self.plate)

    def _get_instruction(self) -> str:
        return "Pick up the apple and place it on the plate."

    def get_object(self):
        return dict(
            apple=dict(obj_name=self.apple.mj_obj.root_body, obj_type="body"),
            plate=dict(obj_name=self.plate.mj_obj.root_body, obj_type="body"),
        )

    def get_subtask_term_signals(self):
        # Signal when apple is lifted off the table
        obj_z = self.sim.data.body_xpos[self.obj_body_id(self.apple.mj_obj.name)][2]
        table_pos = self.sim.data.body_xpos[self.obj_body_id(self.table.mj_obj.name)]
        table_z = table_pos[2] + self.table.mj_obj.top_offset[2]
        return dict(apple_lifted=int(obj_z - table_z > self.LIFT_OFFSET))

    @staticmethod
    def task_config():
        task = DexMGConfigHelper.AttrDict()
        # Pick up apple
        task.task_spec_0.subtask_1 = dict(
            object_ref="apple",
            subtask_term_signal="apple_lifted",
            subtask_term_offset_range=(5, 10),
            selection_strategy="random",
            selection_strategy_kwargs=None,
            action_noise=0.05,
            num_interpolation_steps=5,
            num_fixed_steps=0,
            apply_noise_during_interpolation=False,
        )
        # Place on plate
        task.task_spec_0.subtask_2 = dict(
            object_ref="plate",
            subtask_term_signal=None,
            subtask_term_offset_range=None,
            selection_strategy="random",
            selection_strategy_kwargs=None,
            action_noise=0.05,
            num_interpolation_steps=5,
            num_fixed_steps=0,
            apply_noise_during_interpolation=False,
        )
        task.task_spec_1.subtask_1 = dict(
            object_ref=None,
            subtask_term_signal=None,
            subtask_term_offset_range=None,
            selection_strategy="random",
            selection_strategy_kwargs=None,
            action_noise=0.05,
            num_interpolation_steps=5,
            num_fixed_steps=0,
            apply_noise_during_interpolation=False,
        )
        return task.to_dict()

    def _reset_internal(self):
        super()._reset_internal()
        if not self.deterministic_reset:
            self._randomize_table_rgba()

    def _randomize_table_rgba(self):
        randomize_materials_rgba(
            rng=self.rng, mjcf_obj=self.table.mj_obj, gradient=self.TABLE_GRADIENT, linear=True
        )


class ManipAppleToPlateDC(ManipAppleToPlate):
    pass  # The LabEnvMixin will be added when registering


class ManipCubeToZone(LMSimpleEnv, DexMGConfigHelper):
    """
    Single table with cube and blue zone close together - no navigation required.
    Robot's lower body remains static throughout the task.
    """

    SCENE_SCALE = SceneScaleConfig(planar_scale=1.0)
    TABLE_GRADIENT: Gradient = Gradient(
        np.array([0.68, 0.34, 0.07, 1.0]), np.array([1.0, 1.0, 1.0, 1.0])
    )
    LIFT_OFFSET = 0.1

    def _get_objects(self) -> list[SceneObject]:
        self.table = SceneObject(
            ObjectConfig(
                name="table",
                mjcf_path="objects/omniverse/locomanip/lab_table/model.xml",
                scale=1.0,
                static=True,
                sampler_config=SamplingConfig(
                    x_range=np.array([0.0, 0.0]),
                    y_range=np.array([0.0, 0.0]),
                    reference_pos=np.array([0.5, 0, 0]),
                    rotation=np.array([np.pi * 0.5, np.pi * 0.5]),
                ),
            )
        )
        
        self.cube = SceneObject(
            ObjectConfig(
                name="red_cube",
                mjcf_path="objects/blocks/red_block.xml",
                static=False,
                scale=1.0,
                sampler_config=SamplingConfig(
                    x_range=np.array([-0.05, -0.05]),
                    y_range=np.array([0.1, 0.1]),
                    rotation=np.array([0.0, 0.0]),
                    reference_pos=np.array([0.4, 0, self.table.mj_obj.top_offset[2]]),
                ),
            )
        )
        
        self.zone = SceneObject(
            ObjectConfig(
                name="blue_zone",
                mjcf_path="objects/zones/blue_zone.xml",
                scale=1.0,
                static=True,
                sampler_config=SamplingConfig(
                    x_range=np.array([0.0, 0.0]),
                    y_range=np.array([-0.2, -0.2]),
                    rotation=np.array([0.0, 0.0]),
                    reference_pos=np.array([0.4, 0, self.table.mj_obj.top_offset[2] + 0.001]),
                ),
            )
        )
        
        return [self.table, self.cube, self.zone]

    def _get_success_criteria(self) -> SuccessCriteria:
        # Success when red block is fully within the edges of the blue zone (in xy plane)
        # Using IsClose instead of IsInContact since zone has no collision mesh
        return IsClose(self.cube, self.zone, max_distance=0.14, use_xy_only=True)

    def _get_instruction(self) -> str:
        return "Push the red block to the blue zone."

    def get_object(self):
        return dict(
            cube=dict(obj_name=self.cube.mj_obj.root_body, obj_type="body"),
            zone=dict(obj_name=self.zone.mj_obj.root_body, obj_type="body"),
        )

    def get_subtask_term_signals(self):
        return dict()  # No intermediate signals for pushing

    @staticmethod
    def task_config():
        task = DexMGConfigHelper.AttrDict()
        # Push cube to zone - single subtask
        task.task_spec_0.subtask_1 = dict(
            object_ref="zone",  # Target destination
            subtask_term_signal=None,  # No intermediate signal needed
            subtask_term_offset_range=None,
            selection_strategy="random",
            selection_strategy_kwargs=None,
            action_noise=0.05,
            num_interpolation_steps=5,
            num_fixed_steps=0,
            apply_noise_during_interpolation=False,
        )
        task.task_spec_1.subtask_1 = dict(
            object_ref=None,
            subtask_term_signal=None,
            subtask_term_offset_range=None,
            selection_strategy="random",
            selection_strategy_kwargs=None,
            action_noise=0.05,
            num_interpolation_steps=5,
            num_fixed_steps=0,
            apply_noise_during_interpolation=False,
        )
        return task.to_dict()

    def _reset_internal(self):
        super()._reset_internal()
        if not self.deterministic_reset:
            self._randomize_table_rgba()

    def _randomize_table_rgba(self):
        randomize_materials_rgba(
            rng=self.rng, mjcf_obj=self.table.mj_obj, gradient=self.TABLE_GRADIENT, linear=True
        )


class ManipCubeToZoneDC(ManipCubeToZone):
    pass  # The LabEnvMixin will be added when registering