"""
Locomotion tasks that extend manipulation tasks with additional scene obstacles.
These tasks require navigation in addition to manipulation.
"""

import numpy as np
from robocasa.environments.locomanipulation.manip import ManipBlockToZoneRight
from robocasa.utils.scene.configs import ObjectConfig, SamplingConfig
from robocasa.utils.scene.scene import SceneObject
from robocasa.utils.scene.success_criteria import IsRobotInRange, SuccessCriteria


class LocoWalkToTable(ManipBlockToZoneRight):
    """
    Extends ManipBlockToZoneRight with obstacles behind the robot.
    Adds a large cube and large sphere that the robot must navigate around
    to reach the table and complete the cube-to-zone task.
    """

    def _get_objects(self) -> list[SceneObject]:
        # Get the base objects from parent (table, cube, zone)
        objects = super()._get_objects()
        
        # Add large green cube behind the robot (negative x, left side)
        self.large_block = SceneObject(
            ObjectConfig(
                name="large_block_obstacle",
                mjcf_path="obstacles/obstacle_green_block.xml",
                static=True,
                scale=1.0,
                sampler_config=SamplingConfig(
                    x_range=np.array([-0.6, -0.6]),
                    y_range=np.array([0.8, 0.8]),  # Left side
                    rotation=np.array([0.0, 0.0]),
                    reference_pos=np.array([0, 0, 0]),  # Behind robot
                ),
            )
        )
        
        # Add large blue sphere behind the robot (negative x, right side)
        self.large_sphere = SceneObject(
            ObjectConfig(
                name="large_sphere_obstacle",
                mjcf_path="obstacles/obstacle_blue_sphere.xml",
                static=True,
                scale=1.0,
                sampler_config=SamplingConfig(
                    x_range=np.array([-1.0, -1.0]),
                    y_range=np.array([-0.6, -0.6]),  # Right side
                    rotation=np.array([0.0, 0.0]),
                    reference_pos=np.array([0, 0, 0]),  # Behind robot
                ),
            )
        )
        
        # Return all objects including the new obstacles
        objects.extend([self.large_block, self.large_sphere])
        return objects

    def _get_instruction(self) -> str:
        return "Walk to the table."

    def _get_success_criteria(self) -> SuccessCriteria:
        """Success when robot is close to the table (within 0.8 units in xy plane)"""
        return IsRobotInRange(target=self.table, threshold=0.8, planar=True)


class LocoWalkToTableDC(LocoWalkToTable):
    pass  # The LabEnvMixin will be added when registering
