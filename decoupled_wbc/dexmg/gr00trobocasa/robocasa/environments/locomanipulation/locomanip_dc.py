from robocasa import (
    LMNavPickBottle,
    LMPnPAppleToPlate,
)
from decoupled_wbc.dexmg.gr00trobocasa.robocasa.environments.locomanipulation.manip import ManipAppleToPlate, ManipCubeToZone
from robocasa.models.scenes.lab_arena import LabArena


class LabEnvMixin:
    MUJOCO_ARENA_CLS = LabArena


class LMNavPickBottleDC(LabEnvMixin, LMNavPickBottle): ...


class LMPnPAppleToPlateDC(LabEnvMixin, LMPnPAppleToPlate): ...


class ManipAppleToPlateDC(LabEnvMixin, ManipAppleToPlate): ...


class ManipCubeToZoneDC(LabEnvMixin, ManipCubeToZone): ...