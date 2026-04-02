from robocasa import (
    LMNavPickBottle,
    LMPnPAppleToPlate,
)
from robocasa.environments.locomanipulation.manip import ManipAppleToPlate, ManipBlockToZoneLeft, ManipBlockToZoneRight
from robocasa.environments.locomanipulation.locomotion import LocoWalkToTable
from robocasa.models.scenes.lab_arena import LabArena


class LabEnvMixin:
    MUJOCO_ARENA_CLS = LabArena


class LMNavPickBottleDC(LabEnvMixin, LMNavPickBottle): ...


class LMPnPAppleToPlateDC(LabEnvMixin, LMPnPAppleToPlate): ...


class ManipAppleToPlateDC(LabEnvMixin, ManipAppleToPlate): ...


class ManipBlockToZoneRightDC(LabEnvMixin, ManipBlockToZoneRight): ...
class ManipBlockToZoneLeftDC(LabEnvMixin, ManipBlockToZoneLeft): ...


class LocoWalkToTableDC(LabEnvMixin, LocoWalkToTable): ...

