from robocasa import (
    LMNavPickBottle,
    LMPnPAppleToPlate,
)
from decoupled_wbc.dexmg.gr00trobocasa.robocasa.environments.locomanipulation.manip import ManipAppleToPlate, ManipCubeToZone
from robocasa.environments.locomanipulation.locomanip_static import LMPnPAppleToPlateStatic, LMPnPCubeToZoneStatic
from robocasa.models.scenes.lab_arena import LabArena


class LabEnvMixin:
    MUJOCO_ARENA_CLS = LabArena


class LMNavPickBottleDC(LabEnvMixin, LMNavPickBottle): ...


class LMPnPAppleToPlateDC(LabEnvMixin, LMPnPAppleToPlate): ...


class ManipAppleToPlateDC(LabEnvMixin, ManipAppleToPlate): ...


class ManipCubeToZoneDC(LabEnvMixin, ManipCubeToZone): ...


class LMPnPAppleToPlateStaticDC(LabEnvMixin, LMPnPAppleToPlateStatic): ...


class LMPnPCubeToZoneStaticDC(LabEnvMixin, LMPnPCubeToZoneStatic): ...
