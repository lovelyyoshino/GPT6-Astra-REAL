"""Pinned robot geometry for joint-space contracts, separate from telemetry.

Construction reads the reviewed manufacturer constants without executing them.
Subsequent matrix/radius calculations have no I/O. This identifies a numerical
model, not a physical robot, tool attachment, controller zero, or free corridor.
"""
import copy
import math

from .model_compatibility import fk_matrix, load_model_catalog


class OfficialJointModel:
    def __init__(self, catalog_spec, model):
        models, source = load_model_catalog(catalog_spec)
        if source["recognized_official_snapshot"] is not True or model not in models:
            raise ValueError("A recognized pinned manufacturer joint model is required")
        self._mdh = tuple(tuple(row) for row in models[model]["mdh"])
        self._source = {"mode": "model_joint_geometry_v1", "model": model,
                        "sdk_commit": source["commit"], "constants_sha256": source["sha256"]}

    @property
    def source(self):
        return copy.deepcopy(self._source)

    def matrix(self, joints_rad):
        return fk_matrix(self._mdh, joints_rad)

    def flange_radii(self):
        """Conservative lever arms for the flange, excluding attached bodies."""
        return [abs(row[0]) + sum(abs(link[0])+abs(link[1]) for link in self._mdh[index+1:])
                for index, row in enumerate(self._mdh)]

    def radii(self, attachment_radius_m, link_body_allowance_m):
        if (type(attachment_radius_m) not in (int, float) or not math.isfinite(attachment_radius_m)
                or attachment_radius_m <= 0 or type(link_body_allowance_m) not in (int, float)
                or link_body_allowance_m != .060):
            raise ValueError("A sourced positive attachment bound and existing 60 mm body allowance are required")
        return [r + attachment_radius_m + link_body_allowance_m for r in self.flange_radii()]
