"""Pure role projection of the canonical plug recipe, never an actuator."""
import copy
import re

from .task_roles import resolve_task_roles


_ARM_WORDS = re.compile(
    r"\b(left|right)(?=(?: arm| fingers| gripper| grasp| stabilizes| transfers|\-arm)\b)")


def render_plug_recipe(canonical_recipe, *, worker_arm="right", support_arm="left"):
    """Change arm assignments without changing the scene's left target socket.

    The canonical source is kept intact. The resulting recipe is still offline
    and must be frozen alongside the matching live task's explicit arm roles.
    """
    worker, support = resolve_task_roles(
        {"worker_arm": worker_arm, "support_arm": support_arm})
    if (type(canonical_recipe) is not dict
            or canonical_recipe.get("task_id") != "plug_transfer_left"
            or canonical_recipe.get("schema_version") != "astra_recipe_v1"):
        raise ValueError("Canonical plug_transfer_left recipe required")
    result = copy.deepcopy(canonical_recipe)
    if (worker, support) == ("right", "left"):
        return result
    mapping = {"right": worker, "left": support}

    def text(value):
        return _ARM_WORDS.sub(lambda match: mapping[match.group(1)], value)

    result["goal"] = text(result["goal"])
    result["constraints"] = [text(value) for value in result["constraints"]]
    for step in result["steps"]:
        step["arm"] = mapping[step["arm"]]
        step["goal"] = text(step["goal"])
        if "evidence" in step:
            step["evidence"] = [
                mapping[value.split("_", 1)[0]] + "_gripper_clear"
                if value in ("left_gripper_clear", "right_gripper_clear") else value
                for value in step["evidence"]]
    return result
