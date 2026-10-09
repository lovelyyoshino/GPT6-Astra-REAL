"""Frozen plug worker/support identity; no device access or task mutation."""

PROFILE_KEY = "_pair_task_roles"
ROLE_KEYS = frozenset(("worker_arm", "support_arm"))


def resolve_task_roles(task):
    """Return (worker, support), preserving the legacy right/left default.

    Explicit roles are an inseparable pair. This reads but never normalizes or
    rewrites old serialized task/episode contracts.
    """
    if type(task) is not dict:
        raise ValueError("Task role container must be an object")
    present = ROLE_KEYS.intersection(task)
    if not present:
        return "right", "left"
    if present != ROLE_KEYS:
        raise ValueError("worker_arm and support_arm must be frozen together")
    worker, support = task["worker_arm"], task["support_arm"]
    if type(worker) is not str or type(support) is not str or {worker, support} != {"left", "right"}:
        raise ValueError("Worker and support must be distinct left/right arms")
    return worker, support


def role_fields(task):
    """Return only explicit validated fields, leaving legacy bytes unchanged."""
    worker, support = resolve_task_roles(task)
    return {"worker_arm": worker, "support_arm": support} if ROLE_KEYS.intersection(task) else {}
