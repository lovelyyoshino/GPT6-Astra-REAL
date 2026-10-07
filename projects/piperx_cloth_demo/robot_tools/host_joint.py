"""Same-live-host bridge for durable MOVE_J cancellation and hold facts.

No robot imports, sender, or permission token. The adapter still owns the final
fresh sample, mode/geometry checks and exact-frame interposer. Cancellation
first blocks ordinary dispatch; disk latency never leaves it enabled. A slow
or failed cancel publication can lose the opportunity to hold, never revive it.
"""
from __future__ import annotations

import copy
import json
import math
import threading
import uuid


# This blocks the action worker's RX, so keep it below one 50 ms feedback
# interval. This is a wait deadline, not a real-time scheduling guarantee.
# Slow storage loses this hold opportunity; it never extends the wait or
# authorizes a target on an old sample. The old physical target may still move.
CANCEL_PUBLICATION_WAIT_S = .025


class HostJointBridgeError(RuntimeError):
    pass


def _wire(value):
    # Official SDK snapshots retain IntEnum instances. Freeze their actual JSON
    # representation before entering the ledger's intentionally strict schema.
    return json.loads(json.dumps(value, allow_nan=False))


class HostJointBridge:
    """One already-claimed original action; this object cannot be restored.

    Construct after ledger.begin and before starting that action's worker.
    The host stores it only for its active joint action and calls cancel(reason)
    instead of its ordinary _fault for an explicit client cancellation. EOF,
    watchdog, device faults and generic errors must use the ordinary fault path.
    Do not call cancel while holding a lock needed by the action worker.
    """

    def __init__(self, host, event_id):
        self.host, self.ledger = host, host.ledger
        self.owner, self.run_id, self.deadline = host.owner, host.run_id, host.deadline
        original = self.ledger.event(event_id)
        if (original is None or original["owner"] != self.owner or original["status"] != "pending"
                or original["payload"].get("kind") != "joint"):
            raise HostJointBridgeError("Bridge requires this host's pending joint action")
        # PairHost sets this from the admitted scene before constructing the
        # bridge. A hold skips the ordinary fault guard, never its scene expiry.
        self.rgb_deadline = getattr(host, "dispatch_rgb_deadline", None)
        now = host.clock()
        if (type(self.rgb_deadline) not in (int, float) or not math.isfinite(self.rgb_deadline)
                or self.rgb_deadline <= 0 or type(now) not in (int, float)
                or not math.isfinite(now) or not 0 <= now <= self.rgb_deadline):
            raise HostJointBridgeError("Bridge requires the original finite current RGB deadline")
        self._last_hold_check_at = now
        self.event_id = event_id
        self._lock = threading.Lock()
        self._published = threading.Event()
        self._state = "idle"
        self._request = self._cancel_receipt = self._error = None
        self._reason = None
        self._worker = None

    def _scope(self, *, worker=False):
        if (self.host.ledger is not self.ledger or self.host.owner != self.owner
                or self.host.run_id != self.run_id or self.host.deadline != self.deadline
                or self.host.active_event_id != self.event_id):
            raise HostJointBridgeError("Original live host/action scope changed")
        if worker and (self._worker is None or threading.current_thread() is not self._worker):
            raise HostJointBridgeError("Only the original action worker may continue hold bookkeeping")

    def _fail(self, message):
        self.host.fault_event.set()
        with self._lock:
            self._fail_locked(message)

    def _fail_locked(self, message):
        self.host.fault_event.set()
        if self._error is None:
            self._error = str(message)
        self._state = "failed"
        self._published.set()

    def invalidate(self, reason):
        """Sticky local abort from EOF/watchdog/ordinary host fault; zero IO.

        The host's normal fault path retains durable fault/ownership facts.
        Because the cancellation already set fault_event, a separate local
        failure is required to distinguish this later failure from its request.
        A crashed process cannot restore this bridge or its ledger worker.
        """
        self._fail(reason)

    def check_active(self):
        """Final in-memory hold check; no SQLite, device call or host state lock.

        The adapter calls this after its final new feedback validation and
        immediately before the exact frame send. It cannot make check+CAN send
        atomic, and never claims that an already-started frame was prevented.
        """
        try:
            self._scope(worker=True)
        except Exception as exc:
            self._fail("Original hold scope became invalid: " + str(exc))
            raise HostJointBridgeError(self._error) from exc
        with self._lock:
            if self._state != "ready" or self._error is not None:
                raise HostJointBridgeError(self._error or "Hold cancellation is not active")
            try:
                now = self.host.clock()
            except Exception as exc:
                self._fail_locked("Hold clock failed: " + str(exc))
                raise HostJointBridgeError(self._error) from exc
            current = getattr(self.host, "dispatch_rgb_deadline", None)
            if (type(current) not in (int, float) or not math.isfinite(current)
                    or current != self.rgb_deadline):
                self._fail_locked("Original RGB deadline changed during hold")
            elif (type(now) not in (int, float) or not math.isfinite(now)
                  or now < self._last_hold_check_at):
                self._fail_locked("Hold clock became invalid or regressed")
            elif now > self.rgb_deadline or now >= self.deadline:
                self._fail_locked("Original RGB action window or task deadline expired during hold")
            else:
                self._last_hold_check_at = now
            if self._error is not None:
                raise HostJointBridgeError(self._error)

    def _persist_failure_locked(self, message):
        """One factual fault write while the host's fault-record lock is held."""
        self._fail(message)
        try:
            self.ledger.fault(self.owner, ("Explicit hold cancellation failed: " + str(message))[:4096])
        except BaseException as exc:
            self.host._fault_record_error = type(exc).__name__ + ": " + str(exc)

    def cancel(self, reason):
        """Explicit client cancellation only; never wait for the device worker.

        The pending marker precedes fault_event, so the adapter can wait with
        zero TX for publication instead of mistaking disk latency for no request.
        _fault_record_lock prevents normal _fault from winning that same typed
        cancellation race. Independent faults already recorded still win.
        """
        if type(reason) is not str or not reason.strip() or len(reason) > 4096:
            raise ValueError("Cancellation reason must be a nonempty string of at most 4096 characters")
        with self._lock:
            if self._state != "idle":
                if self._reason != reason:
                    raise HostJointBridgeError("Cancellation is immutable once requested")
                if self._state == "ready":
                    return copy.deepcopy(self._cancel_receipt)
                if self._state == "failed":
                    raise HostJointBridgeError(self._error)
                return {"status": "cancellation_pending", "software_cancelled": True,
                        "physical_stop_verified": None, "original_event_id": self.event_id}
            self._reason, self._state = reason, "pending"
            hold_id = "hold_" + uuid.uuid4().hex
            self.host.fault_event.set()
        # Never wait on a competing fault writer or the device lock. An earlier
        # normal fault owns this shutdown; the bridge must not reclassify it.
        if not self.host._fault_record_lock.acquire(blocking=False):
            self._fail("Another fault writer already owns cancellation")
            raise HostJointBridgeError(self._error)
        try:
            if self.host._fault_record_attempted:
                self._fail("A prior host fault cannot become a hold cancellation")
                raise HostJointBridgeError(self._error)
            self.host._fault_record_attempted = True
            self.host._fault_reason = ("Explicit hold cancellation: " + reason)[:4096]
            try:
                self._scope()
                facts = self.ledger.request_hold_cancel(self.owner, self.event_id, reason)
            except BaseException as exc:
                self.host._fault_record_error = type(exc).__name__ + ": " + str(exc)
                self._persist_failure_locked(exc)
                raise HostJointBridgeError("Durable explicit cancellation failed: " + str(exc)) from exc
            with self._lock:
                failure = self._error
                if failure is None:
                    self._request = {"hold_event_id": hold_id, "reason": "explicit_client_cancel"}
                    self._cancel_receipt = {"status": "cancellation_requested", "software_cancelled": True,
                        "original_event_id": self.event_id, "hold_event_id": hold_id,
                        "cancellation": facts, "physical_stop_verified": None}
                    self._state = "ready"
                    self._published.set()
                    result = copy.deepcopy(self._cancel_receipt)
            if failure is not None:
                self._persist_failure_locked(failure)
                raise HostJointBridgeError(failure)
            return result
        finally:
            self.host._fault_record_lock.release()

    def cancellation_request(self):
        """Poll with at most 25 ms requested wait; a timeout forfeits this hold.

        There is no feedback sampling inside this wait and it is not continuous
        motion monitoring. Ready publication still needs the adapter's new RX
        checks. Failure unwinds to the host's fault diagnostics, never a retry.
        """
        self._scope()
        with self._lock:
            state = self._state
        if state == "idle":
            return None
        if state == "pending":
            now = self.host.clock()
            if type(now) not in (int, float) or not math.isfinite(now) or not 0 <= now < self.deadline:
                self._fail("Cancellation publication has no valid original deadline")
            else:
                # Monotonic Event timeout also bounds a frozen/bad wall clock.
                remaining = min(CANCEL_PUBLICATION_WAIT_S, self.deadline-now)
                if not self._published.wait(remaining):
                    self._fail("Cancellation publication exceeded its bounded wait")
        with self._lock:
            if self._state != "ready" or self._error is not None:
                raise HostJointBridgeError(self._error or "Cancellation is not durably published")
            return copy.deepcopy(self._request)

    def record_original(self, event):
        self._scope()
        event = _wire(event)
        if (event.get("event_id") != self.event_id or event.get("deadline_at") != self.deadline
                or event.get("identity", {}).get("owner") != self.owner
                or event.get("identity", {}).get("run_id") != self.run_id):
            raise HostJointBridgeError("Adapter original event changed its frozen host binding")
        with self._lock:
            if self._worker is not None and self._worker is not threading.current_thread():
                raise HostJointBridgeError("Original send cannot move to another worker")
        result = self.ledger.record_original_send(self.owner, self.event_id, event)
        with self._lock:
            self._worker = threading.current_thread()
        return result

    def claim_hold(self, original_id, hold_id, payload):
        self._scope(worker=True)
        request = self.cancellation_request()
        if original_id != self.event_id or request is None or request["hold_event_id"] != hold_id:
            raise HostJointBridgeError("Hold does not match this explicit cancellation")
        self.check_active()
        return self.ledger.begin_hold(self.owner, self.event_id, hold_id, _wire(payload))

    def _bound_hold(self, hold_id):
        self._scope(worker=True)
        with self._lock:
            if self._request is None or self._request["hold_event_id"] != hold_id:
                raise HostJointBridgeError("Hold event changed")

    def record_frame_begin(self, hold_id, index, frame):
        self._bound_hold(hold_id)
        self.check_active()
        return self.ledger.begin_hold_frame(self.owner, hold_id, index, _wire(frame))

    def record_frame_return(self, hold_id, index, outcome, error=None):
        self._bound_hold(hold_id)
        return self.ledger.finish_hold_frame(self.owner, hold_id, index, outcome, error=error)

    def finish_hold(self, hold_id, receipt):
        self._bound_hold(hold_id)
        return self.ledger.finish_hold(self.owner, hold_id, _wire(receipt))
