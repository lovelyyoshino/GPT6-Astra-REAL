#!/usr/bin/env python3
"""Visible host-terminal probe: receive CAN only and read ROS graph; no commands."""
import json
import os
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from right_pick.robot import inspect_environment, check_ros_master


def main():
    report = inspect_environment()
    report["checked_at"] = time.time()
    report["ros_master_uri"] = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    report["can_interfaces"] = {}
    for interface in ("can1", "can2"):
        root = Path('/sys/class/net') / interface
        info = {"exists": root.exists()}
        for field in ("operstate", "flags"):
            try:
                info[field] = (root / field).read_text().strip()
            except OSError:
                info[field] = None
        info["usb_device"] = (root / 'device').resolve().name if (root / 'device').exists() else None
        report["can_interfaces"][interface] = info
    # Inspect only process names/script basenames, never environment or arguments
    # that might include credentials. This does not start or stop a controller.
    report["relevant_processes"] = []
    for process in Path('/proc').glob('[0-9]*'):
        try:
            args = (process / 'cmdline').read_bytes().split(b'\0')
            names = [Path(a.decode(errors='replace')).name for a in args[:3] if a]
            selected = [n for n in names if n in ('rosmaster', 'roscore', 'roslaunch')
                        or (n.startswith('piper_') and n.endswith('.py'))]
            if selected:
                report["relevant_processes"].append({"pid": int(process.name), "programs": selected})
        except (OSError, ValueError):
            continue
    report["can_receive_probes"] = {}
    # Passive SocketCAN subscriptions; zero send/sendto calls.
    for interface in ("can1", "can2"):
        sock = None
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.settimeout(0.1)
            sock.bind((interface,))
            deadline = time.monotonic() + 0.6
            count = 0
            local_count = 0
            remote_count = 0
            ids = set()
            id_counts = {}
            while time.monotonic() < deadline:
                try:
                    frame, ancillary, flags, address = sock.recvmsg(16)
                    count += 1
                    if flags & socket.MSG_DONTROUTE:
                        local_count += 1
                    else:
                        remote_count += 1
                    if len(frame) >= 4:
                        identifier = int.from_bytes(frame[:4], sys.byteorder) & socket.CAN_EFF_MASK
                        ids.add(identifier)
                        key = '0x%03X' % identifier
                        id_counts[key] = id_counts.get(key, 0) + 1
                except socket.timeout:
                    pass
            report["can_receive_probes"][interface] = {"received_frames":count,
                "local_origin_frames":local_count, "nonlocal_origin_frames":remote_count,
                "observed_ids":sorted(ids), "frame_id_counts":id_counts, "transmitted_frames":0}
        except OSError as exc:
            report["can_receive_probes"][interface] = {"error":type(exc).__name__, "reason":str(exc), "transmitted_frames":0}
        finally:
            if sock is not None:
                sock.close()
    try:
        report["ros_master"] = check_ros_master(2)
        import http.client
        import xmlrpc.client
        class Transport(xmlrpc.client.Transport):
            def make_connection(self, host):
                return http.client.HTTPConnection(host, timeout=2)
        with xmlrpc.client.ServerProxy(os.environ.get("ROS_MASTER_URI","http://localhost:11311"), transport=Transport()) as master:
            code, message, state = master.getSystemState("/right_pick_readonly_host_probe")
            if code == 1:
                report["bridge_parameters"] = {}
                nodes = {node for group in state for _, names in group for node in names}
                for node in ("/master_arm/bridge", "/right_arm/bridge"):
                    if node not in nodes:
                        continue
                    for key in ("can_port", "mode", "auto_enable"):
                        name = node + '/' + key
                        result = master.getParam("/right_pick_readonly_host_probe", name)
                        report["bridge_parameters"][name] = result[2] if result[0] == 1 else None
        if code == 1:
            report["ros_graph"] = {"publishers":state[0],"subscribers":state[1],"services":state[2]}
    except Exception as exc:
        report["ros_master"] = {"error":type(exc).__name__, "reason":str(exc)}
    report["video_devices"] = sorted(str(p) for p in Path('/dev').glob('video*'))
    report["note"] = "Read-only probe. CAN receipt can include other host publishers; receipt alone is not motor-enable or fresh-joint proof."
    output = ROOT / "runs" / ("host_probe_" + str(time.time_ns()) + ".json")
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(report,ensure_ascii=False,indent=2))
    print("Saved: " + str(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
