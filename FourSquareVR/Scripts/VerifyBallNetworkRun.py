"""Compare received complete snapshots with their exact published sequence; stdlib only.

This verifies transport evidence. It does not qualify Steam, headset feel, visual quality or Quest.
"""
import argparse
import datetime as dt
import json
import pathlib
import re

parser = argparse.ArgumentParser()
parser.add_argument("case")
args = parser.parse_args()
if not re.fullmatch(r"[A-Za-z0-9_-]+", args.case):
    parser.error("invalid case")
root = pathlib.Path(__file__).resolve().parents[1]
logs = root / "Saved/Logs/BallStage5"

def timestamp(line):
    m = re.match(r"\[([\d.:-]+)\]", line)
    return dt.datetime.strptime(m[1], "%Y.%m.%d-%H.%M.%S:%f").timestamp() if m else 0

def snapshots(lines):
    result = {}
    for line in lines:
        if "NETSNAP " in line:
            values = dict(re.findall(r"(\w+)=([\w.]+)", line))
            result[int(values["Seq"])] = tuple(int(values[k]) for k in ("Frame", "Generation", "Timebase", "Config", "CRC"))
    return result

def summaries(lines, marker):
    return [(timestamp(line), dict(re.findall(r"(\w+)=([-\w.]+)", line))) for line in lines if marker in line]

server = (logs / f"{args.case}Server.log").read_text(errors="replace").splitlines()
reference = snapshots(server)
result = {"case": args.case, "server_snapshots": len(reference), "clients": {}, "note": "Payload bytes exclude Unreal/IP/Steam overhead; gaps are observations, not exact packet-loss percentages."}
ok = bool(reference)
for role in ("Client1", "Client2Late"):
    lines = (logs / f"{args.case}{role}.log").read_text(errors="replace").splitlines()
    received = snapshots(lines)
    bad = [seq for seq, state in received.items() if reference.get(seq) != state]
    samples = [(time, row) for time, row in summaries(lines, "NETTEST Hands Local=1") if int(row.get("SentPackets", "0")) > 0]
    rate = {}
    if len(samples) > 1:
        a, b = samples[0], samples[-1]
        span = b[0] - a[0]
        rate = {"pose_packets_per_second": (int(b[1]["SentPackets"])-int(a[1]["SentPackets"]))/span,
                "pose_payload_bytes_per_second": (int(b[1]["PoseBytes"])-int(a[1]["PoseBytes"]))/span,
                "last_RTT_ms": float(b[1]["RTTms"]), "last_uncertainty_ms": float(b[1]["Uncertaintyms"])}
    guard = any("ClientAuthorityGuard=PASS" in line for line in lines)
    denied = any("ServerSubmitPoses" in line and ("No owning connection" in line or "not be processed" in line) for line in lines)
    row = {"received_snapshots": len(received), "mismatches": bad, "generations": sorted({v[1] for v in received.values()}),
           "timebases": sorted({v[2] for v in received.values()}), "client_state_guard_passed": guard,
           "unowned_RPC_denial_observed": denied, **rate}
    result["clients"][role] = row
    ok &= len(received) >= 10 and not bad and guard and denied and len(row["generations"]) >= 2

hands = summaries(server, "NETTEST Hands")
result["server_participants"] = sorted({int(row["Participant"]) for _, row in hands if int(row["Participant"])})
result["maximum_accepted_by_participant"] = {str(pid): max(int(row.get("Accepted", "0")) for _, row in hands if int(row["Participant"]) == pid) for pid in result["server_participants"]}
result["configuration_boundaries_passed"] = sum("ConfigurationBoundary=PASS" in line for line in server)
result["resets_observed"] = sum("ResetGeneration=" in line for line in server)
result["last_simulation"] = next((line.split("NETTEST Simulation ", 1)[1] for line in reversed(server) if "NETTEST Simulation " in line), "")
result["fatal_or_errors"] = [line for line in server if "Fatal error:" in line or "LogHandballBall: Error:" in line]
ok &= len(result["server_participants"]) >= 2 and all(n > 10 for n in result["maximum_accepted_by_participant"].values())
ok &= bool(result["configuration_boundaries_passed"] and result["resets_observed"]) and not result["fatal_or_errors"]
result["transport_checks_passed"] = bool(ok)
destination = root / "Saved/Automation/BallStage5" / f"{args.case}Summary.json"
destination.write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
raise SystemExit(0 if ok else 1)
