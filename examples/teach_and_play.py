"""
SO-101 Follower 티칭 & 플레이백 데모

[티칭]
  토크 OFF → 손으로 팔을 이동 → ENTER로 웨이포인트 기록 → JSON 저장

[재생]
  저장된 웨이포인트를 순서대로 보간하며 재생

Usage:
  # 티칭 (웨이포인트 기록)
  python examples/teach_and_play.py --port COM4 --teach

  # 재생
  python examples/teach_and_play.py --port COM4 --play

  # 재생 옵션 (파일 지정, 3회 반복, 웨이포인트 간 3초, 부드러운 보간)
  python examples/teach_and_play.py --port COM4 --play --file basket_pick.json --repeat 3 --delay 3.0 --interp-steps 30
"""

import argparse
import json
import time
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

MOTORS = {
    "shoulder_pan":  Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex":    Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex":    Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll":    Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

DEFAULT_FILE = Path("examples/waypoints.json")


def read_positions(bus: FeetechMotorsBus) -> dict[str, int]:
    raw = bus.sync_read("Present_Position", normalize=False)
    return {k: int(v) for k, v in raw.items()}


def write_positions(bus: FeetechMotorsBus, positions: dict[str, int]) -> None:
    bus.sync_write("Goal_Position", positions, normalize=False)


def interpolate(start: dict, end: dict, steps: int):
    """두 웨이포인트 사이를 선형 보간, steps개의 중간 위치를 yield."""
    for step in range(1, steps + 1):
        alpha = step / steps
        yield {
            name: int(start[name] + (end[name] - start[name]) * alpha)
            for name in start
        }


# ─────────────────────────────────────────────
# 티칭
# ─────────────────────────────────────────────
def teach(port: str, waypoints_file: Path) -> None:
    bus = FeetechMotorsBus(port=port, motors=MOTORS)
    bus.connect()

    try:
        bus.disable_torque()
        for motor in MOTORS:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        waypoints: list[dict] = []

        print("\n=== TEACH MODE ===")
        print("토크 OFF — 팔을 원하는 위치로 직접 이동하세요.")
        print("ENTER : 현재 위치 기록")
        print("'q'   : 저장 후 종료\n")

        idx = 1
        while True:
            raw = input(f"Waypoint {idx:02d}> 위치 잡고 ENTER (종료: q): ").strip().lower()
            if raw == "q":
                break

            pos = read_positions(bus)
            waypoints.append(pos)

            col_w = max(len(n) for n in pos) + 1
            print("  기록됨:")
            for name, val in pos.items():
                print(f"    {name:<{col_w}} = {val}")
            print()
            idx += 1

        if waypoints:
            waypoints_file.parent.mkdir(parents=True, exist_ok=True)
            with open(waypoints_file, "w") as f:
                json.dump(waypoints, f, indent=2)
            print(f"총 {len(waypoints)}개 웨이포인트 저장 → {waypoints_file}")
        else:
            print("기록된 웨이포인트가 없습니다.")

    finally:
        bus.disable_torque()
        bus.disconnect()
        print("연결 해제.")


# ─────────────────────────────────────────────
# 재생
# ─────────────────────────────────────────────
def play(
    port: str,
    waypoints_file: Path,
    delay: float,
    repeat: int,
    interp_steps: int,
) -> None:
    if not waypoints_file.exists():
        raise FileNotFoundError(f"웨이포인트 파일 없음: {waypoints_file}")

    with open(waypoints_file) as f:
        waypoints: list[dict] = json.load(f)

    if not waypoints:
        print("재생할 웨이포인트가 없습니다.")
        return

    print(f"\n=== PLAY MODE ===")
    print(f"웨이포인트 {len(waypoints)}개  |  {repeat}회 반복  |  딜레이 {delay}s  |  보간 {interp_steps}스텝\n")

    bus = FeetechMotorsBus(port=port, motors=MOTORS)
    bus.connect()

    try:
        bus.enable_torque()
        for motor in MOTORS:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        step_delay = delay / interp_steps

        for run in range(repeat):
            if repeat > 1:
                print(f"─── Run {run + 1}/{repeat} ───")

            current = read_positions(bus)

            for i, target in enumerate(waypoints):
                label = f"Waypoint {i + 1:02d}/{len(waypoints)}"
                print(f"  → {label}  (gripper={target.get('gripper', '?')})")

                for interp_pos in interpolate(current, target, interp_steps):
                    write_positions(bus, interp_pos)
                    time.sleep(step_delay)

                current = target

            print(f"  Run {run + 1} 완료.\n")

        print("재생 완료.")

    except KeyboardInterrupt:
        print("\n[Ctrl+C] 중단 — 토크 OFF")

    finally:
        bus.disable_torque()
        bus.disconnect()
        print("연결 해제.")


# ─────────────────────────────────────────────
# 진입점
# ─────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="SO-101 Follower 티칭 & 플레이백",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", default="COM4", help="시리얼 포트 (기본: COM4)")
    parser.add_argument(
        "--file", type=Path, default=DEFAULT_FILE,
        help=f"웨이포인트 JSON 파일 (기본: {DEFAULT_FILE})",
    )

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--teach", action="store_true", help="티칭 모드: 웨이포인트 기록")
    mode.add_argument("--play",  action="store_true", help="재생 모드: 웨이포인트 실행")

    parser.add_argument(
        "--delay", type=float, default=2.0,
        help="웨이포인트 간 이동 시간(초) (기본: 2.0)",
    )
    parser.add_argument(
        "--repeat", type=int, default=1,
        help="반복 횟수 (기본: 1)",
    )
    parser.add_argument(
        "--interp-steps", type=int, default=20,
        help="보간 스텝 수 — 클수록 부드러움 (기본: 20)",
    )

    args = parser.parse_args()

    if args.teach:
        teach(port=args.port, waypoints_file=args.file)
    else:
        play(
            port=args.port,
            waypoints_file=args.file,
            delay=args.delay,
            repeat=args.repeat,
            interp_steps=args.interp_steps,
        )
