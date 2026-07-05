"""
Calibrate a single motor without touching other motors.

Motor name → ID mapping:
  shoulder_pan=1, shoulder_lift=2, elbow_flex=3,
  wrist_flex=4, wrist_roll=5, gripper=6

Usage examples:
  # 기본 (homing + range 자동)
  python examples/calibrate_single_motor.py --port COM4 --id leader --motor wrist_roll

  # 인터랙티브 범위 기록 (비-full-turn 모터만 유효)
  python examples/calibrate_single_motor.py --port COM4 --id leader --motor shoulder_pan --record-range

  # 범위 직접 지정 (Present_Position 기준)
  python examples/calibrate_single_motor.py --port COM4 --id leader --motor shoulder_pan --range-min 500 --range-max 3500

주의:
  - wrist_roll은 360도 회전 모터이므로 --record-range / --range-min/max 무시됨
  - 하드웨어 position limit은 Actual_Position (raw) 기준으로 저장됨
  - wrist_roll 안전 제한은 follower config의 max_relative_target 사용 권장
"""

import argparse
import json
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

# 360도 회전 모터: 하드웨어 position limit 사용 불가 (wrap-around 문제)
FULL_TURN_MOTORS = {"wrist_roll"}

ENCODER_MAX = 4095  # STS3215 12-bit


def load_calibration(fpath: Path) -> dict:
    if fpath.is_file():
        with open(fpath) as f:
            return json.load(f)
    return {}


def save_calibration(fpath: Path, calib: dict) -> None:
    fpath.parent.mkdir(parents=True, exist_ok=True)
    with open(fpath, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"Calibration saved → {fpath}")


def calibrate_motor(
    port: str,
    arm_id: str,
    motor_name: str,
    range_min_present: int | None = None,
    range_max_present: int | None = None,
    record_range: bool = False,
) -> None:
    calib_fpath = (
        Path.home()
        / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader"
        / f"{arm_id}.json"
    )

    is_full_turn = motor_name in FULL_TURN_MOTORS

    if is_full_turn and (record_range or range_min_present or range_max_present):
        print(
            f"[경고] '{motor_name}'은 360도 회전 모터입니다.\n"
            "  하드웨어 position limit은 wrap-around 문제로 신뢰할 수 없어 무시됩니다.\n"
            "  안전 제한은 follower config의 max_relative_target을 사용하세요."
        )

    bus = FeetechMotorsBus(port=port, motors=MOTORS)
    bus.connect()

    try:
        bus.disable_torque()
        bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)

        # --- Step 1: homing 초기화 (이 모터만) ---
        bus.write("Homing_Offset", motor_name, 0, normalize=False)
        bus.write("Min_Position_Limit", motor_name, 0, normalize=False)
        bus.write("Max_Position_Limit", motor_name, ENCODER_MAX, normalize=False)

        # --- Step 2: 중간 위치로 이동 → raw 위치 읽기 ---
        input(f"\n'{motor_name}'을 범위의 중간(CENTER) 위치로 이동하고 ENTER...")
        raw_center = int(bus.read("Present_Position", motor_name, normalize=False))
        print(f"  Raw center position: {raw_center}")

        # --- Step 3: homing_offset 계산 및 기록 ---
        # Feetech: Present_Position = Actual - Homing_Offset
        # 목표: 중간 위치에서 Present_Position = 2047
        homing_offset = raw_center - (ENCODER_MAX // 2)
        bus.write("Homing_Offset", motor_name, homing_offset, normalize=False)
        print(f"  Homing_Offset: {homing_offset}  (center → Present={ENCODER_MAX // 2})")

        # --- Step 4: 범위 결정 ---
        if is_full_turn:
            # full-turn 모터: 하드웨어 제한 없음, 소프트웨어 정규화용 전체 범위
            range_min_present = 0
            range_max_present = ENCODER_MAX
            hw_min = 0
            hw_max = ENCODER_MAX
            print(f"  Full-turn motor → hardware limit: [0, {ENCODER_MAX}] (제한 없음)")

        elif record_range:
            input(f"\n'{motor_name}'을 안전한 최솟값(MIN) 위치로 이동하고 ENTER...")
            range_min_present = int(bus.read("Present_Position", motor_name, normalize=False))
            print(f"  Min recorded (Present): {range_min_present}")

            input(f"'{motor_name}'을 안전한 최댓값(MAX) 위치로 이동하고 ENTER...")
            range_max_present = int(bus.read("Present_Position", motor_name, normalize=False))
            print(f"  Max recorded (Present): {range_max_present}")

            if range_min_present > range_max_present:
                range_min_present, range_max_present = range_max_present, range_min_present
                print("  (min/max 자동 교환)")

            # Present_Position → Actual_Position 변환하여 EEPROM에 기록
            # Actual = Present + homing_offset
            hw_min = max(0, min(ENCODER_MAX, range_min_present + homing_offset))
            hw_max = max(0, min(ENCODER_MAX, range_max_present + homing_offset))
            print(f"  Software range (Present): [{range_min_present}, {range_max_present}]")
            print(f"  Hardware limit (Actual):  [{hw_min}, {hw_max}]")

        elif range_min_present is not None and range_max_present is not None:
            hw_min = max(0, min(ENCODER_MAX, range_min_present + homing_offset))
            hw_max = max(0, min(ENCODER_MAX, range_max_present + homing_offset))
            print(f"  Software range (Present): [{range_min_present}, {range_max_present}]")
            print(f"  Hardware limit (Actual):  [{hw_min}, {hw_max}]")

        else:
            # 기본값: full range
            range_min_present = 0
            range_max_present = ENCODER_MAX
            hw_min = 0
            hw_max = ENCODER_MAX
            print(f"  Range (default full): [0, {ENCODER_MAX}]")

        # --- Step 5: 하드웨어 EEPROM에 Actual 기준 제한 기록 ---
        bus.write("Min_Position_Limit", motor_name, hw_min, normalize=False)
        bus.write("Max_Position_Limit", motor_name, hw_max, normalize=False)

        # --- Step 6: JSON 업데이트 (Present_Position 기준, 다른 모터 유지) ---
        motor = MOTORS[motor_name]
        calib = load_calibration(calib_fpath)
        calib[motor_name] = {
            "id": motor.id,
            "drive_mode": 0,
            "homing_offset": homing_offset,
            "range_min": range_min_present,   # Present_Position 기준 (lerobot 정규화용)
            "range_max": range_max_present,
        }
        save_calibration(calib_fpath, calib)

    finally:
        bus.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="단일 모터 캘리브레이션 (다른 모터 설정 유지)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", required=True, help="시리얼 포트 (예: COM4)")
    parser.add_argument("--id", required=True, dest="arm_id", help="팔 ID (캘리브레이션 파일명)")
    parser.add_argument(
        "--motor",
        required=True,
        choices=list(MOTORS.keys()),
        help="캘리브레이션할 모터 이름",
    )
    parser.add_argument("--range-min", type=int, default=None, help="최소 범위 (Present_Position 기준)")
    parser.add_argument("--range-max", type=int, default=None, help="최대 범위 (Present_Position 기준)")
    parser.add_argument("--record-range", action="store_true", help="손으로 이동해 min/max 직접 기록")
    args = parser.parse_args()

    calibrate_motor(
        port=args.port,
        arm_id=args.arm_id,
        motor_name=args.motor,
        range_min_present=args.range_min,
        range_max_present=args.range_max,
        record_range=args.record_range,
    )
