#!/usr/bin/env python3
"""Compare the embedded UR5 FK/IK implementation with Robotics Toolbox.

The reference robot is constructed with the exact DH constants documented by
this repository. This avoids silently comparing against the different d1
value used by the toolbox's convenience UR5 model.
"""

import argparse
import ctypes
import math
import pathlib
import random
import statistics
import subprocess
import tempfile
import time

import numpy as np
import roboticstoolbox as rtb


JOINT_MIN = np.array([-2 * math.pi, -2 * math.pi, -math.pi,
                      -2 * math.pi, -2 * math.pi, -2 * math.pi])
JOINT_MAX = -JOINT_MIN
DH_A = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
DH_D = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
DH_ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]


class RobotPose(ctypes.Structure):
    _fields_ = [("value", (ctypes.c_float * 4) * 4)]


class Solution(ctypes.Structure):
    _fields_ = [("joint", ctypes.c_float * 6)]


class Score(ctypes.Structure):
    _fields_ = [("travel_cost", ctypes.c_float),
                ("limit_cost", ctypes.c_float),
                ("singularity_cost", ctypes.c_float),
                ("total_cost", ctypes.c_float),
                ("singular", ctypes.c_uint8)]


def build_library(root):
    output = pathlib.Path(tempfile.gettempdir()) / "robot_kinematics_validation.so"
    command = ["cc", "-shared", "-fPIC", "-std=c11", "-O2", "-I",
               str(root / "firmware/app"), str(root / "firmware/app/kinematics.c"),
               "-lm", "-o", str(output)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    return output


def load_embedded(root):
    library = ctypes.CDLL(str(build_library(root)))
    library.robot_kinematics_fk.argtypes = [ctypes.POINTER(ctypes.c_float),
                                             ctypes.POINTER(RobotPose)]
    library.robot_kinematics_fk.restype = ctypes.c_int
    library.robot_kinematics_ik.argtypes = [ctypes.POINTER(RobotPose),
                                             ctypes.POINTER(ctypes.c_float),
                                             ctypes.POINTER(Solution),
                                             ctypes.POINTER(ctypes.c_uint8)]
    library.robot_kinematics_ik.restype = ctypes.c_int
    library.robot_kinematics_select_best.argtypes = [ctypes.POINTER(Solution),
                                                      ctypes.c_uint8,
                                                      ctypes.POINTER(ctypes.c_float),
                                                      ctypes.POINTER(ctypes.c_uint8),
                                                      ctypes.POINTER(Score)]
    library.robot_kinematics_select_best.restype = ctypes.c_int
    return library


def make_reference_robot():
    links = [rtb.RevoluteDH(a=a, d=d, alpha=alpha)
             for a, d, alpha in zip(DH_A, DH_D, DH_ALPHA)]
    return rtb.DHRobot(links, name="UR5_project_dh")


def pose_from_ctypes(pose):
    return np.array([[pose.value[row][column] for column in range(4)]
                     for row in range(4)], dtype=float)


def pose_to_ctypes(matrix):
    pose = RobotPose()
    for row in range(4):
        for column in range(4):
            pose.value[row][column] = float(matrix[row, column])
    return pose


def rotation_error(left, right):
    relative = left[:3, :3].T @ right[:3, :3]
    skew = np.array([relative[2, 1] - relative[1, 2],
                     relative[0, 2] - relative[2, 0],
                     relative[1, 0] - relative[0, 1]])
    sine = 0.5 * float(np.linalg.norm(skew))
    cosine = 0.5 * float(np.trace(relative) - 1.0)
    return float(math.atan2(sine, cosine))


def make_cases(count, seed):
    if count < 100:
        raise ValueError("count must be at least 100")
    random_generator = random.Random(seed)
    cases = []
    strata = (count // 4, count // 4, count // 4, count - 3 * (count // 4))
    for index in range(strata[0]):
        cases.append(np.array([random_generator.uniform(float(low), float(high))
                               for low, high in zip(JOINT_MIN * 0.75, JOINT_MAX * 0.75)]))
    for index in range(strata[1]):
        cases.append(np.array([random_generator.uniform(float(low), float(high))
                               for low, high in zip(JOINT_MIN * 0.98, JOINT_MAX * 0.98)]))
    for index in range(strata[2]):
        joint = np.array([random_generator.uniform(float(low), float(high))
                          for low, high in zip(JOINT_MIN * 0.75, JOINT_MAX * 0.75)])
        joint[index % 6] = float(JOINT_MAX[index % 6] - 0.01)
        cases.append(joint)
    for index in range(strata[3]):
        joint = np.array([random_generator.uniform(float(low), float(high))
                          for low, high in zip(JOINT_MIN * 0.75, JOINT_MAX * 0.75)])
        joint[2] = 0.001 if index % 2 == 0 else -0.001
        joint[4] = 0.001 if index % 3 == 0 else 0.8
        cases.append(joint)
    return cases


def summarize(values):
    if not values:
        return {"mean": None, "max": None, "p95": None}
    return {"mean": statistics.fmean(values),
            "max": max(values),
            "p95": float(np.percentile(values, 95))}


def run(root, count, seed, output):
    embedded = load_embedded(root)
    reference = make_reference_robot()
    cases = make_cases(count, seed)
    fk_position_errors = []
    fk_rotation_errors = []
    ik_position_errors = []
    ik_rotation_errors = []
    fk_times = []
    reference_fk_times = []
    ik_times = []
    ik_success = 0
    selection_success = 0
    status_counts = {}

    for joints in cases:
        start = time.perf_counter_ns()
        reference_pose = np.asarray(reference.fkine(joints).A, dtype=float)
        reference_fk_times.append((time.perf_counter_ns() - start) / 1000.0)
        joint_array = (ctypes.c_float * 6)(*map(float, joints))
        embedded_pose = RobotPose()
        start = time.perf_counter_ns()
        fk_status = embedded.robot_kinematics_fk(joint_array, ctypes.byref(embedded_pose))
        fk_times.append((time.perf_counter_ns() - start) / 1000.0)
        if fk_status != 0:
            raise RuntimeError(f"FK rejected valid case with status {fk_status}")
        embedded_matrix = pose_from_ctypes(embedded_pose)
        fk_position_errors.append(float(np.linalg.norm(
            embedded_matrix[:3, 3] - reference_pose[:3, 3])))
        fk_rotation_errors.append(rotation_error(embedded_matrix, reference_pose))

        target = pose_to_ctypes(reference_pose)
        solutions = (Solution * 8)()
        solution_count = ctypes.c_uint8(0)
        start = time.perf_counter_ns()
        ik_status = embedded.robot_kinematics_ik(
            ctypes.byref(target), joint_array, solutions, ctypes.byref(solution_count))
        ik_times.append((time.perf_counter_ns() - start) / 1000.0)
        status_counts[ik_status] = status_counts.get(ik_status, 0) + 1
        if ik_status == 0 and solution_count.value > 0:
            ik_success += 1
            best_index = ctypes.c_uint8(0)
            score = Score()
            select_status = embedded.robot_kinematics_select_best(
                solutions, solution_count.value, joint_array,
                ctypes.byref(best_index), ctypes.byref(score))
            if select_status == 0:
                selection_success += 1
                selected = np.array(solutions[best_index.value].joint, dtype=float)
                selected_array = (ctypes.c_float * 6)(*map(float, selected))
                selected_embedded_pose = RobotPose()
                selected_fk_status = embedded.robot_kinematics_fk(
                    selected_array, ctypes.byref(selected_embedded_pose))
                if selected_fk_status != 0:
                    raise RuntimeError(
                        f"selected IK solution rejected by embedded FK: {selected_fk_status}")
                selected_pose = pose_from_ctypes(selected_embedded_pose)
                ik_position_errors.append(float(np.linalg.norm(
                    selected_pose[:3, 3] - reference_pose[:3, 3])))
                ik_rotation_errors.append(rotation_error(selected_pose, reference_pose))

    report = []
    report.append("# UR5 运动学精度对比分析")
    report.append("")
    report.append(f"- 测试样本：{count} 组，随机种子：{seed}")
    report.append("- 基准：Robotics Toolbox Python `1.4.2`，使用项目文档中的精确标准 DH 参数构造 `DHRobot`。")
    report.append("- 被测实现：`firmware/app/kinematics.c`，通过 host `ctypes` 调用，与 ARM 固件共用源码。")
    report.append("- 样本分层：常规工作区、接近关节边界、限位附近、q3/q5 近奇异区域，各占约四分之一。")
    report.append("")
    report.append("## 结果")
    report.append("")
    report.append("| 指标 | 均值 | P95 | 最大值 | 单位 |")
    report.append("|---|---:|---:|---:|---|")
    for name, values, unit in [
        ("FK 位置误差", fk_position_errors, "m"),
        ("FK 姿态误差", fk_rotation_errors, "rad"),
        ("IK 选中解位置误差", ik_position_errors, "m"),
        ("IK 选中解姿态误差", ik_rotation_errors, "rad"),
        ("FK 求解耗时", fk_times, "us"),
        ("Robotics Toolbox FK 耗时", reference_fk_times, "us"),
        ("IK 求解耗时", ik_times, "us"),
    ]:
        stats = summarize(values)
        report.append(f"| {name} | {stats['mean']:.6g} | {stats['p95']:.6g} | "
                      f"{stats['max']:.6g} | {unit} |")
    report.append("")
    report.append(f"- IK 候选生成成功率：`{ik_success}/{count}` "
                  f"({100.0 * ik_success / count:.2f}%)")
    report.append(f"- 最优解筛选成功率：`{selection_success}/{count}` "
                  f"({100.0 * selection_success / count:.2f}%)")
    report.append(f"- IK 状态分布：`{status_counts}`，其中 `0` 为 `ROBOT_KINEMATICS_OK`。")
    report.append("")
    report.append("## 分析")
    report.append("")
    report.append("FK 误差直接衡量嵌入式标准 DH 链与 Robotics Toolbox 同一模型的数值一致性；IK 误差使用嵌入式筛选候选回代到嵌入式 FK 后，与基准目标位姿比较。")
    report.append("耗时为 Linux host 上的 C 共享库调用耗时，不能等同于 Cortex-M4F 实时耗时，只用于比较算法调用开销趋势；部署性能仍应结合 QEMU 或硬件 cycle counter 测量。")
    report.append("")
    pathlib.Path(output).write_text("\n".join(report) + "\n", encoding="utf-8")
    print("\n".join(report))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=128)
    parser.add_argument("--seed", type=int, default=20260911)
    parser.add_argument("--output", default="docs/kinematics_validation_report.md")
    args = parser.parse_args()
    root = pathlib.Path(__file__).resolve().parents[2]
    run(root, args.count, args.seed, root / args.output)


if __name__ == "__main__":
    main()