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


def case_stratum(index, count):
    quarter = count // 4
    if index < quarter:
        return "常规工作区"
    if index < 2 * quarter:
        return "接近关节边界"
    if index < 3 * quarter:
        return "限位附近"
    return "q3/q5 近奇异区域"


def summarize(values):
    if not values:
        return {"mean": None, "max": None, "p95": None}
    return {"mean": statistics.fmean(values),
            "max": max(values),
            "p95": float(np.percentile(values, 95))}


def run(root, count, seed):
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
    failure_counts = {}
    stratum_counts = {}
    stratum_errors = {}

    for case_index, joints in enumerate(cases):
        stratum = case_stratum(case_index, count)
        stratum_counts.setdefault(stratum, {"成功": 0, "候选生成失败": 0,
                                             "全部候选奇异": 0,
                                             "其他筛选失败": 0})
        stratum_errors.setdefault(stratum, {"fk_p": [], "fk_r": [],
                                            "ik_p": [], "ik_r": []})
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
        stratum_errors[stratum]["fk_p"].append(fk_position_errors[-1])
        stratum_errors[stratum]["fk_r"].append(fk_rotation_errors[-1])

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
                stratum_counts[stratum]["成功"] += 1
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
                stratum_errors[stratum]["ik_p"].append(ik_position_errors[-1])
                stratum_errors[stratum]["ik_r"].append(ik_rotation_errors[-1])
            else:
                failure_reason = ("全部候选奇异" if select_status == -3
                                  else "其他筛选失败")
                stratum_counts[stratum][failure_reason] += 1
                failure_counts[failure_reason] = failure_counts.get(failure_reason, 0) + 1
        else:
            stratum_counts[stratum]["候选生成失败"] += 1
            failure_counts["候选生成失败"] = failure_counts.get("候选生成失败", 0) + 1

    print(f"运动学精度验证：{count} 组样本，种子 {seed}")
    for name, values, unit in [
        ("FK 位置误差", fk_position_errors, "m"),
        ("FK 姿态误差", fk_rotation_errors, "rad"),
        ("IK 选中解位置误差", ik_position_errors, "m"),
        ("IK 选中解姿态误差", ik_rotation_errors, "rad"),
        ("FK 求解耗时", fk_times, "us"),
        ("IK 求解耗时", ik_times, "us"),
    ]:
        stats = summarize(values)
        print(f"  {name}: 均值 {stats['mean']:.6g} {unit}"
              f"  P95 {stats['p95']:.6g}  最大 {stats['max']:.6g}")
    print(f"  IK 候选生成 {ik_success}/{count}，最优解筛选 {selection_success}/{count}，"
          f"状态分布 {status_counts}")
    for stratum, counts in stratum_counts.items():
        print(f"  {stratum}: {counts}")
    for stratum, errors in stratum_errors.items():
        if not errors["ik_p"]:
            print(f"  {stratum} 误差：无通过筛选的样本")
            continue
        print(f"  {stratum} 误差：FK 位置 均值 "
              f"{statistics.fmean(errors['fk_p']):.3g} "
              f"最大 {max(errors['fk_p']):.3g} m；"
              f"FK 姿态 均值 {statistics.fmean(errors['fk_r']):.3g} "
              f"最大 {max(errors['fk_r']):.3g} rad；"
              f"IK 选中解位置 均值 {statistics.fmean(errors['ik_p']):.3g} "
              f"最大 {max(errors['ik_p']):.3g} m；"
              f"IK 姿态 均值 {statistics.fmean(errors['ik_r']):.3g} "
              f"最大 {max(errors['ik_r']):.3g} rad")
    print(f"  失败统计：{failure_counts}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=128)
    parser.add_argument("--seed", type=int, default=20260911)
    args = parser.parse_args()
    run(pathlib.Path(__file__).resolve().parents[2], args.count, args.seed)


if __name__ == "__main__":
    main()