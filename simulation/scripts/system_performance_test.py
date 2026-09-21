#!/usr/bin/env python3
import argparse
import json
import logging
import pathlib
import subprocess
import sys
import tempfile
import time

import pybullet as p

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/scripts"))

from protocol import CMD_SET_OBSTACLES, obstacles_payload  # noqa: E402
from qemu_client import QemuRobotClient  # noqa: E402
from qemu_pybullet_pick_place import (SCENARIOS, joint_torque_limits,  # noqa: E402
                                      load_scene, run_cycle)

APP = ROOT / "firmware/app"
FIRMWARE = ROOT / "firmware/build/robot_firmware.elf"
SOURCES = ("cartesian_trajectory.c", "artificial_potential_field.c",
           "kinematics.c", "joint_pid.c")
ARRIVAL_LIMIT_M = 0.005
PLACE_LIMIT_M = 0.005
ATTEMPTS = 6

COST_SOURCE = r'''
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "artificial_potential_field.h"
#include "cartesian_trajectory.h"
#include "joint_pid.h"
#include "kinematics.h"

#define JN ROBOT_KINEMATICS_JOINT_COUNT
#define OB 2U

static double now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double) ts.tv_sec * 1e6 + (double) ts.tv_nsec * 1e-3;
}

static int cmp(const void *a, const void *b)
{
    double x = *(const double *) a;
    double y = *(const double *) b;
    return x < y ? -1 : x > y;
}

static void stats(const char *label, double *values, int count)
{
    double sum = 0.0;
    int index;

    if (count == 0) {
        return;
    }
    qsort(values, (size_t) count, sizeof(double), cmp);
    for (index = 0; index < count; index++) {
        sum += values[index];
    }
    printf("%s: 均值 %.2f us，P95 %.2f，最大 %.2f\n", label, sum / (double) count,
           values[(int) (0.95 * (double) (count - 1))], values[count - 1]);
}

static void build_obstacles(robot_apf_obstacle_t *o, const robot_pose_t *s,
                            const robot_pose_t *e)
{
    float mid_x = 0.5f * (s->value[0][3] + e->value[0][3]);
    float mid_y = 0.5f * (s->value[1][3] + e->value[1][3]);
    float mid_z = 0.5f * (s->value[2][3] + e->value[2][3]);

    memset(o, 0, sizeof(robot_apf_obstacle_t) * OB);
    o[0].minimum[0] = mid_x - 0.04f; o[0].minimum[1] = mid_y + 0.06f;
    o[0].minimum[2] = 0.0f;
    o[0].maximum[0] = mid_x + 0.04f; o[0].maximum[1] = mid_y + 0.16f;
    o[0].maximum[2] = mid_z + 0.10f;
    o[0].clearance_m = 0.02f; o[0].influence_radius = 0.20f;
    o[0].repulsive_gain = 0.02f;
    o[1] = o[0];
    o[1].minimum[1] = mid_y - 0.16f;
    o[1].maximum[1] = mid_y - 0.06f;
}

static void measure_apf(const robot_pose_t *start, robot_apf_obstacle_t *obstacles,
                       int repeats)
{
    robot_apf_config_t config;
    float nominal[3];
    float adjusted[3];
    volatile float sink = 0.0f;
    double begin;
    int index;

    robot_apf_config_init(&config);
    if (robot_apf_set_obstacles(&config, obstacles, OB) != 0) {
        return;
    }
    nominal[0] = start->value[0][3];
    nominal[1] = start->value[1][3];
    nominal[2] = start->value[2][3];
    begin = now_us();
    for (index = 0; index < repeats; index++) {
        (void) robot_apf_adjust_target(&config, nominal, nominal, adjusted);
        sink += adjusted[0];
    }
    printf("APF 单次修正（%d 次平均）: %.3f us\n", repeats,
           (now_us() - begin) / (double) repeats);
    if (sink > 1.0e30f) {
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    int cycles = argc > 2 && strcmp(argv[1], "--cycles") == 0 ? atoi(argv[2]) : 2000;
    robot_cartesian_trajectory_t traj;
    robot_joint_pid_t pid[JN];
    robot_pose_t start;
    robot_pose_t end;
    robot_apf_obstacle_t obstacles[OB];
    robot_pid_config_t pid_config = {2.5f, 0.3f, 0.025f, 1.0f, 2.0f, 0.0f, 0.01f};
    float joint[JN] = {0.0f, -1.20f, 1.20f, -1.60f, -1.57f, 0.0f};
    float feedback[JN];
    double *traj_times;
    double *pid_times;
    double *total_times;
    int traj_count = 0;
    int pid_count = 0;
    int total_count = 0;
    int cycle;
    uint8_t index;

    if (cycles <= 0) {
        return 1;
    }
    traj_times = malloc(sizeof(double) * (size_t) cycles);
    pid_times = malloc(sizeof(double) * (size_t) cycles);
    total_times = malloc(sizeof(double) * (size_t) cycles);
    if (traj_times == NULL || pid_times == NULL || total_times == NULL) {
        return 1;
    }
    if (robot_kinematics_fk(joint, &start) != ROBOT_KINEMATICS_OK) {
        return 1;
    }
    end = start;
    end.value[0][3] += 0.30f;
    end.value[2][3] -= 0.10f;
    build_obstacles(obstacles, &start, &end);
    if (robot_cartesian_plan_line(&traj, &start, &end, joint, 2.0f, 0.01f)
        != ROBOT_CARTESIAN_OK) {
        return 1;
    }
    if (robot_cartesian_set_obstacles(&traj, obstacles, OB) != ROBOT_CARTESIAN_OK) {
        return 1;
    }
    for (index = 0U; index < JN; index++) {
        if (robot_joint_pid_init(&pid[index], &pid_config) != ROBOT_PID_OK) {
            return 1;
        }
    }
    memcpy(feedback, joint, sizeof(feedback));
    for (cycle = 0; cycle < cycles; cycle++) {
        robot_cartesian_status_t status;
        double begin = now_us();
        double traj_cost;
        double pid_cost;

        status = robot_cartesian_update(&traj, 0.01f, traj.output_joint);
        traj_cost = now_us() - begin;
        traj_times[traj_count++] = traj_cost;
        if (status == ROBOT_CARTESIAN_COMPLETE) {
            memcpy(feedback, joint, sizeof(feedback));
            if (robot_cartesian_plan_line(&traj, &start, &end, joint, 2.0f, 0.01f)
                != ROBOT_CARTESIAN_OK) {
                return 1;
            }
            (void) robot_cartesian_set_obstacles(&traj, obstacles, OB);
            continue;
        }
        if (status != ROBOT_CARTESIAN_OK) {
            return 1;
        }
        begin = now_us();
        for (index = 0U; index < JN; index++) {
            float command = 0.0f;
            if (robot_joint_pid_update(&pid[index], traj.output_joint[index],
                                       feedback[index], &command) == ROBOT_PID_OK) {
                feedback[index] += command * 0.01f;
            }
        }
        pid_cost = now_us() - begin;
        pid_times[pid_count++] = pid_cost;
        total_times[total_count++] = traj_cost + pid_cost;
    }
    printf("单控制周期开销（主机原生 -O2，%d 个周期，2 个障碍）\n", cycles);
    stats("插补 + 避障修正 + 周期 IK", traj_times, traj_count);
    stats("6 关节增量式 PID + 前馈", pid_times, pid_count);
    stats("单周期合计", total_times, total_count);
    measure_apf(&start, obstacles, 1000000);
    free(traj_times);
    free(pid_times);
    free(total_times);
    return 0;
}
'''

GDB_SOURCE = r'''
import json
import gdb

STACK_WORDS = {"task_init": 512, "communication": 2048, "path": 1024,
               "pid": 2048, "status": 384, "IDLE": 128}
HEAP_TOTAL_BYTES = 64 * 1024
FILL_BYTE = 0xA5


def walk(list_value, collected):
    item_type = gdb.lookup_type("ListItem_t").pointer()
    end = list_value["xListEnd"].address
    node = list_value["xListEnd"]["pxNext"]
    while int(node) != int(end):
        item = node.cast(item_type)
        collected.append(int(item["pvOwner"]))
        node = item["pxNext"]


def collect():
    collected = []
    ready = gdb.parse_and_eval("pxReadyTasksLists")
    count = int(gdb.parse_and_eval("sizeof(pxReadyTasksLists) / sizeof(List_t)"))
    for index in range(count):
        walk(ready[index], collected)
    for name in ("pxDelayedTaskList", "pxOverflowDelayedTaskList"):
        pointer = gdb.parse_and_eval(name)
        if int(pointer) != 0:
            walk(pointer.dereference(), collected)
    walk(gdb.parse_and_eval("xSuspendedTaskList"), collected)
    unique = []
    for address in collected:
        if address not in unique:
            unique.append(address)
    return unique


def usage(address):
    tcb = gdb.Value(address).cast(gdb.lookup_type("TCB_t").pointer())
    name = tcb["pcTaskName"].string()
    words = STACK_WORDS.get(name, min(STACK_WORDS.values()))
    size = words * 4
    data = gdb.selected_inferior().read_memory(int(tcb["pxStack"]), size).tobytes()
    free = 0
    for byte in data:
        if byte != FILL_BYTE:
            break
        free += 1
    return {"task": name, "stack_words": words, "stack_bytes": size,
            "free_bytes": free, "used_bytes": size - free,
            "used_percent": round(100.0 * (size - free) / size, 1)}


payload = {"heap_total_bytes": HEAP_TOTAL_BYTES,
           "heap_free_bytes": int(gdb.parse_and_eval("xFreeBytesRemaining")),
           "heap_min_ever_free_bytes": int(
               gdb.parse_and_eval("xMinimumEverFreeBytesRemaining")),
           "tasks": [usage(address) for address in collect()]}
print("RESOURCE_JSON " + json.dumps(payload, ensure_ascii=False))
'''


def run_cycle_cost(cycles):
    source = pathlib.Path(tempfile.gettempdir()) / "robot_control_cycle_cost.c"
    binary = pathlib.Path(tempfile.gettempdir()) / "robot_control_cycle_cost"
    source.write_text(COST_SOURCE, encoding="utf-8")
    command = ["cc", "-O2", "-std=c11", "-Wall", "-Wextra", "-I", str(APP),
               str(source)]
    command.extend(str(APP / name) for name in SOURCES)
    command.extend(["-lm", "-o", str(binary)])
    subprocess.run(command, check=True, capture_output=True, text=True)
    subprocess.run([str(binary), "--cycles", str(cycles)], check=True)


def sample_resources(port, probe):
    command = ["gdb-multiarch", "-batch", "-q", str(FIRMWARE),
               "-ex", f"target remote :{port}", "-x", str(probe)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=60.0)
    for line in result.stdout.splitlines():
        if line.startswith("RESOURCE_JSON "):
            return json.loads(line[len("RESOURCE_JSON "):])
    raise RuntimeError(f"资源采样失败：{result.stdout[-300:]} {result.stderr[-300:]}")


def prepare(args, scenario, logger):
    if p.isConnected():
        p.disconnect()
    client = QemuRobotClient(byte_interval_s=args.byte_interval,
                             gdb_port=args.gdb_port, logger=logger)
    robot_id, joint_ids, tool_link, obstacle_ids, payload_id = load_scene(
        scenario["obstacles"], False)
    torque_limits = joint_torque_limits(robot_id, joint_ids)
    response = client.request(CMD_SET_OBSTACLES,
                              obstacles_payload(scenario["obstacles"]))
    if response["response_code"] != 0:
        raise RuntimeError("障碍物下发被拒绝")
    return client, (robot_id, joint_ids, tool_link, torque_limits, obstacle_ids,
                    payload_id)


def run_stability(args, probe):
    scenario = next((item for item in SCENARIOS if item["name"] == args.scenario),
                    None)
    if scenario is None:
        print(f"未知场景 {args.scenario}；可选："
              + ", ".join(item["name"] for item in SCENARIOS))
        return 1
    logger = logging.getLogger("qemu.system_performance")
    logger.setLevel(logging.DEBUG if args.log_file else logging.INFO)
    if args.log_file:
        handler = logging.FileHandler(args.log_file, mode="w", encoding="utf-8")
        handler.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)s %(name)s: %(message)s"))
        logger.addHandler(handler)

    cycles = []
    client = None
    ids = None
    try:
        client, ids = prepare(args, scenario, logger)
        first = sample_resources(args.gdb_port, probe)
        for index in range(1, args.cycles + 1):
            for attempt in range(1, ATTEMPTS + 1):
                try:
                    started = time.monotonic()
                    result = run_cycle(client, *ids, scenario, args.leg_duration)
                    break
                except RuntimeError as error:
                    print(f"循环 {index} 第 {attempt} 次中断，重启 QEMU 后重试："
                          f"{error}")
                    if attempt == ATTEMPTS:
                        raise
                    if client is not None:
                        client.close()
                    client, ids = prepare(args, scenario, logger)
            result["wall_s"] = time.monotonic() - started
            cycles.append(result)
        last = sample_resources(args.gdb_port, probe)
    finally:
        if client is not None:
            client.close()
        if p.isConnected():
            p.disconnect()

    first_cycle, final_cycle = cycles[0], cycles[-1]
    total_steps = sum(sum(leg["metrics"]["steps"] for leg in cycle["legs"])
                      for cycle in cycles)
    passed = sum(1 for cycle in cycles
                 if cycle["arrival"] <= ARRIVAL_LIMIT_M
                 and cycle["place_error"] <= PLACE_LIMIT_M
                 and cycle["collision"] == 0 and cycle["error"] == 0
                 and cycle["state"] == 2)
    stack_first = {task["task"]: task for task in first["tasks"]}
    print(f"长跑稳定性（场景 {args.scenario}，{len(cycles)} 个循环，"
          f"{total_steps} 个控制周期，总挂钟 "
          f"{sum(cycle['wall_s'] for cycle in cycles):.1f} s）")
    print(f"  到位误差：首个 {first_cycle['arrival'] * 1000:.4f} mm，末个 "
          f"{final_cycle['arrival'] * 1000:.4f} mm，全程最大 "
          f"{max(cycle['arrival'] for cycle in cycles) * 1000:.4f} mm")
    print(f"  放置精度：首个 {first_cycle['place_error'] * 1000:.4f} mm，末个 "
          f"{final_cycle['place_error'] * 1000:.4f} mm，全程最大 "
          f"{max(cycle['place_error'] for cycle in cycles) * 1000:.4f} mm")
    print(f"  状态 "
          f"{sorted({(cycle['state'], cycle['error']) for cycle in cycles})}，"
          f"碰撞合计 {sum(cycle['collision'] for cycle in cycles)}，"
          f"判据（{ARRIVAL_LIMIT_M * 1000:.0f} mm / {PLACE_LIMIT_M * 1000:.0f} mm）"
          f"通过 {passed}/{len(cycles)}")
    print(f"  堆剩余 {first['heap_free_bytes'] / 1024:.1f} KiB → "
          f"{last['heap_free_bytes'] / 1024:.1f} KiB，历史最小剩余 "
          f"{last['heap_min_ever_free_bytes'] / 1024:.1f} KiB"
          f"（总 {last['heap_total_bytes'] / 1024:.0f} KiB）")
    for task in sorted(last["tasks"], key=lambda item: item["task"]):
        print(f"  栈 {task['task']:<14}{stack_first[task['task']]['used_bytes']:>5} B"
              f"（{stack_first[task['task']]['used_percent']:>4.1f}%）→ "
              f"{task['used_bytes']:>5} B（{task['used_percent']:>4.1f}%），"
              f"配置 {task['stack_words']} words")
    return 0


def main():
    parser = argparse.ArgumentParser(description="系统级性能与资源测试")
    parser.add_argument("--scenario", default="S1")
    parser.add_argument("--cycles", type=int, default=6, help="连续作业循环数")
    parser.add_argument("--cost-cycles", type=int, default=4000)
    parser.add_argument("--gdb-port", type=int, default=3333)
    parser.add_argument("--leg-duration", type=float, default=2.0)
    parser.add_argument("--byte-interval", type=float, default=0.0002)
    parser.add_argument("--log-file", default=None)
    args = parser.parse_args()

    probe = pathlib.Path(tempfile.gettempdir()) / "robot_gdb_resource_probe.py"
    probe.write_text(GDB_SOURCE, encoding="utf-8")
    run_cycle_cost(args.cost_cycles)
    return run_stability(args, probe)


if __name__ == "__main__":
    sys.exit(main())
