"""Interactive terminal entry point for DeepSeek-planned Zekeep tasks."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math

import rclpy

from .deepseek import DeepSeekPlanner
from .planner import plan_as_dict
from .robot import RobotTools
from .web_network import llm_network


COLORS = {'red': '红', 'blue': '蓝', 'green': '绿', 'purple': '紫', 'gray': '灰', 'yellow': '黄'}
OBJECT_NAMES = {
    **{color: f'{name}色物块' for color, name in COLORS.items()},
    **{f'{color} block': f'{name}色物块' for color, name in COLORS.items()},
    'block': '物块', 'pen': '笔', 'cardboard box': '纸箱', 'paper cup': '纸杯',
    'transparent tape roll': '透明胶带卷', 'tissue box': '纸巾盒', 'tissue paper': '纸巾',
}


def _same_box(left, right):
    if left is None or right is None:
        return False
    intersection = max(0, min(left[2], right[2])-max(left[0], right[0])) * max(0, min(left[3], right[3])-max(left[1], right[1]))
    union = (left[2]-left[0])*(left[3]-left[1]) + (right[2]-right[0])*(right[3]-right[1]) - intersection
    return union > 0 and intersection/union > .8


def _describe_detections(raw):
    try:
        data = json.loads(raw)
    except (ValueError, TypeError):
        return '检测结果格式异常，无法整理物体列表。'
    if not isinstance(data, dict):
        return '检测结果格式异常，无法整理物体列表。'
    items = data.get('observations', data.get('detections'))
    if not isinstance(items, list):
        return '检测结果格式异常，无法整理物体列表。'
    if not items:
        return '当前未确认到已配置类别的物体；这不代表桌面上没有东西。'
    rows = []
    for item in items:
        if not isinstance(item, dict):
            continue
        name = item.get('class_name') or item.get('color')
        confidence = item.get('confidence')
        if (not isinstance(name, str) or not name.strip() or type(confidence) not in (int, float)
                or not math.isfinite(confidence) or not 0 <= confidence <= 1):
            continue
        box = item.get('bbox')
        if (not isinstance(box, list) or len(box) != 4
                or not all(type(v) in (int, float) and math.isfinite(v) for v in box)
                or box[2] <= box[0] or box[3] <= box[1]):
            box = None
        hits = item.get('hits', 0)
        rows.append(dict(name=OBJECT_NAMES.get(name.strip().lower(), '其他物体'), bbox=box,
                         confidence=confidence, hits=hits if type(hits) is int and hits >= 0 else 0,
                         uncertain=confidence < .5))
    objects = []
    for row in sorted(rows, key=lambda item: (item['hits'], item['confidence']), reverse=True):
        # Near-identical boxes share one display count even when class labels flicker.
        match = next((other for other in objects if _same_box(row['bbox'], other['bbox'])), None)
        if match is None:
            objects.append(row)
        elif match['name'] != row['name']:
            match['uncertain'] = True
    if not objects:
        return '没有有效识别记录，暂时无法判断画面中有什么。'
    counts = Counter(item['name'] for item in objects)
    uncertain = Counter(item['name'] for item in objects if item['uncertain'])
    window = data.get('observation_window_s')
    scope = f'最近 {window:g} 秒内' if type(window) in (int, float) and math.isfinite(window) and window > 0 else '当前画面'
    lines = [f'{scope}识别到 {len(objects)} 个物体：']
    for name, count in counts.items():
        note = ''
        if uncertain[name]:
            note = '（识别待确认）' if uncertain[name] == count else f'（其中 {uncertain[name]} 个识别待确认）'
        lines.append(f'- {name}：{count} 个{note}')
    lines.append('以上是相机识别结果，可能漏检或误识别。')
    return '\n'.join(lines)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="zekeep")
    parser.add_argument("--timeout", type=float, default=30.0, help="DeepSeek API 超时秒数")
    parser.add_argument("--dry-run", action="store_true", help="只生成计划，不调用 ROS")
    parser.add_argument("task", nargs="*", help="可选的一次性任务；省略则进入交互模式")
    return parser.parse_args()


def _run_task(planner, robot, task: str, dry_run: bool) -> bool:
    plan = planner.plan(task)
    if dry_run or not plan.steps or any(step.tool != 'detect_blocks' for step in plan.steps):
        print(json.dumps(plan_as_dict(plan), ensure_ascii=False, indent=2))
    if not plan.steps:
        print("未生成可执行动作。")
        return False
    if dry_run:
        print("dry-run：未调用 ROS。")
        return True
    if plan.requires_confirmation and input("输入 yes 执行：").strip() != "yes":
        print("已取消。")
        return False
    try:
        for step in plan.steps:
            result = robot.execute(step.tool, step.arguments)
            print(_describe_detections(result) if step.tool == 'detect_blocks' else f"[{step.tool}] {result}")
    except (Exception, KeyboardInterrupt):
        robot.stop_best_effort()
        raise
    return True


@llm_network()
def main() -> None:
    args = _arguments()
    try:
        planner = DeepSeekPlanner(timeout_s=args.timeout)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    robot = None
    if not args.dry_run:
        rclpy.init()
        robot = RobotTools(args.namespace, autostart=True)
    exit_code = 0
    try:
        if args.task:
            if not _run_task(
                planner, robot, " ".join(args.task), args.dry_run
            ):
                exit_code = 1
        else:
            print("输入任务；exit 退出。LLM 不会自动使能机械臂。")
            while args.dry_run or rclpy.ok():
                try:
                    task = input("任务> ").strip()
                except EOFError:
                    break
                if task in {"exit", "quit"}:
                    break
                if not task:
                    continue
                try:
                    _run_task(planner, robot, task, args.dry_run)
                except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
                    print(f"失败: {exc}")
                    exit_code = 1
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"失败: {exc}")
        exit_code = 1
    finally:
        if robot is not None:
            robot.destroy_node()
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
