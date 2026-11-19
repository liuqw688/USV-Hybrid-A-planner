#!/usr/bin/env python3
"""停止当前用户启动的 USV_systems 节点、启动器和工程脚本。"""

import argparse
import os
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).resolve().parent
PROC = Path('/proc')


def package_names():
    """读取工作空间中的 ROS 包名。"""
    names = set()
    for manifest in (WORKSPACE / 'src').rglob('package.xml'):
        try:
            name = ET.parse(manifest).getroot().findtext('name')
            if name:
                names.add(name.strip())
        except (OSError, ET.ParseError):
            print(f'无法读取包清单：{manifest}', flush=True)
    return names


def read_process(pid):
    """读取本用户进程的命令、父进程和启动时间。"""
    directory = PROC / str(pid)
    try:
        if directory.stat().st_uid != os.getuid():
            return None
        arguments = [os.fsdecode(arg) for arg in
                     directory.joinpath('cmdline').read_bytes().split(b'\0') if arg]
        stat = directory.joinpath('stat').read_text()
        fields = stat[stat.rfind(')') + 2:].split()
        if not arguments or fields[0] == 'Z':
            return None
        return arguments, int(fields[1]), fields[19]
    except (OSError, IndexError, ValueError):
        return None


def workspace_path(argument, pid):
    """判断可执行文件或脚本路径是否位于本工作空间。"""
    path = Path(argument)
    try:
        if not path.is_absolute():
            path = (PROC / str(pid) / 'cwd').resolve(strict=True) / path
        path = Path(os.path.normpath(path))
        return path.is_relative_to(WORKSPACE)
    except OSError:
        return False


def matches(arguments, pid, packages):
    """只检查可执行文件、解释器的脚本参数及 ROS 包启动命令。"""
    for index, argument in enumerate(arguments[:2]):
        if index + 2 >= len(arguments):
            break
        if (Path(argument).name == 'ros2' and arguments[index + 1] in ('launch', 'run')
                and arguments[index + 2] in packages):
            return True
    executable = Path(arguments[0]).name
    if '/' in arguments[0] and workspace_path(arguments[0], pid):
        return True
    if executable.startswith('python') or executable in ('bash', 'sh'):
        for argument in arguments[1:]:
            if argument in ('-u', '-B', '-I', '-O', '-OO', '-s', '-S', '-E', '--'):
                continue
            if argument.startswith('-'):
                return False
            return workspace_path(argument, pid)
    return False


def still_running(pid, start_time):
    process = read_process(pid)
    return process is not None and process[2] == start_time


def wait_for_exit(targets, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not any(still_running(pid, start) for pid, start in targets.items()):
            return
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description='停止本工作空间由当前用户启动的 ROS 节点和脚本。')
    parser.add_argument('-n', '--dry-run', action='store_true', help='仅显示匹配进程')
    args = parser.parse_args()
    packages = package_names()
    processes = {}
    for directory in PROC.iterdir():
        if directory.name.isdigit():
            pid = int(directory.name)
            process = read_process(pid)
            if process:
                processes[pid] = process

    # 排除本脚本及其祖先进程，避免终止执行命令的终端或 shell。
    protected = set()
    pid = os.getpid()
    while pid and pid not in protected:
        protected.add(pid)
        pid = processes[pid][1] if pid in processes else 0

    selected = {pid for pid, (argv, _, _) in processes.items()
                if pid not in protected and matches(argv, pid, packages)}
    # launch 的 RViz 等子进程可能不位于本工作空间，需要一并停止。
    while True:
        children = {pid for pid, (_, parent, _) in processes.items()
                    if parent in selected and pid not in selected and pid not in protected}
        if not children:
            break
        selected.update(children)
    targets = {pid: processes[pid][2] for pid in selected}
    if not targets:
        print('未发现本工作空间的后台进程。')
        return 0
    for pid in sorted(targets):
        print(f"  PID {pid}: {' '.join(processes[pid][0])}", flush=True)
    if args.dry_run:
        print('预览完成，未停止进程。')
        return 0

    for sig, seconds in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        remaining = [pid for pid, start in targets.items() if still_running(pid, start)]
        if not remaining:
            break
        print(f'发送 {sig.name}: {sorted(remaining)}', flush=True)
        for pid in remaining:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                print(f'无法向 PID {pid} 发送信号：权限不足。')
        wait_for_exit(targets, seconds)
    remaining = [pid for pid, start in targets.items() if still_running(pid, start)]
    if remaining:
        print(f'以下进程未退出：{sorted(remaining)}')
        return 1
    print('USV_systems 后台进程已停止。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
