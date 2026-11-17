#!/usr/bin/env python3

import os
import sys

RIGHT_ZERO = [
    "001#7FFE7FE7AD9997FF",
    "002#7FF57FE7AD9997FF",
    "003#80028007AD9997FF",
    "004#80027FE7AD9997FF",
    "005#80077FE0C40A37FF",
    "006#7FEA7FD0FD0A37FF",
    "007#80037FE0CC0A37FF",
    "008#7FFF7FD0830A37FF",
]

LEFT_ZERO = [
    "001#7FFF7FE7AD9997FF",
    "002#7FED7FD7AD9997FF",
    "003#7FFB7FD7AD9997FF",
    "004#800F7FE7AD9997FF",
    "005#7FFF7FE0C40A37FF",
    "006#7FF37FE0FD0A37FF",
    "007#80137FD0CC0A37FF",
    "008#7FF77FE0830A37FF",
]

def hex_to_dec(s):
    return int(s, 16)

def dec_to_hex(n):
    return f"{n:04X}"

def get_position(line):
    data = line.split('#')[1]
    return data[:4]

def get_rest(line):
    data = line.split('#')[1]
    return data[4:]

def get_joint_id(line):
    return line.split('#')[0]

def process_file(input_file, output_file):
    filename = os.path.basename(input_file)
    zero_arr = LEFT_ZERO if filename.startswith('L') else RIGHT_ZERO
    
    with open(input_file, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]
    
    total_lines = len(lines)
    
    enable_frames = lines[:8]
    body_frames = lines[8:-8]
    disable_frames = lines[-8:]
    
    first_group = body_frames[:8]
    last_group = body_frames[-8:]
    
    result = []
    
    result.extend(enable_frames)
    result.extend(zero_arr)
    
    interpolate_frames_start = []
    max_frames_start = 0
    
    for i in range(8):
        zero_line = zero_arr[i]
        action_line = first_group[i]
        
        zero_pos = hex_to_dec(get_position(zero_line))
        action_pos = hex_to_dec(get_position(action_line))
        rest = get_rest(action_line)
        joint_id = get_joint_id(zero_line)
        
        diff = abs(action_pos - zero_pos)
        max_frames_start = max(max_frames_start, diff)
        interpolate_frames_start.append({
            'start': zero_pos,
            'end': action_pos,
            'rest': rest,
            'joint_id': joint_id,
            'step': 1 if action_pos > zero_pos else -1
        })
    
    for frame_idx in range(max_frames_start):
        if frame_idx % 10 != 0:
            continue
        
        all_reached = True
        for j in range(8):
            info = interpolate_frames_start[j]
            current = info['start'] + info['step'] * (frame_idx + 1)
            
            if not ((info['step'] > 0 and current >= info['end']) or \
                    (info['step'] < 0 and current <= info['end'])):
                all_reached = False
                break
        
        if all_reached:
            break
        
        for j in range(8):
            info = interpolate_frames_start[j]
            current = info['start'] + info['step'] * (frame_idx + 1)
            
            if info['step'] > 0 and current >= info['end']:
                current = info['end']
            elif info['step'] < 0 and current <= info['end']:
                current = info['end']
            
            pos_hex = dec_to_hex(current)
            result.append(f"{info['joint_id']}#{pos_hex}{info['rest']}")
    
    result.extend(body_frames)
    
    interpolate_frames_end = []
    max_frames_end = 0
    
    for i in range(8):
        action_line = last_group[i]
        zero_line = zero_arr[i]
        
        action_pos = hex_to_dec(get_position(action_line))
        zero_pos = hex_to_dec(get_position(zero_line))
        rest = get_rest(action_line)
        joint_id = get_joint_id(zero_line)
        
        diff = abs(zero_pos - action_pos)
        max_frames_end = max(max_frames_end, diff)
        interpolate_frames_end.append({
            'start': action_pos,
            'end': zero_pos,
            'rest': rest,
            'joint_id': joint_id,
            'step': 1 if zero_pos > action_pos else -1
        })
    
    for frame_idx in range(max_frames_end):
        if frame_idx % 3 != 0:
            continue
        
        all_reached = True
        for j in range(8):
            info = interpolate_frames_end[j]
            current = info['start'] + info['step'] * (frame_idx + 1)
            
            if not ((info['step'] > 0 and current >= info['end']) or \
                    (info['step'] < 0 and current <= info['end'])):
                all_reached = False
                break
        
        if all_reached:
            break
        
        for j in range(8):
            info = interpolate_frames_end[j]
            current = info['start'] + info['step'] * (frame_idx + 1)
            
            if info['step'] > 0 and current >= info['end']:
                current = info['end']
            elif info['step'] < 0 and current <= info['end']:
                current = info['end']
            
            pos_hex = dec_to_hex(current)
            result.append(f"{info['joint_id']}#{pos_hex}{info['rest']}")
    
    result.extend(zero_arr)
    result.extend(disable_frames)
    
    with open(output_file, 'w') as f:
        for line in result:
            f.write(line + '\n')

DIR = "/home/vlai/x_air/publish/modules/src/xarm_action/1"

def get_output_path(input_file):
    base, ext = os.path.splitext(input_file)
    return base + "_smoothed" + ext

def list_log_files(directory):
    files = []
    for f in sorted(os.listdir(directory)):
        if f.endswith('.log') and not f.endswith('_smoothed.log'):
            full_path = os.path.join(directory, f)
            if os.path.isfile(full_path):
                files.append(full_path)
    return files

def process_target(target):
    target = os.path.expanduser(target)
    if os.path.isfile(target):
        print(f"\nProcessing: {target}")
        output_file = get_output_path(target)
        process_file(target, output_file)
        print(f"Output: {output_file}")
    elif os.path.isdir(target):
        log_files = list_log_files(target)
        if not log_files:
            print(f"No .log files found in {target}")
            return
        print(f"\nFound {len(log_files)} .log file(s) in {target}:")
        for i, f in enumerate(log_files, 1):
            print(f"  [{i}] {f}")
        print(f"  [{len(log_files)+1}] Process ALL files")
        
        choice = input(f"\nSelect file to process [1-{len(log_files)+1}]: ").strip()
        
        if choice == str(len(log_files) + 1):
            for f in log_files:
                print(f"\nProcessing: {f}")
                output_file = get_output_path(f)
                process_file(f, output_file)
                print(f"Output: {output_file}")
        else:
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(log_files):
                    f = log_files[idx]
                    print(f"\nProcessing: {f}")
                    output_file = get_output_path(f)
                    process_file(f, output_file)
                    print(f"Output: {output_file}")
                else:
                    print("Invalid selection")
            except ValueError:
                print("Invalid input")
    else:
        print(f"Error: '{target}' is not a valid file or directory")

print("=" * 50)
print("  Smooth Action - 机械臂动作平滑处理工具")
print("=" * 50)

while True:
    print(f"\n默认目录: {DIR}")
    path = input("输入要处理的文件或目录路径 (q 退出): ").strip()
    
    if path.lower() == 'q':
        print("退出")
        break
    
    if not path:
        path = DIR
    
    process_target(path)
    print("\n" + "-" * 50)
