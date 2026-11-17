#!/usr/bin/env python3

import sys
import os

JOINT_SYMMETRY_TYPE = {
    '001': 'MIRROR',
    '002': 'MIRROR',
    '003': 'STRONG_MIRROR',
    '004': 'DIRECT',
    '005': 'MIRROR',
    '006': 'MIRROR',
    '007': 'MIRROR',
    '008': 'STRONG_MIRROR'
}

BASE_VALUE = 0x10000

def mirror_position(pos_hex):
    value = int(pos_hex, 16)
    mirrored = BASE_VALUE - value
    return format(mirrored & 0xFFFF, '04X')

def process_line(line):
    line = line.strip()
    if not line:
        return None
    
    if '#' not in line:
        return None
    
    joint_part, hex_value = line.split('#', 1)
    joint_id = joint_part.strip().zfill(3)
    
    if len(hex_value) != 16:
        return None
    
    if hex_value == 'FFFFFFFFFFFFFFFC' or hex_value == 'FFFFFFFFFFFFFFFD':
        return f"{joint_id}#{hex_value}"
    
    pos_part = hex_value[:4]
    rest_part = hex_value[4:]
    
    symmetry_type = JOINT_SYMMETRY_TYPE.get(joint_id, 'DIRECT')
    
    if symmetry_type in ('STRONG_MIRROR', 'MIRROR'):
        new_pos = mirror_position(pos_part)
    else:
        new_pos = pos_part
    
    new_hex_value = new_pos + rest_part
    
    return f"{joint_id}#{new_hex_value}"

def process_file(input_path, output_path):
    if not os.path.exists(input_path):
        print(f"错误：输入文件不存在: {input_path}")
        return False
    
    with open(input_path, 'r') as f:
        lines = f.readlines()
    
    mirrored_lines = []
    for line in lines:
        result = process_line(line)
        if result:
            mirrored_lines.append(result)
    
    with open(output_path, 'w') as f:
        for line in mirrored_lines:
            f.write(line + '\n')
    
    frame_count = len(mirrored_lines) // 8
    print(f"镜像文件已生成: {output_path}")
    print(f"处理行数: {len(mirrored_lines)}")
    print(f"处理帧数: {frame_count}")
    return True

def compare_files(file1, file2):
    if not os.path.exists(file1) or not os.path.exists(file2):
        print("错误：文件不存在")
        return
    
    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        lines1 = [l.strip() for l in f1.readlines() if l.strip()]
        lines2 = [l.strip() for l in f2.readlines() if l.strip()]
    
    min_len = min(len(lines1), len(lines2))
    matches = 0
    mismatches = 0
    max_diff = 0
    max_diff_info = None
    
    for i in range(min_len):
        line1 = lines1[i]
        line2 = lines2[i]
        
        if line1 == line2:
            matches += 1
        else:
            mismatches += 1
            if '#' in line1 and '#' in line2:
                joint1, hex1 = line1.split('#', 1)
                joint2, hex2 = line2.split('#', 1)
                
                if joint1 == joint2:
                    val1 = int(hex1[:4], 16)
                    val2 = int(hex2[:4], 16)
                    diff = abs(val1 - val2)
                    if diff > max_diff:
                        max_diff = diff
                        max_diff_info = f"行{i+1}, 关节{joint1}: {hex1[:4]} vs {hex2[:4]}, 差值: {diff}"
    
    total = matches + mismatches
    accuracy = (matches / total) * 100 if total > 0 else 0
    
    print(f"\n=== 对比结果 ===")
    print(f"匹配行数: {matches}")
    print(f"不匹配行数: {mismatches}")
    print(f"准确率: {accuracy:.2f}%")
    if max_diff_info:
        print(f"最大差异: {max_diff_info}")

def main():
    if len(sys.argv) < 3:
        print("用法: python mirror_arm.py <输入文件> <输出文件> [--compare <对比文件>]")
        print("示例: python mirror_arm.py Rax1.log Lax1_mirrored.log")
        print("示例: python mirror_arm.py Rax1.log Lax1_mirrored.log --compare Lax1.log")
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_path = sys.argv[2]
    
    success = process_file(input_path, output_path)
    if not success:
        sys.exit(1)
    
    if len(sys.argv) >= 5 and sys.argv[3] == '--compare':
        compare_file = sys.argv[4]
        compare_files(output_path, compare_file)

if __name__ == '__main__':
    main()