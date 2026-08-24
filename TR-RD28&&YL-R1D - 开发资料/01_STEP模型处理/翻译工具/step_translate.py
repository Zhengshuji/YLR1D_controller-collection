#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import sys
import csv

def read_translations(csv_file):
    """
    读取CSV翻译文件（GB2312编码），格式：序号,原名称,英文翻译
    返回字典 {原名称: 英文翻译}
    """
    trans = {}
    with open(csv_file, 'r', encoding='gb2312') as f:
        reader = csv.reader(f)
        # 尝试跳过表头（如果第一行包含“序号”等字眼）
        first_row = next(reader, None)
        if first_row:
            # 如果第一行不是“序号”开头，则视为数据行，重新处理该行
            if not (first_row[0].strip() == '序号' or '原名称' in first_row[0]):
                # 如果第一行是数据，则处理它
                if len(first_row) >= 3:
                    chinese = first_row[1].strip()
                    english = first_row[2].strip()
                    if chinese and english:
                        trans[chinese] = english
        # 读取剩余行
        for row in reader:
            if len(row) >= 3:
                chinese = row[1].strip()
                english = row[2].strip()
                if chinese and english:
                    trans[chinese] = english
    return trans

def replace_chinese_in_quotes(text, trans_dict):
    """
    只替换单引号内的内容，要求整个字符串完全匹配字典的键
    """
    def replace_match(m):
        inner = m.group(1)          # 引号内的内容
        if inner in trans_dict:     # 完全匹配
            return f"'{trans_dict[inner]}'"
        else:
            return m.group(0)       # 保持原样

    pattern = re.compile(r"'([^']*)'")
    return pattern.sub(replace_match, text)

import os
# 获取当前脚本所在目录
current_path = os.path.dirname(os.path.abspath(__file__))
# 设置工作目录为当前文件夹
os.chdir(current_path)

def main():
    if len(sys.argv) == 1:
        input_file = "../Models/origin/test.STEP"
        output_file = "../Models/translated/test_tr.STEP"
        csv_file = "translate.csv"
    elif len(sys.argv) < 4:
        print("用法: python step_translate.py <输入STEP文件> <输出STEP文件> <translate.csv>")
        sys.exit(1)
    else:
        input_file = sys.argv[1]
        output_file = sys.argv[2]
        csv_file = sys.argv[3]

    # 读取翻译表
    trans_dict = read_translations(csv_file)

    # 读取STEP文件，自动探测编码（UTF-8优先，失败则GB2312）
    try:
        with open(input_file, 'r', encoding='gb2312') as f:
            content = f.read()
    except UnicodeDecodeError:
        with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()

    # 执行替换
    new_content = replace_chinese_in_quotes(content, trans_dict)

    # 写入输出文件（统一使用UTF-8）
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print("处理完成！")

if __name__ == "__main__":
    main()