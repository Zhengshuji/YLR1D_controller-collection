#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import sys

def extract_chinese_quoted_strings(input_file, output_file):
    """
    从 STEP 文件中提取所有含有中文字符的 '...' 字符串，
    自动探测编码（优先 GB2312/GBK），去重后按行输出。
    """
    pattern = re.compile(r"'([^']*)'")
    result_set = set()

    # 尝试多种编码
    encodings = ['gb2312','utf-8','gbk']
    content = None
    for enc in encodings:
        try:
            with open(input_file, 'r', encoding=enc) as f:
                content = f.read()
            break
        except (UnicodeDecodeError, LookupError):
            continue
    if content is None:
        print(f"错误：无法用常见编码读取文件 '{input_file}'，请检查文件编码。")
        sys.exit(1)

    # 按行处理（避免一次性大文件内存问题，但已读取全部，也可逐行）
    for line in content.splitlines():
        for match in pattern.finditer(line):
            quoted = match.group(0)      # 完整的带引号字符串，如 '...'
            inner = match.group(1)       # 引号内部内容
            # 检查内部是否包含中文字符（基本 CJK 统一表意汉字）
            if any('\u4e00' <= ch <= '\u9fff' for ch in inner):
                result_set.add(quoted)   # 保留完整引号

    # 排序后写入输出文件
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            for item in sorted(result_set):
                f.write(item + '\n')
        print(f"提取完成，共 {len(result_set)} 个不重复名称，已保存至 '{output_file}'")
    except Exception as e:
        print(f"写入文件时发生错误：{e}")
        sys.exit(1)

import os
# 获取当前脚本所在目录
current_path = os.path.dirname(os.path.abspath(__file__))
# 设置工作目录为当前文件夹
os.chdir(current_path)

if __name__ == '__main__':
    if len(sys.argv) == 1:
        #src_path = "GB2312中文.txt"
        #obj_path = "result.txt"
        src_path = "../Models/origin/test.STEP"
        obj_path = "result.txt"
        extract_chinese_quoted_strings(src_path, obj_path)
    elif len(sys.argv) != 3:
        print("用法：python extract_chinese_names.py <输入STEP文件> <输出txt文件>")
        print("示例：python extract_chinese_names.py model/input/test.STEP result.txt")
        sys.exit(1)
    else:
        extract_chinese_quoted_strings(sys.argv[1], sys.argv[2])