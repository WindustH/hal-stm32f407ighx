#!/usr/bin/env python3
import os
import re
import subprocess  # <- 新增
import sys  # <- 新增


def update_cmake(cmake_path, target_state):
    """
    (Setter) 更新 CMakeLists.txt 文件。
    它会移除所有已知的 BOARD_... 定义，然后添加指定的 target_state。
    """
    try:
        with open(cmake_path, "r", encoding="utf-8") as f:
            content = f.read()

        # 1. 移除任何已存在的 BOARD_GIMBAL 或 BOARD_CHASSIS 定义
        content = re.sub(r"\s+BOARD_GIMBAL\s*?\n", "\n", content, flags=re.MULTILINE)
        content = re.sub(r"\s+BOARD_CHASSIS\s*?\n", "\n", content, flags=re.MULTILINE)

        # 2. 准备新的定义行
        new_def_line = f"    BOARD_{target_state}\n"

        # 3. 找到 target_compile_definitions(...) 块，并在 ) 之前插入新定义
        pattern = re.compile(
            r"(target_compile_definitions\([\s\S]*?PRIVATE[\s\S]*?)(\s*\))",
            re.MULTILINE,
        )
        new_content = pattern.sub(
            r"\1\n" + new_def_line.strip() + r"\2", content, count=1
        )

        if new_content == content:
            pattern_fallback = re.compile(
                r"(target_compile_definitions\([\s\S]*?)(\s*\))", re.MULTILINE
            )
            new_content = pattern_fallback.sub(
                r"\1\n" + new_def_line.strip() + r"\2", content, count=1
            )

        if new_content == content:  # 如果两种情况都失败
            print(
                f"Error: Could not find 'target_compile_definitions' block in {cmake_path}."
            )
            return False  # <- 失败

        # 4. 清理可能因重复删除而产生的多余空行
        new_content = re.sub(
            r"\n\s*\n(\s*\))", r"\n\1", new_content, flags=re.MULTILINE
        )

        with open(cmake_path, "w", encoding="utf-8") as f:
            f.write(new_content)
        return True  # <- 成功

    except Exception as e:
        print(f"Error updating {cmake_path}: {e}")
        return False


def update_clangd(clangd_path, target_state):
    """
    (Setter) 更新 .clangd 文件。
    移除所有已知的 -DBOARD_... 定义，然后添加指定的 target_state。
    """
    try:
        with open(clangd_path, "r", encoding="utf-8") as f:
            content = f.read()

        # 1. 移除任何已存在的 -DBOARD_... 定义
        content = re.sub(
            r"^\s*-\s*DBOARD_GIMBAL,?\s*?\n", "", content, flags=re.MULTILINE
        )
        content = re.sub(
            r"^\s*-\s*DBOARD_CHASSIS,?\s*?\n", "", content, flags=re.MULTILINE
        )

        # 2. 准备新的定义行
        new_def_line = f"      -DBOARD_{target_state},\n"

        # 3. 找到 'CompileFlags: Add: [' 块并插入新定义
        pattern_multiline = re.compile(
            r"(CompileFlags:\s*Add:\s*\[\s*\n)", re.MULTILINE
        )
        new_content = pattern_multiline.sub(r"\1" + new_def_line, content, count=1)

        if new_content == content:  # 匹配失败
            pattern_singleline = re.compile(
                r"(CompileFlags:\s*Add:\s*\[)", re.MULTILINE
            )
            new_content = pattern_singleline.sub(
                r"\1\n" + new_def_line, content, count=1
            )

        if new_content == content:  # 两种都失败
            print(f"Error: Could not find 'CompileFlags: Add:' block in {clangd_path}.")
            return False  # <- 失败

        with open(clangd_path, "w", encoding="utf-8") as f:
            f.write(new_content)
        return True  # <- 成功

    except Exception as e:
        print(f"Error updating {clangd_path}: {e}")
        return False


def run_cmake_configure(build_dir, source_dir="."):
    """
    (新增) 运行 CMake 配置命令来重新生成 compile_commands.json
    """
    # 确保构建目录存在
    if not os.path.isdir(build_dir):
        print(f"Build directory '{build_dir}' not found. Creating it...")
        try:
            os.makedirs(build_dir)
        except OSError as e:
            print(f"Error creating directory {build_dir}: {e}")
            return False

    # 假设你正在使用 Ninja (现代 CMake 的标准)
    # 如果你使用 Make, 把 "-G", "Ninja" 删掉
    cmake_command = [
        "cmake",
        "-S",
        source_dir,
        "-B",
        build_dir,
        "-G",
        "Ninja",  # 如果你不用 Ninja，请删除此行
    ]

    print(f"--- Running CMake Configure ---")
    print(f"CMD: {' '.join(cmake_command)}")

    try:
        # 运行命令
        result = subprocess.run(
            cmake_command, capture_output=True, text=True, check=True
        )
        print(result.stdout)
        print(f"--- CMake Configure Finished ---")
        return True
    except subprocess.CalledProcessError as e:
        # 如果 CMake 失败 (例如 CMakeLists.txt 中有语法错误)
        print(f"Error: CMake configuration failed.")
        print(e.stderr)
        return False
    except FileNotFoundError:
        print(f"Error: 'cmake' command not found. Is it installed and in your PATH?")
        return False


def switch_board_definition(
    cmake_path="CMakeLists.txt", clangd_path=".clangd", cmake_build_dir="build"
):
    """
    主函数：在 CMakeLists.txt 和 .clangd 中切换编译宏。
    然后自动运行 cmake configure 来刷新 clangd。
    """
    # 检查所有需要的文件是否存在
    if not os.path.isfile(cmake_path):
        print(f"Error: {cmake_path} not found.")
        return
    if not os.path.isfile(clangd_path):
        print(f"Error: {clangd_path} not found.")
        return

    # 1. 读取 CMakeLists.txt 来决定当前状态
    try:
        with open(cmake_path, "r", encoding="utf-8") as f:
            cmake_content = f.read()
    except Exception as e:
        print(f"Error reading {cmake_path}: {e}")
        return

    # 2. 在 target_compile_definitions 块中查找状态
    definitions_block = ""
    match = re.search(
        r"target_compile_definitions\([\s\S]*?PRIVATE([\s\S]*?)\)",
        cmake_content,
        re.MULTILINE,
    )
    if match:
        definitions_block = match.group(1)
    else:
        print(
            f"Warning: Could not find 'target_compile_definitions(...PRIVATE...)' block in {cmake_path}."
        )

    is_gimbal_active = "BOARD_GIMBAL" in definitions_block
    is_chassis_active = "BOARD_CHASSIS" in definitions_block

    target_state = None

    # 3. 决定目标状态
    if is_gimbal_active and not is_chassis_active:
        target_state = "CHASSIS"
        print("Current state: BOARD_GIMBAL. Switching to BOARD_CHASSIS.")
    elif is_chassis_active and not is_gimbal_active:
        target_state = "GIMBAL"
        print("Current state: BOARD_CHASSIS. Switching to BOARD_GIMBAL.")
    elif not is_gimbal_active and not is_chassis_active:
        target_state = "GIMBAL"
        print("Current state: None. Defaulting to BOARD_GIMBAL.")
    else:
        print("Error: Both BOARD_GIMBAL and BOARD_CHASSIS are active. Cleaning up...")
        target_state = "GIMBAL"

    # 4. 调用更新函数
    cmake_ok = update_cmake(cmake_path, target_state)
    clangd_ok = update_clangd(clangd_path, target_state)

    if not (cmake_ok and clangd_ok):
        print("Failed to update config files. Aborting CMake run.")
        return

    print(
        f"Successfully set state to BOARD_{target_state} in {cmake_path} and {clangd_path}."
    )

    # 5. (新增) 自动运行 CMake
    if not run_cmake_configure(cmake_build_dir):
        print("Script finished, but CMake configuration failed.")
        print("Clangd highlighting might be incorrect.")
    else:
        print("Script finished. Clangd should update automatically.")


if __name__ == "__main__":
    # --------------------------------------------------------------------
    # (重要)
    # !!! 确保 'cmake_build_dir' 指向你正确的 CMake 构建目录 !!!
    # --------------------------------------------------------------------
    switch_board_definition(
        cmake_path="CMakeLists.txt",
        clangd_path=".clangd",
        cmake_build_dir="build",  # <--- 修改这里，例如 "build", "build-debug"
    )
