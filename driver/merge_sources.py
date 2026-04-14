import os
from pathlib import Path

# ===== 配置区 =====
SCRIPT_DIR = Path(__file__).resolve().parent          # 当前脚本所在目录
ROOT_DIR = SCRIPT_DIR                                 # 扫描根目录
OUTPUT_FILE = SCRIPT_DIR.parent / f"{SCRIPT_DIR.name}_merged_sources.txt"
#ROOT_DIR = r"./."          # 要扫描的根目录（Windows 路径）
#OUTPUT_FILE = r"./.merged_sources.txt"  # 输出文件
EXTENSIONS = {".cpp", ".h"}              # 需要合并的后缀
# ==================

def is_target_file(path: Path) -> bool:
    return path.suffix.lower() in EXTENSIONS

def main():
    root = Path(ROOT_DIR)
    files = sorted(p for p in root.rglob("*") if p.is_file() and is_target_file(p))

    print(f"Found {len(files)} source files")

    with open(OUTPUT_FILE, "w", encoding="utf-8") as out:
        for file in files:
            rel_path = file.relative_to(root)

            out.write("\n")
            out.write("=" * 80 + "\n")
            out.write(f"FILE: {rel_path}\n")
            out.write("=" * 80 + "\n\n")

            try:
                with open(file, "r", encoding="utf-8", errors="ignore") as f:
                    out.write(f.read())
            except Exception as e:
                out.write(f"\n[ERROR reading file: {e}]\n")

    print(f"All files merged into: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
