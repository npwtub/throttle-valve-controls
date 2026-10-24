"""Find the Windows host compiler when MSYS2 is not on PlatformIO's PATH."""

import os
import shutil
from pathlib import Path

Import("env")  # Provided by PlatformIO/SCons.

if os.name == "nt" and not shutil.which("g++", path=env["ENV"].get("PATH")):
    # Prefer an explicit installation override, then standard MSYS2 locations.
    msys_root = Path(os.environ.get("MSYS2_ROOT", "C:/msys64"))
    for variant in ("ucrt64", "mingw64"):
        compiler_dir = msys_root / variant / "bin"
        if (compiler_dir / "g++.exe").is_file():
            env.PrependENVPath("PATH", str(compiler_dir))
            break
