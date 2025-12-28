#!/usr/bin/env python3
"""Diagnose PyBullet installation issues."""
import sys
import platform
import subprocess

def run_command(cmd):
    """Run a command and return output."""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip(), result.stderr.strip(), result.returncode
    except Exception as e:
        return "", str(e), 1

print("=" * 60)
print("PyBullet Installation Diagnostics")
print("=" * 60)
print()

# Python version
print(f"Python Version: {sys.version}")
print(f"Python Executable: {sys.executable}")
print()

# Platform info
print(f"Platform: {platform.platform()}")
print(f"Architecture: {platform.machine()}")
print(f"Processor: {platform.processor()}")
print()

# Check if we're in venv
in_venv = hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)
print(f"In Virtual Environment: {in_venv}")
if in_venv:
    print(f"Venv Path: {sys.prefix}")
print()

# Pip version
stdout, stderr, code = run_command(f"{sys.executable} -m pip --version")
if code == 0:
    print(f"Pip Version: {stdout}")
else:
    print(f"Pip Error: {stderr}")
print()

# Check available PyBullet versions
print("Checking available PyBullet versions...")
stdout, stderr, code = run_command(f"{sys.executable} -m pip search pybullet 2>&1 || {sys.executable} -m pip index versions pybullet 2>&1 || echo 'Search not available'")
if "pybullet" in stdout.lower() or "pybullet" in stderr.lower():
    print(stdout[:500] if stdout else stderr[:500])
else:
    print("Cannot query PyPI directly. Trying alternative method...")
print()

# Try to import pybullet
print("Checking if PyBullet is already installed...")
try:
    import pybullet
    print(f"✓ PyBullet is installed: {pybullet.__version__}")
    import pybullet_data
    print(f"✓ PyBullet data path: {pybullet_data.getDataPath()}")
except ImportError:
    print("✗ PyBullet is not installed")
print()

# Recommendations
print("=" * 60)
print("RECOMMENDATIONS:")
print("=" * 60)

arch = platform.machine()
python_version = f"{sys.version_info.major}.{sys.version_info.minor}"

if arch == "arm64" or "arm" in arch.lower():
    print("⚠️  You're on ARM architecture (Apple Silicon)")
    print("   PyBullet should work, but try:")
    print("   1. pip install --upgrade pip")
    print("   2. pip install pybullet --no-cache-dir")
    print("   3. If that fails, use conda: conda install -c conda-forge pybullet")
elif python_version >= "3.12":
    print(f"⚠️  Python {python_version} detected")
    print("   PyBullet may not have pre-built wheels for Python 3.12+")
    print("   Try:")
    print("   1. Use Python 3.11 or earlier")
    print("   2. Or use conda: conda install -c conda-forge pybullet")
else:
    print(f"✓ Python {python_version} should be compatible")
    print("   Try these in order:")
    print("   1. pip install --upgrade pip setuptools wheel")
    print("   2. pip install pybullet --no-cache-dir")
    print("   3. pip install pybullet --no-build-isolation")

print()
print("If all else fails, use conda:")
print("   conda create -n pybullet_env python=3.11")
print("   conda activate pybullet_env")
print("   conda install -c conda-forge pybullet")

