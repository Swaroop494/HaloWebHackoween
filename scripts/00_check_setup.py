#!/usr/bin/env python3
"""Check if PyBullet simulation environment is set up correctly."""
import sys
import os

def check_setup():
    """Verify all required components are available."""
    print("Checking PyBullet simulation setup...\n")
    
    errors = []
    warnings = []
    
    # Check Python version
    print(f"✓ Python version: {sys.version.split()[0]}")
    
    # Check required packages
    required_packages = {
        'pybullet': 'PyBullet',
        'pybullet_data': 'PyBullet Data',
        'numpy': 'NumPy',
        'gymnasium': 'Gymnasium',
        'stable_baselines3': 'Stable-Baselines3',
    }
    
    print("\nChecking required packages:")
    for module, name in required_packages.items():
        try:
            mod = __import__(module)
            version = getattr(mod, '__version__', 'unknown')
            print(f"  ✓ {name}: {version}")
        except ImportError:
            print(f"  ✗ {name}: NOT INSTALLED")
            errors.append(f"{name} is not installed. Run: pip install {module}")
    
    # Check URDF file
    print("\nChecking project files:")
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_path = os.path.join(project_root, "urdf", "diff_drive_robot.urdf")
    
    if os.path.exists(urdf_path):
        print(f"  ✓ Robot URDF found: {urdf_path}")
    else:
        print(f"  ✗ Robot URDF missing: {urdf_path}")
        errors.append("Robot URDF file not found")
    
    # Check robot class
    try:
        from scripts.diff_drive_robot import DiffDriveRobot
        print("  ✓ DiffDriveRobot class importable")
    except ImportError as e:
        print(f"  ✗ DiffDriveRobot class: {e}")
        errors.append("Cannot import DiffDriveRobot class")
    
    # Check PyBullet data path
    try:
        import pybullet_data
        data_path = pybullet_data.getDataPath()
        print(f"  ✓ PyBullet data path: {data_path}")
    except Exception as e:
        warnings.append(f"PyBullet data path check failed: {e}")
    
    # Summary
    print("\n" + "="*50)
    if errors:
        print("❌ SETUP INCOMPLETE")
        print("\nErrors found:")
        for error in errors:
            print(f"  - {error}")
        print("\nTo fix, run:")
        print("  pip install -r requirements.txt")
        return False
    elif warnings:
        print("⚠️  SETUP COMPLETE (with warnings)")
        for warning in warnings:
            print(f"  - {warning}")
        return True
    else:
        print("✅ SETUP COMPLETE")
        print("\nYou can now run:")
        print("  python scripts/01_test_pybullet.py")
        print("  python scripts/02_simple_navigation.py")
        return True

if __name__ == "__main__":
    success = check_setup()
    sys.exit(0 if success else 1)

