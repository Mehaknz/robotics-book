import os
import argparse

def find_ros2_packages(workspace_path):
    """Finds all ROS 2 packages in a given workspace."""
    packages = []
    for root, dirs, files in os.walk(workspace_path):
        if "package.xml" in files:
            packages.append(root)
    return packages

def check_package_integrity(package_path):
    """Checks the integrity of a single ROS 2 package."""
    missing_items = []
    
    # Check for package.xml
    if not os.path.exists(os.path.join(package_path, "package.xml")):
        missing_items.append("package.xml")

    # Check for launch directory and .launch.py files
    launch_dir = os.path.join(package_path, "launch")
    if not os.path.isdir(launch_dir):
        missing_items.append("launch/ directory")
    else:
        if not any(f.endswith(".launch.py") for f in os.listdir(launch_dir)):
            missing_items.append("No .launch.py files in launch/")

    # Check for src directory and .py files
    src_dir = os.path.join(package_path, "src")
    if not os.path.isdir(src_dir):
        missing_items.append("src/ directory")
    else:
        if not any(f.endswith(".py") for f in os.listdir(src_dir)):
            missing_items.append("No .py files in src/")

    # Check for urdf directory and .urdf files
    urdf_dir = os.path.join(package_path, "urdf")
    if os.path.isdir(urdf_dir):
        if not any(f.endswith(".urdf") for f in os.listdir(urdf_dir)):
            missing_items.append("No .urdf files in urdf/")

    # Check for Isaac Sim assets (.usd files)
    assets_dir = os.path.join(package_path, "assets")
    if os.path.isdir(assets_dir):
        if not any(f.endswith(".usd") for f in os.listdir(assets_dir)):
            missing_items.append("No .usd files in assets/")

    return missing_items

def main():
    parser = argparse.ArgumentParser(description="Scan a ROS 2 workspace for missing files.")
    parser.add_argument("workspace_path", type=str, help="The path to the ROS 2 workspace.")
    args = parser.parse_args()

    workspace_path = args.workspace_path

    if not os.path.isdir(workspace_path):
        print(f"Error: Workspace path '{workspace_path}' not found.")
        return

    print(f"Scanning ROS 2 workspace at: {workspace_path}")
    
    ros2_packages = find_ros2_packages(workspace_path)

    if not ros2_packages:
        print("No ROS 2 packages found in the workspace.")
        return

    print(f"Found {len(ros2_packages)} ROS 2 packages.")
    print("-" * 30)

    for pkg_path in ros2_packages:
        pkg_name = os.path.basename(pkg_path)
        print(f"Checking package: {pkg_name}")
        
        missing = check_package_integrity(pkg_path)
        
        if missing:
            print(f"  [INCOMPLETE] Package '{pkg_name}' is missing the following:")
            for item in missing:
                print(f"    - {item}")
        else:
            print(f"  [COMPLETE] Package '{pkg_name}' seems to be structured correctly.")
        print("-" * 30)

if __name__ == "__main__":
    main()
