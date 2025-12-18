---
id: 0009
title: Create ROS 2 Workspace Scanner Script
stage: green
date: 2025-12-08
surface: agent
model: gemini
feature: physical-ai-humanoid-robotics-book
branch: main
user: Hira Ali
command: python ros2_workspace_scanner.py "C:\Users\Hira Ali\OneDrive\Desktop\hack_yt\ros2_ws_local"
labels: [ros2, python, workspace-scanner]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - ros2_workspace_scanner.py
tests:
 - none
---

## Prompt

I am working on a Physical AI and Humanoid Robotics project using ROS 2, Gazebo, NVIDIA Isaac Sim, and Jetson edge devices. I suspect some critical files or folders are missing in my workspace, which may break my robot simulation or ROS 2 nodes.
I need a Python script or method that can:

Scan my ROS 2 workspace and all packages.

Identify missing essential files, including:

package.xml

launch/ folder with .launch.py files

src/ folder with .py files

urdf/ folder with .urdf files

Any Isaac Sim .usd or asset files if applicable

Produce a readable report showing which packages or folders are incomplete or missing files.
Please provide a ready-to-run solution suitable for a ROS 2 / Isaac Sim project.

## Response snapshot

I have created a Python script that scans your ROS 2 workspace for missing essential files and folders.

### How to Run the Script

1.  **Save the script:** The script `ros2_workspace_scanner.py` is already created in the root of your project.

2.  **Open a terminal:** Navigate to the root of your project directory where the script is located.

3.  **Run the script:** Execute the script from your terminal by passing the path to your ROS 2 workspace as an argument.

    For example, if your ROS 2 workspace is located at `C:\Users\Hira Ali\OneDrive\Desktop\hack_yt\ros2_ws_local`, you would run:

    ```bash
    python ros2_workspace_scanner.py "C:\Users\Hira Ali\OneDrive\Desktop\hack_yt\ros2_ws_local"
    ```

### Interpreting the Output

The script will print a report to the console, indicating which packages are `[COMPLETE]` and which are `[INCOMPLETE]`. For incomplete packages, it will list the specific files or folders that are missing.

This will help you quickly identify and fix any structural issues in your ROS 2 workspace.

## Outcome

- ✅ Impact: Provided a script to scan and validate ROS 2 workspaces.
- 🧪 Tests: No new tests were added.
- 📁 Files: Created `ros2_workspace_scanner.py`.
- 🔁 Next prompts: User can now run the script and validate their workspace.
- 🧠 Reflection: The script provides a good starting point for workspace validation.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none
