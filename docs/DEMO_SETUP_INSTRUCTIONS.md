# Final Demo Setup & Execution Instructions

This guide provides the exact steps to get the "Physical AI & Humanoid Robotics" demo running on your system.

---

### **Part 1: Prerequisites & System Setup**

1.  **Hardware:**
    *   A desktop PC with an **NVIDIA RTX 4090** GPU.
    *   A microphone for voice commands.

2.  **Operating System:**
    *   **Ubuntu 22.04**.

3.  **NVIDIA Isaac Sim:**
    *   Install **NVIDIA Isaac Sim 2024** via the NVIDIA Omniverse Launcher.
    *   Ensure it runs correctly on its own before proceeding.

4.  **ROS 2:**
    *   Install **ROS 2 Humble Hawksbill** (the `ros-humble-desktop-full` version). Follow the official ROS 2 installation guide for Ubuntu.
    *   Remember to source ROS 2 in your `.bashrc` file:
        ```bash
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        ```

5.  **OpenAI API Key:**
    *   You need an OpenAI account with API access.
    *   Create an API key from your OpenAI dashboard.
    *   Set the key as an environment variable. Add this line to your `~/.bashrc` file, replacing `your_key_here` with your actual key.
        ```bash
        echo "export OPENAI_API_KEY='your_key_here'" >> ~/.bashrc
        source ~/.bashrc
        ```
    *   Verify by typing `echo $OPENAI_API_KEY` in a new terminal.

---

### **Part 2: Creating the ROS 2 Workspace**

1.  **Create the Workspace Directories:**
    Open a terminal and run these commands to create the folder structure for your ROS 2 package.
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    mkdir -p humanoid_control/launch humanoid_control/humanoid_control
    ```

2.  **Create the Source Files:**
    Navigate into the `~/ros2_ws/src/humanoid_control` directory. Create the following files and copy-paste the content for each from the `DEMO_SOURCE_CODE.md` file you have.

    *   `package.xml`
    *   `setup.py`
    *   `launch/demo.launch.py`
    *   `humanoid_control/voice_input_node.py`
    *   `humanoid_control/llm_action_node.py`

    *Tip: Use a text editor like VS Code to create and save these files easily.*

---

### **Part 3: Installing Dependencies & Building**

1.  **Install Python Libraries:**
    These libraries are required for the Python scripts to function.
    ```bash
    pip install openai
    pip install SpeechRecognition
    # The following might require `sudo apt install` if pip fails
    pip install pyaudio 
    ```
    *Note: If `pip install pyaudio` fails, you may need to install its dependencies first: `sudo apt-get install portaudio19-dev python3-pyaudio`*

2.  **Build the ROS 2 Workspace:**
    Navigate to the root of your workspace (`~/ros2_ws`) and run `colcon build`.
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select humanoid_control
    ```

3.  **Source the Workspace:**
    After the build completes, you need to source your local setup file.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
    *Pro-Tip: Add this line to your `.bashrc` as well, so you don't have to source it in every new terminal: `echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc`*

---

### **Part 4: Running the Demo (The Final Step!)**

You will need **two terminals** and the **Isaac Sim application** running simultaneously.

1.  **Terminal 1: Launch the ROS 2 Nodes**
    *   Open a new terminal. It should have both ROS 2 and your workspace sourced.
    *   Run the demo launch file:
        ```bash
        ros2 launch humanoid_control demo.launch.py
        ```
    *   You should see both the `voice_input_node` and `llm_action_node` start up and print "ready" messages. The voice node will calibrate and then say "Listening for command...".

2.  **Isaac Sim: Run the Controller Script**
    *   Launch **NVIDIA Isaac Sim**.
    *   Go to the menu `Window -> Script Editor`.
    *   Create a new file and save it as `isaac_sim_controller.py` in a memorable location (e.g., your Documents folder).
    *   Copy-paste the full content of `isaac_sim_controller.py` from `DEMO_SOURCE_CODE.md` into the Script Editor.
    *   Press the "Run" button (the green play icon) in the Script Editor.
    *   The simulation environment will load. You will see a ground plane, a humanoid, a table, and the cups. In Isaac Sim's console logs, you should see "Isaac Controller Node is ready...".

3.  **Execute the Voice Command!**
    *   With everything running, speak clearly into your microphone:
        > **"Okay robot, please pick up the red cup and bring it to me."**

4.  **Watch the Magic Happen:**
    *   **Terminal 1:** You will see the `voice_input_node` transcribe your speech, and the `llm_action_node` generate the JSON plan.
    *   **Isaac Sim:** The robot in the simulation will execute the plan step-by-step. It will move to the table, the red cup will attach to its "hand," and it will return to its starting area.

You have now successfully run the full, end-to-end Physical AI demo.
