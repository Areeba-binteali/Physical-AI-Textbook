---
id: 03-first-ros-program
title: "Chapter 3: Your First ROS 2 Program"
---



# Chapter 3: Your First ROS 2 Program

### Learning Goals

You've learned the concepts of nodes and topics, and you've seen some code examples. Now it's time to put it all together and create a complete, runnable ROS 2 program from scratch. This chapter will walk you through the entire development workflow. By the end, you will be able to:

-   Create a ROS 2 workspace.
-   Create a ROS 2 Python package.
-   Understand the structure of a Python package (`setup.py`, `package.xml`).
-   Write a simple "talker" node (a publisher).
-   Use `colcon` to build your package.
-   Source your workspace and run your node.

---

### The ROS 2 Development Workflow

Developing a ROS 2 application follows a standard pattern:

1.  **Create a Workspace**: A dedicated directory to hold all your related ROS packages.
2.  **Create a Package**: A container for your nodes, launch files, and configuration.
3.  **Write Code**: Implement your nodes in Python or C++.
4.  **Declare Dependencies**: Specify which other ROS 2 packages your code relies on.
5.  **Build**: Compile your code and generate executables using the `colcon` build tool.
6.  **Source & Run**: Make your executables available in your terminal and run them.

Let's go through each step.

---

### Step 1: Create a Workspace

A ROS 2 workspace is a directory containing other packages. It's standard practice to create a `src` subdirectory within your workspace to hold the source code of your packages.

Open a terminal and run the following commands:
```bash
# We recommend creating a dedicated folder for your ROS 2 development
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
You now have a workspace located at `~/ros2_ws`. The `src` folder is where you will place your new packages.

---

### Step 2: Create a ROS 2 Python Package

Now, navigate into the `src` directory and use the `ros2 pkg create` command to create a new package for our talker node.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_pkg
```

This command creates a new directory named `my_first_pkg` with the following structure:
```
my_first_pkg/
├── my_first_pkg/
│   └── __init__.py
├── resource/
│   └── my_first_pkg
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
└── setup.py
```

-   **`my_first_pkg/my_first_pkg/`**: This is where your Python node files will go.
-   **`package.xml`**: An XML file containing metadata about your package, such as its name, version, author, and dependencies.
-   **`setup.py`**: The build script for your Python package. It tells ROS 2 how to install your package and where to find its executables.

---

### Step 3: Write Your "Talker" Node

Let's create the publisher node we discussed in the previous chapter. Create a new file named `talker.py` inside the `my_first_pkg/my_first_pkg/` directory.

**File: `~/ros2_ws/src/my_first_pkg/my_first_pkg/talker.py`**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Talker node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from ROS 2 at {self.get_clock().now()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    talker_node = TalkerNode()
    rclpy.spin(talker_node)
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Step 4: Declare Dependencies and Entry Points

Now we need to tell ROS 2 about our node.

#### `package.xml`
First, declare the dependencies. Open `package.xml` and add these lines inside the `<package>` tags. This tells ROS that our package depends on `rclpy` (the Python client library) and `std_msgs` (for the `String` message type).
```xml
  <!-- Add these dependency tags -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

#### `setup.py`
Next, we need to create an "entry point" so ROS 2 knows how to run our `talker.py` script. Open `setup.py` and modify the `entry_points` dictionary. This tells `colcon` to create an executable script named `talker` that runs the `main` function from our `talker.py` file.
```python
    entry_points={
        'console_scripts': [
            'talker = my_first_pkg.talker:main',
        ],
    },
```

---

### Step 5: Build the Package

Navigate back to the root of your workspace (`~/ros2_ws`) and run the `colcon build` command.
```bash
cd ~/ros2_ws
colcon build
```
`colcon` is the ROS 2 build tool. It will automatically find your package, process the `setup.py` and `package.xml` files, and install the results into the `install` directory.

You should see output indicating a successful build.
```
Starting >>> my_first_pkg
Finished <<< my_first_pkg [2.32s]

Summary: 1 package finished [2.53s]
```

---

### Step 6: Source the Workspace and Run

Before you can run your new node, you need to add your workspace's `install` directory to your shell's path. This is called "sourcing an overlay".

In the same terminal where you ran the build, run:
```bash
source install/setup.bash
```
**IMPORTANT**: You need to do this in every new terminal you open to make your workspace's packages available.

Now, you can finally run your node!
```bash
ros2 run my_first_pkg talker
```
You should see the "Publishing..." messages printed to your screen.

To see the messages being published, open a **new terminal**, source the workspace again (`source ~/ros2_ws/install/setup.bash`), and use `ros2 topic echo`:
```bash
ros2 topic echo /chatter
```
You have successfully created, built, and run your first ROS 2 program!

---

### Exercises

1.  What is the purpose of the `src` directory in a ROS 2 workspace?
2.  In `setup.py`, what does the line `'talker = my_first_pkg.talker:main'` do?
3.  Why is it necessary to run `source install/setup.bash` after building a workspace?

---

### Quiz

1.  What is the command to create a new ROS 2 Python package?
    a) `ros2 create pkg`
    b) `ros2 pkg new --python`
    c) `ros2 pkg create --build-type ament_python <package_name>`
    d) `colcon create pkg <package_name>`

2.  Which file is used to declare the dependencies of a package?
    a) `setup.py`
    b) `package.xml`
    c) `dependencies.txt`
    d) `CMakeLists.txt`

3.  What does the `colcon build` command do?
    a) It runs the Python code directly.
    b) It finds packages in the `src` directory, compiles them, and installs them into the `install` directory.
    c) It downloads dependencies from the internet.
    d) It deletes all the files in the `build` directory.
