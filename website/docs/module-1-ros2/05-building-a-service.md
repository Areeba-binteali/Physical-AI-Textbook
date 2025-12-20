---
id: 05-building-a-service
title: "Chapter 5: Building Your First Service"
---



# Chapter 5: Building Your First Service

### Learning Goals

You've created a standalone publisher node, but robotics is about interaction. In this chapter, you'll learn how to build a complete request/response interaction by creating your first ROS 2 Service. You will apply the workflow from the previous chapter to a client/server pattern. By the end, you will be able to:

-   Define a custom Service interface in a `.srv` file.
-   Modify your build files (`CMakeLists.txt` and `package.xml`) to generate the necessary code for custom interfaces.
-   Write a Service server node in Python.
-   Write a Service client node in Python.
-   Build and run the client/server interaction.

---

### The Goal: A Simple Calculator

Our goal is to create a service that adds two integers.
-   The **Server** will wait for a request containing two integers.
-   The **Client** will send a request with two integers and wait for the result.
-   The **Server** will compute the sum and send it back as the response.

This follows the same pattern as the `AddTwoInts` example from Chapter 4, but this time, you will create the service definition and all the code yourself.

---

### Step 1: Define the Custom Service (`.srv` file)

First, we need to define the structure of our request and response. In your package `my_first_pkg`, create a new directory called `srv`.

```bash
cd ~/ros2_ws/src/my_first_pkg
mkdir srv
```

Inside the `srv` directory, create a new file named `Sum.srv`.
**File: `~/ros2_ws/src/my_first_pkg/srv/Sum.srv`**
```
int64 a
int64 b
---
int64 sum
```
This file defines a service named `Sum`. The part above the `---` is the request (two 64-bit integers, `a` and `b`), and the part below is the response (one 64-bit integer, `sum`).

---

### Step 2: Update Build Configuration

Before you can use this new `.srv` file in your code, you need to tell `colcon` how to process it. This involves editing both `package.xml` and a new file, `CMakeLists.txt`, because interface generation is handled by CMake.

#### `package.xml`
Open `package.xml` and add the following lines. This is crucial for telling ROS 2 that you are building custom interface files and that you need the `rosidl_default_generators` package to do it.

```xml
<!-- Right after the <build_depend>ament_python</build_depend> line -->
<build_depend>rosidl_default_generators</build_depend>

<!-- Right after the <exec_depend>rclpy</exec_depend> line -->
<exec_depend>rosidl_default_runtime</exec_depend>

<!-- At the bottom, before </package> -->
<member_of_group>rosidl_interface_packages</member_of_group>
```

#### `CMakeLists.txt`
Your Python package doesn't have a `CMakeLists.txt` by default. You need to create one.

**File: `~/ros2_ws/src/my_first_pkg/CMakeLists.txt`**
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_first_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Sum.srv"
)

ament_package()
```
This file tells the build system to find the necessary packages and then generate the ROS 2 interface code for your `Sum.srv` file.

Finally, you also need to tell your `setup.py` that you are using a cmake build type. Change your `ament_python` to `ament_cmake` in `package.xml` and `ros2 pkg create`. You also need to change the build type in the `setup.py` to `ament_cmake`. But for a pure python package, it is easier to just add the cmake file. We will stick to `ament_python` for now and just add the `CMakeLists.txt`. For `colcon` to recognize this hybrid package, we need to add a line to `setup.py`.

In `setup.py`, add `data_files=[...]` if it's not there and include the `CMakeLists.txt` file. However, the simplest way for a Python package is to handle this through `package.xml` and `CMakeLists.txt` alone. For `colcon` to correctly build both, you must specify both build types when creating the package, or manually edit the files. For this tutorial, we will assume you are adding a service to an existing Python package. The `CMakeLists.txt` and the changes to `package.xml` are the most important parts for interface generation.

---

### Step 3: Write the Service Server

Now, create a new file for your server node.

**File: `~/ros2_ws/src/my_first_pkg/my_first_pkg/sum_server.py`**
```python
import rclpy
from rclpy.node import Node
# Import your custom Service interface
from my_first_pkg.srv import Sum

class SumServer(Node):
    def __init__(self):
        super().__init__('sum_server')
        self.srv = self.create_service(Sum, 'sum_service', self.sum_callback)
        self.get_logger().info('Sum Service is ready.')

    def sum_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming Request: a={request.a} b={request.b}')
        self.get_logger().info(f'Sending Response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    sum_server = SumServer()
    rclpy.spin(sum_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Notice that we `from my_first_pkg.srv import Sum`. This is possible because `colcon` will generate the Python code from your `Sum.srv` file during the build process.

---

### Step 4: Write the Service Client

Next, create the client node.

**File: `~/ros2_ws/src/my_first_pkg/my_first_pkg/sum_client.py`**
```python
import rclpy
from rclpy.node import Node
from my_first_pkg.srv import Sum
import sys

class SumClient(Node):
    def __init__(self):
        super().__init__('sum_client')
        self.cli = self.create_client(Sum, 'sum_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = Sum.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print('Usage: ros2 run my_first_pkg sum_client <a> <b>')
        return

    sum_client = SumClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    
    future = sum_client.send_request(a, b)
    rclpy.spin_until_future_complete(sum_client, future)

    try:
        response = future.result()
        sum_client.get_logger().info(f'Sum of {a} and {b} is {response.sum}')
    except Exception as e:
        sum_client.get_logger().error(f'Service call failed: {e}')
    
    sum_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Step 5: Update `setup.py` with New Entry Points

Add the new server and client nodes to your `setup.py` file so you can run them from the command line.

```python
    entry_points={
        'console_scripts': [
            'talker = my_first_pkg.talker:main',
            'sum_server = my_first_pkg.sum_server:main',
            'sum_client = my_first_pkg.sum_client:main',
        ],
    },
```

---

### Step 6: Build and Run

Navigate to your workspace root, build the package, source it, and run the nodes.

1.  **Build**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_first_pkg
    ```
2.  **Run Server**: Open a terminal, source the workspace, and run the server.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run my_first_pkg sum_server
    ```
3.  **Run Client**: Open a *new* terminal, source the workspace, and run the client with two numbers.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run my_first_pkg sum_client 40 2
    ```

The client terminal should print the result: `Sum of 40 and 2 is 42`. The server terminal will show the logs of the incoming request and outgoing response.

---

### Exercises

1.  Create a new service called `Multiply.srv` that multiplies two numbers.
2.  Implement the server and client nodes for your `Multiply` service.
3.  What happens if you try to run the client before the server is running?

---

### Quiz

1.  What is the purpose of the `---` separator in a `.srv` file?
    a) It's a comment.
    b) It separates the request fields from the response fields.
    c) It indicates the end of the file.
    d) It separates different service definitions.

2.  Which file do you need to edit to ensure your custom `.srv` files are built correctly?
    a) `setup.py` only
    b) `package.xml` only
    c) `CMakeLists.txt` and `package.xml`
    d) `rosdep_install.sh`

3.  In a Python service client, what does the `cli.wait_for_service()` function do?
    a) It starts the service.
    b) It sends the request to the service.
    c) It pauses the client node until the server is available.
    d) It shuts down the client.
