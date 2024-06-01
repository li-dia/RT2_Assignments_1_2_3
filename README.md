# Research Track 2 (RT2) Assignments

This repository contains three assignments for the Research Track 2 (RT2) course. Each assignment focuses on different aspects of robotics and data analysis, ranging from enhanced documentation to interactive user interfaces and statistical performance comparisons.

## Table of Contents
- [Assignment 1: Enhanced Documentation](#assignment-1-enhanced-documentation)
- [Assignment 2: Interactive Jupyter Notebook Interface](#assignment-2-interactive-jupyter-notebook-interface)
- [Assignment 3: Statistical Analysis](#assignment-3-statistical-analysis)

## Assignment 1: Enhanced Documentation

This assignment builds upon the second assignment from the previous semester, with the main addition being comprehensive documentation using Sphinx. The documentation is designed to provide a detailed overview of the code, its functionality, and usage instructions.

### Key Features
- **Sphinx Documentation**: Detailed documentation generated using Sphinx.
- **GitHub Pages**: The documentation is hosted on GitHub Pages for easy access.

### Accessing the Documentation
The generated documentation is available [here](https://li-dia.github.io/RT2_Assignments_1_2_3/).

## Assignment 2: Interactive Jupyter Notebook Interface

This assignment involves creating a Jupyter Notebook-based user interface for the second assignment of the RT1 course. The goal is to replace the existing user interface node with an interactive notebook that provides real-time information about the robot's position.

### Key Features
- **Widgets for User Interaction**: The notebook utilizes Jupyter widgets to enhance user interaction.
- **Real-Time Position Visualization**: The robot's position and path to target positions are visualized in real-time.
- **Target Status Indicators**: Reached targets are marked with green squares, while not-reached targets are marked with red Xs.

### Visualization
Below are some screenshots of the interactive notebook visualizing the robot's path and target status:

![Robot Path and Target Status](https://github.com/li-dia/RT2_Assignments_1_2_3/assets/118188149/0f0bd620-fd61-4364-b9dd-afaa356170c9)
*Visualization of the robot's path and target positions, with green squares indicating reached targets and red Xs indicating not-reached targets.*


![Jupyter Notebook Widgets](https://github.com/li-dia/RT2_Assignments_1_2_3/assets/118188149/b3ceb29e-b48f-4bbb-a42c-024d5cffd388)

*Buttons and text widgets for setting target coordinates and controlling the robot.*

### How to Use

1. Ensure that you have set up your ROS workspace correctly.

   ```bash
   cd path/to/your/ros_workspace
   catkin_make
   source devel/setup.bash
2. Clone the Repository:
```bash
   git clone https://github.com/li-dia/RT-Assignment2-Lidia-Loubar.git
  cd RT-Assignment2-Lidia-Loubar
```
3. Run the ROS Nodes using the Launch File:
Execute the following command to launch the needed nodes:

```bash
  roslaunch assignment_2_2023 assignment1.launch
```
4. Open the Jupyter Notebook and run all the cells:
```bash
   jupyter notebook RT2_Assignment2.ipynb
```
  
5. To set the target, enter the coordinates in the Target X and Target Y widgets, then click on the "Set Goal" button. To cancel, click on the "Cancel Goal" button.

## Assignment 3: Statistical Analysis
This assignment involves performing a statistical analysis to compare the performance of two code implementations for the first assignment of the RT1 course: my code and my colleague's code.

### Key Features
