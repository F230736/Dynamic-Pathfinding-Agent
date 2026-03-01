# Dynamic Pathfinding Agent (AI Assignment 2)
**Aleeza Ahmad (23F-0736)**
---
## Project Overview

This project implements a **Dynamic Pathfinding Agent** that navigates a grid environment using informed search algorithms.
It supports real-time path re-planning when obstacles appear dynamically during execution.

Algorithms implemented:

* A* Search
* Greedy Best-First Search (GBFS)

Heuristics:

* Manhattan Distance
* Euclidean Distance

The application provides a full graphical interface to visualize:

* Explored nodes
* Frontier nodes
* Final optimal path
* Dynamic obstacle spawning and re-planning

The GUI is built using Python’s Tkinter library.

---

## Features

* Adjustable grid size (rows, columns, cell size)
* Random obstacle generation with adjustable density
* Interactive map editor (place walls, set start, set goal, erase)
* Visualization of search exploration and final path
* Dynamic obstacles that trigger automatic re-planning
* Performance metrics:

  * Nodes visited
  * Path cost
  * Execution time
  * Re-plan count
  * Status updates

---

## Requirements

Make sure you have **Python 3.8 or higher** installed.

No external libraries are required because the project uses built-in Python modules:

* tkinter
* heapq
* math
* random
* time
* collections

---

## Installation

1. Clone or download the project files.
2. Ensure Python is installed on your system.

To verify Python installation:

```bash
python --version
```

No additional dependencies are required.
(There is **no need** to run `pip install`.)

---

## How to Run the Program

Navigate to the folder containing the file:

```
pathfinding_agent.py
```

Then run:

```bash
python pathfinding_agent.py
```

The GUI window will open showing the grid and control panel.

---

## How to Use

### 1. Configure Grid

* Set Rows, Columns, and Cell Size
* Click **Apply Grid**

### 2. Generate Map

* Set obstacle percentage
* Click **Generate Random Map**

### 3. Edit Map (Optional)

Use edit modes:

* Place Wall
* Set Start
* Set Goal
* Erase

Click or drag on the grid to modify cells.

### 4. Select Algorithm

Choose one:

* A* Search
* Greedy Best-First Search (GBFS)

### 5. Select Heuristic

* Manhattan Distance (best for grid movement)
* Euclidean Distance

### 6. Dynamic Mode (Optional)

Enable:

* Dynamic obstacles spawning during execution
* Automatic path re-planning

### 7. Run Search

Click:

```
▶ START SEARCH
```

Controls:

* ⏹ STOP → Pause search
* ⏵ RESUME → Continue paused search
* ↺ RESET VISUAL → Clear path and visualization

---

## Visualization Legend

| Color        | Meaning        |
| ------------ | -------------- |
| Green        | Start Node     |
| Red          | Goal Node      |
| Cyan         | Final Path     |
| Yellow       | Frontier Nodes |
| Blue         | Visited Nodes  |
| Gray         | Wall           |
| White Circle | Agent Position |

---

## Algorithms Summary

### A* Search

Formula:

```
f(n) = g(n) + h(n)
```

* Optimal and complete
* Finds shortest path
* Slightly slower and higher memory usage

### Greedy Best-First Search (GBFS)

Formula:

```
f(n) = h(n)
```

* Faster and lightweight
* Not guaranteed optimal
* May fail in complex maze environments

---

## Dynamic Re-Planning Logic

* Random obstacles appear during agent movement
* If obstacle blocks current path:

  * The agent automatically re-runs the search from current position
  * New path is computed in real time

---

## Test Scenarios Included

1. A* Best Case (open grid)
2. A* Worst Case (dense maze)
3. GBFS Best Case
4. GBFS Worst Case (maze trap)
5. A* vs GBFS comparison
6. Dynamic obstacle spawning mode
7. Interactive map editing
8. No-path scenario handling

---
## Notes

* The program uses only built-in Python libraries, so it runs on any system with Python installed.
* Dynamic mode visually demonstrates real-time AI decision making and adaptive path planning.

---

## Conclusion
This project demonstrates intelligent navigation using informed search algorithms with dynamic environment handling, making it suitable for applications like robotics navigation, game AI, and autonomous agents.
