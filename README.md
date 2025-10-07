# Delivery Cost Sharing & Route Load Balancing

## Project Overview

This project is a prototype implementation for a **Multi-Objective Vehicle Routing Problem (VRP)** focusing on distributing delivery workload among a fleet of drivers. The primary goal is to analyze the trade-off between **Efficiency (Minimizing Total Distance)** and **Fairness (Balancing Driver Load)**.

The application, built with Python and **Tkinter**, allows users to compare the performance of two distinct algorithmic approaches: the **Greedy Route Solver** and the **DP Partition Heuristic**.

### Use Cases
* Balance load across drivers (Fairness).
* Minimize overall system distance (Efficiency).
* Compare Greedy vs. DP coverage performance.

### Techniques Implemented
* **Greedy (Nearest Neighbor):** Used for initial route generation and as a comparison baseline.
* **Dynamic Programming (DP) Partition Heuristic:** Used to model the load-balancing problem and minimize load variance.

---

## Getting Started

### Prerequisites
* Python 3.x
* The standard `tkinter` library (usually included with Python installation).

### Installation
1.  Save the Python code provided in the next section as `delivery_optimizer.py`.
2.  Run the application from your terminal:
    ```bash
    python delivery_optimizer.py
    ```

---

## How to Demo the Features

The application is designed for direct comparison. Follow these steps to demonstrate the core project features:

### 1. Configure the Scenario
* Set **Number of Drivers (K)** (e.g., 4).
* Set **Max Route Distance ($L_{max}$)** (e.g., 75.0 units).

### 2. Run 1: Greedy Route Solver (Efficiency Baseline)
1.  Select **"Greedy Route Solver (Efficiency)"** from the dropdown menu.
2.  Click **"Run Optimization."**
3.  **Observation:** Note the output. You should see a relatively **low Total System Distance** but a **high Load Imbalance** ($\max - \min$ value).

### 3. Run 2: DP Partition Heuristic (Fairness Optimization)
1.  Select **"DP Partition (Fairness)"** from the dropdown menu.
2.  Click **"Run Optimization."**
3.  **Observation:** Compare the metrics. You should see a **significantly lower Load Imbalance**. The Total System Distance may be slightly higher, illustrating the **trade-off** between fairness and efficiency.

---

## Output Metrics Explained

The application provides the following key metrics to evaluate performance:

| Metric | Goal | Interpretation |
| :--- | :--- | :--- |
| **Total System Distance** | **Minimize** | Measures **Efficiency** (lower is better). |
| **Load Imbalance ($\max - \min$)** | **Minimize to Zero** | Measures **Fairness** (lower is better). |

---

## Academic Details

| Semester | S.E. Semester III – Computer Engineering |
| :--- | :--- |
| Subject | Analysis of Algorithm |
| Project Title | Delivery Cost Sharing and Route Load Balancing |

