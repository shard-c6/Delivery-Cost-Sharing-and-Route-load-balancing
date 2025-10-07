import tkinter as tk
from tkinter import ttk, messagebox
import math
import random

# --- 1. CORE ALGORITHMIC LOGIC ---

def calculate_distance(p1, p2):
    """Calculates Euclidean distance (simplified for prototype)."""
    return math.sqrt((p1['lat'] - p2['lat'])**2 + (p1['lon'] - p2['lon'])**2)

def generate_sample_points(n):
    """Generates random delivery points and depot."""
    points = {}
    for i in range(1, n + 1):
        points[f'P{i}'] = {'lat': random.uniform(1, 10), 'lon': random.uniform(1, 10)}
    return points, {'lat': 5.0, 'lon': 5.0} # Depot

def _generate_base_routes(points_data, depot_coords, max_d):
    """
    Phase 1: Generates a set of locally efficient mini-routes (tasks). 
    Uses a Greedy Nearest Neighbor approach constrained by max_d.
    """
    unassigned_points = set(points_data.keys())
    temp_routes = []
    
    while unassigned_points:
        route_path = ['DEPOT']
        current_dist = 0
        
        # Start at a random unassigned point (or closest to Depot)
        start_point = min(unassigned_points, key=lambda p: calculate_distance(depot_coords, points_data[p]))
        unassigned_points.remove(start_point)
        route_path.append(start_point)
        
        while True:
            best_next_point = None
            min_next_distance = float('inf')
            last_coords = points_data.get(route_path[-1], depot_coords)

            for p_name in list(unassigned_points):
                p_coords = points_data[p_name]
                dist = calculate_distance(last_coords, p_coords)
                dist_to_depot = calculate_distance(p_coords, depot_coords)
                
                # Constraint check: distance to next point + remaining distance back to Depot
                if current_dist + dist + dist_to_depot <= max_d:
                    if dist < min_next_distance:
                        min_next_distance = dist
                        best_next_point = p_name

            if best_next_point:
                route_path.append(best_next_point)
                current_dist += min_next_distance
                unassigned_points.remove(best_next_point)
            else:
                break
        
        if len(route_path) > 1:
            final_load = current_dist + calculate_distance(points_data.get(route_path[-1], depot_coords), depot_coords)
            temp_routes.append({'path': route_path + ['DEPOT'], 'load': final_load})
            
    return temp_routes


def dp_partition_solver(mini_routes, num_drivers):
    """
    Technique 2: DP Partition Heuristic (Fairness). 
    Assigns mini-routes to K drivers to equalize loads.
    (Implemented as a greedy heuristic for the K-way Partition Problem).
    """
    sorted_routes = sorted(mini_routes, key=lambda r: r['load'], reverse=True)
    
    driver_data = {
        i: {'route_paths': [], 'distance': 0.0}
        for i in range(1, num_drivers + 1)
    }

    # Assign largest routes to the current lowest-loaded driver
    for route in sorted_routes:
        min_load_driver_id = min(driver_data, key=lambda k: driver_data[k]['distance'])
        driver_data[min_load_driver_id]['distance'] += route['load']
        driver_data[min_load_driver_id]['route_paths'].append(route['path'])

    # Format output
    final_driver_data = {}
    for id, data in driver_data.items():
        full_route_sequence = []
        for path in data['route_paths']:
            full_route_sequence.extend(path[:-1])
        
        if full_route_sequence:
            full_route_sequence.append('DEPOT')
        
        final_driver_data[id] = {
            'route': full_route_sequence if full_route_sequence else ['DEPOT'],
            'distance': data['distance']
        }
    
    return final_driver_data

def sequential_greedy_solver(mini_routes, num_drivers):
    """
    Technique 1: Simplified Greedy Route Assignment (Efficiency Baseline).
    Assigns mini-routes in a round-robin fashion, prioritizing system efficiency over fair balance.
    """
    driver_data = {i: {'route_paths': [], 'distance': 0.0} for i in range(1, num_drivers + 1)}
    
    # Assign sequentially (round-robin)
    for i, route in enumerate(mini_routes):
        driver_id = (i % num_drivers) + 1
        driver_data[driver_id]['distance'] += route['load']
        driver_data[driver_id]['route_paths'].append(route['path'])

    # Format output
    final_driver_data = {}
    for id, data in driver_data.items():
        full_route_sequence = []
        for path in data['route_paths']:
            full_route_sequence.extend(path[:-1])
        
        if full_route_sequence:
            full_route_sequence.append('DEPOT')
        
        final_driver_data[id] = {
            'route': full_route_sequence if full_route_sequence else ['DEPOT'],
            'distance': data['distance']
        }
    
    return final_driver_data

# --- 2. TKINTER GUI IMPLEMENTATION ---

class DeliveryOptimizerApp:
    def __init__(self, master):
        self.master = master
        master.title("AOA Project: VRP Comparison")
        self.points_data, self.depot_coords = generate_sample_points(25)
        self.num_drivers = tk.IntVar(value=4)
        self.max_route_dist = tk.DoubleVar(value=75.0)
        self.algorithm_choice = tk.StringVar(value="DP Partition (Fairness)")
        self.create_widgets()

    def create_widgets(self):
        # Configuration Frame
        config_frame = ttk.LabelFrame(self.master, text="Configuration & Algorithm", padding="10")
        config_frame.grid(row=0, column=0, padx=10, pady=10, sticky="n")

        # ... (GUI input elements - similar to previous response)

        ttk.Label(config_frame, text="Number of Drivers:").grid(row=0, column=0, sticky="w", pady=5)
        ttk.Entry(config_frame, textvariable=self.num_drivers).grid(row=0, column=1, pady=5, padx=5)

        ttk.Label(config_frame, text="Max Route Distance:").grid(row=1, column=0, sticky="w", pady=5)
        ttk.Entry(config_frame, textvariable=self.max_route_dist).grid(row=1, column=1, pady=5, padx=5)
        
        ttk.Label(config_frame, text="Optimization Technique:").grid(row=2, column=0, sticky="w", pady=5)
        ttk.Combobox(config_frame, textvariable=self.algorithm_choice, 
                     values=["DP Partition (Fairness)", "Greedy Route Solver (Efficiency)"], 
                     state="readonly").grid(row=2, column=1, pady=5, padx=5)
        
        ttk.Label(config_frame, text=f"Total Points: {len(self.points_data)}").grid(row=3, column=0, columnspan=2, pady=10)
        
        ttk.Button(config_frame, text="Run Optimization", command=self.run_optimization).grid(row=4, column=0, columnspan=2, pady=10)

        # Output Frame
        output_frame = ttk.LabelFrame(self.master, text="Optimization Results", padding="10")
        output_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")
        
        # Metrics Display
        self.total_dist_label = ttk.Label(output_frame, text="Total System Distance: N/A")
        self.total_dist_label.grid(row=0, column=0, sticky="w", pady=5)
        
        self.imbalance_label = ttk.Label(output_frame, text="Load Imbalance (max-min): N/A")
        self.imbalance_label.grid(row=1, column=0, sticky="w", pady=5)
        
        ttk.Separator(output_frame, orient='horizontal').grid(row=2, column=0, sticky="ew", pady=10)

        # Route Details Table
        self.tree = ttk.Treeview(output_frame, columns=('Distance', 'Route'), show='headings', height=10)
        self.tree.heading('Distance', text='Total Distance')
        self.tree.heading('Route', text='Assigned Route Sequence')
        self.tree.column('Distance', width=120, anchor='center')
        self.tree.column('Route', width=400)
        self.tree.grid(row=3, column=0, sticky="nsew")
        
        vsb = ttk.Scrollbar(output_frame, orient="vertical", command=self.tree.yview)
        vsb.grid(row=3, column=1, sticky='ns')
        self.tree.configure(yscrollcommand=vsb.set)

    def run_optimization(self):
        try:
            num_d = self.num_drivers.get()
            max_d = self.max_route_dist.get()
            choice = self.algorithm_choice.get()
            
            if num_d <= 0 or max_d <= 0:
                messagebox.showerror("Error", "Inputs must be positive.")
                return

            # Phase 1: Generate Base Mini-Routes (Efficiency focus, common input for both)
            mini_routes = _generate_base_routes(self.points_data, self.depot_coords, max_d)
            if not mini_routes:
                 messagebox.showinfo("Result", "No routes could be generated under the given constraints.")
                 return

            # Phase 2: Execute Chosen Algorithm
            if "DP Partition" in choice:
                driver_data = dp_partition_solver(mini_routes, num_d)
            elif "Greedy Route Solver" in choice:
                driver_data = sequential_greedy_solver(mini_routes, num_d)
            else:
                return 

            # Update Metrics and UI
            self._update_ui(driver_data)
            
        except Exception as e:
            messagebox.showerror("Runtime Error", f"Error: {e}")

    def _update_ui(self, driver_data):
        """Updates the metrics and the Treeview table."""
        distances = [data['distance'] for data in driver_data.values()]
        
        if not distances:
            self.total_dist_label.config(text="Total System Distance: 0.00 units")
            self.imbalance_label.config(text="Load Imbalance (max-min): 0.00 units")
            return

        total_dist = sum(distances)
        max_dist = max(distances)
        min_dist = min(distances)
        imbalance = max_dist - min_dist

        self.total_dist_label.config(text=f"Total System Distance: {total_dist:.2f} units")
        self.imbalance_label.config(text=f"Load Imbalance (max-min): {imbalance:.2f} units")

        self.tree.delete(*self.tree.get_children())

        for driver_id, data in driver_data.items():
            route_str = " -> ".join(data['route'])
            
            tag = ''
            if data['distance'] == max_dist and imbalance > 0:
                tag = 'max_load'
            elif data['distance'] == min_dist and imbalance > 0:
                tag = 'min_load'
            
            self.tree.insert('', tk.END, text=f"Driver {driver_id}", 
                             values=(f"{data['distance']:.2f}", route_str), 
                             tags=(tag,))

        self.tree.tag_configure('max_load', background='salmon')
        self.tree.tag_configure('min_load', background='lightgreen')

if __name__ == "__main__":
    # Create an initial set of sample points and run the app
    root = tk.Tk()
    app = DeliveryOptimizerApp(root)
    root.mainloop()