import tkinter as tk
from tkinter import ttk, messagebox
import math
import random

# --- 1. CORE ALGORITHMIC LOGIC (Same as before) ---

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
    """Phase 1: Generates a set of locally efficient mini-routes (tasks)."""
    # [Code for _generate_base_routes function - same as previous response]
    unassigned_points = set(points_data.keys())
    temp_routes = []
    
    while unassigned_points:
        route_path = ['DEPOT']
        current_dist = 0
        
        # Start at a random unassigned point (or closest to Depot)
        if not unassigned_points: break
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
    """Technique 2: DP Partition Heuristic (Fairness)."""
    # [Code for dp_partition_solver function - same as previous response]
    sorted_routes = sorted(mini_routes, key=lambda r: r['load'], reverse=True)
    driver_data = {i: {'route_paths': [], 'distance': 0.0} for i in range(1, num_drivers + 1)}

    for route in sorted_routes:
        min_load_driver_id = min(driver_data, key=lambda k: driver_data[k]['distance'])
        driver_data[min_load_driver_id]['distance'] += route['load']
        driver_data[min_load_driver_id]['route_paths'].append(route['path'])

    final_driver_data = {}
    for id, data in driver_data.items():
        full_route_sequence = []
        for path in data['route_paths']:
            full_route_sequence.extend(path[:-1])
        if full_route_sequence: full_route_sequence.append('DEPOT')
        
        final_driver_data[id] = {
            'route': full_route_sequence if full_route_sequence else ['DEPOT'],
            'distance': data['distance']
        }
    return final_driver_data

def sequential_greedy_solver(mini_routes, num_drivers):
    """Technique 1: Simplified Greedy Route Assignment (Efficiency Baseline)."""
    # [Code for sequential_greedy_solver function - same as previous response]
    driver_data = {i: {'route_paths': [], 'distance': 0.0} for i in range(1, num_drivers + 1)}
    
    for i, route in enumerate(mini_routes):
        driver_id = (i % num_drivers) + 1
        driver_data[driver_id]['distance'] += route['load']
        driver_data[driver_id]['route_paths'].append(route['path'])

    final_driver_data = {}
    for id, data in driver_data.items():
        full_route_sequence = []
        for path in data['route_paths']:
            full_route_sequence.extend(path[:-1])
        if full_route_sequence: full_route_sequence.append('DEPOT')
        
        final_driver_data[id] = {
            'route': full_route_sequence if full_route_sequence else ['DEPOT'],
            'distance': data['distance']
        }
    return final_driver_data

# --- 2. TKINTER GUI IMPLEMENTATION ---

class DeliveryOptimizerApp:
    def __init__(self, master):
        self.master = master
        master.title("VRP: Algorithmic Comparison")
        
        self.points_data, self.depot_coords = generate_sample_points(30) # Increased points for better test
        self.num_drivers = tk.IntVar(value=5)
        self.max_route_dist = tk.DoubleVar(value=50.0)
        self.algorithm_results = {} # Store results for both algorithms

        self.create_menu()
        self.create_widgets()
        
    def create_menu(self):
        """Creates the Menu Bar for configuration and actions."""
        menubar = tk.Menu(self.master)
        self.master.config(menu=menubar)

        # Config Menu (Inputs)
        config_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Configuration", menu=config_menu)
        config_menu.add_command(label="Set Parameters", command=self.open_config_window)
        
        # Run Menu (Action)
        run_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Run Analysis", menu=run_menu)
        run_menu.add_command(label="Run All Algorithms & Compare", command=self.run_all_optimizations)
        
        # Help Menu
        menubar.add_command(label="About", command=lambda: messagebox.showinfo("About", "Delivery Cost Sharing Optimizer Prototype."))

    def open_config_window(self):
        """Opens a separate window for setting main input parameters."""
        config_window = tk.Toplevel(self.master)
        config_window.title("Set Optimization Parameters")
        
        frame = ttk.Frame(config_window, padding="15")
        frame.grid(row=0, column=0)

        # Driver Count
        ttk.Label(frame, text="Number of Drivers (K):").grid(row=0, column=0, sticky="w", pady=5)
        ttk.Entry(frame, textvariable=self.num_drivers, width=10).grid(row=0, column=1, pady=5)

        # Max Route Distance
        ttk.Label(frame, text="Max Route Distance (L_max):").grid(row=1, column=0, sticky="w", pady=5)
        ttk.Entry(frame, textvariable=self.max_route_dist, width=10).grid(row=1, column=1, pady=5)

        # Save Button
        ttk.Button(frame, text="Close & Save", command=config_window.destroy).grid(row=2, column=0, columnspan=2, pady=15)
        
    def create_widgets(self):
        """Creates the main output area with the Tabbed Interface."""
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.master.grid_rowconfigure(0, weight=1)
        self.master.grid_columnconfigure(0, weight=1)

        # Tabbed Output Interface (Notebook)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)

        # Create two tabs for the algorithms
        self.greedy_tab = ttk.Frame(self.notebook, padding="10")
        self.dp_tab = ttk.Frame(self.notebook, padding="10")
        
        self.notebook.add(self.greedy_tab, text="1. Greedy (Efficiency)")
        self.notebook.add(self.dp_tab, text="2. DP Partition (Fairness)")
        
        # Initialize output elements for both tabs
        self._setup_output_tab(self.greedy_tab, "greedy")
        self._setup_output_tab(self.dp_tab, "dp")
        
    def _setup_output_tab(self, tab, tag):
        """Sets up the metrics and treeview table within a single tab."""
        # Metrics
        metrics_frame = ttk.LabelFrame(tab, text="Summary Metrics", padding="10")
        metrics_frame.pack(fill='x', pady=5)

        setattr(self, f'{tag}_total_dist_label', ttk.Label(metrics_frame, text="Total System Distance: N/A", font=('Arial', 10, 'bold')))
        getattr(self, f'{tag}_total_dist_label').grid(row=0, column=0, sticky="w", padx=10, pady=2)
        
        setattr(self, f'{tag}_imbalance_label', ttk.Label(metrics_frame, text="Load Imbalance (max-min): N/A", font=('Arial', 10, 'bold')))
        getattr(self, f'{tag}_imbalance_label').grid(row=0, column=1, sticky="w", padx=10, pady=2)
        
        # Table
        ttk.Label(tab, text="Driver Route Details").pack(pady=5)
        
        tree = ttk.Treeview(tab, columns=('Distance', 'Route'), show='headings', height=10)
        tree.heading('Distance', text='Total Distance')
        tree.heading('Route', text='Assigned Route Sequence')
        tree.column('Distance', width=120, anchor='center')
        tree.column('Route', width=500)
        tree.pack(fill='both', expand=True)
        setattr(self, f'{tag}_tree', tree)
        
    def run_all_optimizations(self):
        """Executes both algorithms sequentially and updates both tabs."""
        try:
            num_d = self.num_drivers.get()
            max_d = self.max_route_dist.get()
            
            if num_d <= 0 or max_d <= 0:
                messagebox.showerror("Error", "Parameters must be set to positive values.")
                return

            # Phase 1: Generate Base Mini-Routes
            mini_routes = _generate_base_routes(self.points_data, self.depot_coords, max_d)
            if not mini_routes:
                 messagebox.showinfo("Result", "No routes could be generated under the given constraints.")
                 return

            # Phase 2: Run both algorithms
            greedy_data = sequential_greedy_solver(mini_routes, num_d)
            dp_data = dp_partition_solver(mini_routes, num_d)
            
            # Update the UI for each algorithm
            self._update_tab_ui(self.greedy_tab, "greedy", greedy_data)
            self._update_tab_ui(self.dp_tab, "dp", dp_data)
            
            self.notebook.select(self.dp_tab) # Default to the Fairness result
            
        except Exception as e:
            messagebox.showerror("Runtime Error", f"Error during optimization: {e}")

    def _update_tab_ui(self, tab_widget, tag, driver_data):
        """Calculates metrics and populates the table for a single algorithm tab."""
        distances = [data['distance'] for data in driver_data.values()]
        
        total_dist = sum(distances)
        max_dist = max(distances) if distances else 0
        min_dist = min(distances) if distances else 0
        imbalance = max_dist - min_dist

        # Update Metrics Labels
        getattr(self, f'{tag}_total_dist_label').config(text=f"Total System Distance: {total_dist:.2f} units")
        getattr(self, f'{tag}_imbalance_label').config(text=f"Load Imbalance (max-min): {imbalance:.2f} units")

        # Update Treeview Table
        tree = getattr(self, f'{tag}_tree')
        tree.delete(*tree.get_children())

        for driver_id, data in driver_data.items():
            route_str = " -> ".join(data['route'])
            
            tag_name = ''
            if data['distance'] == max_dist and imbalance > 0:
                tag_name = 'max_load'
            elif data['distance'] == min_dist and imbalance > 0:
                tag_name = 'min_load'
            
            tree.insert('', tk.END, text=f"Driver {driver_id}", 
                             values=(f"{data['distance']:.2f}", route_str), 
                             tags=(tag_name,))

        # Configure highlighting tags
        tree.tag_configure('max_load', background='salmon')
        tree.tag_configure('min_load', background='lightgreen')

# --- 3. RUN THE APPLICATION ---

if __name__ == "__main__":
    root = tk.Tk()
    app = DeliveryOptimizerApp(root)
    root.mainloop()