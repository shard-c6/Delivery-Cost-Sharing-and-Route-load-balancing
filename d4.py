import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math
import random
import csv
import datetime

# -------------------------
# Core helpers
# -------------------------

def calculate_distance(p1, p2):
    return math.sqrt((p1['lat'] - p2['lat'])**2 + (p1['lon'] - p2['lon'])**2)

def generate_sample_points(n, seed=None, bounds=(1, 10)):
    if seed is not None:
        random.seed(seed)
    low, high = bounds
    pts = {}
    for i in range(1, n + 1):
        pts[f'P{i}'] = {'lat': random.uniform(low, high), 'lon': random.uniform(low, high)}
    # fixed depot at center for simplicity
    return pts, {'lat': (low + high) / 2.0, 'lon': (low + high) / 2.0}

def compute_sequence_distance(seq, points_data, depot_coords):
    """Compute distance of a sequence of nodes (may include 'DEPOT')."""
    if not seq:
        return 0.0
    total = 0.0
    prev = depot_coords
    for node in seq:
        curr = depot_coords if node == 'DEPOT' else points_data[node]
        total += calculate_distance(prev, curr)
        prev = curr
    return total

# -------------------------
# Mini-route generation
# -------------------------
def generate_base_mini_routes(points_data, depot_coords, max_d):
    """
    Greedy nearest-neighbor grouping into mini-routes.
    Each mini-route is {'path': ['DEPOT','Px','Py','DEPOT'], 'load': distance}
    """
    unassigned = set(points_data.keys())
    mini_routes = []

    def route_len(seq_no_depot):
        if not seq_no_depot:
            return 0.0
        prev = depot_coords
        s = 0.0
        for n in seq_no_depot:
            s += calculate_distance(prev, points_data[n])
            prev = points_data[n]
        s += calculate_distance(prev, depot_coords)
        return s

    while unassigned:
        # seed = closest to depot
        seed = min(unassigned, key=lambda p: calculate_distance(depot_coords, points_data[p]))
        seq = [seed]
        unassigned.remove(seed)

        improved = True
        while improved:
            improved = False
            last = points_data[seq[-1]]
            best = None
            best_d = float('inf')
            for p in list(unassigned):
                d = calculate_distance(last, points_data[p])
                if route_len(seq + [p]) <= max_d and d < best_d:
                    best = p
                    best_d = d
            if best:
                seq.append(best)
                unassigned.remove(best)
                improved = True

        load = route_len(seq)
        mini_routes.append({'path': ['DEPOT'] + seq + ['DEPOT'], 'load': load})

    return mini_routes

# -------------------------
# Greedy (Efficiency) solver
# -------------------------
def efficient_greedy_solver(mini_routes, num_drivers, points_data, depot_coords):
    """
    For each mini-route (in input order), place it on the driver that causes
    the smallest marginal increase in that driver's total route distance.
    Returns driver_data: {id: {'route': [...], 'distance': float}}
    """
    # initialize drivers with empty route sequence (starting at DEPOT)
    drivers = {i: {'seq': ['DEPOT'], 'load': 0.0} for i in range(1, num_drivers + 1)}

    def marginal_inc(driver_seq, mini_path):
        # driver_seq: list like ['DEPOT', ... , 'DEPOT' or lastpoint]
        # compute old and new distances robustly
        # normalize seq to ensure trailing DEPOT
        old_seq = driver_seq[:] if driver_seq[-1] == 'DEPOT' else driver_seq[:] + ['DEPOT']
        # new seq: drop trailing DEPOT if present, append mini_path without leading DEPOT, then DEPOT
        base = driver_seq[:] if driver_seq[-1] != 'DEPOT' else driver_seq[:-1]
        add_nodes = [n for n in mini_path if n != 'DEPOT']
        new_seq = base + add_nodes + ['DEPOT']
        old_d = compute_sequence_distance(old_seq, points_data, depot_coords)
        new_d = compute_sequence_distance(new_seq, points_data, depot_coords)
        return new_d - old_d

    for task in mini_routes:
        best_driver = None
        best_inc = float('inf')
        for d_id, d in drivers.items():
            inc = marginal_inc(d['seq'], task['path'])
            if inc < best_inc:
                best_inc = inc
                best_driver = d_id
        # assign
        d = drivers[best_driver]
        # update sequence
        if d['seq'][-1] == 'DEPOT':
            d_seq_base = d['seq'][:-1]
        else:
            d_seq_base = d['seq'][:]
        d_seq_base.extend([n for n in task['path'] if n != 'DEPOT'])
        d_seq_base.append('DEPOT')
        d['seq'] = d_seq_base
        # we compute final load as full sequence distance to be robust
        d['load'] = compute_sequence_distance(d['seq'], points_data, depot_coords)

    final = {}
    for i, d in drivers.items():
        final[i] = {'route': d['seq'] if d['seq'] else ['DEPOT'], 'distance': d['load']}
    return final

# -------------------------
# DP Exact (Fairness) solver
# -------------------------
def dp_exact_partition_solver(mini_routes, num_drivers, m_threshold=22):
    """
    Minimize makespan: binary search a cap and check feasibility via DFS.
    Returns driver_data dict (like other solvers) OR None if instance too large.
    """
    loads = [r['load'] for r in mini_routes]
    tasks = list(enumerate(loads))
    m = len(tasks)
    if m > m_threshold:
        return None  # signal fallback

    total = sum(loads)
    lo = max(loads) if loads else 0.0
    hi = total
    tasks_sorted = sorted(tasks, key=lambda x: x[1], reverse=True)

    def can_pack(cap):
        bins = [0.0] * num_drivers
        seen = set()
        def dfs(idx):
            if idx == m:
                return True
            key = (idx, tuple(sorted(bins)))
            if key in seen:
                return False
            seen.add(key)
            load = tasks_sorted[idx][1]
            prev = -1.0
            for b in range(num_drivers):
                if bins[b] == prev:
                    continue
                if bins[b] + load <= cap + 1e-9:
                    bins[b] += load
                    if dfs(idx + 1):
                        return True
                    bins[b] -= load
                    prev = bins[b]
                if abs(bins[b]) < 1e-12:
                    break
            return False
        return dfs(0)

    # trivial
    if m == 0:
        return {i: {'route': ['DEPOT'], 'distance': 0.0} for i in range(1, num_drivers + 1)}

    left, right = lo, hi
    best_cap = right
    while right - left > 1e-4:
        mid = (left + right) / 2.0
        if can_pack(mid):
            best_cap = mid
            right = mid
        else:
            left = mid

    # reconstruct a feasible partition using first-fit decreasing (should succeed)
    bins = [{'load': 0.0, 'tasks': []} for _ in range(num_drivers)]
    for idx, load in tasks_sorted:
        placed = False
        order = sorted(range(num_drivers), key=lambda i: bins[i]['load'])
        for i in order:
            if bins[i]['load'] + load <= best_cap + 1e-9:
                bins[i]['load'] += load
                bins[i]['tasks'].append(idx)
                placed = True
                break
        if not placed:
            i = min(range(num_drivers), key=lambda k: bins[k]['load'])
            bins[i]['load'] += load
            bins[i]['tasks'].append(idx)

    # convert tasks back to sequences by concatenating mini_routes' paths
    final = {}
    for i in range(num_drivers):
        seq = []
        for t in bins[i]['tasks']:
            # append mini_routes[t]['path'] without trailing 'DEPOT'
            seq.extend(mini_routes[t]['path'][:-1])
        if seq:
            seq.append('DEPOT')
        else:
            seq = ['DEPOT']
        final[i+1] = {'route': seq, 'distance': bins[i]['load']}
    return final

# -------------------------
# GUI Application
# -------------------------
class DeliveryOptimizerApp:
    def __init__(self, master):
        self.master = master
        master.title("Delivery Route: Greedy (Efficiency) vs DP (Fairness)")

        # default parameters
        self.num_points_var = tk.IntVar(value=20)
        self.seed_var = tk.IntVar(value=0)
        self.num_drivers = tk.IntVar(value=4)
        self.max_route_dist = tk.DoubleVar(value=60.0)
        self.algorithm_choice = tk.StringVar(value="Greedy (Efficiency)")

        # generate initial points
        self.points_data, self.depot_coords = generate_sample_points(self.num_points_var.get(), seed=self.seed_var.get())

        self._build_ui()
        self._draw_initial_info()

    def _build_ui(self):
        # left: controls
        cframe = ttk.LabelFrame(self.master, text="Inputs (expected)", padding=8)
        cframe.grid(row=0, column=0, sticky="nw", padx=8, pady=8)

        ttk.Label(cframe, text="# Delivery Points:").grid(row=0, column=0, sticky='w')
        ttk.Entry(cframe, textvariable=self.num_points_var, width=8).grid(row=0, column=1, sticky='w', padx=4)

        ttk.Label(cframe, text="Random Seed:").grid(row=1, column=0, sticky='w')
        ttk.Entry(cframe, textvariable=self.seed_var, width=8).grid(row=1, column=1, sticky='w', padx=4)

        ttk.Label(cframe, text="# Drivers (K):").grid(row=2, column=0, sticky='w')
        ttk.Entry(cframe, textvariable=self.num_drivers, width=8).grid(row=2, column=1, sticky='w', padx=4)

        ttk.Label(cframe, text="Max Route Dist (L_max):").grid(row=3, column=0, sticky='w')
        ttk.Entry(cframe, textvariable=self.max_route_dist, width=8).grid(row=3, column=1, sticky='w', padx=4)

        ttk.Label(cframe, text="Algorithm:").grid(row=4, column=0, sticky='w', pady=(6,0))
        ttk.Combobox(cframe, textvariable=self.algorithm_choice, state='readonly',
                     values=["Greedy (Efficiency)", "DP Exact (Fairness)"]).grid(row=4, column=1, sticky='w', padx=4)

        ttk.Button(cframe, text="Regenerate Points", command=self._regenerate_points).grid(row=5, column=0, columnspan=2, pady=8)
        ttk.Button(cframe, text="Run", command=self._run).grid(row=6, column=0, columnspan=2, pady=8)
        ttk.Button(cframe, text="Export CSV", command=self._export_csv).grid(row=7, column=0, columnspan=2, pady=4)

        # right: outputs
        of = ttk.LabelFrame(self.master, text="Outputs (expected)", padding=8)
        of.grid(row=0, column=1, sticky="ne", padx=8, pady=8)

        self.total_label = ttk.Label(of, text="Total System Distance: N/A")
        self.total_label.grid(row=0, column=0, sticky='w')
        self.imbalance_label = ttk.Label(of, text="Load Imbalance (max-min): N/A")
        self.imbalance_label.grid(row=1, column=0, sticky='w')

        ttk.Separator(of, orient='horizontal').grid(row=2, column=0, sticky='ew', pady=6)

        # tree for per-driver route
        self.tree = ttk.Treeview(of, columns=('Distance', 'Route'), show='headings', height=12)
        self.tree.heading('Distance', text='Total Distance')
        self.tree.heading('Route', text='Assigned Route Sequence')
        self.tree.column('Distance', width=110, anchor='center')
        self.tree.column('Route', width=420)
        self.tree.grid(row=3, column=0, sticky='nsew')
        vsb = ttk.Scrollbar(of, orient="vertical", command=self.tree.yview)
        vsb.grid(row=3, column=1, sticky='ns')
        self.tree.configure(yscrollcommand=vsb.set)

        # small canvas to show depot & points (simple)
        self.canvas = tk.Canvas(self.master, width=640, height=360, bg='white')
        self.canvas.grid(row=1, column=0, columnspan=2, padx=8, pady=8)

    def _draw_initial_info(self):
        self._draw_map(None)
        self._update_points_label()

    def _update_points_label(self):
        pass  # could add a label showing count; already visible in inputs

    def _regenerate_points(self):
        try:
            n = int(self.num_points_var.get())
            seed = int(self.seed_var.get())
            if n <= 0:
                messagebox.showerror("Error", "Number of points must be > 0")
                return
            self.points_data, self.depot_coords = generate_sample_points(n, seed=seed)
            self.tree.delete(*self.tree.get_children())
            self.total_label.config(text="Total System Distance: N/A")
            self.imbalance_label.config(text="Load Imbalance (max-min): N/A")
            self._draw_map(None)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _run(self):
        try:
            num_d = int(self.num_drivers.get())
            max_d = float(self.max_route_dist.get())
            if num_d <= 0 or max_d <= 0:
                messagebox.showerror("Error", "Drivers and L_max must be positive")
                return

            mini_routes = generate_base_mini_routes(self.points_data, self.depot_coords, max_d)

            if self.algorithm_choice.get().startswith("Greedy"):
                driver_data = efficient_greedy_solver(mini_routes, num_d, self.points_data, self.depot_coords)
            else:
                dp_res = dp_exact_partition_solver(mini_routes, num_d, m_threshold=22)
                if dp_res is None:
                    messagebox.showwarning("DP skipped", f"DP exact skipped because #mini-routes ({len(mini_routes)}) > threshold (22). Falling back to greedy.")
                    driver_data = efficient_greedy_solver(mini_routes, num_d, self.points_data, self.depot_coords)
                else:
                    driver_data = dp_res

            self._update_ui(driver_data)
            # draw chosen solution on map (prefer driver_data)
            self._draw_map(driver_data)
        except Exception as e:
            messagebox.showerror("Runtime Error", str(e))

    def _update_ui(self, driver_data):
        self.tree.delete(*self.tree.get_children())
        distances = [d['distance'] for d in driver_data.values()] if driver_data else []
        if not distances:
            self.total_label.config(text="Total System Distance: N/A")
            self.imbalance_label.config(text="Load Imbalance (max-min): N/A")
            return
        total = sum(distances)
        mx = max(distances)
        mn = min(distances)
        imbalance = mx - mn
        self.total_label.config(text=f"Total System Distance: {total:.2f} units")
        self.imbalance_label.config(text=f"Load Imbalance (max-min): {imbalance:.2f} units")
        for i, data in driver_data.items():
            route_str = " -> ".join(data['route'])
            tag = ''
            if data['distance'] == mx and imbalance > 0:
                tag = 'max_load'
            elif data['distance'] == mn and imbalance > 0:
                tag = 'min_load'
            self.tree.insert('', tk.END, values=(f"{data['distance']:.2f}", route_str), tags=(tag,))
        self.tree.tag_configure('max_load', background='salmon')
        self.tree.tag_configure('min_load', background='lightgreen')

    def _draw_map(self, driver_data):
        self.canvas.delete('all')
        w = self.canvas.winfo_width() or 640
        h = self.canvas.winfo_height() or 360
        padding = 20
        all_lats = [p['lat'] for p in self.points_data.values()] + [self.depot_coords['lat']]
        all_lons = [p['lon'] for p in self.points_data.values()] + [self.depot_coords['lon']]
        min_lat, max_lat = min(all_lats), max(all_lats)
        min_lon, max_lon = min(all_lons), max(all_lons)
        def project(coord):
            lat, lon = coord['lat'], coord['lon']
            lat_span = max(1e-6, max_lat - min_lat)
            lon_span = max(1e-6, max_lon - min_lon)
            x = padding + ( (lon - min_lon) / lon_span ) * (w - 2*padding)
            y = padding + (1 - ( (lat - min_lat) / lat_span )) * (h - 2*padding)
            return x, y
        # points
        pos = {}
        for name, coord in self.points_data.items():
            x, y = project(coord)
            pos[name] = (x, y)
            self.canvas.create_oval(x-3, y-3, x+3, y+3, fill='black')
            self.canvas.create_text(x+6, y, text=name, anchor='w', font=('Arial', 8))
        # depot
        dx, dy = project(self.depot_coords)
        self.canvas.create_rectangle(dx-5, dy-5, dx+5, dy+5, fill='red')
        self.canvas.create_text(dx+8, dy-8, text='DEPOT', anchor='w', font=('Arial', 9, 'bold'))
        # draw routes if provided
        if driver_data:
            palette = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd','#8c564b','#e377c2','#7f7f7f','#bcbd22','#17becf']
            ids = sorted(driver_data.keys())
            for idx, d in enumerate(ids):
                seq = driver_data[d]['route']
                color = palette[idx % len(palette)]
                coords = []
                for node in seq:
                    if node == 'DEPOT':
                        coords.append((dx, dy))
                    else:
                        coords.append(pos[node])
                flat = []
                for (x,y) in coords:
                    flat.extend([x,y])
                if len(flat) >= 4:
                    self.canvas.create_line(*flat, fill=color, width=2, smooth=True)
                    # label
                    if len(coords) > 1:
                        px, py = coords[1]
                        self.canvas.create_text(px, py-10, text=f'D{d}', anchor='s', fill=color, font=('Arial',9,'bold'))
        # footnote
        self.canvas.create_text(8, h-8, text=f"Points: {len(self.points_data)}  Drivers: {self.num_drivers.get()}  L_max: {self.max_route_dist.get():.1f}", anchor='w', font=('Arial', 9, 'italic'))

    def _export_csv(self):
        if not self.tree.get_children():
            messagebox.showinfo("No data", "Run an optimization first")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV","*.csv")])
        if not path:
            return
        # export tree rows
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['DriverID', 'Distance', 'Route'])
            for i, item in enumerate(self.tree.get_children(), start=1):
                vals = self.tree.item(item)['values']
                writer.writerow([i, vals[0], vals[1]])
        messagebox.showinfo("Saved", f"Exported to {path}")

# -------------------------
# Run
# -------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = DeliveryOptimizerApp(root)
    root.mainloop()
