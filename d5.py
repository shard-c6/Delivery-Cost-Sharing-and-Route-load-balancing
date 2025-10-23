# import tkinter as tk
# from tkinter import ttk, messagebox, filedialog
# import math
# import random
# import csv
# import datetime

# # -------------------------
# # Utilities
# # -------------------------
# def calculate_distance(p1, p2):
#     return math.sqrt((p1['lat'] - p2['lat'])**2 + (p1['lon'] - p2['lon'])**2)

# def generate_sample_points(n, seed=None, bounds=(1, 10)):
#     """Generate n random points and a central depot. Optional seed for reproducibility."""
#     if seed is not None:
#         random.seed(seed)
#     low, high = bounds
#     pts = {}
#     for i in range(1, n + 1):
#         pts[f'P{i}'] = {'lat': random.uniform(low, high), 'lon': random.uniform(low, high)}
#     return pts, {'lat': (low + high) / 2.0, 'lon': (low + high) / 2.0}

# def compute_sequence_distance(seq, points_data, depot_coords):
#     if not seq:
#         return 0.0
#     total = 0.0
#     prev = depot_coords
#     for node in seq:
#         curr = depot_coords if node == 'DEPOT' else points_data[node]
#         total += calculate_distance(prev, curr)
#         prev = curr
#     return total

# # -------------------------
# # Mini-route generation (unchanged logic)
# # -------------------------
# def generate_base_mini_routes(points_data, depot_coords, max_d):
#     unassigned = set(points_data.keys())
#     mini_routes = []

#     def route_len(seq_no_depot):
#         if not seq_no_depot:
#             return 0.0
#         prev = depot_coords
#         s = 0.0
#         for n in seq_no_depot:
#             s += calculate_distance(prev, points_data[n])
#             prev = points_data[n]
#         s += calculate_distance(prev, depot_coords)
#         return s

#     while unassigned:
#         seed = min(unassigned, key=lambda p: calculate_distance(depot_coords, points_data[p]))
#         seq = [seed]
#         unassigned.remove(seed)

#         improved = True
#         while improved:
#             improved = False
#             last = points_data[seq[-1]]
#             best = None
#             best_d = float('inf')
#             for p in list(unassigned):
#                 d = calculate_distance(last, points_data[p])
#                 if route_len(seq + [p]) <= max_d and d < best_d:
#                     best = p
#                     best_d = d
#             if best:
#                 seq.append(best)
#                 unassigned.remove(best)
#                 improved = True

#         load = route_len(seq)
#         mini_routes.append({'path': ['DEPOT'] + seq + ['DEPOT'], 'load': load})

#     return mini_routes

# # -------------------------
# # Solvers
# # -------------------------
# def efficient_greedy_solver(mini_routes, num_drivers, points_data, depot_coords):
#     """Assign each mini-route to the driver causing smallest marginal increase in that driver's route (fuel-efficient)."""
#     drivers = {i: {'seq': ['DEPOT'], 'load': 0.0} for i in range(1, num_drivers + 1)}

#     def marginal_inc(driver_seq, mini_path):
#         old_seq = driver_seq[:] if driver_seq[-1] == 'DEPOT' else driver_seq[:] + ['DEPOT']
#         base = driver_seq[:] if driver_seq[-1] != 'DEPOT' else driver_seq[:-1]
#         add_nodes = [n for n in mini_path if n != 'DEPOT']
#         new_seq = base + add_nodes + ['DEPOT']
#         old_d = compute_sequence_distance(old_seq, points_data, depot_coords)
#         new_d = compute_sequence_distance(new_seq, points_data, depot_coords)
#         return new_d - old_d

#     for task in mini_routes:
#         best_driver, best_inc = None, float('inf')
#         for d_id, d in drivers.items():
#             inc = marginal_inc(d['seq'], task['path'])
#             if inc < best_inc:
#                 best_inc = inc
#                 best_driver = d_id
#         # assign to best_driver
#         d = drivers[best_driver]
#         if d['seq'][-1] == 'DEPOT':
#             base = d['seq'][:-1]
#         else:
#             base = d['seq'][:]
#         base.extend([n for n in task['path'] if n != 'DEPOT'])
#         base.append('DEPOT')
#         d['seq'] = base
#         d['load'] = compute_sequence_distance(d['seq'], points_data, depot_coords)

#     final = {}
#     for i, d in drivers.items():
#         final[i] = {'route': d['seq'] if d['seq'] else ['DEPOT'], 'distance': d['load']}
#     return final

# def dp_exact_partition_solver(mini_routes, num_drivers, m_threshold=22):
#     """Binary-search makespan + DFS feasibility. Returns None if too many mini-routes (fallback)."""
#     loads = [r['load'] for r in mini_routes]
#     tasks = list(enumerate(loads))
#     m = len(tasks)
#     if m > m_threshold:
#         return None

#     total = sum(loads)
#     lo = max(loads) if loads else 0.0
#     hi = total
#     tasks_sorted = sorted(tasks, key=lambda x: x[1], reverse=True)

#     def can_pack(cap):
#         bins = [0.0] * num_drivers
#         seen = set()
#         def dfs(idx):
#             if idx == m:
#                 return True
#             key = (idx, tuple(sorted(bins)))
#             if key in seen:
#                 return False
#             seen.add(key)
#             load = tasks_sorted[idx][1]
#             prev = -1.0
#             for b in range(num_drivers):
#                 if bins[b] == prev:
#                     continue
#                 if bins[b] + load <= cap + 1e-9:
#                     bins[b] += load
#                     if dfs(idx + 1):
#                         return True
#                     bins[b] -= load
#                     prev = bins[b]
#                 if abs(bins[b]) < 1e-12:
#                     break
#             return False
#         return dfs(0)

#     if m == 0:
#         return {i: {'route': ['DEPOT'], 'distance': 0.0} for i in range(1, num_drivers + 1)}

#     left, right = lo, hi
#     best_cap = right
#     while right - left > 1e-4:
#         mid = (left + right) / 2.0
#         if can_pack(mid):
#             best_cap = mid
#             right = mid
#         else:
#             left = mid

#     # Reconstruct one feasible partition (first-fit decreasing)
#     bins = [{'load': 0.0, 'tasks': []} for _ in range(num_drivers)]
#     for idx, load in tasks_sorted:
#         placed = False
#         for i in sorted(range(num_drivers), key=lambda x: bins[x]['load']):
#             if bins[i]['load'] + load <= best_cap + 1e-9:
#                 bins[i]['load'] += load
#                 bins[i]['tasks'].append(idx)
#                 placed = True
#                 break
#         if not placed:
#             i = min(range(num_drivers), key=lambda k: bins[k]['load'])
#             bins[i]['load'] += load
#             bins[i]['tasks'].append(idx)

#     final = {}
#     for i in range(num_drivers):
#         seq = []
#         for t in bins[i]['tasks']:
#             seq.extend(mini_routes[t]['path'][:-1])
#         if seq:
#             seq.append('DEPOT')
#         else:
#             seq = ['DEPOT']
#         final[i+1] = {'route': seq, 'distance': bins[i]['load']}
#     return final

# # -------------------------
# # Styled Tkinter UI (aligned cards)
# # -------------------------
# class DeliveryOptimizerApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Delivery Route: Greedy vs DP (Mapped & Polished)")

#         # Use a clean ttk theme if available
#         style = ttk.Style()
#         try:
#             style.theme_use('clam')
#         except Exception:
#             pass
#         style.configure('Card.TLabelframe', padding=12, relief='raised')
#         style.configure('Card.TButton', padding=6)
#         style.configure('Header.TLabel', font=('Segoe UI', 10, 'bold'))
#         style.configure('Small.TLabel', font=('Segoe UI', 9))

#         # Variables
#         self.num_points_var = tk.IntVar(value=20)
#         self.seed_var = tk.IntVar(value=0)
#         self.num_drivers = tk.IntVar(value=4)
#         self.max_route_dist = tk.DoubleVar(value=60.0)
#         self.algorithm_choice = tk.StringVar(value="Greedy (Efficiency)")

#         # Data
#         self.points_data, self.depot_coords = generate_sample_points(self.num_points_var.get(), seed=self.seed_var.get())

#         self._build_layout()
#         self._draw_map(None)

#     def _build_layout(self):
#         # Top-level grid: left controls, right outputs + canvas
#         self.master.columnconfigure(0, weight=0)
#         self.master.columnconfigure(1, weight=1)
#         self.master.rowconfigure(1, weight=1)

#         # Controls card (left)
#         ctrl = ttk.LabelFrame(self.master, text="Inputs (expected)", style='Card.TLabelframe')
#         ctrl.grid(row=0, column=0, rowspan=2, sticky="nw", padx=12, pady=12)

#         # Use an internal grid with consistent padding
#         for r in range(8):
#             ctrl.rowconfigure(r, pad=6)
#         ctrl.columnconfigure(0, weight=1)
#         ctrl.columnconfigure(1, weight=1)

#         ttk.Label(ctrl, text="# Delivery Points:", style='Small.TLabel').grid(row=0, column=0, sticky='w')
#         ttk.Entry(ctrl, textvariable=self.num_points_var, width=8).grid(row=0, column=1, sticky='e')

#         ttk.Label(ctrl, text="Random Seed:", style='Small.TLabel').grid(row=1, column=0, sticky='w')
#         ttk.Entry(ctrl, textvariable=self.seed_var, width=8).grid(row=1, column=1, sticky='e')

#         ttk.Label(ctrl, text="# Drivers (K):", style='Small.TLabel').grid(row=2, column=0, sticky='w')
#         ttk.Entry(ctrl, textvariable=self.num_drivers, width=8).grid(row=2, column=1, sticky='e')

#         ttk.Label(ctrl, text="Max Route Dist (L_max):", style='Small.TLabel').grid(row=3, column=0, sticky='w')
#         ttk.Entry(ctrl, textvariable=self.max_route_dist, width=8).grid(row=3, column=1, sticky='e')

#         ttk.Label(ctrl, text="Algorithm:", style='Small.TLabel').grid(row=4, column=0, sticky='w')
#         algo_combobox = ttk.Combobox(ctrl, textvariable=self.algorithm_choice, state='readonly',
#                      values=["Greedy (Efficiency)", "DP Exact (Fairness)", "Compare Both"])
#         algo_combobox.grid(row=4, column=1, sticky='e')
#         algo_combobox.current(0)

#         # brief explanation of the seed shown inline
#         ttk.Label(ctrl, text="Seed controls randomness for repeatable point sets.", style='Small.TLabel', wraplength=180).grid(row=5, column=0, columnspan=2, sticky='w')

#         ttk.Button(ctrl, text="Regenerate Points", style='Card.TButton', command=self._regenerate_points).grid(row=6, column=0, columnspan=2, sticky='ew')
#         ttk.Button(ctrl, text="Run", style='Card.TButton', command=self._run).grid(row=7, column=0, sticky='ew', pady=(6,0))
#         ttk.Button(ctrl, text="Export CSV", style='Card.TButton', command=self._export_csv).grid(row=7, column=1, sticky='ew', pady=(6,0))

#         # Output card (right, top)
#         out = ttk.LabelFrame(self.master, text="Outputs (expected)", style='Card.TLabelframe')
#         out.grid(row=0, column=1, sticky="new", padx=12, pady=12)
#         out.columnconfigure(0, weight=1)

#         self.total_label = ttk.Label(out, text="Total System Distance: N/A", style='Header.TLabel')
#         self.total_label.grid(row=0, column=0, sticky='w', padx=4, pady=(0,2))
#         self.imbalance_label = ttk.Label(out, text="Load Imbalance (max-min): N/A", style='Header.TLabel')
#         self.imbalance_label.grid(row=1, column=0, sticky='w', padx=4, pady=(0,8))

#         ttk.Separator(out, orient='horizontal').grid(row=2, column=0, sticky='ew', pady=6)

#         self.tree = ttk.Treeview(out, columns=('Distance', 'Route'), show='headings', height=8)
#         self.tree.heading('Distance', text='Total Distance')
#         self.tree.heading('Route', text='Assigned Route Sequence')
#         self.tree.column('Distance', width=120, anchor='center')
#         self.tree.column('Route', width=600)
#         self.tree.grid(row=3, column=0, sticky='nsew')
#         out.rowconfigure(3, weight=1)

#         vsb = ttk.Scrollbar(out, orient="vertical", command=self.tree.yview)
#         vsb.grid(row=3, column=1, sticky='ns')
#         self.tree.configure(yscrollcommand=vsb.set)

#         # Map card (right, bottom)
#         map_card = ttk.LabelFrame(self.master, text="Map (visual)", style='Card.TLabelframe')
#         map_card.grid(row=1, column=1, sticky="nsew", padx=12, pady=(0,12))
#         map_card.columnconfigure(0, weight=1)
#         map_card.rowconfigure(0, weight=1)

#         self.canvas = tk.Canvas(map_card, width=800, height=420, bg='white')
#         self.canvas.grid(row=0, column=0, sticky='nsew')

#     # -------------------------
#     # Actions
#     # -------------------------
#     def _regenerate_points(self):
#         try:
#             n = int(self.num_points_var.get())
#             seed = int(self.seed_var.get())
#             if n <= 0:
#                 messagebox.showerror("Error", "Number of points must be > 0")
#                 return
#             self.points_data, self.depot_coords = generate_sample_points(n, seed=seed)
#             self.tree.delete(*self.tree.get_children())
#             self.total_label.config(text="Total System Distance: N/A")
#             self.imbalance_label.config(text="Load Imbalance (max-min): N/A")
#             self._draw_map(None)
#             messagebox.showinfo("Regenerated", f"Generated {n} points (seed={seed}).")
#         except Exception as e:
#             messagebox.showerror("Error", str(e))

#     def _run(self):
#         try:
#             num_d = int(self.num_drivers.get())
#             max_d = float(self.max_route_dist.get())
#             if num_d <= 0 or max_d <= 0:
#                 messagebox.showerror("Error", "Drivers and L_max must be positive")
#                 return

#             mini_routes = generate_base_mini_routes(self.points_data, self.depot_coords, max_d)

#             choice = self.algorithm_choice.get()
#             # --- explicit and correct mapping ---
#             if choice == "Greedy (Efficiency)":
#                 driver_data = efficient_greedy_solver(mini_routes, num_d, self.points_data, self.depot_coords)
#                 # show single result
#                 self._update_ui(driver_data, label_suffix="(Greedy)")
#                 self._draw_map(driver_data)
#             elif choice == "DP Exact (Fairness)":
#                 dp_res = dp_exact_partition_solver(mini_routes, num_d, m_threshold=22)
#                 if dp_res is None:
#                     messagebox.showwarning("DP skipped", f"DP exact skipped because #mini-routes ({len(mini_routes)}) > threshold (22). Falling back to Greedy.")
#                     driver_data = efficient_greedy_solver(mini_routes, num_d, self.points_data, self.depot_coords)
#                     self._update_ui(driver_data, label_suffix="(Greedy-fallback)")
#                     self._draw_map(driver_data)
#                 else:
#                     self._update_ui(dp_res, label_suffix="(DP)")
#                     self._draw_map(dp_res)
#             elif choice == "Compare Both":
#                 greedy_res = efficient_greedy_solver(mini_routes, num_d, self.points_data, self.depot_coords)
#                 dp_res = dp_exact_partition_solver(mini_routes, num_d, m_threshold=22)
#                 if dp_res is None:
#                     # show greedy and LPT fallback as DP wasn't feasible
#                     messagebox.showwarning("DP skipped for Compare", f"DP exact skipped (#{len(mini_routes)} > threshold). Showing Greedy vs Greedy-fallback.")
#                     dp_res = efficient_greedy_solver(mini_routes, num_d, self.points_data, self.depot_coords)
#                 # Present both results by opening a small comparison window
#                 self._show_comparison_window(greedy_res, dp_res)
#                 # draw preferred (DP if available else greedy)
#                 pref = dp_res if dp_res is not None else greedy_res
#                 self._draw_map(pref)
#             else:
#                 messagebox.showerror("Error", "Unknown algorithm selection.")
#         except Exception as e:
#             messagebox.showerror("Runtime Error", str(e))

#     def _update_ui(self, driver_data, label_suffix=""):
#         self.tree.delete(*self.tree.get_children())
#         distances = [d['distance'] for d in driver_data.values()] if driver_data else []
#         if not distances:
#             self.total_label.config(text="Total System Distance: N/A")
#             self.imbalance_label.config(text="Load Imbalance (max-min): N/A")
#             return
#         total = sum(distances)
#         mx = max(distances)
#         mn = min(distances)
#         imbalance = mx - mn
#         self.total_label.config(text=f"Total System Distance: {total:.2f} units {label_suffix}")
#         self.imbalance_label.config(text=f"Load Imbalance (max-min): {imbalance:.2f} units")

#         for i, data in driver_data.items():
#             route_str = " -> ".join(data['route'])
#             tag = ''
#             if data['distance'] == mx and imbalance > 0:
#                 tag = 'max_load'
#             elif data['distance'] == mn and imbalance > 0:
#                 tag = 'min_load'
#             self.tree.insert('', tk.END, iid=str(i), values=(f"{data['distance']:.2f}", route_str), tags=(tag,))
#         self.tree.tag_configure('max_load', background='salmon')
#         self.tree.tag_configure('min_load', background='lightgreen')

#     def _draw_map(self, driver_data):
#         self.canvas.delete('all')
#         w = self.canvas.winfo_width() or 800
#         h = self.canvas.winfo_height() or 420
#         padding = 20
#         all_lats = [p['lat'] for p in self.points_data.values()] + [self.depot_coords['lat']]
#         all_lons = [p['lon'] for p in self.points_data.values()] + [self.depot_coords['lon']]
#         min_lat, max_lat = min(all_lats), max(all_lats)
#         min_lon, max_lon = min(all_lons), max(all_lons)
#         def project(coord):
#             lat, lon = coord['lat'], coord['lon']
#             lat_span = max(1e-6, max_lat - min_lat)
#             lon_span = max(1e-6, max_lon - min_lon)
#             x = padding + ( (lon - min_lon) / lon_span ) * (w - 2*padding)
#             y = padding + (1 - ( (lat - min_lat) / lat_span )) * (h - 2*padding)
#             return x, y

#         pos = {}
#         for name, coord in self.points_data.items():
#             x, y = project(coord)
#             pos[name] = (x, y)
#             self.canvas.create_oval(x-3, y-3, x+3, y+3, fill='black')
#             self.canvas.create_text(x+6, y, text=name, anchor='w', font=('Segoe UI',8))

#         dx, dy = project(self.depot_coords)
#         self.canvas.create_rectangle(dx-5, dy-5, dx+5, dy+5, fill='red')
#         self.canvas.create_text(dx+8, dy-8, text='DEPOT', anchor='w', font=('Segoe UI',9,'bold'))

#         if driver_data:
#             palette = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd','#8c564b','#e377c2','#7f7f7f','#bcbd22','#17becf']
#             ids = sorted(driver_data.keys())
#             for idx, d in enumerate(ids):
#                 seq = driver_data[d]['route']
#                 color = palette[idx % len(palette)]
#                 coords = []
#                 for node in seq:
#                     coords.append((dx, dy) if node == 'DEPOT' else pos[node])
#                 flat = []
#                 for (x,y) in coords:
#                     flat.extend([x,y])
#                 if len(flat) >= 4:
#                     self.canvas.create_line(*flat, fill=color, width=2, smooth=True)
#                     if len(coords) > 1:
#                         px, py = coords[1]
#                         self.canvas.create_text(px, py-10, text=f'D{d}', anchor='s', fill=color, font=('Segoe UI',9,'bold'))
#         self.canvas.create_text(8, h-8, text=f"Points: {len(self.points_data)}  Drivers: {self.num_drivers.get()}  L_max: {self.max_route_dist.get():.1f}", anchor='w', font=('Segoe UI',9,'italic'))

#     def _show_comparison_window(self, greedy_res, dp_res):
#         # small two-column window to compare metrics side-by-side
#         win = tk.Toplevel(self.master)
#         win.title("Compare: Greedy vs DP")
#         win.geometry("820x380")
#         left = ttk.LabelFrame(win, text="Greedy (Efficiency)", padding=8)
#         left.place(x=8, y=8, width=400, height=360)
#         right = ttk.LabelFrame(win, text="DP Exact (Fairness)", padding=8)
#         right.place(x=412, y=8, width=400, height=360)

#         def populate(frame, data):
#             dists = [v['distance'] for v in data.values()]
#             total = sum(dists)
#             mx = max(dists) if dists else 0.0
#             mn = min(dists) if dists else 0.0
#             ttk.Label(frame, text=f"Total Distance: {total:.2f}").pack(anchor='w')
#             ttk.Label(frame, text=f"Max: {mx:.2f}  Min: {mn:.2f}  Imbalance: {mx-mn:.2f}").pack(anchor='w', pady=(0,6))
#             tv = ttk.Treeview(frame, columns=('D','R'), show='headings', height=12)
#             tv.heading('D', text='Distance')
#             tv.heading('R', text='Route')
#             tv.column('D', width=80, anchor='center')
#             tv.column('R', width=280)
#             tv.pack(fill='both', expand=True)
#             for i, (k, v) in enumerate(sorted(data.items())):
#                 tv.insert('', tk.END, values=(f"{v['distance']:.2f}", " -> ".join(v['route'])))

#         populate(left, greedy_res)
#         populate(right, dp_res)

#     def _export_csv(self):
#         if not self.tree.get_children():
#             messagebox.showinfo("No data", "Run an optimization first")
#             return
#         path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV","*.csv")])
#         if not path:
#             return
#         with open(path, 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(['DriverID', 'Distance', 'Route'])
#             for iid in self.tree.get_children():
#                 vals = self.tree.item(iid)['values']
#                 writer.writerow([iid, vals[0], vals[1]])
#         messagebox.showinfo("Saved", f"Exported to {path}")

# # -------------------------
# # Run
# # -------------------------
# if __name__ == "__main__":
#     root = tk.Tk()
#     app = DeliveryOptimizerApp(root)
#     root.mainloop()







#----------------------------------------------------------------------------------------

#-----------------------------------------------------------------------------------------

