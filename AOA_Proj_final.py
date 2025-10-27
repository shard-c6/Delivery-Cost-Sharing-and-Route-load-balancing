import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math, random, csv, sys, traceback

# ---------------- Utilities ----------------
def calculate_distance(p1, p2):
    return math.sqrt((p1['lat'] - p2['lat'])**2 + (p1['lon'] - p2['lon'])**2)

def generate_sample_points(n, seed=None, bounds=(1,10)):
    """Generate n points and a central depot. Seed -> repeatable experiments."""
    if seed is not None:
        random.seed(seed)
    low, high = bounds
    pts = {}
    for i in range(1, n+1):
        pts[f'P{i}'] = {'lat': random.uniform(low, high), 'lon': random.uniform(low, high)}
    depot = {'lat': (low+high)/2.0, 'lon': (low+high)/2.0}
    return pts, depot

def compute_sequence_distance(seq, points_data, depot_coords):
    """Distance of a full sequence like ['DEPOT','P3','P5','DEPOT']."""
    if not seq:
        return 0.0
    total = 0.0
    prev = depot_coords
    for node in seq:
        curr = depot_coords if node == 'DEPOT' else points_data[node]
        total += calculate_distance(prev, curr)
        prev = curr
    return total

def nearest_neighbor_sequence(point_names, points_data, depot_coords):
    """Order given points from depot using a NN heuristic, return ['DEPOT',..., 'DEPOT']."""
    if not point_names:
        return ['DEPOT']
    remaining = set(point_names)
    seq = ['DEPOT']
    current = depot_coords
    while remaining:
        best = None
        best_d = float('inf')
        for p in remaining:
            d = calculate_distance(current, points_data[p])
            if d < best_d:
                best = p
                best_d = d
        seq.append(best)
        remaining.remove(best)
        current = points_data[best]
    seq.append('DEPOT')
    return seq

# ---------------- Greedy route builder (Efficiency) ----------------
def route_building_greedy(points_data, depot_coords, num_drivers, max_d):
    """
    Build K routes directly: each driver keeps taking the nearest feasible unassigned point
    while route + return-to-depot <= L_max. Any leftover points are appended to the
    least-loaded driver (may exceed L_max in the tail).
    """
    unassigned = set(points_data.keys())
    drivers = {i: {'route': ['DEPOT'], 'distance': 0.0} for i in range(1, num_drivers+1)}

    for d in range(1, num_drivers+1):
        seq = ['DEPOT']
        curr_dist = 0.0
        last = depot_coords
        while True:
            best = None
            best_d = float('inf')
            for p in list(unassigned):
                d_to_p = calculate_distance(last, points_data[p])
                p_to_depot = calculate_distance(points_data[p], depot_coords)
                if curr_dist + d_to_p + p_to_depot <= max_d:
                    if d_to_p < best_d:
                        best = p
                        best_d = d_to_p
            if best is None:
                break
            seq.append(best)
            curr_dist += best_d
            last = points_data[best]
            unassigned.remove(best)
        if len(seq) > 1 and seq[-1] != 'DEPOT':
            seq.append('DEPOT')
        drivers[d]['route'] = seq
        drivers[d]['distance'] = compute_sequence_distance(seq, points_data, depot_coords)
        if not unassigned:
            break

    # append any remaining to least-loaded drivers (keeps outputs different from DP)
    while unassigned:
        p = unassigned.pop()
        k = min(drivers.keys(), key=lambda i: drivers[i]['distance'])
        r = drivers[k]['route']
        if r[-1] == 'DEPOT':
            r = r[:-1] + [p, 'DEPOT']
        else:
            r = r + [p, 'DEPOT']
        drivers[k]['route'] = r
        drivers[k]['distance'] = compute_sequence_distance(r, points_data, depot_coords)

    return drivers

# ---------------- DP (Fairness) on POINTS ONLY ----------------
def _dp_find_bins(loads, num_drivers, m_threshold=32):
    """
    Exact k-partition (minimize makespan) via binary search on cap + DFS feasibility.
    Returns bins [{'load':float,'tasks':[idx,...]}, ...] or None if too many tasks.
    """
    tasks = list(enumerate(loads))
    m = len(tasks)
    if m > m_threshold:
        return None
    total = sum(loads)
    lo = max(loads) if loads else 0.0
    hi = total
    tasks_sorted = sorted(tasks, key=lambda x: x[1], reverse=True)

    def can_pack(cap):
        bins = [0.0]*num_drivers
        seen = set()
        def dfs(i):
            if i == m:
                return True
            key = (i, tuple(sorted(bins)))
            if key in seen:
                return False
            seen.add(key)
            w = tasks_sorted[i][1]
            prev = -1.0
            for b in range(num_drivers):
                if bins[b] == prev:
                    continue
                if bins[b] + w <= cap + 1e-9:
                    bins[b] += w
                    if dfs(i+1):
                        return True
                    bins[b] -= w
                    prev = bins[b]
                if abs(bins[b]) < 1e-12:
                    break
            return False
        return dfs(0)

    if m == 0:
        return [{'load':0.0,'tasks':[]} for _ in range(num_drivers)]

    left, right = lo, hi
    best = right
    while right - left > 1e-4:
        mid = (left + right)/2.0
        if can_pack(mid):
            best = mid
            right = mid
        else:
            left = mid

    # build feasible bins with first-fit decreasing under 'best'
    bins = [{'load':0.0,'tasks':[]} for _ in range(num_drivers)]
    for idx, w in tasks_sorted:
        placed = False
        for b in sorted(range(num_drivers), key=lambda x: bins[x]['load']):
            if bins[b]['load'] + w <= best + 1e-9:
                bins[b]['load'] += w
                bins[b]['tasks'].append(idx)
                placed = True
                break
        if not placed:
            b = min(range(num_drivers), key=lambda x: bins[x]['load'])
            bins[b]['load'] += w
            bins[b]['tasks'].append(idx)
    return bins

def _lpt_bins(loads, num_drivers):
    """Largest Processing Time heuristic: fast fallback when exact DP is too big."""
    order = sorted(list(enumerate(loads)), key=lambda x: x[1], reverse=True)
    bins = [{'load':0.0,'tasks':[]} for _ in range(num_drivers)]
    for idx, w in order:
        b = min(range(num_drivers), key=lambda x: bins[x]['load'])
        bins[b]['load'] += w
        bins[b]['tasks'].append(idx)
    return bins

def dp_fairness_on_points(points_data, depot_coords, num_drivers, m_threshold=32):
    """
    Fairness objective: minimize max driver load by partitioning points (atomic tasks).
    Each point's atomic load = round-trip to depot (depot->point->depot).
    After partitioning, order each driver's assigned points with NN and compute true distance.
    """
    point_names = sorted(points_data.keys())
    loads = [2.0 * calculate_distance(depot_coords, points_data[name]) for name in point_names]

    bins = _dp_find_bins(loads, num_drivers, m_threshold=m_threshold)
    exact_used = True
    if bins is None:  # too many tasks; use LPT fallback (still point-based, so outputs differ)
        bins = _lpt_bins(loads, num_drivers)
        exact_used = False

    result = {}
    for i in range(num_drivers):
        pts = [point_names[t] for t in bins[i]['tasks']]
        seq = nearest_neighbor_sequence(pts, points_data, depot_coords)
        dist = compute_sequence_distance(seq, points_data, depot_coords)
        result[i+1] = {'route': seq, 'distance': dist}

    return result, exact_used

# ---------------- Polished Tkinter UI (cards aligned) ----------------
class DeliveryOptimizerApp:
    def __init__(self, root):
        self.root = root
        root.title("Delivery Route: Greedy (Efficiency) vs DP (Fairness) — Points-only")
        root.geometry("1080x720")

        style = ttk.Style()
        try:
            style.theme_use('clam')
        except:
            pass
        style.configure('Card.TLabelframe', padding=10)
        style.configure('Header.TLabel', font=('Segoe UI', 10, 'bold'))
        style.configure('Small.TLabel', font=('Segoe UI', 9))

        # Inputs
        self.num_points_var = tk.IntVar(value=12)
        self.seed_var = tk.IntVar(value=0)
        self.num_drivers = tk.IntVar(value=4)
        self.max_route_dist = tk.DoubleVar(value=60.0)
        self.algorithm_choice = tk.StringVar(value="Compare Both")  # default to side-by-side

        # Data
        self.points_data, self.depot_coords = generate_sample_points(self.num_points_var.get(), seed=self.seed_var.get())

        self._build_layout()
        self._draw_map(None)

    def _build_layout(self):
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)

        # Controls card
        ctrl = ttk.LabelFrame(self.root, text="Inputs (expected)", style='Card.TLabelframe')
        ctrl.grid(row=0, column=0, rowspan=2, sticky='nw', padx=12, pady=12)
        for r in range(8):
            ctrl.rowconfigure(r, pad=6)
        ctrl.columnconfigure(0, weight=1)
        ctrl.columnconfigure(1, weight=1)

        ttk.Label(ctrl, text="# Delivery Points:", style='Small.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Entry(ctrl, textvariable=self.num_points_var, width=8).grid(row=0, column=1, sticky='e')

        ttk.Label(ctrl, text="Random Seed:", style='Small.TLabel').grid(row=1, column=0, sticky='w')
        ttk.Entry(ctrl, textvariable=self.seed_var, width=8).grid(row=1, column=1, sticky='e')
        ttk.Label(ctrl, text="(Seed = repeatable experiments)", style='Small.TLabel').grid(row=2, column=0, columnspan=2, sticky='w')

        ttk.Label(ctrl, text="# Drivers (K):", style='Small.TLabel').grid(row=3, column=0, sticky='w')
        ttk.Entry(ctrl, textvariable=self.num_drivers, width=8).grid(row=3, column=1, sticky='e')

        ttk.Label(ctrl, text="Max Route Dist (L_max):", style='Small.TLabel').grid(row=4, column=0, sticky='w')
        ttk.Entry(ctrl, textvariable=self.max_route_dist, width=8).grid(row=4, column=1, sticky='e')

        ttk.Label(ctrl, text="Algorithm:", style='Small.TLabel').grid(row=5, column=0, sticky='w')
        algo = ttk.Combobox(ctrl, textvariable=self.algorithm_choice, state='readonly',
                            values=["Greedy (Efficiency)", "DP Exact (Fairness)", "Compare Both"])
        algo.grid(row=5, column=1, sticky='e')
        algo.current(2)

        ttk.Button(ctrl, text="Regenerate Points", command=self._regenerate_points).grid(row=6, column=0, columnspan=2, sticky='ew')
        ttk.Button(ctrl, text="Run", command=self._run).grid(row=7, column=0, sticky='ew')
        ttk.Button(ctrl, text="Export CSV", command=self._export_csv).grid(row=7, column=1, sticky='ew')

        # Outputs card
        out = ttk.LabelFrame(self.root, text="Outputs (expected)", style='Card.TLabelframe')
        out.grid(row=0, column=1, sticky='new', padx=12, pady=12)
        out.columnconfigure(0, weight=1)

        self.total_label = ttk.Label(out, text="Total System Distance: N/A", style='Header.TLabel')
        self.total_label.grid(row=0, column=0, sticky='w')
        self.imbalance_label = ttk.Label(out, text="Load Imbalance (max-min): N/A", style='Header.TLabel')
        self.imbalance_label.grid(row=1, column=0, sticky='w')

        ttk.Separator(out, orient='horizontal').grid(row=2, column=0, sticky='ew', pady=8)

        self.tree = ttk.Treeview(out, columns=('Distance','Route'), show='headings', height=8)
        self.tree.heading('Distance', text='Total Distance')
        self.tree.heading('Route', text='Assigned Route Sequence')
        self.tree.column('Distance', width=120, anchor='center')
        self.tree.column('Route', width=700)
        self.tree.grid(row=3, column=0, sticky='nsew')
        vsb = ttk.Scrollbar(out, orient='vertical', command=self.tree.yview)
        vsb.grid(row=3, column=1, sticky='ns')
        self.tree.configure(yscrollcommand=vsb.set)

        # Map card
        map_card = ttk.LabelFrame(self.root, text="Map (visual)", style='Card.TLabelframe')
        map_card.grid(row=1, column=1, sticky='nsew', padx=12, pady=(0,12))
        map_card.columnconfigure(0, weight=1)
        map_card.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(map_card, bg='white', width=800, height=420)
        self.canvas.grid(row=0, column=0, sticky='nsew')

    def _regenerate_points(self):
        try:
            n = int(self.num_points_var.get())
            seed = int(self.seed_var.get())
            if n <= 0: raise ValueError("Number of points must be > 0")
            self.points_data, self.depot_coords = generate_sample_points(n, seed=seed)
            self.tree.delete(*self.tree.get_children())
            self.total_label.config(text="Total System Distance: N/A")
            self.imbalance_label.config(text="Load Imbalance (max-min): N/A")
            self._draw_map(None)
            messagebox.showinfo("Regenerated", f"Generated {n} points (seed={seed}).")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _run(self):
        try:
            K = int(self.num_drivers.get())
            Lmax = float(self.max_route_dist.get())
            if K <= 0 or Lmax <= 0:
                messagebox.showerror("Error", "Drivers and L_max must be positive")
                return

            choice = self.algorithm_choice.get()
            if choice == "Greedy (Efficiency)":
                res = route_building_greedy(self.points_data, self.depot_coords, K, Lmax)
                self._update_ui(res, "(Greedy)")
                self._draw_map(res)

            elif choice == "DP Exact (Fairness)":
                res, exact_used = dp_fairness_on_points(self.points_data, self.depot_coords, K, m_threshold=32)
                suffix = "(DP exact)" if exact_used else "(DP LPT fallback)"
                self._update_ui(res, suffix)
                self._draw_map(res)

            else:  # Compare Both
                greedy = route_building_greedy(self.points_data, self.depot_coords, K, Lmax)
                dpres, exact_used = dp_fairness_on_points(self.points_data, self.depot_coords, K, m_threshold=32)
                self._show_comparison_window(greedy, dpres, exact_used)
                # draw DP by default (fairness view)
                self._draw_map(dpres)
        except Exception as e:
            tb = traceback.format_exc()
            print(tb, file=sys.stderr)
            messagebox.showerror("Runtime Error", f"{e}\n\nSee console for details.")

    def _update_ui(self, driver_data, label_suffix=""):
        self.tree.delete(*self.tree.get_children())
        distances = [v['distance'] for v in driver_data.values()] if driver_data else []
        if not distances:
            self.total_label.config(text="Total System Distance: N/A")
            self.imbalance_label.config(text="Load Imbalance (max-min): N/A")
            return
        total = sum(distances); mx = max(distances); mn = min(distances); imb = mx - mn
        self.total_label.config(text=f"Total System Distance: {total:.2f} {label_suffix}")
        self.imbalance_label.config(text=f"Load Imbalance (max-min): {imb:.2f}")

        for k in sorted(driver_data.keys()):
            d = driver_data[k]
            route_str = " -> ".join(d['route'])
            tag = ''
            if d['distance'] == mx and imb > 0: tag = 'max_load'
            elif d['distance'] == mn and imb > 0: tag = 'min_load'
            self.tree.insert('', tk.END, iid=str(k), values=(f"{d['distance']:.2f}", route_str), tags=(tag,))
        self.tree.tag_configure('max_load', background='salmon')
        self.tree.tag_configure('min_load', background='lightgreen')

    def _draw_map(self, driver_data):
        self.canvas.delete('all')
        w = self.canvas.winfo_width() or 800
        h = self.canvas.winfo_height() or 420
        pad = 20
        all_lats = [p['lat'] for p in self.points_data.values()] + [self.depot_coords['lat']]
        all_lons = [p['lon'] for p in self.points_data.values()] + [self.depot_coords['lon']]
        min_lat, max_lat = min(all_lats), max(all_lats)
        min_lon, max_lon = min(all_lons), max(all_lons)

        def project(coord):
            lat, lon = coord['lat'], coord['lon']
            lat_span = max(1e-6, max_lat - min_lat)
            lon_span = max(1e-6, max_lon - min_lon)
            x = pad + ((lon - min_lon) / lon_span) * (w - 2*pad)
            y = pad + (1 - ((lat - min_lat) / lat_span)) * (h - 2*pad)
            return x, y

        pos = {}
        for name, coord in self.points_data.items():
            x, y = project(coord); pos[name] = (x, y)
            self.canvas.create_oval(x-3, y-3, x+3, y+3, fill='black')
            self.canvas.create_text(x+6, y, text=name, anchor='w', font=('Segoe UI', 8))

        dx, dy = project(self.depot_coords)
        self.canvas.create_rectangle(dx-5, dy-5, dx+5, dy+5, fill='red')
        self.canvas.create_text(dx+8, dy-8, text='DEPOT', anchor='w', font=('Segoe UI', 9, 'bold'))

        if driver_data:
            palette = ['#1f77b4','#ff7f0e','#2ca02c','#d62728',
                       '#9467bd','#8c564b','#e377c2','#7f7f7f',
                       '#bcbd22','#17becf']
            ids = sorted(driver_data.keys())
            for idx, d in enumerate(ids):
                seq = driver_data[d]['route']
                color = palette[idx % len(palette)]
                coords = []
                for node in seq:
                    coords.append((dx, dy) if node == 'DEPOT' else pos.get(node, (dx, dy)))
                flat = []
                for (x, y) in coords: flat.extend([x, y])
                if len(flat) >= 4:
                    self.canvas.create_line(*flat, fill=color, width=2, smooth=True)
                    if len(coords) > 1:
                        px, py = coords[1]
                        self.canvas.create_text(px, py-10, text=f'D{d}', anchor='s', fill=color, font=('Segoe UI', 9, 'bold'))
        self.canvas.create_text(8, h-8,
                                text=f"Points: {len(self.points_data)}  Drivers: {self.num_drivers.get()}  L_max: {self.max_route_dist.get():.1f}",
                                anchor='w', font=('Segoe UI', 9, 'italic'))

    def _show_comparison_window(self, greedy_res, dp_res, dp_exact_used):
        win = tk.Toplevel(self.root)
        win.title("Compare: Greedy (Efficiency) vs DP (Fairness, points)")
        win.geometry("860x400")
        left = ttk.LabelFrame(win, text="Greedy (Efficiency)", padding=8)
        right = ttk.LabelFrame(win, text=f"DP Fairness — Points ({'Exact' if dp_exact_used else 'LPT fallback'})", padding=8)
        left.place(x=8, y=8, width=420, height=380)
        right.place(x=432, y=8, width=420, height=380)

        def populate(frame, data):
            dists = [v['distance'] for v in data.values()]
            total = sum(dists); mx = max(dists) if dists else 0.0; mn = min(dists) if dists else 0.0
            ttk.Label(frame, text=f"Total Distance: {total:.2f}").pack(anchor='w')
            ttk.Label(frame, text=f"Max: {mx:.2f}  Min: {mn:.2f}  Imbalance: {mx-mn:.2f}").pack(anchor='w', pady=(0,6))
            tv = ttk.Treeview(frame, columns=('D','R'), show='headings', height=12)
            tv.heading('D', text='Distance'); tv.heading('R', text='Route')
            tv.column('D', width=80, anchor='center'); tv.column('R', width=300)
            tv.pack(fill='both', expand=True)
            for k,v in sorted(data.items()):
                tv.insert('', tk.END, values=(f"{v['distance']:.2f}", " -> ".join(v['route'])))
        populate(left, greedy_res)
        populate(right, dp_res)

    def _export_csv(self):
        if not self.tree.get_children():
            messagebox.showinfo("No data", "Run an optimization first")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV","*.csv")])
        if not path:
            return
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['DriverID','Distance','Route'])
            for iid in self.tree.get_children():
                vals = self.tree.item(iid)['values']
                writer.writerow([iid, vals[0], vals[1]])
        messagebox.showinfo("Saved", f"Exported to {path}")

# ---------------- Run ----------------
if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = DeliveryOptimizerApp(root)
        root.mainloop()
    except Exception as e:
        tb = traceback.format_exc()
        print(tb, file=sys.stderr)
        try:
            tk.Tk().withdraw()
            messagebox.showerror("Fatal error", f"Application failed to start:\n{e}\n\nSee console for traceback.")
        except:
            pass
        sys.exit(1)