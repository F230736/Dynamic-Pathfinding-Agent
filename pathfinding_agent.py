"""
Dynamic Pathfinding Agent - Tkinter Implementation
Implements GBFS and A* with Manhattan & Euclidean heuristics
Dynamic obstacle spawning with real-time re-planning
FIXED: Dynamic obstacles now properly spawn ON the path to trigger re-planning
"""

import tkinter as tk
from tkinter import messagebox
import heapq
import math
import random
import time
from collections import defaultdict

# ─── Color Palette ───────────────────────────────────────────────────────────
C_BG         = "#0f0f1a"
C_PANEL      = "#1a1a2e"
C_BORDER     = "#2a2a4a"
C_EMPTY      = "#1e1e3a"
C_WALL       = "#4a4a7a"
C_START      = "#00ff88"
C_GOAL       = "#ff4466"
C_FRONTIER   = "#ffd700"
C_VISITED    = "#4466ff"
C_PATH       = "#00ffcc"
C_TEXT       = "#e0e0ff"
C_ACCENT     = "#7c5cbf"
C_BTN        = "#2d2b55"
C_BTN_HOV    = "#3d3b75"
C_SUCCESS    = "#00ff88"
C_WARN       = "#ff9900"

CELL_MIN = 12
CELL_MAX = 60


class PathfindingApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dynamic Pathfinding Agent")
        self.configure(bg=C_BG)
        self.resizable(True, True)

        # Grid state
        self.rows = 20
        self.cols = 25
        self.cell_size = 28
        self.grid = {}
        self.start = (0, 0)
        self.goal  = (self.rows - 1, self.cols - 1)
        self.path  = []
        self.visited_cells = set()
        self.frontier_cells = set()

        # Agent state
        self.agent_pos = None
        self.running = False
        self.replans = 0
        self.paused = False
        self._resume_path = None
        self._resume_step = None
        self.total_nodes = 0
        self.exec_time_ms = 0
        self.path_cost = 0

        # Edit mode
        self.edit_mode = "wall"
        self.drag_painting = False

        self._build_ui()
        self._init_grid()
        self._draw_grid()

    # ─────────────────────────────────────────────────────────────────────────
    # UI Construction
    # ─────────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # Left panel with scrollbar
        left_outer = tk.Frame(self, bg=C_PANEL, width=250)
        left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 5), pady=10)
        left_outer.pack_propagate(False)

        canvas_scroll = tk.Canvas(left_outer, bg=C_PANEL, highlightthickness=0, width=230)
        scrollbar = tk.Scrollbar(left_outer, orient=tk.VERTICAL, command=canvas_scroll.yview)
        canvas_scroll.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas_scroll.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        left = tk.Frame(canvas_scroll, bg=C_PANEL)
        left_window = canvas_scroll.create_window((0, 0), window=left, anchor="nw")

        def _on_frame_configure(e):
            canvas_scroll.configure(scrollregion=canvas_scroll.bbox("all"))

        def _on_canvas_configure(e):
            canvas_scroll.itemconfig(left_window, width=e.width)

        left.bind("<Configure>", _on_frame_configure)
        canvas_scroll.bind("<Configure>", _on_canvas_configure)

        def _on_mousewheel(e):
            canvas_scroll.yview_scroll(int(-1 * (e.delta / 120)), "units")

        canvas_scroll.bind_all("<MouseWheel>", _on_mousewheel)

        tk.Label(left, text="⬡ PATHFINDER", font=("Courier", 14, "bold"),
                 fg=C_ACCENT, bg=C_PANEL).pack(pady=(15, 5))
        tk.Label(left, text="Dynamic Navigation Agent", font=("Courier", 8),
                 fg="#666688", bg=C_PANEL).pack(pady=(0, 15))

        self._section(left, "GRID CONFIG")
        self._labeled_spin(left, "Rows", 5, 50, self.rows, "rows_var")
        self._labeled_spin(left, "Cols", 5, 60, self.cols, "cols_var")
        self._labeled_spin(left, "Cell Size", CELL_MIN, CELL_MAX, self.cell_size, "cell_var")
        self._btn(left, "Apply Grid", self._apply_grid)

        self._section(left, "MAP TOOLS")
        self._labeled_spin(left, "Obstacle %", 5, 70, 30, "density_var")
        self._btn(left, "Generate Random Map", self._random_map)
        self._btn(left, "Clear Map", self._clear_map)

        self._section(left, "EDIT MODE")
        self.edit_var = tk.StringVar(value="wall")
        modes = [("Place Wall", "wall"), ("Set Start", "start"),
                 ("Set Goal", "goal"), ("Erase", "erase")]
        for txt, val in modes:
            tk.Radiobutton(left, text=txt, variable=self.edit_var, value=val,
                           bg=C_PANEL, fg=C_TEXT, selectcolor=C_BTN,
                           activebackground=C_PANEL, font=("Courier", 9),
                           command=lambda v=val: setattr(self, 'edit_mode', v)
                           ).pack(anchor=tk.W, padx=15)

        self._section(left, "ALGORITHM")
        self.algo_var = tk.StringVar(value="astar")
        for txt, val in [("A* Search", "astar"), ("Greedy BFS", "gbfs")]:
            tk.Radiobutton(left, text=txt, variable=self.algo_var, value=val,
                           bg=C_PANEL, fg=C_TEXT, selectcolor=C_BTN,
                           activebackground=C_PANEL, font=("Courier", 9)
                           ).pack(anchor=tk.W, padx=15)

        self._section(left, "HEURISTIC")
        self.heur_var = tk.StringVar(value="manhattan")
        for txt, val in [("Manhattan", "manhattan"), ("Euclidean", "euclidean")]:
            tk.Radiobutton(left, text=txt, variable=self.heur_var, value=val,
                           bg=C_PANEL, fg=C_TEXT, selectcolor=C_BTN,
                           activebackground=C_PANEL, font=("Courier", 9)
                           ).pack(anchor=tk.W, padx=15)

        self._section(left, "DYNAMIC MODE")
        self.dynamic_var = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text="Enable Dynamic Obstacles",
                       variable=self.dynamic_var, bg=C_PANEL, fg=C_TEXT,
                       selectcolor=C_BTN, activebackground=C_PANEL,
                       font=("Courier", 9)).pack(anchor=tk.W, padx=15)
        self._labeled_spin(left, "Spawn Prob %", 1, 30, 8, "spawn_var")
        self._labeled_spin(left, "Step Delay ms", 10, 500, 80, "delay_var")

        tk.Frame(left, bg=C_BORDER, height=1).pack(fill=tk.X, padx=10, pady=10)
        self._btn(left, "▶  START SEARCH", self._start_search, fg=C_SUCCESS)
        self._btn(left, "⏹  STOP", self._stop_search, fg=C_WARN)
        self._btn(left, "⏵  RESUME", self._resume_search, fg="#00ccff")
        self._btn(left, "↺  RESET VISUAL", self._reset_visual)

        # Right panel
        right = tk.Frame(self, bg=C_BG)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 10), pady=10)

        # Metrics bar
        metrics = tk.Frame(right, bg=C_PANEL, pady=6)
        metrics.pack(fill=tk.X, pady=(0, 8))

        self.metric_labels = {}
        metrics_data = [
            ("nodes_visited", "Nodes Visited", "0"),
            ("path_cost",     "Path Cost",     "0"),
            ("exec_time",     "Exec Time",     "0 ms"),
            ("replans",       "Re-plans",      "0"),
            ("status",        "Status",        "IDLE"),
        ]
        for key, label, val in metrics_data:
            f = tk.Frame(metrics, bg=C_PANEL)
            f.pack(side=tk.LEFT, padx=15)
            tk.Label(f, text=label, font=("Courier", 7), fg="#666688",
                     bg=C_PANEL).pack()
            lbl = tk.Label(f, text=val, font=("Courier", 11, "bold"),
                           fg=C_ACCENT, bg=C_PANEL)
            lbl.pack()
            self.metric_labels[key] = lbl

        # Canvas
        canvas_frame = tk.Frame(right, bg=C_BORDER, bd=1)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(canvas_frame, bg=C_BG, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>",
                         lambda e: setattr(self, 'drag_painting', False))

        # Legend
        legend = tk.Frame(right, bg=C_PANEL, pady=4)
        legend.pack(fill=tk.X, pady=(8, 0))
        items = [("Start", C_START), ("Goal", C_GOAL), ("Path", C_PATH),
                 ("Frontier", C_FRONTIER), ("Visited", C_VISITED), ("Wall", C_WALL)]
        for name, color in items:
            f = tk.Frame(legend, bg=C_PANEL)
            f.pack(side=tk.LEFT, padx=10)
            tk.Frame(f, bg=color, width=14, height=14).pack(side=tk.LEFT)
            tk.Label(f, text=f" {name}", font=("Courier", 8), fg=C_TEXT,
                     bg=C_PANEL).pack(side=tk.LEFT)

    def _section(self, parent, text):
        tk.Label(parent, text=text, font=("Courier", 8, "bold"),
                 fg="#888aaa", bg=C_PANEL).pack(anchor=tk.W, padx=15, pady=(12, 2))

    def _labeled_spin(self, parent, label, lo, hi, default, attr):
        f = tk.Frame(parent, bg=C_PANEL)
        f.pack(fill=tk.X, padx=15, pady=1)
        tk.Label(f, text=label, font=("Courier", 8), fg=C_TEXT,
                 bg=C_PANEL, width=12, anchor=tk.W).pack(side=tk.LEFT)
        var = tk.IntVar(value=default)
        setattr(self, attr, var)
        tk.Spinbox(f, from_=lo, to=hi, textvariable=var, width=5,
                   bg=C_BTN, fg=C_TEXT, insertbackground=C_TEXT,
                   buttonbackground=C_BTN, relief=tk.FLAT,
                   font=("Courier", 9)).pack(side=tk.RIGHT)

    def _btn(self, parent, text, cmd, fg=C_TEXT):
        btn = tk.Button(parent, text=text, command=cmd, font=("Courier", 9, "bold"),
                        bg=C_BTN, fg=fg, activebackground=C_BTN_HOV,
                        activeforeground=fg, relief=tk.FLAT, pady=5, cursor="hand2")
        btn.pack(fill=tk.X, padx=10, pady=2)
        btn.bind("<Enter>", lambda e: btn.config(bg=C_BTN_HOV))
        btn.bind("<Leave>", lambda e: btn.config(bg=C_BTN))
        return btn

    # ─────────────────────────────────────────────────────────────────────────
    # Grid Init & Drawing
    # ─────────────────────────────────────────────────────────────────────────
    def _init_grid(self):
        self.grid = {}
        for r in range(self.rows):
            for c in range(self.cols):
                self.grid[(r, c)] = 'empty'
        self.grid[self.start] = 'start'
        self.grid[self.goal]  = 'goal'

    def _apply_grid(self):
        self.rows = self.rows_var.get()
        self.cols = self.cols_var.get()
        self.cell_size = self.cell_var.get()
        self.start = (0, 0)
        self.goal  = (self.rows - 1, self.cols - 1)
        self._init_grid()
        self.path = []
        self.visited_cells = set()
        self.frontier_cells = set()
        self._draw_grid()
        self._update_metrics()

    def _draw_grid(self):
        self.canvas.delete("all")
        cs = self.cell_size
        path_set = set(self.path)
        for r in range(self.rows):
            for c in range(self.cols):
                self._draw_cell_at(r, c, cs, path_set)
        # Draw agent on top
        if self.agent_pos:
            self._draw_agent(self.agent_pos)

    def _draw_cell_at(self, r, c, cs, path_set=None):
        if path_set is None:
            path_set = set(self.path)
        x1, y1 = c * cs + 2, r * cs + 2
        x2, y2 = x1 + cs - 2, y1 + cs - 2
        state = self.grid.get((r, c), 'empty')
        if state == 'start':
            color = C_START
        elif state == 'goal':
            color = C_GOAL
        elif state == 'wall':
            color = C_WALL
        elif (r, c) in path_set:
            color = C_PATH
        elif (r, c) in self.frontier_cells:
            color = C_FRONTIER
        elif (r, c) in self.visited_cells:
            color = C_VISITED
        else:
            color = C_EMPTY
        self.canvas.create_rectangle(x1, y1, x2, y2,
                                     fill=color, outline=C_BG, width=1)

    def _draw_agent(self, pos):
        r, c = pos
        cs = self.cell_size
        x1, y1 = c * cs + 2, r * cs + 2
        cx = x1 + cs // 2 - 2
        cy = y1 + cs // 2 - 2
        rad = cs // 3
        self.canvas.create_oval(cx - rad, cy - rad, cx + rad, cy + rad,
                                fill="#ffffff", outline=C_ACCENT, width=2)

    def _draw_cell_color(self, r, c, color):
        cs = self.cell_size
        x1, y1 = c * cs + 2, r * cs + 2
        x2, y2 = x1 + cs - 2, y1 + cs - 2
        self.canvas.create_rectangle(x1, y1, x2, y2,
                                     fill=color, outline=C_BG, width=1)

    def _update_cell(self, r, c):
        """Redraw a single cell based on current state."""
        cs = self.cell_size
        path_set = set(self.path)
        # Erase old agent circle if needed
        self._draw_cell_at(r, c, cs, path_set)
        if (r, c) == self.agent_pos:
            self._draw_agent((r, c))

    # ─────────────────────────────────────────────────────────────────────────
    # Map Tools
    # ─────────────────────────────────────────────────────────────────────────
    def _random_map(self):
        density = self.density_var.get() / 100.0
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) == self.start or (r, c) == self.goal:
                    self.grid[(r, c)] = 'start' if (r, c) == self.start else 'goal'
                elif random.random() < density:
                    self.grid[(r, c)] = 'wall'
                else:
                    self.grid[(r, c)] = 'empty'
        self.path = []
        self.visited_cells = set()
        self.frontier_cells = set()
        self._draw_grid()

    def _clear_map(self):
        self._init_grid()
        self.path = []
        self.visited_cells = set()
        self.frontier_cells = set()
        self.agent_pos = None
        self._draw_grid()
        self._update_metrics()

    def _reset_visual(self):
        self.path = []
        self.visited_cells = set()
        self.frontier_cells = set()
        self.agent_pos = None
        self.running = False
        self.paused = False
        self._resume_path = None
        self._resume_step = None
        self._draw_grid()
        self._update_status("IDLE")

    # ─────────────────────────────────────────────────────────────────────────
    # Mouse Interaction
    # ─────────────────────────────────────────────────────────────────────────
    def _canvas_to_cell(self, x, y):
        cs = self.cell_size
        c = x // cs
        r = y // cs
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return (r, c)
        return None

    def _on_click(self, event):
        if self.running:
            return
        self.drag_painting = True
        cell = self._canvas_to_cell(event.x, event.y)
        if cell:
            self._apply_edit(cell)

    def _on_drag(self, event):
        if self.running or not self.drag_painting:
            return
        cell = self._canvas_to_cell(event.x, event.y)
        if cell:
            self._apply_edit(cell)

    def _apply_edit(self, cell):
        mode = self.edit_var.get()
        r, c = cell
        if mode == "wall":
            if cell != self.start and cell != self.goal:
                self.grid[cell] = 'wall'
        elif mode == "erase":
            if cell != self.start and cell != self.goal:
                self.grid[cell] = 'empty'
        elif mode == "start":
            old = self.start
            self.grid[old] = 'empty'
            self.grid[cell] = 'start'
            self.start = cell
        elif mode == "goal":
            old = self.goal
            self.grid[old] = 'empty'
            self.grid[cell] = 'goal'
            self.goal = cell
        self._update_cell(r, c)

    # ─────────────────────────────────────────────────────────────────────────
    # Heuristics
    # ─────────────────────────────────────────────────────────────────────────
    def _heuristic(self, a, b):
        if self.heur_var.get() == "manhattan":
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        else:
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    # ─────────────────────────────────────────────────────────────────────────
    # Search Algorithm  (A* and GBFS share this function)
    # ─────────────────────────────────────────────────────────────────────────
    def _neighbors(self, pos):
        r, c = pos
        result = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.grid.get((nr, nc)) != 'wall':
                    result.append((nr, nc))
        return result

    def _search(self, start, goal):
        """
        Returns (path, visited_order, frontier_snapshots).
        Works for both A* and GBFS depending on algo_var.
        """
        algo = self.algo_var.get()
        h = self._heuristic

        open_set   = []
        came_from  = {}
        g_score    = defaultdict(lambda: float('inf'))
        g_score[start] = 0
        counter    = 0

        if algo == "astar":
            heapq.heappush(open_set, (g_score[start] + h(start, goal), counter, start))
        else:
            heapq.heappush(open_set, (h(start, goal), counter, start))

        open_dict      = {start: True}
        visited        = []
        frontier_snaps = []

        while open_set:
            _, _, current = heapq.heappop(open_set)
            open_dict.pop(current, None)

            if current == goal:
                # Reconstruct path
                path = []
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()
                return path, visited, frontier_snaps

            visited.append(current)

            for nb in self._neighbors(current):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score[nb]:
                    came_from[nb] = current
                    g_score[nb]   = tentative_g
                    counter += 1
                    f = (tentative_g + h(nb, goal)) if algo == "astar" else h(nb, goal)
                    heapq.heappush(open_set, (f, counter, nb))
                    open_dict[nb] = True

            frontier_snaps.append(set(open_dict.keys()))

        return [], visited, frontier_snaps

    # ─────────────────────────────────────────────────────────────────────────
    # Search Execution
    # ─────────────────────────────────────────────────────────────────────────
    def _start_search(self):
        if self.running:
            return
        self._reset_visual()
        self.running   = True
        self.replans   = 0
        self.paused    = False
        self._resume_path = None
        self._resume_step = None
        self._update_status("SEARCHING...")

        t0 = time.time()
        path, visited, frontier_snaps = self._search(self.start, self.goal)
        elapsed = (time.time() - t0) * 1000

        if not path:
            self._update_status("NO PATH FOUND")
            self.running = False
            messagebox.showwarning("No Path", "No path found! Try clearing some walls.")
            return

        self.exec_time_ms = elapsed
        self.total_nodes  = len(visited)
        self.path_cost    = len(path) - 1
        self._update_metrics_values()
        self._animate_exploration(visited, frontier_snaps, path, 0)

    # ─────────────────────────────────────────────────────────────────────────
    # Exploration Animation
    # ─────────────────────────────────────────────────────────────────────────
    def _animate_exploration(self, visited, frontier_snaps, final_path, idx):
        if not self.running:
            return
        if idx < len(visited):
            cell = visited[idx]
            if cell != self.start and cell != self.goal:
                old_frontier = set(self.frontier_cells)
                self.visited_cells.add(cell)
                if idx < len(frontier_snaps):
                    self.frontier_cells = frontier_snaps[idx] - self.visited_cells
                    self.frontier_cells.discard(self.start)
                    self.frontier_cells.discard(self.goal)
                else:
                    self.frontier_cells = set()
                for fc in old_frontier - self.frontier_cells:
                    self._update_cell(*fc)
                self._draw_cell_color(*cell, C_VISITED)
                for fc in self.frontier_cells:
                    self._draw_cell_color(*fc, C_FRONTIER)
            self.after(20, lambda: self._animate_exploration(
                visited, frontier_snaps, final_path, idx + 1))
        else:
            # Draw final path in green over visited
            self.frontier_cells = set()
            self.path = final_path
            path_set  = set(final_path)
            for cell in self.visited_cells:
                if cell not in path_set:
                    self._draw_cell_color(*cell, C_VISITED)
            for cell in final_path:
                if cell != self.start and cell != self.goal:
                    self._draw_cell_color(*cell, C_PATH)
            self.after(300, lambda: self._animate_agent(final_path, 0))

    # ─────────────────────────────────────────────────────────────────────────
    # Agent Animation  ←  FIXED DYNAMIC OBSTACLE LOGIC HERE
    # ─────────────────────────────────────────────────────────────────────────
    def _animate_agent(self, path, step):
        if not self.running:
            return

        if step >= len(path):
            self._update_status("GOAL REACHED ✓")
            self.running = False
            return

        # ── Move agent ──────────────────────────────────────────────────────
        prev_pos       = self.agent_pos
        self.agent_pos = path[step]

        # Redraw previous cell (restore path/visited color)
        if prev_pos and prev_pos != self.start and prev_pos != self.goal:
            if prev_pos in set(self.path):
                self._draw_cell_color(*prev_pos, C_PATH)
            elif prev_pos in self.visited_cells:
                self._draw_cell_color(*prev_pos, C_VISITED)
            else:
                self._draw_cell_color(*prev_pos, C_EMPTY)

        # Draw agent at new position
        self._draw_cell_color(*self.agent_pos, C_EMPTY)  # clear first
        self._draw_agent(self.agent_pos)

        # ── Dynamic obstacle logic ───────────────────────────────────────────
        # Only try to spawn if dynamic mode ON and there are future steps
        if self.dynamic_var.get() and step < len(path) - 1:
            prob = self.spawn_var.get() / 100.0
            if random.random() < prob:
                self._try_spawn_obstacle(path, step)
                # After spawning, path may have been replaced — read from self
                path = self.path
                step = self._resume_step - 1 if self._resume_step else step

        # ── Schedule next step ───────────────────────────────────────────────
        # Save resume state BEFORE scheduling
        self._resume_path = path
        self._resume_step = step + 1
        delay = self.delay_var.get()
        self.after(delay, lambda: self._animate_agent(path, step + 1))

    def _try_spawn_obstacle(self, current_path, current_step):
        """
        Attempt to spawn a new obstacle.
        Strategy:
          1. With 60% chance, try to place it on the remaining path
             (this guarantees a re-plan and is visually interesting).
          2. With 40% chance, place it anywhere empty off-path.
        After spawning, check if remaining path is blocked and re-plan if so.
        """
        remaining_path = current_path[current_step + 1:]   # steps ahead of agent
        remaining_set  = set(remaining_path)

        # Build candidate lists
        on_path_candidates = [
            cell for cell in remaining_path
            if cell != self.goal and cell != self.start
        ]
        off_path_candidates = [
            (r, c)
            for r in range(self.rows)
            for c in range(self.cols)
            if self.grid[(r, c)] == 'empty'
            and (r, c) not in remaining_set
            and (r, c) != self.start
            and (r, c) != self.goal
            and (r, c) != self.agent_pos
        ]

        # Choose where to spawn
        new_wall = None
        if on_path_candidates and random.random() < 0.6:
            # Place ON the path → guaranteed re-plan
            new_wall = random.choice(on_path_candidates)
        elif off_path_candidates:
            # Place off-path → no re-plan needed
            new_wall = random.choice(off_path_candidates)
        elif on_path_candidates:
            new_wall = random.choice(on_path_candidates)

        if new_wall is None:
            return  # nowhere to spawn

        # Place the wall
        self.grid[new_wall] = 'wall'
        self._draw_cell_color(*new_wall, C_WALL)

        # Check if it blocks our remaining path
        if new_wall in remaining_set:
            # ── RE-PLAN ──────────────────────────────────────────────────────
            self._update_status("RE-PLANNING...")
            self.replans += 1

            t0 = time.time()
            new_path, new_visited, _ = self._search(self.agent_pos, self.goal)
            elapsed = (time.time() - t0) * 1000

            self.exec_time_ms += elapsed
            self.total_nodes  += len(new_visited)

            if new_path:
                # Clear old path color, draw new path
                old_path_set = set(self.path)
                self.path = new_path
                new_path_set = set(new_path)

                # Repaint cells that were path but aren't anymore
                for cell in old_path_set - new_path_set:
                    if cell != self.start and cell != self.goal and cell != self.agent_pos:
                        if cell in self.visited_cells:
                            self._draw_cell_color(*cell, C_VISITED)
                        else:
                            self._draw_cell_color(*cell, C_EMPTY)

                # Paint new path green
                for cell in new_path:
                    if cell != self.start and cell != self.goal and cell != self.agent_pos:
                        self._draw_cell_color(*cell, C_PATH)

                self.path_cost = len(new_path) - 1
                self.visited_cells.update(new_visited)
                self._update_metrics_values()

                # Update resume pointers so _animate_agent uses new path from step 0
                self._resume_path = new_path
                self._resume_step = 0
                # Overwrite local path reference so caller sees new path
                # (we mutate via self.path; caller reads self._resume_path)

            else:
                self._update_status("PATH BLOCKED - NO RE-ROUTE")
                self.running = False
        # else: wall spawned off-path, no re-plan needed

    # ─────────────────────────────────────────────────────────────────────────
    # Stop / Resume
    # ─────────────────────────────────────────────────────────────────────────
    def _stop_search(self):
        self.running = False
        self.paused  = True
        self._update_status("PAUSED")

    def _resume_search(self):
        if not self.paused or self._resume_path is None:
            self._update_status("NOTHING TO RESUME")
            return
        self.running = True
        self.paused  = False
        self._update_status("RESUMED...")
        self._animate_agent(self._resume_path, self._resume_step)

    # ─────────────────────────────────────────────────────────────────────────
    # Metrics
    # ─────────────────────────────────────────────────────────────────────────
    def _update_metrics_values(self):
        self.metric_labels["nodes_visited"].config(text=str(self.total_nodes))
        self.metric_labels["path_cost"].config(text=str(self.path_cost))
        self.metric_labels["exec_time"].config(text=f"{self.exec_time_ms:.1f} ms")
        self.metric_labels["replans"].config(text=str(self.replans))

    def _update_metrics(self):
        for key in ("nodes_visited", "path_cost"):
            self.metric_labels[key].config(text="0")
        self.metric_labels["exec_time"].config(text="0 ms")
        self.metric_labels["replans"].config(text="0")

    def _update_status(self, text):
        colors = {
            "GOAL REACHED ✓":           C_SUCCESS,
            "RE-PLANNING...":           C_WARN,
            "SEARCHING...":             C_FRONTIER,
            "RESUMED...":               C_FRONTIER,
            "PAUSED":                   C_WARN,
            "NO PATH FOUND":            C_GOAL,
            "PATH BLOCKED - NO RE-ROUTE": C_GOAL,
        }
        self.metric_labels["status"].config(
            text=text, fg=colors.get(text, C_ACCENT))


if __name__ == "__main__":
    app = PathfindingApp()
    app.mainloop()
