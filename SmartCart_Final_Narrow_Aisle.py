"""SmartCart supermarket animation: A* routing + OR-Tools Guided Local Search.
Install: pip install matplotlib ortools
Run: python SmartCart_Final_Narrow_Aisle.py
"""
import heapq
import itertools
import math
import time
import json
from pathlib import Path
from collections import defaultdict
from functools import lru_cache

import matplotlib
try:
    matplotlib.use('TkAgg')
except Exception:
    pass
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button
from matplotlib.patches import Rectangle, FancyBboxPatch, Polygon
from matplotlib.transforms import Affine2D
try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2 as enums
    HAVE_ORTOOLS = True
except ImportError:
    HAVE_ORTOOLS = False

plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei', 'Noto Sans CJK TC', 'PingFang TC', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# S1-S11: long-side / wall shelves; E1-E4: short endcaps.
CATALOG = {
    'S1': ('電器・生活', ['電池', '燈泡', '延長線']),
    'S2': ('清潔用品', ['洗衣精', '洗碗精', '清潔劑']),
    'S3': ('調味料', ['醬油', '食鹽', '砂糖']),
    'S4': ('罐頭食品', ['鮪魚罐頭', '玉米罐頭', '番茄罐頭']),
    'S5': ('餅乾點心', ['餅乾', '洋芋片', '巧克力']),
    'S6': ('沖泡飲品', ['咖啡', '奶茶', '可可粉']),
    'S7': ('飲料・冰品', ['礦泉水', '紅茶', '冰淇淋']),
    'S8': ('麵包・蛋糕', ['吐司', '貝果', '蛋糕']),
    'S9': ('零食', ['海苔', '堅果', '果乾']),
    'S10': ('日用品', ['衛生紙', '牙膏', '牙刷']),
    'S11': ('米麵雜糧', ['白米', '義大利麵', '燕麥']),
    'E1': ('端架・水果', ['蘋果', '香蕉', '橘子']),
    'E2': ('端架・蔬菜', ['高麗菜', '番茄', '洋蔥']),
    'E3': ('端架・乳品', ['牛奶', '優格', '起司']),
    'E4': ('端架・特價', ['雞蛋', '豆腐', '豆漿']),
}
# 商品資料儲存在程式旁的 JSON；可透過「商品管理」自行新增、修改與刪除。
CATALOG_FILE = Path(__file__).with_name('smartcart_products.json')
def load_catalog():
    if CATALOG_FILE.exists():
        try:
            data = json.loads(CATALOG_FILE.read_text(encoding='utf-8'))
            for shelf in CATALOG:
                record = data.get(shelf, {})
                if isinstance(record, dict):
                    items = record.get('items', {})
                    if isinstance(items, dict):
                        CATALOG[shelf] = (str(record.get('category', CATALOG[shelf][0])),
                                          {str(p): int(level) for p, level in items.items()
                                           if str(p).strip() and 1 <= int(level) <= 5})
        except (ValueError, OSError, TypeError) as exc:
            print('商品資料讀取失敗，使用預設清單：', exc)

def save_catalog():
    data = {s: {'category': category, 'items': items}
            for s, (category, items) in CATALOG.items()}
    CATALOG_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')

def rebuild_products():
    global PRODUCTS
    PRODUCTS = {p: (s, level)
                for s, (_, items) in CATALOG.items()
                for p, level in items.items()}

CATALOG = {s: (category, {p: i for i, p in enumerate(items, 1)})
           for s, (category, items) in CATALOG.items()}
load_catalog()
rebuild_products()

# Layout coordinates are a schematic model, not a certified as-built store survey.
# Rectangles represent physical obstacles. Navigation nodes lie outside rectangles.
OBSTACLES = [
    ('Island A', (3.00, 3.50, 0.90, 3.60)),
    ('Island B', (7.00, 3.50, 0.90, 3.60)),
    ('Left continuous S1/S2', (0.35, 3.60, 0.45, 3.85)),
    ('Right continuous S9/S10', (11.20, 3.60, 0.45, 3.85)),
    ('Top continuous S7/S8', (3.35, 9.35, 5.80, 0.45)),
    ('Bottom left', (0.25, 0.25, 2.40, 0.45)),
]

# Directed aisle graph; lower redundant horizontal aisle removed, bottom aisle retained.
X = [1.40, 2.40, 4.60, 6.30, 8.50, 10.50]
Y = [0.80, 1.85, 2.70, 4.20, 5.40, 6.60, 7.80, 8.70]
POS = {}
GRAPH = defaultdict(list)
for i, x in enumerate(X):
    for j, y in enumerate(Y):
        POS[f'N{i}_{j}'] = (x, y)

# Each existing EDGE is a directed lane. The two parallel lanes of an aisle
# provide opposite travel directions; these are NOT centreline offsets.
# Vertical lanes, from left to right: ↑ ↓ ↓ ↑ ↓ ↑
VERTICAL_NORTH = {1, 3, 5}
# Horizontal lane directions; row j=1 is deliberately absent.
# Top and bottom parallel cross-aisles form opposing one-way lanes.
HORIZONTAL_EAST = {0, 3, 4, 6}

def add_edge(a, b):
    """Add exactly one legal directed arc on the existing grey EDGE."""
    w = math.dist(POS[a], POS[b])
    xa, ya = POS[a]
    xb, yb = POS[b]
    if abs(xa - xb) < 1e-9:
        north = X.index(xa) in VERTICAL_NORTH
        source, target = ((a, b) if ya < yb else (b, a)) if north else ((a, b) if ya > yb else (b, a))
    else:
        east = Y.index(ya) in HORIZONTAL_EAST
        source, target = ((a, b) if xa < xb else (b, a)) if east else ((a, b) if xa > xb else (b, a))
    GRAPH[source].append((target, w))

# A horizontal edge is allowed only if it does not intersect shelf footprints.
def intersects_rect(a, b, rect, eps=0.025):
    x, y, w, h = rect
    x1, y1 = a
    x2, y2 = b
    if abs(y1-y2) < 1e-8:
        return (y-eps <= y1 <= y+h+eps and max(min(x1,x2), x-eps) < min(max(x1,x2), x+w+eps))
    if abs(x1-x2) < 1e-8:
        return (x-eps <= x1 <= x+w+eps and max(min(y1,y2), y-eps) < min(max(y1,y2), y+h+eps))
    raise ValueError('Only orthogonal graph edges are allowed')

def is_clear(a, b):
    return not any(intersects_rect(a, b, rect) for _, rect in OBSTACLES)

for i in range(len(X)):
    for j in range(len(Y)):
        a = f'N{i}_{j}'
        if i+1 < len(X) and j != 1:  # Remove the redundant lower cross-aisle entirely.
            b = f'N{i+1}_{j}'
            if is_clear(POS[a], POS[b]): add_edge(a, b)
        if j+1 < len(Y):
            b = f'N{i}_{j+1}'
            if is_clear(POS[a], POS[b]): add_edge(a, b)

# Add a dedicated traffic-only node 17 near the entrance by subdividing an
# existing eastbound EDGE; this does not add a shortcut or a reverse arc.
TRAFFIC_NODE_17 = 'T17'
POS[TRAFFIC_NODE_17] = (5.65, 0.80)
for k, (target, weight) in enumerate(GRAPH['N2_0']):
    if target == 'N3_0':
        del GRAPH['N2_0'][k]
        GRAPH['N2_0'].append((TRAFFIC_NODE_17, math.dist(POS['N2_0'], POS[TRAFFIC_NODE_17])))
        GRAPH[TRAFFIC_NODE_17].append(('N3_0', math.dist(POS[TRAFFIC_NODE_17], POS['N3_0'])))
        break
else:
    raise RuntimeError('Cannot insert traffic node 17 into the eastbound entrance EDGE')

# Each shelf is assigned to a safe, existing directed EDGE node.
# Several shelves may share a stop: the customer can leave the cart and walk
# to either side of the aisle. Distances are illustrative and should be
# checked against the actual supermarket layout before physical deployment.
STOP_TO_NODE = {
    'Entrance': 'N3_0', 'Parking': 'N5_0',
}

# S11 mirrors the Parking display area (same width, height and lower y); no duplicate backing rectangle.
# Display locations of shelves: central island long sides, wall units, endcaps.
SHELF_RECTS = {
    'S1':(.35,5.525,.45,1.925),'S2':(.35,3.6,.45,1.925),
    'S3':(3.0,3.5,.45,3.6),'S4':(3.45,3.5,.45,3.6),
    'S5':(7.0,3.5,.45,3.6),'S6':(7.45,3.5,.45,3.6),
    'S7':(3.35,9.35,2.90,.45),'S8':(6.25,9.35,2.90,.45),
    'S9':(11.2,5.525,.45,1.925),'S10':(11.2,3.6,.45,1.925),
    'S11':(.25,.38,2.40,.83),
    'E1':(3.0,7.1,.90,.40),'E2':(3.0,3.1,.90,.40),
    'E3':(7.0,7.1,.90,.40),'E4':(7.0,3.1,.90,.40),
}
for e in ('E1','E2','E3','E4'):
    OBSTACLES.append((e, SHELF_RECTS[e]))
for a, neighbors in GRAPH.items():
    for b, _ in neighbors:
        assert is_clear(POS[a], POS[b]), f'Edge crosses a shelf: {a} -> {b}'

# Candidate pick-up locations must be reachable from the actual shelf face.
# S5 is on the LEFT face of island B: N4 is across the solid island and
# must not be treated as an S5 pickup. S11 is serviced at E2 lower-left.
# Multiple candidates need not lie on opposite sides of a shelf.
# Candidate pick-up locations: legal approaches around a shelf are eligible.
# These are pre-defined feasible walking-access locations on existing lanes;
# the route solver chooses whichever minimizes the WHOLE trip.
# Only shelves with a physically plausible shared stopping position may be
# serviced together; adjacent IDs do not imply accessibility.
# Multiple shelves may be serviced at the same stop, but no shelf is forced
# to use a particular stop or permanently skipped.
# Hand-drawn numbered pickup points (1..27). Each maps to an EXISTING
# intersection of the directed lane graph, so adding stops cannot create
# illegal shortcuts or opposite-direction movement.
# 17 was marked near the entrance in the drawing; 1 is the entrance itself.
NUMBERED_STOPS = {
    1: ('Entrance', 'N3_0'),
    2: ('E2 / S11', 'N1_2'),
    3: ('S2', 'N0_3'), 4: ('S2', 'N1_3'),
    5: ('S3', 'N1_4'),
    6: ('S1', 'N0_5'), 7: ('S1', 'N1_5'),
    8: ('E1', 'N1_6'),
    9: ('S7', 'N2_7'), 10: ('S7', 'N3_7'),
    11: ('E1 / E3', 'N3_6'),
    12: ('S4 / S5', 'N3_4'),
    13: ('E2 / E4', 'N3_2'),
    14: ('E2 / E4', 'N3_1'),
    15: ('S4 / S5', 'N3_5'),
    16: ('E3 / E1', 'N4_6'),
    17: ('Traffic only (no shelf)', TRAFFIC_NODE_17),
    18: ('S8', 'N4_7'), 19: ('S8', 'N5_7'),
    20: ('E3', 'N5_6'),
    21: ('S9', 'N5_5'), 22: ('S9', 'N5_4'),
    23: ('S6', 'N4_4'),
    24: ('S10', 'N4_3'), 25: ('S10', 'N5_3'),
    26: ('E4', 'N4_2'),
    27: ('Parking', 'N5_0'),
}
# Stop coverage is explicit: an aisle point may service two shelves only
# where the customer can physically reach both without crossing an island.
# In particular, S5 is never picked from the S6 side of island B.
SHELF_PICKUP_NODES = {
    'S1': ('N0_5', 'N1_5'),
    'S2': ('N0_3', 'N1_3'),
    'S3': ('N1_4',),
    'S4': ('N3_4', 'N3_5'),
    'S5': ('N3_4', 'N3_5'),
    'S6': ('N4_4',),
    'S7': ('N2_7', 'N3_7'),
    'S8': ('N4_7', 'N5_7'),
    'S9': ('N5_5', 'N5_4'),
    'S10': ('N4_3', 'N5_3'),
    'S11': ('N1_2',),
    'E1': ('N1_6', 'N3_6', 'N4_6'),
    'E2': ('N1_2', 'N3_2', 'N3_1'),
    'E3': ('N3_6', 'N4_6', 'N5_6'),
    'E4': ('N3_2', 'N3_1', 'N4_2'),
}

for shelf, nodes in SHELF_PICKUP_NODES.items():
    for node in nodes:
        STOP_TO_NODE[f'P_{node}'] = node
STOP_COVERS = defaultdict(set)
for shelf, nodes in SHELF_PICKUP_NODES.items():
    for node in nodes:
        STOP_COVERS[f'P_{node}'].add(shelf)

# Convert each actual graph stop to its handwritten ID. 17 is traffic-only.
NODE_TO_NUMBER = {node: number for number, (_label, node) in NUMBERED_STOPS.items()}

def pickup_direction(node, incoming_heading, shelf):
    """Return shelf position relative to the cart's heading at the pickup stop.

    The cart heading is based on its last nonzero traveled segment, not the
    north-up map view.  If a shelf is diagonally behind the cart, give the
    side plus a front/rear qualifier rather than guessing only 'left/right'.
    """
    cx, cy = POS[node]
    sx, sy, sw, sh = SHELF_RECTS[shelf]
    dx, dy = sx + sw / 2 - cx, sy + sh / 2 - cy
    forward = math.cos(incoming_heading) * dx + math.sin(incoming_heading) * dy
    left = -math.sin(incoming_heading) * dx + math.cos(incoming_heading) * dy
    if abs(left) < 0.08:
        return '正前方' if forward >= 0 else '後方'
    side = '左手邊' if left > 0 else '右手邊'
    if abs(forward) > abs(left) * 1.5:
        return ('前方偏' if forward > 0 else '後方偏') + ('左' if left > 0 else '右')
    return side

# The existing paired edges ARE the lanes. No lateral animation offset.
RIGHT_OFFSET = 0.0

# Every picking point is an aisle node; entrance/parking are separate terminals.
def astar(start, end):
    src, dst = STOP_TO_NODE[start], STOP_TO_NODE[end]
    heap = [(math.dist(POS[src], POS[dst]), 0.0, src)]
    best, prev = {src:0.0}, {src:None}
    while heap:
        _, g, u = heapq.heappop(heap)
        if g > best[u]+1e-10: continue
        if u == dst:
            path = []
            while u is not None:
                path.append(u)
                u = prev[u]
            return g, path[::-1]
        for v, w in GRAPH[u]:
            ng = g+w
            if ng < best.get(v, math.inf)-1e-10:
                best[v], prev[v] = ng, u
                heapq.heappush(heap, (ng+math.dist(POS[v], POS[dst]), ng, v))
    raise RuntimeError(f'Unreachable: {start} -> {end}')

def solve_gls(stops, distance_matrix):
    """Optional GLS refinement for a fixed set of selected stop nodes."""
    if not HAVE_ORTOOLS:
        return None
    manager = pywrapcp.RoutingIndexManager(len(stops), 1, [0], [len(stops)-1])
    routing = pywrapcp.RoutingModel(manager)
    def transit(i, j):
        return distance_matrix[manager.IndexToNode(i)][manager.IndexToNode(j)]
    callback = routing.RegisterTransitCallback(transit)
    routing.SetArcCostEvaluatorOfAllVehicles(callback)
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = enums.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = enums.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromMilliseconds(250)
    solution = routing.SolveWithParameters(params)
    if solution is None:
        return None
    order, idx = [], routing.Start(0)
    while not routing.IsEnd(idx):
        order.append(manager.IndexToNode(idx))
        idx = solution.Value(routing.NextVar(idx))
    order.append(manager.IndexToNode(idx))
    return [stops[i] for i in order]

@lru_cache(maxsize=None)
def cached_astar(a, b):
    return astar(a, b)

def plan_route(names):
    """Joint stop-choice and route-order optimization (exact subset DP).

    A stop may cover several shelves. Directed A* distances encode all
    one-way edges; GLS can refine the order but cannot relax traffic rules.
    """
    shelves = list(dict.fromkeys(PRODUCTS[p][0] for p in names))
    full_mask = (1 << len(shelves)) - 1
    candidates = sorted({f'P_{node}' for shelf in shelves
                         for node in SHELF_PICKUP_NODES[shelf]})
    cover = {stop: sum(1 << i for i, shelf in enumerate(shelves)
                       if shelf in STOP_COVERS[stop]) for stop in candidates}
    places = ['Entrance', *candidates, 'Parking']
    paths = {}
    for a in places:
        for b in places:
            if a == b:
                paths[a, b] = (0.0, [STOP_TO_NODE[a]])
            else:
                try:
                    paths[a, b] = cached_astar(a, b)
                except RuntimeError:
                    pass
    # Each DP state is (covered shelves, last stop). Exact for these 15 shelves.
    dp = {(0, 'Entrance'): 0.0}
    parent = {}
    for _ in range(len(shelves)):
        nxt = dict(dp)
        for (mask, last), cost in dp.items():
            for stop in candidates:
                newmask = mask | cover[stop]
                if newmask == mask or (last, stop) not in paths:
                    continue
                nc = cost + paths[last, stop][0]
                key = (newmask, stop)
                if nc < nxt.get(key, math.inf) - 1e-9:
                    nxt[key] = nc
                    parent[key] = (mask, last)
        dp = nxt
    choices = [(cost + paths[last, 'Parking'][0], (mask, last))
               for (mask, last), cost in dp.items()
               if mask == full_mask and (last, 'Parking') in paths]
    if not choices:
        raise RuntimeError('找不到符合單向 EDGE 的完整取貨路線')
    best_cost, state = min(choices)
    reverse = []
    while state != (0, 'Entrance'):
        reverse.append(state[1])
        state = parent[state]
    route = ['Entrance', *reversed(reverse), 'Parking']
    # Optional GLS on the exact DP's chosen stops (never accept a worse route).
    if HAVE_ORTOOLS and len(route) > 3:
        huge = 10**9
        matrix = [[round(paths[a,b][0]*1000) if (a,b) in paths else huge
                   for b in route] for a in route]
        refined = solve_gls(route, matrix)
        if refined and all((a,b) in paths for a,b in zip(refined,refined[1:])):
            ref_cost = sum(paths[a,b][0] for a,b in zip(refined,refined[1:]))
            if ref_cost < best_cost - 1e-9:
                route, best_cost = refined, ref_cost
    return route, paths, best_cost


fig, ax = plt.subplots(figsize=(14.2, 9), facecolor='#f5f7fa')
plt.subplots_adjust(left=.06, right=.80, top=.92, bottom=.16)
ax.set_aspect('equal')
ax.set(xlim=(-.85,12.65),ylim=(-.55,10.75),title='SmartCart | 有向 A* + 智慧取貨導航 | 禁止逆向')
ax.set_facecolor('#fcfdff')
ax.set_xticks([])
ax.set_yticks([])
for spine in ax.spines.values(): spine.set_visible(False)

ax.add_patch(Rectangle((.05,.25),11.85,9.8,fill=False,edgecolor='#536579',lw=2,zorder=1))
ax.add_patch(FancyBboxPatch((9.30,.38),2.40,.83,boxstyle='round,pad=.04',
                             facecolor='#dcfce7',edgecolor='#22a06b',lw=1.5,zorder=1))
ax.text(10.50,.62,'PARKING',ha='center',va='center',fontsize=10,color='#147d4e',weight='bold')

drawn_edges = set()
for a, nbrs in GRAPH.items():
    for b, _ in nbrs:
        edge = frozenset((a,b))
        if edge in drawn_edges:
            continue
        drawn_edges.add(edge)
        x1,y1=POS[a];x2,y2=POS[b]
        ax.plot([x1,x2],[y1,y2],color='#e2eaf2',lw=5,zorder=0,solid_capstyle='round')
        # Direction arrows on both vertical and horizontal existing edges.
        mx,my=(x1+x2)/2,(y1+y2)/2
        dx,dy=x2-x1,y2-y1
        norm=math.hypot(dx,dy)
        ux,uy=dx/norm,dy/norm
        ax.annotate('',xy=(mx+ux*.16,my+uy*.16),xytext=(mx-ux*.16,my-uy*.16),
                    arrowprops=dict(arrowstyle='->',color='#9ab0c7',lw=1.2),zorder=2)
for x,y in POS.values():
    ax.plot(x,y,'.',color='#adc1d5',ms=4,zorder=1)

patches = {}
for shelf,(x,y,w,h) in SHELF_RECTS.items():
    endcap=shelf.startswith('E')
    patch=FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.015,rounding_size=0.045',
                         facecolor='#c9d9e8',
                         edgecolor='#6686a4',lw=1.4,zorder=3)
    ax.add_patch(patch)
    patches[shelf]=patch
    if shelf in ('S3','S4','S5','S6'):
        ax.text(x+w/2,y+h/2,shelf,ha='center',va='center',fontsize=9,weight='bold',
                rotation=90,color='#24415d',zorder=4)
    else:
        ax.text(x+w/2,y+h/2,shelf,ha='center',va='center',fontsize=9,weight='bold',
                color='#24415d',zorder=4)

# Presentation mode: pickup locations and their numbers stay in the model,
# but are not rendered as red points or numbered labels.

for s,color in [('Entrance','#27a477'),('Parking','#27a477')]:
    x,y=POS[STOP_TO_NODE[s]]
    ax.scatter(x,y,s=170,c=color,edgecolors='white',linewidths=2,zorder=6)
    if s == 'Entrance':
        ax.text(x,y-.40,'ENTRANCE',ha='center',fontsize=9,weight='bold',color=color,zorder=6)

# Terminal node IDs are hidden as well; entrance and parking names remain visible.

# Physical-scale top-view cart (assumed 0.85 m long x 0.55 m wide).
# Unlike a matplotlib marker, this polygon scales with the map's metre coordinates.
CART_LENGTH_M = 0.85
CART_WIDTH_M = 0.55
cart_shape = Polygon([(-.425,-.275),(.425,-.275),(.425,.275),(-.425,.275)],
                     closed=True,facecolor='#2457dc',edgecolor='white',lw=1.8,zorder=9)
ax.add_patch(cart_shape)
cart_front = Polygon([(.19,-.20),(.34,0),(.19,.20)],closed=True,
                     facecolor='white',edgecolor='none',zorder=10)
ax.add_patch(cart_front)
def move_cart(x,y,angle=0):
    transform = Affine2D().rotate(angle).translate(x,y) + ax.transData
    cart_shape.set_transform(transform)
    cart_front.set_transform(transform)

# Dimension annotations: schematic coordinate metres, not measured store dimensions.
def dimension(x1,y1,x2,y2,label,vertical=False):
    ax.annotate('',xy=(x2,y2),xytext=(x1,y1),
                arrowprops=dict(arrowstyle='<->',color='#526477',lw=1.0),zorder=11)
    ax.text((x1+x2)/2+(.17 if vertical else 0), (y1+y2)/2+ (0 if vertical else .15),
            label,rotation=90 if vertical else 0,ha='center',va='center',
            fontsize=8,color='#34485c',zorder=11,
            bbox=dict(facecolor='#fcfdff',edgecolor='none',pad=.7))

dimension(.05,10.25,11.90,10.25,'11.85 m（地圖座標）')
dimension(-.42,.25,-.42,10.05,'9.80 m（地圖座標）',True)
dimension(3.00,7.72,3.90,7.72,'0.90 m')
dimension(3.15,3.50,3.15,7.10,'3.60 m',True)
dimension(3.90,5.80,7.00,5.80,'3.10 m')
dimension(7.90,5.80,11.20,5.80,'3.30 m')
# Removed overlapping note at top-left; dimension arrows remain visible.
trail,=ax.plot([],[],color='#2457dc',lw=3.3,zorder=7)
stop_dot,=ax.plot([],[],marker='o',color='#2457dc',markersize=0,alpha=0,
                  markeredgecolor='white',markeredgewidth=2,linestyle='None',zorder=8)
status=ax.text(.02,.98,'請輸入商品，按 GO 開始動畫（到站顯示 Node／左右／層數）',transform=ax.transAxes,va='top',
               fontsize=10,bbox=dict(facecolor='white',edgecolor='#b7c5d4',boxstyle='round,pad=.5'),zorder=12)
progress=ax.text(1.02,.95,'購物進度\n\n尚未開始',transform=ax.transAxes,va='top',fontsize=10,
                 linespacing=1.6,bbox=dict(facecolor='white',edgecolor='#b7c5d4',
                 boxstyle='round,pad=.8'),zorder=12)
textbox=TextBox(plt.axes([.19,.065,.49,.05]),'購物清單：',initial='蘋果,牛奶,餅乾,衛生紙,義大利麵,電池,延長線')
button=Button(plt.axes([.72,.065,.10,.05]),'GO')
timer=fig.canvas.new_timer(interval=45)
state={'frames':[],'index':0}

def tick():
    i=state['index'];frames=state['frames']
    if i>=len(frames):
        timer.stop();return
    x,y,msg,done,stopping=frames[i]
    # Heading is captured on the incoming edge during frame generation.
    state['heading'] = state['frame_headings'][i]
    move_cart(x,y,state.get('heading',0.0))
    trail.set_data([f[0] for f in frames[:i+1]],[f[1] for f in frames[:i+1]])
    stop_dot.set_data([x] if stopping else [],[y] if stopping else [])
    status.set_text(msg)
    progress.set_text('購物進度\n\n'+('\n'.join('✓ '+p for p in done) if done else '尚未拿取商品'))
    state['index']=i+1
    fig.canvas.draw_idle()

timer.add_callback(tick)

def go(_event):
    timer.stop()
    raw=textbox.text.replace('，',',').replace('、',',').replace('；',',')
    names=list(dict.fromkeys(p.strip() for p in raw.split(',') if p.strip()))
    bad=[p for p in names if p not in PRODUCTS]
    if not names or bad:
        status.set_text('尚未建檔：'+('、'.join(bad) if bad else '請輸入商品')+'（可按商品管理新增）')
        fig.canvas.draw_idle();return
    t0 = time.perf_counter()
    try:
        route, paths, length = plan_route(names)
    except Exception as exc:
        status.set_text(f'規劃失敗：{exc}')
        fig.canvas.draw_idle()
        return
    # Attribute each product to the FIRST visited stop that can service it.
    chosen = defaultdict(list)
    for product in names:
        shelf, level = PRODUCTS[product]
        stop = next((stop for stop in route[1:-1]
                     if shelf in STOP_COVERS[stop]), None)
        if stop is None:
            status.set_text(f'規劃錯誤：{product} 無取貨位置')
            fig.canvas.draw_idle()
            return
        chosen[stop].append((product, level, shelf))
    print('購物清單：', '、'.join(names))
    print('27 個編號節點（17 僅交通）：', ', '.join(f'{n}={lab}' for n,(lab,_) in NUMBERED_STOPS.items()))
    print('合法有向 EDGE 路線：', ' → '.join(route))
    for stop, items in chosen.items():
        print(f'{stop} 停靠取貨：', '、'.join(f'{p}({s})' for p,_,s in items))
    print(f'距離 {length:.2f} m，規劃 {(time.perf_counter()-t0)*1000:.0f} ms')
    selected_shelves={shelf for items in chosen.values() for _,_,shelf in items}
    for s,patch in patches.items():
        patch.set_facecolor('#438fbd' if s in selected_shelves else '#c9d9e8')
    sx,sy=POS[STOP_TO_NODE['Entrance']]
    frames=[(sx,sy,'從 Entrance 出發（Node 1）',(),False)]
    frame_headings=[0.0]
    heading=0.0
    done=[]
    # Drive exactly on the A* directed graph edges, without any offset.
    for a,b in zip(route,route[1:]):
        nodepath=paths[a,b][1]
        if len(nodepath)>1:
            coords=[POS[n] for n in nodepath]
            for (x1,y1),(x2,y2) in zip(coords,coords[1:]):
                steps=max(2,int(math.dist((x1,y1),(x2,y2))*13))
                heading=math.atan2(y2-y1,x2-x1)
                for k in range(1,steps+1):
                    t=k/steps
                    frames.append((x1+(x2-x1)*t,y1+(y2-y1)*t,
                                   f'沿單向 EDGE 行駛，前往 Node {NODE_TO_NUMBER.get(STOP_TO_NODE[b], b)} ...',tuple(done),False))
                    frame_headings.append(heading)
        if b in chosen:
            items = chosen[b]
            stop_node = STOP_TO_NODE[b]
            stop_id = NODE_TO_NUMBER.get(stop_node, '?')
            msg = f'抵達 Node {stop_id}｜' + '｜'.join(
                f'{pickup_direction(stop_node, heading, shelf)} {shelf} 貨架 第{level}層：{product}'
                for product,level,shelf in items)
            done.extend(p for p,_,_ in items)
            x,y=POS[STOP_TO_NODE[b]]
            frames.extend([(x,y,msg,tuple(done),True)]*40)
            frame_headings.extend([heading]*40)
            print(msg)
    px,py=POS[STOP_TO_NODE['Parking']]
    frames.extend([(px,py,'已返回 Parking（Node 27），購物完成！',tuple(done),False)]*40)
    frame_headings.extend([heading]*40)
    assert len(frames) == len(frame_headings)
    trail.set_data([],[])
    stop_dot.set_data([],[])
    move_cart(sx,sy)
    progress.set_text('購物進度\n\n準備出發')
    state.update(frames=frames,frame_headings=frame_headings,index=0,heading=0.0)
    timer.start()
    fig.canvas.draw_idle()

def open_catalog(_event):
    # TkAgg 下使用 Tkinter 商品管理視窗，不會中斷 Matplotlib 動畫。
    import tkinter as tk
    from tkinter import ttk, messagebox
    root = fig.canvas.manager.window
    win = tk.Toplevel(root)
    win.title('SmartCart｜商品管理（自訂貨架與層數）')
    win.geometry('690x530')
    win.transient(root)
    frame = ttk.Frame(win, padding=12)
    frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='商品資料庫：新增商品後，直接在主視窗購物清單輸入商品名稱即可。',
              font=('Microsoft JhengHei', 10)).pack(anchor='w', pady=(0,8))
    columns = ('商品', '貨架', '層數', '分類')
    tree = ttk.Treeview(frame, columns=columns, show='headings', height=14)
    for key, width in [('商品',180),('貨架',75),('層數',65),('分類',150)]:
        tree.heading(key,text=key)
        tree.column(key,width=width,anchor='center' if key in ('貨架','層數') else 'w')
    scroller = ttk.Scrollbar(frame, orient='vertical', command=tree.yview)
    tree.configure(yscrollcommand=scroller.set)
    tree.pack(side='top', fill='both', expand=True)
    scroller.place(in_=tree, relx=1, rely=0, relheight=.68, anchor='ne')
    def refresh():
        for row in tree.get_children(): tree.delete(row)
        for shelf,(category,items) in CATALOG.items():
            for product, level in sorted(items.items(), key=lambda kv:(kv[1],kv[0])):
                tree.insert('', 'end', values=(product,shelf,f'第{level}層',category))
    refresh()
    edit = ttk.Frame(frame)
    edit.pack(fill='x', pady=(12,4))
    ttk.Label(edit,text='商品名稱').grid(row=0,column=0,sticky='w')
    ttk.Label(edit,text='貨架').grid(row=0,column=1,sticky='w',padx=8)
    ttk.Label(edit,text='層數').grid(row=0,column=2,sticky='w',padx=8)
    product_var=tk.StringVar()
    shelf_var=tk.StringVar(value='S1')
    level_var=tk.StringVar(value='1')
    ttk.Entry(edit,textvariable=product_var,width=23).grid(row=1,column=0,sticky='we')
    ttk.Combobox(edit,textvariable=shelf_var,values=list(CATALOG),state='readonly',width=9).grid(row=1,column=1,padx=8)
    ttk.Combobox(edit,textvariable=level_var,values=['1','2','3','4','5'],state='readonly',width=7).grid(row=1,column=2,padx=8)
    def choose(_event):
        selection=tree.selection()
        if not selection:return
        product,shelf,level,_=tree.item(selection[0],'values')
        product_var.set(product);shelf_var.set(shelf);level_var.set(level.replace('第','').replace('層',''))
    tree.bind('<<TreeviewSelect>>',choose)
    def add_or_update():
        product=product_var.get().strip()
        shelf=shelf_var.get()
        if not product or shelf not in CATALOG:
            messagebox.showwarning('資料不完整','請填入商品名稱與貨架。',parent=win);return
        level=int(level_var.get())
        # 商品名稱全店唯一；修改位置時會從舊貨架移除。
        for old_shelf,(_,items) in CATALOG.items():
            items.pop(product,None)
        CATALOG[shelf][1][product]=level
        save_catalog();rebuild_products();refresh()
        status.set_text(f'已儲存：{product}｜{shelf}｜第{level}層')
        fig.canvas.draw_idle()
    def delete_selected():
        product=product_var.get().strip()
        if product not in PRODUCTS:
            messagebox.showinfo('找不到商品','請先點選表格中的商品。',parent=win);return
        shelf,_=PRODUCTS[product]
        del CATALOG[shelf][1][product]
        save_catalog();rebuild_products();refresh()
        product_var.set('')
        status.set_text(f'已刪除商品：{product}')
        fig.canvas.draw_idle()
    controls=ttk.Frame(frame)
    controls.pack(fill='x',pady=10)
    ttk.Button(controls,text='新增／更新商品',command=add_or_update).pack(side='left',padx=(0,8))
    ttk.Button(controls,text='刪除所選商品',command=delete_selected).pack(side='left',padx=(0,8))
    ttk.Button(controls,text='完成',command=win.destroy).pack(side='right')
    ttk.Label(frame,text='同一貨架可以放多種商品，同一層也可以放多種商品；層數可選 1–5。',
              foreground='#526477').pack(anchor='w')

# 按 M 開啟商品管理，保持主畫面簡潔。
def on_key(event):
    if event.key == 'm':
        open_catalog(event)
fig.canvas.mpl_connect('key_press_event', on_key)
button.on_clicked(go)
textbox.on_submit(go)
move_cart(*POS[STOP_TO_NODE['Entrance']])
print('=== SmartCart | Directed A* + candidate stop optimization ===')
print('有向 EDGE 禁止逆向；候選停靠點聯合最佳化；車身轉角碰撞尚未驗證。')
print('輸入商品，逗號分隔；例：蘋果,牛奶,餅乾,衛生紙')
print('按鍵 M：開啟商品管理視窗')
for s,(cat,items) in CATALOG.items():
    print(f'{s:>3} {cat}: '+ '、'.join(f'{p}(第{lv}層)' for p,lv in items.items()))
plt.show()
