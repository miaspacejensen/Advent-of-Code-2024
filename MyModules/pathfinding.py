from collections import deque 
import heapq

def bfs(grid, start_pos, target_pos, directions, obstacles=[]):

    '''
    Breadth First Search (BFS)
    - Unweighted graphs.
    - With or without obstacles.
    '''

    n_rows = len(grid)
    n_cols = len(grid[0])

    queue = deque([start_pos])

    visited_positions = set()
    visited_positions.add(start_pos)
    
    parent = {}
    while queue:
        current_pos = queue.popleft()
        if current_pos == target_pos:
            path = []
            while current_pos:
                path.append(current_pos)
                current_pos = parent.get(current_pos)
            reversed_path = path[::-1] 
            return reversed_path
        
        for dr, dc in directions:
            r = current_pos[0] + dr 
            c = current_pos[1] + dc 

            neighbour_pos = (r, c)

            if 0 <= r < n_rows and 0 <= c < n_cols and neighbour_pos not in visited_positions:
                if obstacles == [] or grid[r][c] not in obstacles:
                    visited_positions.add(neighbour_pos)
                    parent[neighbour_pos] = current_pos
                    queue.append(neighbour_pos)
    
    return None

def astar(grid, start_pos, target_pos, directions, obstacles=[]):
    
    '''
    A*
    - Weighted or uneighted graphs.
    - With or without obstacles.
    - Heuristic-based search.
    - Harnesses BFS but minimises exploration.
    '''

    def manhattan_dist(pos_a, pos_b):
        row_diff = pos_a[0] - pos_b[0]
        col_diff = pos_a[1] - pos_b[1]
        return abs(row_diff) + abs(col_diff)

    n_rows = len(grid)
    n_cols = len(grid[0])

    open_set = [(0, start_pos)]

    came_from = {}

    g_score = {start_pos: 0}
    f_score = {start_pos: manhattan_dist(start_pos, target_pos)}

    while open_set:
        _, current_pos = heapq.heappop(open_set)

        if current_pos == target_pos:
            path = []
            while current_pos in came_from:
                path.append(current_pos)
                current_pos = came_from[current_pos]
            reversed_path = [start_pos] + path[::-1] 
            return reversed_path
        
        for dr, dc in directions:
            r = current_pos[0] + dr 
            c = current_pos[1] + dc 

            neighbour_pos = (r, c)

            if 0 <= r < n_rows and 0 <= c < n_cols:
                if obstacles == [] or grid[r][c] not in obstacles:
                    temp_g_score = g_score[current_pos] + 1
                    if temp_g_score < g_score.get(neighbour_pos, float('inf')):
                        came_from[neighbour_pos] = current_pos
                        g_score[neighbour_pos] = temp_g_score
                        f_score[neighbour_pos] = temp_g_score + manhattan_dist(neighbour_pos, target_pos)
                        heapq.heappush(open_set, (f_score[neighbour_pos], neighbour_pos))
    return None

def dijkstra(grid, start_pos, target_pos, directions, obstacles=[]):

    '''
    Dijkstra
    - Weighted (non-negative) graphs.
    - With or without obstacles.
    - Similar to A* except without heuristic-based search.
    '''

    n_rows = len(grid)
    n_cols = len(grid[0])

    open_set = [(0, start_pos)]

    came_from = {}

    cost_so_far = {start_pos: 0}

    while open_set:
        current_cost, current_pos = heapq.heappop(open_set)

        if current_pos == target_pos:
            path = []
            while current_pos in came_from:
                path.append(current_pos)
                current_pos = came_from[current_pos]
            reversed_path = path[::-1] # [start_pos] + path[::-1] 
            return reversed_path
        
        for dr, dc in directions:
            r = current_pos[0] + dr 
            c = current_pos[1] + dc 

            neighbour_pos = (r, c)

            if 0 <= r < n_rows and 0 <= c < n_cols and grid[r][c] >= 0:
                if obstacles == [] or grid[r][c] not in obstacles:
                    new_cost = current_cost + grid[r][c]
                    if new_cost < cost_so_far.get(neighbour_pos, float('inf')):
                        cost_so_far[neighbour_pos] = new_cost
                        came_from[neighbour_pos] = current_pos 
                        heapq.heappush(open_set, (new_cost, neighbour_pos))
    return None