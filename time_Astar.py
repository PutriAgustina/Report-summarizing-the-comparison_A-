import heapq
import time
from random import randint

# Arah pergerakan: atas, bawah, kiri, kanan
DIRS = [(-1,0), (1,0), (0,-1), (0,1)]

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def generate_grid(size, obstacles):
    grid = [[0 for _ in range(size)] for _ in range(size)]
    for _ in range(obstacles):
        x, y = randint(0, size-1), randint(0, size-1)
        grid[x][y] = 1
    return grid

def neighbors(grid, node):
    n = len(grid)
    result = []
    for dx, dy in DIRS:
        nx, ny = node[0]+dx, node[1]+dy
        if 0 <= nx < n and 0 <= ny < n and grid[nx][ny] == 0:
            result.append((nx, ny))
    return result

def astar(grid, start, goal):
    open_set = [(heuristic(start, goal), 0, start)]
    g_score = {start: 0}
    while open_set:
        _, current_g, current = heapq.heappop(open_set)
        if current == goal:
            return True
        for neighbor in neighbors(grid, current):
            tentative_g = current_g + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
    return False

def run_astar_experiments():
    sizes = [5000, 50000, 500000, 5000000, 50000000]
    obstacles = [10, 100, 1000, 10000, 100000]

    for i in range(len(sizes)):
        n = int(sizes[i]**0.5)
        grid = generate_grid(n, obstacles[i])
        start = (0, 0)
        goal = (n-1, n-1)

        print(f"A* Experiment #{i+1} with {sizes[i]} nodes and {obstacles[i]} obstacles")

        start_time = time.time()
        astar(grid, start, goal)
        duration = (time.time() - start_time) * 1000
        print(f"Execution Time: {duration:.2f} ms\n")

run_astar_experiments()
