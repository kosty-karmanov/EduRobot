import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import heapq

# === Глобальные переменные ===
global_obstacles = np.empty((0, 2))

# Параметры карты
MAP_SIZE = 20.0  # Размер карты +/-10 м (итого 20 м)
map_res = 0.1    # Шаг сетки: 10 см (0.1 м) на клетку
inflate_radius = 0.1  # На 10 см раздуваем препятствия (только для A*)

# Размер сетки в клетках
grid_cells = int(MAP_SIZE / map_res)  # 200 клеток (если 20/0.1)
grid_center = grid_cells // 2  # координата (0,0) в центре

# Начальные позиции (в метрах)
robot_pos = np.array([0.0, 0.0])   # м
target_pos = np.array([8.0, 0.0])  # м

# === ROS2 Нода для /lidar ===
class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global global_obstacles
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Обновляем глобальные препятствия (в СК робота)
        global_obstacles = np.column_stack((x, y))

# === Сетка для визуализации ===
def build_raw_grid(obstacles):
    """Возвращает 2D-массив grid_cells x grid_cells.
       0 = свободно, 1 = занято.
    """
    grid = np.zeros((grid_cells, grid_cells), dtype=np.uint8)

    if obstacles.size == 0:
        return grid

    for (ox, oy) in obstacles:
        j = int(round(ox / map_res)) + grid_center
        i = int(round(oy / map_res)) + grid_center
        if 0 <= i < grid_cells and 0 <= j < grid_cells:
            grid[i, j] = 1

    return grid

def inflate_grid(grid, inflate_r):
    """Принимает grid (0/1), на выходе новый grid (0/1) с «раздутыми» препятствиями."""
    inflated = np.copy(grid)
    inflate_cells = int(np.ceil(inflate_r / map_res))

    occupied_indices = np.argwhere(grid == 1)
    for (i, j) in occupied_indices:
        i_min = max(i - inflate_cells, 0)
        i_max = min(i + inflate_cells, grid_cells - 1)
        j_min = max(j - inflate_cells, 0)
        j_max = min(j + inflate_cells, grid_cells - 1)
        inflated[i_min:i_max+1, j_min:j_max+1] = 1

    return inflated

# === A* ===
def a_star(start_ij, goal_ij, grid):
    """
    start_ij: (i_start, j_start)
    goal_ij:  (i_goal, j_goal)
    grid:     occupancy grid 0/1
    Возвращает список клеток (i, j) от старта до цели или None, если пути нет.
    """
    if grid[start_ij[0], start_ij[1]] == 1 or grid[goal_ij[0], goal_ij[1]] == 1:
        return None

    open_set = []
    g_cost = np.full(grid.shape, np.inf)
    parent = -1 * np.ones((grid.shape[0], grid.shape[1], 2), dtype=int)

    def heuristic(i1, j1, i2, j2):
        return np.sqrt((i1 - i2)**2 + (j1 - j2)**2)

    g_cost[start_ij[0], start_ij[1]] = 0.0
    start_h = heuristic(start_ij[0], start_ij[1], goal_ij[0], goal_ij[1])
    heapq.heappush(open_set, (start_h, start_ij))

    neighbors = [(-1,  0), (1, 0), (0, -1), (0, 1),
                 (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while open_set:
        _, (ci, cj) = heapq.heappop(open_set)
        if (ci, cj) == goal_ij:
            return reconstruct_path(parent, start_ij, goal_ij)

        for di, dj in neighbors:
            ni, nj = ci + di, cj + dj
            if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1]:
                if grid[ni, nj] == 0:
                    step_cost = np.sqrt(di*di + dj*dj)
                    tentative_g = g_cost[ci, cj] + step_cost
                    if tentative_g < g_cost[ni, nj]:
                        g_cost[ni, nj] = tentative_g
                        f_cost = tentative_g + heuristic(ni, nj, goal_ij[0], goal_ij[1])
                        parent[ni, nj] = (ci, cj)
                        heapq.heappush(open_set, (f_cost, (ni, nj)))

    return None

def reconstruct_path(parent, start_ij, goal_ij):
    path = []
    curr = goal_ij
    while curr[0] != -1:
        path.append(curr)
        if curr == start_ij:
            break
        curr = tuple(parent[curr[0], curr[1]])
    path.reverse()
    return path

# === Преобразования координат ===
def world_to_grid(x, y):
    j = int(round(x / map_res)) + grid_center
    i = int(round(y / map_res)) + grid_center
    return (i, j)

def grid_to_world(i, j):
    x = (j - grid_center) * map_res
    y = (i - grid_center) * map_res
    return (x, y)

# === Графика ===
fig, ax = plt.subplots()
scat_robot, = ax.plot([], [], 'bo', markersize=6, label='Робот')
scat_target, = ax.plot([], [], 'rx', markersize=8, label='Цель')
scat_path, = ax.plot([], [], 'g-', linewidth=2, label='Путь A*')
scat_obs = ax.scatter([], [], s=5, c='k', marker='s', label='Препятствия')

ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_aspect('equal', 'box')
ax.grid(True)
ax.legend()
ax.set_title("A* Path Planning")

def init_anim():
    scat_robot.set_data([], [])
    scat_target.set_data([], [])
    scat_path.set_data([], [])
    scat_obs.set_offsets(np.empty((0, 2)))
    return scat_robot, scat_target, scat_path, scat_obs

def update_anim(frame):
    raw_grid = build_raw_grid(global_obstacles)

    inflated_grid = inflate_grid(raw_grid, inflate_radius)

    # 3) Ищем путь
    start_ij = world_to_grid(*robot_pos)
    goal_ij = world_to_grid(*target_pos)
    path_ij = a_star(start_ij, goal_ij, inflated_grid)

    # --- Отрисовка ---
    # Робот
    scat_robot.set_data(robot_pos[0], robot_pos[1])
    # Цель
    scat_target.set_data(target_pos[0], target_pos[1])

    # Препятствия
    occupied = np.argwhere(raw_grid == 1)
    if occupied.size > 0:
        occupied_xy = np.array([grid_to_world(i, j) for i, j in occupied])
        scat_obs.set_offsets(occupied_xy)
    else:
        scat_obs.set_offsets(np.empty((0, 2)))

    # Путь
    if path_ij is not None:
        path_xy = np.array([grid_to_world(i, j) for (i, j) in path_ij])
        scat_path.set_data(path_xy[:,0], path_xy[:,1])
    else:
        scat_path.set_data([], [])

    return scat_robot, scat_target, scat_path, scat_obs

# === Основная функция ===
def main():
    rclpy.init()
    node = LidarSubscriber()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    ani = FuncAnimation(fig, update_anim, init_func=init_anim, interval=500, blit=True)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
