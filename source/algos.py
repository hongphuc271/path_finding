import pygame
from const import *
from maze import SearchSpace

def DFS(g: SearchSpace, sc: pygame.Surface):
    if (g.start.id == g.goal.id):
        print('You\'re already there')
        return

    print('Implement DFS algorithm')

    open_set = [g.start.id]
    closed_set = []
    father = [-1]*g.get_length()

    while len(open_set) > 0:
        node_id = open_set.pop(0)
        if node_id == g.goal.id:
            break
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(YELLOW, sc)
        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if not (neighbor.id in closed_set):
                if neighbor.id in open_set:
                    open_set.remove(neighbor.id)
                open_set.insert(0, neighbor.id)
                father[neighbor.id] = node_id
                if neighbor.id != g.goal.id:
                    g.grid_cells[neighbor.id].set_color(RED, sc)
        closed_set.append(node_id)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(BLUE, sc)
    
    if father[g.goal.id] == -1:
        print('No path found')
        return
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    stroke_path(g, path, sc)

def BFS(g: SearchSpace, sc: pygame.Surface):
    if (g.start.id == g.goal.id):
        print('You\'re already there')
        return

    print('Implement BFS algorithm')

    closed_set = [g.start.id]
    open_set = [g.start.id]
    father = [-1]*g.get_length()

    while len(open_set) > 0:
        node_id = open_set.pop(0)
        if node_id == g.goal.id:
            g.grid_cells[node_id].set_color(PURPLE, sc)
            break
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(YELLOW, sc)
        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if not (neighbor.id in closed_set) and not (neighbor.id in open_set):
                open_set.append(neighbor.id)
                father[neighbor.id] = node_id
                g.grid_cells[neighbor.id].set_color(RED, sc)
        closed_set.append(node_id)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(BLUE, sc)
    
    if father[g.goal.id] == -1:
        print('No path found')
        return
    
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    length = stroke_path(g, path, sc)

    print("Done! Total segments: %d. Total length: %d" % (len(path)-1, length))

def Dijkstra(g: SearchSpace, sc: pygame.Surface):
    if (g.start.id == g.goal.id):
        print('You\'re already there')
        return
    
    print('Implement Dijkstra algorithm')

    closed_set = [g.start.id]
    open_set = [g.start.id]
    father = [-1]*g.get_length()
    dist = [g.get_length()*14]*g.get_length()
    dist[g.start.id] = 0

    while len(open_set) > 0:
        open_set.sort(key=lambda i:dist[i])
        node_id = open_set.pop(0)
        if node_id == g.goal.id:
            g.grid_cells[node_id].set_color(PURPLE, sc)
            break
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(YELLOW, sc)
        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if neighbor.id in closed_set:
                continue
            new_dst = dist[node_id] + get_distance(node_id, neighbor.id)
            if new_dst < dist[neighbor.id] or  not (neighbor.id in open_set):
                dist[neighbor.id] = new_dst
                if not (neighbor.id in open_set):
                    open_set.append(neighbor.id)
                father[neighbor.id] = node_id
                g.grid_cells[neighbor.id].set_color(RED, sc)
        closed_set.append(node_id)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(BLUE, sc)
    
    if father[g.goal.id] == -1:
        print('No path found')
        return
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    length = stroke_path(g, path, sc)

    print("Done! Total segments: %d. Calculated length: %d. Actual length: %d"
          % (len(path)-1, dist[g.goal.id], length))

def AStar(g: SearchSpace, sc: pygame.Surface):
    if (g.start.id == g.goal.id):
        print('You\'re already there')
        return
    
    print('Implement AStar algorithm')

    open_set = [g.start.id]
    closed_set = []
    father = [-1]*g.get_length()
    # G Cost = Chiều dài đường đi từ g.start đến một node
    # H Cost = Khoảng cách Diagonal từ một node đến g.goal
    gcost = [0]*g.get_length()
    hcost = [0]*g.get_length()

    def get_fcost(idx : int) -> int:
        return gcost[idx] + hcost[idx]

    def get_lowest_fcost_id() -> int:
        lowest_fcost_id = open_set[0]
        for id in open_set:
            if get_fcost(id) < get_fcost(lowest_fcost_id):
                lowest_fcost_id = id
        return lowest_fcost_id

    while len(open_set)>0:
        current_id = get_lowest_fcost_id()
        open_set.remove(current_id)
        closed_set.append(current_id)
        if current_id == g.goal.id:
            break
        if current_id != g.start.id:
            g.grid_cells[current_id].set_color(YELLOW, sc)
        for neighbor in g.get_neighbors(g.grid_cells[current_id]):
            if neighbor.id in closed_set:
                continue
            new_gcost = gcost[current_id] + get_distance(current_id, neighbor.id)
            if new_gcost < gcost[neighbor.id] or not(neighbor.id in open_set):
                gcost[neighbor.id] = new_gcost
                hcost[neighbor.id] = get_distance(neighbor.id, g.goal.id)
                father[neighbor.id] = current_id
                if not (neighbor.id in open_set):
                    open_set.append(neighbor.id)
                    if neighbor.id != g.goal.id:
                        g.grid_cells[neighbor.id].set_color(RED, sc)
        if current_id != g.start.id:
            g.grid_cells[current_id].set_color(BLUE, sc)

    if father[g.goal.id] == -1:
        print("No path found")
        return
    
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    length = stroke_path(g, path, sc)

    print("Done! Total segments: %d. Calculated length: %d. Actual length: %d"
          % (len(path)-1, gcost[g.goal.id], length))
    
def stroke_path(g : SearchSpace, path : list[int], sc:pygame.Surface) -> int:
        length = 0
        for i in range(len(path)):
            if i >= len(path) - 1:
                break
            node_id = path[i]
            next_id = path[i+1]
            length += get_distance(node_id, next_id)
            start_point = g.grid_cells[node_id].rect.center
            end_point = g.grid_cells[next_id].rect.center
            pygame.draw.line(surface = sc, color = WHITE, start_pos=start_point, end_pos=end_point, width=4)
            pygame.time.delay(2)
            pygame.display.update()
        return length

#Refs: https://github.com/SebLague/Pathfinding/blob/master/Episode%2003%20-%20astar/Assets/Scripts/Pathfinding.cs
def get_distance(node_a_id : int, node_b_id : int) -> int:
    diff_x = abs(node_a_id//COLS - node_b_id//COLS)
    diff_y = abs(node_a_id%COLS - node_b_id%COLS)
    if diff_x > diff_y:
        return 14*diff_y + 10*(diff_x-diff_y)
    return 14*diff_x + 10*(diff_y-diff_x)