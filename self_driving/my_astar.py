import numpy as np
import matplotlib.pyplot as plt
import heapq

class Node:
    def __init__(self, parent=None, position=None):
        self.g=0
        self.h=0
        self.f=0
        self.j=0 #added change of direction cost

        self.parent=parent
        self.position=position

    def __eq__(self, other_node):
        return self.position==other_node.position

    def __lt__(self, other_node):
        return self.f<other_node.f

    def __gt__(self, other_node):
        return self.f>other_node.f

class Map:
    def __init__(self, rows=1, cols=1):
        self.rows=rows
        self.cols=cols
        self.array=np.zeros((rows, cols), dtype=np.uint8)
        self.buffer_arr=np.zeros((rows, cols), dtype=np.uint8)

    def load_list(self, ls):
        self.array=np.asarray(ls, dtype=np.uint8)
        self.rows=self.array.shape[0]
        self.cols=self.array.shape[1]
        self.buffer()

    def load_array(self, external_array):
        assert sum(external_array.shape)>0, 'map array cannot be empty'
        self.array=external_array
        self.rows=self.array.shape[0]
        self.cols=self.array.shape[1]
        self.buffer()

    #position: x, y. y -> row, x-> column
    def query(self, position):
        return self.array[position[1], position[0]]

    def update(self, position, value):
        self.array[position[1], position[0]]=value

    def obstacles(self, obs_val=10):
        where1s=np.where(self.array==obs_val)
        xs=list(where1s[1])
        ys=list(where1s[0])
        return list(zip(*[xs, ys])), xs, ys

    def fillers(self, filler_val=5):
        where1s=np.where(self.array==filler_val)
        xs=list(where1s[1])
        ys=list(where1s[0])
        return list(zip(*[xs, ys])), xs, ys

    def buffer(self, r=9):
        print('buferring starting')
        obs_xys, obs_xs, obs_ys=self.obstacles()
        self.buffer_arr=np.zeros((self.rows, self.cols), dtype=np.uint8)
        for x, y in zip(obs_xs, obs_ys):
            self.buffer_arr[y-r-1:y+r+1, x-r-1:x+r+1]=1

        filler_xys, filler_xs, filler_ys=self.fillers()        
        for x, y in zip(filler_xs, filler_ys):
            self.buffer_arr[y-r-1:y+r+1, x-r-1:x+r+1]=1
        print('buferring completed')

    def get_buffer(self):
        return self.buffer_arr

    def query_buffer(self, position):
        return self.buffer_arr[position[1], position[0]]

def get_path(node, start_node):
    path=[]
    current_node=node
    while current_node is not None:
        path.append(current_node.position)
        current_node=current_node.parent
    return path[::-1] #reverse to get path from start to end

def astar_search(map: Map, start, end):
    print('path finding starting')
    #create start and end nodes
    start_node=Node(position=start)
    end_node=Node(position=end)

    #create lists to store nodes
    unvisited=[]
    visited=[]

    heapq.heapify(unvisited) 
    heapq.heappush(unvisited, start_node)

    num_cols=map.cols
    num_rows=map.rows
    num_iter=0
    max_iter=num_cols*num_rows//2

    moves=[    ( 0, 1),
            ( 0,-1),
            (-1, 0),
            ( 1, 0),
            ( 1, 1),
            ( 1,-1),
            (-1,-1),
            (-1, 1)]

    cost_lat=1
    cost_diag=1.4142 #2**(1/2)
    cost_dir_change=40

    if map.query_buffer(end)==1:
        return 'destination cannot be reached due to car size'

    while len(unvisited)>0:
        num_iter+=1

        current_node = heapq.heappop(unvisited)
        visited.append(current_node)

        if num_iter>max_iter:
            return 'Too many iterations. Quitting'
            # break

        if current_node==end_node:
            print('Path found')
            return get_path(current_node, start_node)
        
        children=[]
        costs=[]

        for move in moves:
            new_position=(current_node.position[0]+move[0], current_node.position[1]+move[1])

            if (new_position[0]<0 or new_position[0]>num_cols-1 #x
                or new_position[1]<0 or new_position[1]>num_rows-1): #y
                continue

            # if map.query(new_position)!=0:
            if map.query_buffer(new_position)!=0:
                continue

            children.append(Node(parent=current_node, position=new_position))

            if move[0]==0 or move[1]==0:
                costs.append(cost_lat)
            else:
                costs.append(cost_diag)

            # print('child', new_position, 'appended to children list')

        for child, cost in zip(children, costs):
            if sum([child==node for node in visited])>0:
                continue
            
            child.g=current_node.g+cost
            # child.h=((child.position[0]-end_node.position[0])**2 + (child.position[1]-end_node.position[1])**2) ** (1/2)
            delta_x=abs(end[0]-child.position[0])
            delta_y=abs(end[1]-child.position[1])
            child.h=(delta_x+delta_y) + (cost_diag-2)*min(delta_x, delta_y)
            
            # if current_node.parent is not None:
            #     parent_postion=current_node.parent.position
            #     current_postion=current_node.position            
            #     last_move=(current_postion[0]-parent_postion[0], current_postion[1]-parent_postion[1])
            #     current_move=(child.position[0]-current_postion[0], child.position[1]-current_postion[1])
            #     if last_move!=current_move:
            #         child.j=cost_dir_change
            #     else:
            #         child.j=0
            
            child.f=child.g + child.h + child.j

            # if sum([child==node and child.g>node.g for node in unvisited])>0:
            #     continue
            index=None
            for i, node in enumerate(unvisited):
                if child==node:
                    index=i
            if index:
                if child.g>unvisited[index].g:
                    continue
                else:
                    unvisited[index]=unvisited[-1]
                    unvisited.pop()
                    if index<len(unvisited):
                        heapq._siftup(unvisited, index)
                        heapq._siftdown(unvisited, 0, index)

            heapq.heappush(unvisited, child)

    print('could not find a path')

if __name__ == '__main__':
    ls = [[0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 0]]

    ex_map=Map()
    ex_map.load_list(ls)
    print(ex_map.array)
    # print(ex_map.query((1, 2)))
    
    _, xs, ys=ex_map.obstacles()
    plt.plot(xs, ys,  'b.')

    path=astar_search(ex_map, (0,0), (5,4))
    path_xs=list(zip(*path))[0]
    path_ys=list(zip(*path))[1]
    plt.plot(path_xs, path_ys)
    plt.grid(True, color='gray', linewidth=.5)