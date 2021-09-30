from typing import Counter
from numpy.lib.function_base import append
import picar_4wd as fc
import time
from math import sin, cos, radians
import numpy as np
import matplotlib.pyplot as plt
import my_astar as aS
import modded_detect_picamera as dp

ANGLE_RANGE = 180
STEP = 18
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2 #90
min_angle = -ANGLE_RANGE/2 #-90
scan_list = []
angle_list = []

raw_xs=[]
raw_ys=[]
xs_rounded=[]
ys_rounded=[]
xs_adjusted=[]
ys_adjusted=[]
hit_angles=[]
hits_dist_threshold=60

map_range=4000
sensor_x=map_range//2
sensor_y=0

Map=np.zeros((map_range, map_range), dtype=np.uint8)
obs_val=10
filler_val=5
buffer_val=1
left_bound=0
right_bound=0
top_bound=0

interp_xs=[]
interp_ys=[]

def scan_180():
    distances=[]
    angles=[]

    fc.servo.set_angle(max_angle)
    time.sleep(.5)

    current_angle=max_angle
    current_step=-STEP

    while current_angle>=min_angle:
        current_dist=fc.get_distance_at(current_angle)
        # time.sleep(.5)
        distances.append(current_dist)
        angles.append(current_angle)
        current_angle+=current_step
    
    return (distances, angles)

def init_map():
    d, a=scan_180()
    update_map(d, a)

def euclidean_dist(A, B):
    return ((A[0]-B[0])**2 + (A[1]-B[1])**2)**(1/2)

def update_map(distances, angles):
    global Map, raw_xs, raw_ys, xs_rounded, ys_rounded, xs_adjusted, sensor_x, hit_angles, left_bound, right_bound, top_bound
    assert len(distances)==len(angles)
    assert max(distances)<5000
    
    if max(distances)==-2:
        return False

    for d, a in zip(distances, angles):
        if d!=-2 and d<10000: #-2 indicates no object detected so only process if !=-2, also account for abnormally large readings
            a_rad=radians(-a)
            x=d*sin(a_rad)
            y=d*cos(a_rad)
            raw_xs.append(x)
            raw_ys.append(y)
            rounded_x=round(x)
            rounded_y=round(y)
            xs_rounded.append(rounded_x)
            ys_rounded.append(rounded_y)
            x_adjusted=rounded_x+sensor_x
            y_adjusted=rounded_y+sensor_y
            xs_adjusted.append(x_adjusted)
            ys_adjusted.append(y_adjusted)
            Map[y_adjusted, x_adjusted]=obs_val
            hit_angles.append(round(a))

    #interpolate points and add to map
    for i in range(1, len(hit_angles)):
        if hit_angles[i]-hit_angles[i-1]==STEP or hit_angles[i]-hit_angles[i-1]==-STEP:
            A=(xs_adjusted[i-1], ys_adjusted[i-1])
            B=(xs_adjusted[i], ys_adjusted[i])
            if euclidean_dist(A, B)<hits_dist_threshold:
                interpolate_step(A, B)

    print('{} scan hits'.format(len(hit_angles)))
    print('interpolated {} points total'.format(len(interp_xs)))

    margin=80
    left_bound=max(0, min(xs_adjusted)-margin)
    right_bound=min(map_range, max(xs_adjusted)+margin)
    top_bound=min(map_range, max(ys_adjusted)+margin)

    return True

def plot_map_imshow():

    fig, ax=plt.subplots()
    plt.axis([left_bound, right_bound, -10, top_bound])
    plt.grid(True, color='lightsteelblue', linewidth=.3)
    ax.set_xlabel('Absolute coordinates')
    x_sec_ax=ax.secondary_xaxis('top', functions=(lambda x: x-sensor_x, lambda x: x+sensor_x))
    x_sec_ax.set_xlabel('Relative coordinates (to sensor)')
    y_sec_ax=ax.secondary_yaxis('right', functions=(lambda y: y-sensor_y, lambda y: y+sensor_y))
    
    ax.imshow(Map, cmap="Greys", origin='lower')
    
    ax.plot(sensor_x, sensor_y, 'ro', label='sensor', markersize=5)
    
    ax.legend(loc='upper left')

    plt.savefig('./self_driving/output/imshow_plot.png')

def plot_map_plot():
    plt.imshow( Map[0:max(ys_adjusted)+10, sensor_x-100:sensor_x+1], cmap="Greys", origin='lower')
    for x, y, x_a, y_a, a, i in zip(xs_rounded, ys_rounded, xs_adjusted, ys_adjusted, hit_angles, range(len(hit_angles))):
        plt.annotate(str(a)+'o, '+str(x)+', '+str(y), (x_a, y_a), ha='center', color='dimgray', size=6, textcoords='offset points', xytext=(5,5))
    plt.grid(True, color='lightsteelblue', linewidth=.3)
    plt.savefig('./self_driving/output/imshow_plot.png')
    
    fig, ax=plt.subplots()
    plt.axis([left_bound, right_bound, -10, top_bound])
    
    ax.plot(xs_adjusted, ys_adjusted, 'b.', label='obstacles')
    ax.plot(sensor_x, sensor_y, 'ro', label='sensor')
    line=[]
    for x, y, x_a, y_a, a, i in zip(xs_rounded, ys_rounded, xs_adjusted, ys_adjusted, hit_angles, range(len(hit_angles))):
        plt.annotate(str(a)+'o, '+str(x)+', '+str(y), (x_a, y_a), ha='center', color='dimgray', size=6, textcoords='offset points', xytext=(5,5))
        #plot line segments between consecutive scan hits
        if i>0 and (hit_angles[i]!=hit_angles[i-1]-STEP or euclidean_dist((x_a, y_a), (xs_adjusted[i-1],ys_adjusted[i-1]) )>hits_dist_threshold):
            if len(line)>=2:
                line_x, line_y=zip(*line)
                ax.plot(line_x, line_y, linewidth=.5)
                line=[]
            else:
                line=[]
        line.append((x_a, y_a))
    if len(line)>=2:
        line_x, line_y=zip(*line)
        ax.plot(line_x, line_y, linewidth=.5)

    ax.set_xlabel('Absolute coordinates')
    x_sec_ax=ax.secondary_xaxis('top', functions=(lambda x: x-sensor_x, lambda x: x+sensor_x))
    x_sec_ax.set_xlabel('Relative coordinates (to sensor)')
    y_sec_ax=ax.secondary_yaxis('right', functions=(lambda y: y-sensor_y, lambda y: y+sensor_y))

    ax.legend(loc='upper left')
    plt.grid(True, color='gray', linewidth=.5)
    plt.savefig('./self_driving/output/pltplot_plot.png')
    
def interpolate_linear(A, B):
    global interp_xs, interp_ys, Map

    # print('interpolating between {} and {}'.format(A, B))
    if B[0]==A[0]: #vertical line, same x, increment y
        new_ys=list(range(A[1]+1, B[1]))
        new_xs=A[0]*len(new_ys)
    elif B[1]==A[1]: #horizontal line, same y, increment x
        new_xs=list(range(A[0]+1, B[0]))
        new_ys=A[1]*len(new_xs)
    else: #sloped line, calc the equation
        slope=(B[1]-A[1])/(B[0]-A[0])
        intercept=A[1]-slope*A[0]
        new_xs=list(range(min(A[0]+1, B[0]), max(A[0]+1, B[0])))
        new_ys=[round(slope*x+intercept) for x in new_xs]

    if len(new_xs)>0:
        for x, y in zip(new_xs, new_ys):
            Map[y, x]=filler_val

    # print('interpolated {} points between {} and {}'.format(len(new_xs), A, B))

    interp_xs+=new_xs
    interp_ys+=new_ys

def interpolate_step(A, B):
    global interp_xs, interp_ys, Map

    if A==B:
        return
    delta_x=B[0]-A[0]
    delta_y=B[1]-A[1]

    # print('interpolating between {} and {}'.format(A, B))
    if delta_x==0: #vertical line, same x, increment y
        if delta_y>0:
            new_ys=list(range(A[1]+1, B[1]))
        else:
            new_ys=list(range(B[1]+1, A[1]))
        new_xs=A[0]*len(new_ys)
        Map[new_ys, new_xs]=filler_val
    elif delta_y==0: #horizontal line, same y, increment x
        if delta_x>0:
            new_xs=list(range(A[0]+1, B[0]))
        else:
            new_xs=list(range(B[0]+1, A[0]))
        new_ys=A[1]*len(new_xs)
        Map[new_ys, new_xs]=filler_val
    else: #sloped line, calc the equation
        x_incre=np.sign(delta_x)
        y_incre=np.sign(delta_y)
        current_x=A[0]
        current_y=A[1]
        i=np.random.randint(0,2)
        while True:
            if i%2==0 and current_x!=B[0]:
                current_x+=x_incre
            elif i%2!=0 and current_y!=B[1]:
                current_y+=y_incre
            
            if current_x==B[0] and current_y==B[1]:
                break

            Map[current_y, current_x]=filler_val
            
            interp_xs.append(current_x)
            interp_ys.append(current_y)
            
            i+=1

def save_map(file):
    export_array=Map[0:200, 4500:5500]
    np.savetxt(file, export_array)

def load_map(file):
    import_array=np.loadtxt(file, dtype=np.uint8)
    Map[0:200, 4500:5500]=import_array

def find_path_n_plot(dest):
    map_obj=aS.Map()
    map_obj.load_array(Map)
    path=aS.astar_search(map_obj, (sensor_x,sensor_y), dest)
    
    path_xs=list(zip(*path))[0]
    path_ys=list(zip(*path))[1]
    plt.plot(path_xs, path_ys, label='path')
    plt.plot(dest[0], dest[1], 'gx', label='destination')
    plt.legend(loc='upper left')

    plt.savefig('./self_driving/output/path_plot.png')

    plt.figure()
    plt.axis([left_bound, right_bound, -10, top_bound])
    plt.grid(True, color='lightsteelblue', linewidth=.3)
    plt.imshow(map_obj.get_buffer(), cmap="Greys", origin='lower')
    plt.savefig('./self_driving/output/buffer_map.png')

    with open('./self_driving/output/path.txt', 'w') as f:
        f.write(str(path)+'\n')
        f.write('\n'+str(path_xs))
        f.write('\n'+str(path_ys))

    return path

class Move():
    def __init__(self, type, target_angle):
        self.type=type
        self.target_angle=target_angle
    def __eq__(self, other_move):
        return self.type==other_move.type and self.target_angle==other_move.target_angle
    def __str__(self):
        return '({}, {})'.format(self.type, self.target_angle)


def plan_route(path):
    moves=[]
    actions=[]
    
    moves_dict={( 0, 1): Move('forward', 0),
                ( 0,-1): Move('backward', 0),
                (-1, 0): Move('turn', 90),
                ( 1, 0): Move('turn', -90),
                ( 1, 1): Move('turn', -45),
                ( 1,-1): Move('turn', -135),
                (-1,-1): Move('turn', 135),
                (-1, 1): Move('turn', 45) }
    
    for i in range(1, len(path)):
        diff=(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])        
        moves.append(moves_dict[diff])
    count=1
    for i in range(1, len(moves)):
        current_move=moves[i]
        last_move=moves[i-1]
        if current_move==last_move:
            count+=1
        else:
            actions.append((last_move, count))
            count=1
        if i==len(moves)-1:
            actions.append((current_move, count))
    
    print('\nactions:')
    for action in actions:
        print(action[0], end=' ')
        print(action[1])
    print()
    return actions

def execute(actions):
    current_angle=0
    for i in range(len(actions)):
        """
        if this is not the first action:
            calculate angle difference between previous move and current move
            turn the diff angle
        forward(distance=count)
        """
        current_action_move=actions[i][0]
        last_action_move=actions[i-1][0]
        current_action_count=actions[i][1]
        if i==0 and current_action_move.target_angle!=0:
            turn(current_action_move.target_angle)
            current_angle=current_action_move.target_angle
        if i!=0:
            diff_angle=current_action_move.target_angle-last_action_move.target_angle
            if diff_angle!=0:
                turn(diff_angle)
                current_angle+=diff_angle
        if current_angle!=0:
            distance=2**(1/2)*current_action_count
        else:
            distance=current_action_count
        forward(distance)

def forward(distance):
    detect_stopsign()
    power=35
    speed=6 #cm/second
    duration=distance/speed

    print('forward {} cm'.format(distance))

    fc.forward(power)
    time.sleep(duration)
    
    fc.stop()
    time.sleep(.5)

def turn(angle):
    duration=1

    print('turn {}*'.format(angle))

    if angle==-45:
        power=67
        fc.turn_right(power)
    elif angle==-90:
        power=97
        fc.turn_right(power)
    elif angle==45:
        power=67
        fc.turn_left(power)
    elif angle==90:
        power=105
        fc.turn_left(power)
    else:
        print('angle {} not accepted'.format(angle))
    time.sleep(duration)
    fc.stop()
    time.sleep(.5)

def detect_stopsign():
    threshold=0.4
    path='/home/pi/picar-4wd/self_driving/data/'
    labels = dp.load_labels(path+'coco_labels.txt')
    interpreter = dp.Interpreter(path+'detect.tflite')
    objs=dp.scan_once(labels, interpreter, threshold, 0)
    important_objs=['stop sign', 'traffic light']
    
    for obj in objs:
        if obj in important_objs:
            fc.stop()
            print('detected '+obj+', wait 5 sec')
            time.sleep(5)
            return
    print('no stop sign or traffic light detected, continue')

def main():
    init_map()
    plot_map_plot()
    plot_map_imshow()
    path=find_path_n_plot((1990, 75))

    actions=plan_route(path)
    execute(actions)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()