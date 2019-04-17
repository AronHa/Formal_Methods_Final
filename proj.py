# Authors: Collier Crisanti, Aron Harder
# CS6501-003 Sprinig 2019

from z3 import *
import re

def abs2(x):
    return If(x >= 0, x, -x)

s = Solver()

f = open("input.txt","r")
for line in f:
    if line == "\n":
        #Run test
        max_x = 9
        max_y = 9

        # Every waypoint gets visited
        visit_waypoints = [ Or( [Or( [ And(D[d_id][t][0] == w[w_id][0],D[d_id][t][1] == w[w_id][1]) for d_id in range(num_d)] ) for t in range(max_t)] ) for w_id in range(num_w) ]

        # Drone movement
        drone_move = [ abs2(D[d_id][t+1][0]-D[d_id][t][0])+abs2(D[d_id][t+1][1]-D[d_id][t][1])<=1 for t in range(max_t-1) for d_id in range(num_d) ]

        # The drones are never in the same place
        drone_avoid = [ And( [Or(D[d_id1][t][0] != D[d_id2][t][0],D[d_id1][t][1] != D[d_id2][t][1],And(D[d_id1][t][0]==start_x,D[d_id1][t][1]==start_y)) for d_id1 in range(num_d) for d_id2 in range(d_id1+1,num_d)] ) for t in range(max_t) ]

        # The drones do not run into obstacles
        obstacle_avoid = [ And( [And( [Or(D[d_id][t][0] != o[o_id][0],D[d_id][t][1] != o[o_id][1]) for d_id in range(num_d)] ) for t in range(max_t)] ) for o_id in range(num_o) ]

        # The drones do not leave the environment
        env_remain = [ And( [And(D[d_id][t][0] >= 0,D[d_id][t][0] <= max_x,D[d_id][t][1] >= 0,D[d_id][t][1] <= max_y) for d_id in range(num_d)] ) for t in range(max_t) ]

        # Drones must start and end at given position
        start_end_pos = [ And(D[d_id][0][0] == start_x,D[d_id][0][1] == start_y,D[d_id][max_t-1][0] == start_x,D[d_id][max_t-1][1] == start_y) for d_id in range(num_d) ]
        # Drones must start (but not end) at given position
        start_pos = [ And(D[d_id][0][0] == start_x,D[d_id][0][1] == start_y) for d_id in range(num_d) ]

        # Not sure how to do these two
        # Every edge in the network graph is less than alpha
        # The network graph is connected

        s.add(visit_waypoints)
        s.add(start_end_pos)
        #s.add(start_pos)
        s.add(drone_move)
        s.add(drone_avoid)
        s.add(obstacle_avoid)
        s.add(env_remain)

        chk = s.check()
        print(chk)
        #if chk == sat:
        #    print(s.model())
        s.reset()
        #break #Just doing first test case for now...
    else:
        var = line[line.index("{")+1:line.index("}")]
        if line[0] == "W":
            w = re.findall('\((\d+),(\d+)\)',var)
            w = [(int(a[0]),int(a[1])) for a in w]
            num_w = len(w)
        elif line[0] == "D":
            init_coords = re.findall('\((\d+),(\d+)\)',var)
            start_x = int(init_coords[0][0])
            start_y = int(init_coords[0][1])
            #TODO: Need a number of drones
            num_d = 2
            max_t = 40
            D = [ [[Int("x%s_%s" % (i,t)), Int("y%s_%s" % (i,t))] for t in range(max_t)] for i in range(num_d)]
        elif line[0] == "O":
            o = re.findall('\((\d+),(\d+)\)',var)
            o = [(int(a[0]),int(a[1])) for a in o]
            num_o = len(o)
        else:
            print("Unknown token "+line[0]+" detected")

