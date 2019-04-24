# Authors: Collier Crisanti, Aron Harder
# CS6501-003 Spring 2019

from z3 import *
import re
from graph2 import Graph
import datetime

def abs2(x):
    return If(x >= 0, x, -x)
def adj_obs(x,o):
    nearby = 1
    output = []
    for i in range(num_o):
        output.append(If( abs2(x[0]-o[i][0])+abs2(x[1]-o[i][1])<=nearby, 1, 0 ))
    return output

#NOTE: This function a major flaw. Because the array checks itself, it doesn't
#  actually check full coverage, it only checks if the drone is connected to
#  drone 0.
def check_conn(e,num_d,max_t):
    check = [[False for d in range(num_d)] for t in range(max_t) ]
    for t in range(max_t):
        check[t][0] = True
        for d_id1 in range(1,num_d):
            """
            a = []
            for d_id2 in range(num_d):
                if d_id2 == d_id1:
                    continue
                a.append( And(e[t][d_id1][d_id2],check[t][d_id2]) )
            check[t][d_id1] = Or( a )
            """
            #Simplified version of broken code
            check[t][d_id1] = e[t][d_id1][0]
    for t in range(max_t):
        check[t] = And(check[t])
    return And(check)

def check_anet(d,o,num_d,max_t):
    alpha = 10
    Edges = [ [[Bool('e_%s_%s_%s' % (t,i,j)) for i in range(num_d)] for j in range(num_d)] for t in range(max_t) ]
    for t in range(max_t):
        for d_id1 in range(num_d):
            for d_id2 in range(num_d):
                Edges[t][d_id1][d_id2] = dist(d,d_id1,d_id2,o,t) <= alpha**2
    return check_conn(Edges,num_d,max_t)

def dist(d,d_id1,d_id2,o,t):
    gamma = 1
    #NOTE: dist DOES NOT take the sqrt because z3 can't solve for sqrt.
    #  As a workaround, square the other side instead
    return (d[d_id1][t][0]-d[d_id2][t][0])**2+(d[d_id1][t][1]-d[d_id2][t][1])**2+sum(adj_obs(d[d_id1][t],o))*gamma+sum(adj_obs(d[d_id2][t],o))*gamma

s = Solver()
output_model = True

f = open("input.txt","r")
for line in f:
    if line == "\n":
        time1 = datetime.datetime.now()
        print(time1)
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

        # The network graph is connected
        network = check_anet(D,o,num_d,max_t)

        s.add(visit_waypoints)
        s.add(start_end_pos)
        #s.add(start_pos)
        s.add(drone_move)
        s.add(drone_avoid)
        s.add(obstacle_avoid)
        s.add(env_remain)
        s.add(network)

        chk = s.check()
        print(chk)
        time2 = datetime.datetime.now()
        print(time2)
        print(time2-time1)
        if output_model and chk == sat:
            out = s.model()
            for d in range(num_d):
                print("Drone "+str(d))
                for t in range(max_t):
                    print(str(out[D[d][t][0]])+","+str(out[D[d][t][1]]))
                print()
        s.reset()
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
            D = [ [[Int("x%s_%s" % (i,t)), Int("y%s_%s" % (i,t))] for t in range(max_t)] for i in range(num_d)]
        elif line[0] == "O":
            o = re.findall('\((\d+),(\d+)\)',var)
            o = [(int(a[0]),int(a[1])) for a in o]
            num_o = len(o)
        elif line[0] == "t":
            max_t = int(re.findall('\d+',var)[0])
        else:
            print("Unknown token "+line[0]+" detected")

