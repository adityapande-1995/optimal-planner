import numpy as np
import matplotlib.pyplot as plt
import time
from collections import defaultdict
from helper_functions import *
import pickle

class LQR_RRT_planner:
    def __init__(self, Qmat,Rmat, time_limit = 1.0, workspace = 1):
        self.Q = Qmat ; self.R = Rmat
        self.time_limit = time_limit

        # Define workspace
        a = 20 ; b = 100
        if workspace == 1:
            self.path_rects = np.array([
                [[0,0],[a,0],[a,b],[0,b]],
                [[0,b-a],[b,b-a],[b,b],[0,b]]
            ])

            self.start = [0,0,90-2.0,0,0,0] ; self.goal = [b,b,2.0,0,0,0]

        elif workspace == 2:
            H = (b-a)/2.0
            self.path_rects = np.array([
                [[0,0],[a,0],[a,H],[0,H]],
                [[0,H],[b,H],[b,H+a],[0,H+a]],
                [[b-a, b-H],[b,b-H],[b,b],[b-a,b]]
            ])

            self.start = [0,0,90-2.0,0,0,0] ; self.goal = [b,b,2.0,0,0,0]

        elif workspace == 3:
            self.path_rects = np.array([
                [[0,0],[a,0],[a,b],[0,b]],
                [[0,b-a],[b,b-a],[b,b],[0,b]],
                [[b-a,0],[b,0],[b,b],[b-a,b]]
            ])

            self.start = [0,0,90-2.0,0,0,0] ; self.goal = [b,0,2.0,0,0,0]

        elif workspace == 4:
            self.path_rects = np.array([
                [[0,0],[a,0],[a,b],[0,b]],
                [[0,b-a],[a,b],[b,a],[b-a,0]]
            ])

            self.start = [0,0,90-2.0,0,0,0] ; self.goal = [b,a,2.0,0,0,0]

        elif workspace == 5:
            self.path_rects = np.array([
                [[0,0],[a,0],[a,b],[0,b]],
                [[0,b-a],[a,b],[b,a],[b-a,0]],
                [[b-a,0],[b,0],[b,b],[b-a,b]]
            ])

            self.start = [0,0,90-2.0,0,0,0] ; self.goal = [b,b,2.0,0,0,0]


        self.state_names = ["x", "y", "theta", "vx", "vy", "omega"]

        xdot_scale = 0.2 ; angle_scale = 0.2
        self.state_limits = [[0,b],[0,b],[0,360],[-b*xdot_scale,b*xdot_scale],
                             [-b*xdot_scale,b*xdot_scale],[-360*angle_scale, 360*angle_scale]]

        self.nodes = [self.start, self.goal]
        self.graph = defaultdict(list)
        self.parent_node = {}

        self.model = Omni_Vehicle(Q = Qmat, R = Rmat)

    def lqr_cost(self,state1,state2):
        return self.model.get_LQR_cost(list(state1), list(state2))

    # Returns 1 if point is valid
    def is_point_clear(self,point):
        ret = 0
        for i,polygon in enumerate(self.path_rects):
            if is_inside(point,polygon) >= 0 : ret = 1; break

        return i,ret

    # Tells if line between p1 and p2 goes through any obstacle
    def  is_line_clear(self,p1,p2, divisions = 20):
        n = np.linspace(0,1,divisions)
        for fraction in n:
            px = p1[0] + (p2[0] - p1[0])*fraction
            py = p1[1] + (p2[1] - p1[1])*fraction
            _,ret = self.is_point_clear([px,py])
            if ret != 1 : return 0;

        return 1

    def steer(self, closest_node, q_rand):
        time_limit = self.time_limit
        cn_mat = np.matrix(closest_node).T
        qr_mat = np.matrix(q_rand).T

        ta,sa,_ = simulate(self.model, cn_mat, qr_mat, linearization= cn_mat)
        if ta[-1] < time_limit : q_new = q_rand
        else:
            i = np.argmax( np.array(ta) > time_limit )
            q_new = tuple( sa[i].flatten().tolist() )

        return q_new

    def generate_tree(self,r = 2.0, p_goal = 0.05,n = 5000, e = 10 ):
        print("Generating the tree...")
        # Tree init at start
        self.graph[tuple(self.start)] = []
        while len(list(self.graph.keys())) < n:
            # Get q_rand
            choice = np.random.choice(a=["goal", "random"], size = 1, replace = True, p = [p_goal, 1 - p_goal])

            if choice[0] == "goal": q_rand = tuple(self.goal)
            else :
                q_rand = tuple([  np.random.uniform( L, U, 1)[0] for L,U in self.state_limits ])

            # Get nearest node in graph to q_rand
            # Init with the start node first 
            closest_node = tuple(self.start) ;
            closest_dist = self.lqr_cost(q_rand,closest_node)
            for node in self.graph:
                temp_dist = self.lqr_cost(q_rand,node)
                if temp_dist < closest_dist :
                    closest_dist = temp_dist; closest_node = node

            print("\nSelected q_rand ", q_rand)
            # Generate q_new
            cost_limit = 500

            q_new = self.steer(closest_node, q_rand)

            print("q_new", q_new)
            print("closest node", closest_node)
            #time.sleep(1)
            if self.is_line_clear(closest_node, q_new) :
                print("** Line clear between ", closest_node, q_new)
                #time.sleep(1)
                self.nodes.append( list(q_new) )
                #temp_dist = dist(closest_node, q_new)
                temp_dist = self.lqr_cost(q_new,closest_node)
                print("** Joining line with cost ", temp_dist)
                print("Number of nodes in graph: ", len(list(self.graph.keys())))

                self.graph[closest_node].append( (temp_dist, q_new)   )
                self.graph[q_new].append( (temp_dist,closest_node) )
                self.parent_node[q_new] = closest_node

                #if dist(q_new, self.goal) < e adn self.is_line_clear(q_new, self.goal):
                if self.lqr_cost(q_new, self.goal) < cost_limit and self.is_line_clear(q_new,self.goal):
                    print("Tree reached the goal with node !!", q_new, "\n")
                    #temp_dist = dist(self.goal, q_new)
                    temp_dist = self.lqr_cost(q_new,self.goal)

                    goal_tuple = tuple(self.goal)
                    self.graph[goal_tuple].append( (temp_dist, q_new)  )
                    self.graph[q_new].append( (temp_dist, goal_tuple) )
                    if goal_tuple != q_new : self.parent_node[goal_tuple] = q_new
                    
                    return True

        return False

    def smoothen_path(self, debug = 1):
        goal = tuple(self.goal)
        start = tuple(self.start)

        path_len2 = 0
        self.path2 = []
        if self.path_found_flag:
            print("Smothening the path..")
            path2 = [self.path[0]]
            while path2[-1] != goal:
                i = self.path.index(path2[-1])
                if debug: print("Inspecting node ", i, " of ", len(self.path), " coords :", self.path[i])
                if i == len(self.path) - 2 : break
                next_index = i+1
                for j in range(i+1, len(self.path)):
                    
                    if debug: print("  Checking for cheaper path with index, coords: ", j, self.path[j])
                    cost_along = 0
                    for z in range(i,j):
                        incremental_cost = self.lqr_cost(self.path[z], self.path[z+1])
                        print("Adding cost from ",z," to ", z+1, " = ", incremental_cost)
                        cost_along += incremental_cost

                    cost_direct = self.lqr_cost(self.path[i], self.path[j])
                    print("Cost along:" ,cost_along)
                    print("Cost direct:", cost_direct)


                    if self.is_line_clear(self.path[i], self.path[j], 100) and cost_direct < cost_along:
                        next_index = j
                        if debug: print("    Shorter and clear path found from ", self.path[i], self.path[j])

                path2.append(self.path[next_index])
                self.path2 = path2

            print("Path2 - ", self.path2)

            if tuple(self.path2[-1]) != tuple(self.goal) : self.path2.append(self.goal)

    def trace_parents(self):
        current_node = tuple(self.goal)
        print("Starting backtrace--",current_node)
        self.path = [current_node]
        while current_node != tuple(self.start) :
            current_node = self.parent_node[current_node]
            print("node--", current_node)
            self.path.append(current_node)

        self.path = self.path[::-1]
        print("Raw path :",self.path)

        self.path_found_flag = True
        self.smoothen_path()


    def draw(self, display=False):
        fig1 = plt.figure()
        for ob in self.path_rects:
            x = ob[:,0].tolist() + [ob[0,0]]
            y = ob[:,1].tolist() + [ob[0,1]]

            plt.plot(x,y)

        plt.scatter(self.start[0], self.start[1])
        plt.scatter(self.goal[0], self.goal[1])

        self.nodes = np.array(self.nodes)

        print("\nPath :", self.path)
        plt.scatter(self.nodes[:,0], self.nodes[:,1], s = 1)

        for node in self.graph:
            for cost, neb_node in self.graph[node]:
                x = [node[0], neb_node[0]]
                y = [node[1], neb_node[1]]
                plt.plot(x,y, 'black', linewidth='0.1')
                plt.text((node[0] + neb_node[0])/2 , (node[1] + neb_node[1])/2, str(round(cost,1)), size=10)

        plt.xlim(self.state_limits[0][0], self.state_limits[0][1])
        plt.ylim(self.state_limits[1][0], self.state_limits[1][1])


        # Drawing the path
        if self.path_found_flag :
            for i in range(len(self.path) -1):
                x = [self.path[i][0], self.path[i+1][0]]
                y = [self.path[i][1], self.path[i+1][1]]
                plt.plot(x,y,'red', linewidth='2')

            print("\nSmoothened path :", self.path2)
            for i in range(len(self.path2) -1):
                x = [self.path2[i][0], self.path2[i+1][0]]
                y = [self.path2[i][1], self.path2[i+1][1]]
                plt.plot(x,y,'blue', linewidth='2')

        # Drawing states
        no_of_states = len(self.start)
        fig, ax = plt.subplots(no_of_states+1,1)
        fig.suptitle("Target States vs node index in the optimal path")
        for i in range(no_of_states) :
            ax[i].plot([ state[i] for state in self.path2 ], marker='.', linestyle='None')
            ax[i].set_ylabel(self.state_names[i]) ; ax[i].grid()

        costs = [0]
        for i in range(len(self.path2)-1):
            temp_cost = self.lqr_cost( self.path2[i], self.path2[i+1])
            print("Eigenvalues at state ",i,get_cl_eig(self.model, self.path2[i]),flush=True)
            prev_cost = costs[-1]
            print("Cost prev, addition : ", prev_cost, temp_cost, flush=True)
            costs.append(prev_cost + temp_cost)

        print("Final cost :", costs[-1], flush = True)

        print("Cost from direct start to goal: ", self.lqr_cost(self.start, self.goal), flush=True)

        ax[-1].plot(costs)
        ax[-1].set_ylabel("Cumulative LQR cost" ); ax[-1].grid()

        fig1.savefig('path.png')
        fig.savefig('states.png')
        if display: plt.show()


# Main 
if __name__ == "__main__":
    a = LQR_RRT_planner(Qmat = np.diag([1,1,1,0.1,0.1,0.1])*2.0 , Rmat = np.diag([1,1,1,1])*0.5 , time_limit = 1.5, workspace=1)
    if a.generate_tree(p_goal=0.05, e = 20):
        a.trace_parents()

        a.draw(display = False)
        # trajectory --> Time, states, controls 
        trajectory = navigate_path(a.model, a.path2, display = False)

        pickle.dump( trajectory, open( "traj.p", "wb" ) )
        pickle.dump((a.path_rects, a.path2), open("otherdata.p","wb") )
        with open("QR.txt", "w") as file1:
            file1.write("\nQ: \n" + str(a.Q) )
            file1.write("\nR: \n" + str(a.R) )
            file1.write("\nTime: " + str(trajectory[0][-1]) )

        animate(trajectory[0],trajectory[1], a.path_rects, a.path2)

