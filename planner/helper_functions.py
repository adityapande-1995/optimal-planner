#!python3
import numpy as np
import control
import matplotlib.pyplot as plt
import pygame
import time

# Functions for RRT

def which_side_of_line(p1,p2,p):
    d = (p[0]-p1[0])*(p2[1]-p1[1]) - (p[1]-p1[1])*(p2[0]-p1[0])
    if d > 0 : return 1
    if d < 0: return -1
    return 0

# Returns 1 if inside, 0 if on, -1 if outside
def is_inside(point,polygon):
    centroid = np.average(polygon,axis=0)
    inside = 1
    for i in range(polygon.shape[0]):
        s1 = which_side_of_line(polygon[i-1],polygon[i],centroid)
        s2 = which_side_of_line(polygon[i-1],polygon[i],point)
        if s1*s2 < 0 :
            inside = -1; break

    return inside

# Get closed loop eigenvalues of model linearized at state
def get_cl_eig(model, state):
    state = np.matrix(state).reshape((6,1))
    A,B = model.get_A_B(state)
    _,_,E = control.lqr(A,B,model.Q,model.R)
    return E

# Simulate the control system, return state
# eg state_current= np.matrix([0,0,88,0,0,0]).T  
def simulate(model, state_current, state_target, linearization, t0 = 0, dt = 0.01, plot = False):
    # Linearize at state_current
    A,B = model.get_A_B(linearization)
    K,_,_ = control.lqr( A, B, model.Q, model.R )
    time_elapsed = t0

    states_array = []
    control_array = []
    time_array = []

    reached_target = False
    while not reached_target :
        X_dot = np.matrix(A - np.matrix(B)*K) * np.matrix(state_current - state_target)
        control_array.append( -K*np.matrix(state_current - state_target) )

        state_current += X_dot*dt
        time_elapsed += dt
        states_array.append( state_current.copy() )
        time_array.append(time_elapsed)

        if time_elapsed % 10 == 0 : print("Time time: ", time_elapsed)

        difference = abs(state_current - state_target)
        if difference[0] < 2 and difference[1] < 2 and difference[2] < 10 :
            reached_target = True

    states_array = np.array(states_array)
    control_array = np.array(control_array)

    if plot:
        fig, ax = plt.subplots(2,1)
        for i in range(6): ax[0].plot(time_array, states_array[:,i,0])
        ax[0].legend(["x","y","theta","vx", "vy","theta_dot"])
        ax[0].grid() ; ax[1].grid()
        ax[0].set_title("States vs time in s")
        ax[1].set_title("Controls vs time in s")
        for i in range(2): ax[1].plot(time_array, control_array[:,i,0])
        ax[1].legend(["tau_1","tau_2"])
        plt.show()

    return time_array, states_array[:,:,0], control_array[:,:,0]

# Navigate the path obtained from planner
def navigate_path(model, path, plot = True, display = True):
    time_a = []
    states_a = None
    control_a = None

    A,B = model.get_A_B(np.matrix(path[0]).T)
    K_initial,_,_ = control.lqr( A, B, model.Q, model.R )
 
    for i in range(len(path)-1):
        # Simulate
        if i == 0:
            time_c, states_c, controls_c = simulate(model, np.matrix(path[i]).T, np.matrix(path[i+1]).T,
                                                    linearization=np.matrix(path[i]).T,
                                                    t0 = 0, plot=False)
        else:
            time_c, states_c, controls_c = simulate(model, np.matrix(path[i],dtype='float64').T,
                                                    np.matrix(path[i+1],dtype='float64').T,
                                                    linearization= np.matrix(path[i]).T,
                                                    t0 = time_a[-1], plot=False)
        # Append data
        time_a += time_c
        if i != 0:
            states_a = np.vstack((states_a, states_c))
            control_a = np.vstack((control_a, controls_c))
        else:
            states_a = states_c.copy()
            control_a = controls_c.copy()
            
    if plot:
        fig, ax = plt.subplots(3,1, figsize = (20,10))
        for i in range(6): ax[0].plot(time_a, states_a[:,i])
        ax[0].legend(["x","y","theta","vx", "vy","theta_dot"])
        ax[0].grid() ; ax[1].grid() ; ax[2].grid()
        ax[0].set_title("States vs time in s")
        ax[1].set_title("Controls vs time in s")
        ax[2].set_title("Kinetic energy vs time")
        for i in range(4): ax[1].plot(time_a, control_a[:,i])
        ax[1].legend(["tau_1","tau_2","tau_3","tau_4"])

        # 0.5*m*v^2
        energy_linear = 0.5*model.params["m"]*( states_a[:,3]**2 + states_a[:,4]**2  )
        # 0.5*I*w^2
        energy_rot = 0.5*model.params["I"]*( states_a[:,5]**2  )
        ax[2].plot(time_a, energy_linear + energy_rot)
        fig.savefig('time_series.png')
        if display:  plt.show()

    return time_a, states_a, control_a

def animate(time_a,states_a, valid_path, target_path):
    DARK = (100,100,100)
    pygame.init()
    pygame.font.init() # you have to call this at the start, 
    myfont = pygame.font.SysFont('Comic Sans MS', 30)

    screenx , screeny = 600,600
    scale_factor = 5
    screen = pygame.display.set_mode((screenx,screeny))
    pygame.display.set_caption("Navigating the path using LQR controller")
    clock = pygame.time.Clock()
    trail = [[0,screeny]]

    done = False
    i = 0
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        screen.fill(DARK)
        xc, yc, theta = states_a[i,0], states_a[i,1], states_a[i,2]
        xc ,yc = xc*scale_factor, yc*scale_factor

        trail.append([xc, screeny-yc])
        r = 10
        arrow_tip = [xc + 2*r*np.cos(theta*np.pi/180), screeny - yc - 2*r*np.sin(theta*np.pi/180)]
        pygame.draw.line(screen, (255,0,0), [xc,screeny-yc], arrow_tip,3)
        pygame.draw.circle(screen,(0,255,0), [xc, screeny - yc],r,3)
        pygame.draw.lines(screen, (255,165,0),False,trail,3)

        for node in target_path:
            x,y = node[0]*scale_factor, node[1]*scale_factor
            pygame.draw.circle(screen,(255,0,255),[x,screeny-y], r/2,3)

        for rect in valid_path:
            rect = rect*scale_factor
            rect[:,1] = screeny - rect[:,1]
            pygame.draw.lines(screen,(255,255,255),True,rect)

        textsurface = myfont.render("Sim time: "+str(round(time_a[i], 2)), False, (255, 255, 255))
        screen.blit(textsurface,(20,20))

        pygame.display.flip()

        clock.tick(100)

        i += 1
        if i == states_a.shape[0] :
            done = True
            pygame.image.save(screen, "simulation.png")

    time.sleep(1)
    pygame.quit()

# Functions for LQR
# (A,B) should be stabilizable
class Omni_Vehicle:
    # State 'X' = [x,y,theta,x_dot, y_dot, theta_dot] , Control 'u' = [tau_1,..,tau_4]
    def __init__(self, params = {"m":1, "I":1, "r":0.1, "L":0.5}, Q = np.diag([1,1,1,0.1,0.1,0.1]), R = np.eye(4)):
        self.params = params
        self.Q = Q ; self.R = R

    def get_A_B(self, state = np.matrix([0,0,1,0,0,0]).T):
        m = self.params["m"]; I = self.params["I"] ; r = self.params["r"] ; L = self.params["L"]
        theta = state[2,0]
        A = np.zeros((6,6))
        A[0,3] = 1 ; A[1,4] = 1 ; A[2,5] = 1
        temp_1 = np.sin(theta*np.pi/180)/(np.sqrt(2)*m*r)
        temp_2 = np.cos(theta*np.pi/180)/(np.sqrt(2)*m*r)
        B = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],
                      [temp_1, temp_1, temp_1, temp_1],
                      [-temp_2, temp_2, -temp_2, temp_2],
                      [ -L/(I*r), L/(I*r), L/(I*r), -L/(I*r)]])
        return A,B

    # Linearize about state1 !!!!!! ****** IMPORTANT
    def get_LQR_cost(self, state1 = [0,0,1,0,0,0], state2 = [0,0,1,0,0,0]):
        #print("Fetching LQR cost for ", state1, state2)
        state1 = np.matrix(state1).reshape((6,1))
        state2 = np.matrix(state2).reshape((6,1))
        A1,B1 = self.get_A_B(state1)
        
        K1, S1, E1 = control.lqr(A1,B1,self.Q, self.R)
        return ((state2 - state1).T * S1 * (state2 - state1))[0,0]











