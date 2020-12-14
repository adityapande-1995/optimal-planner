#!python3
from sympy import *

# Omni drive robot - 4 wheels
x,y,theta, v_x, v_y, w = symbols('x,y,theta, v_x, v_y, w')
M,L,J,R = symbols('M,L,J,R')
t_1,t_2, t_3, t_4 = symbols('t_1,t_2, t_3, t_4') 

F = Matrix([ v_x, v_y , w,  sin(theta)*(t_1 + t_2 + t_3 + t_4)/(sqrt(2)*M*R), cos(theta)*(t_2 - t_1 + t_4 -t_3)/(sqrt(2)*M*R), L*(t_2 + t_3 - t_1 - t_4)/(J*R) ])
X = Matrix([x,y,theta, v_x, v_y, w])
U = Matrix([t_1,t_2, t_3, t_4])

A = F.jacobian(X).subs([(t_1,0),(t_2,0),(t_3,0), (t_4,0)])
B = F.jacobian(U)
b_s = B.subs([(M,1),(R,0.1),(L,0.5),(J,2),(theta,0.1)])
# X_dot = F
pprint(X)
pprint(F)
pprint(A)
pprint(B)
