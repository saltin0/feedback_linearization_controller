'''
    :project:   Feedback Linearization controller for
                differential drive robots.
    
    :authors:   manyetar@gmail.com
                github:saltin0
    
    :reference: Will be added.
'''

import numpy as np

class FeedbackLinController():
    def __init__(self,Kx=0.0,Ky=0.0,Kr=0.0):
        self.Kx = Kx
        self.Ky = Ky
        self.Kr = Kr

        # Integral error terms
        self.int_x = 0.0
        self.int_y = 0.0

    def set_setpoint(self,x_des,y_des,r_des,x_des_dot, y_des_dot,r_des_dot = 0.0):
        self.x_des = x_des
        self.y_des = y_des
        self.r_des = r_des
        self.x_des_dot = x_des_dot
        self.y_des_dot = y_des_dot
        self.r_des_dot = r_des_dot

    def execute(self,x,y,r,x_dot,y_dot):
        self.int_x += x-self.x_des
        self.int_y += y-self.y_des
        rx = (self.x_des_dot) - self.Kx*(x-self.x_des) - self.Kx*self.int_x/5
        ry = (self.y_des_dot) - self.Ky*(y-self.y_des) - self.Ky*self.int_y/5
        rr = self.r_des_dot - self.Kr*(r-self.r_des)
        self.r_matrix = np.array(([rx,ry,rr]),dtype=np.float32).reshape(3,1)
        self.controller_outputs = np.dot(self.rotation_matrix(self.r_des),self.r_matrix)

        return self.controller_outputs 
    def debugger(self):
        debug_infos = {
            "r_matrix"          : self.r_matrix,
            "controller_out"    : self.controller_outputs
        }
        return debug_infos

    def rotation_matrix(self,psi_des):
        rot_matrix = np.array(([[np.cos(psi_des),np.sin(psi_des),0],[1,1,1]])).reshape(2,3)
        return rot_matrix


FC = FeedbackLinController()
a=FC.rotation_matrix(np.pi)
print(f"Rotation matrix : {a}")


