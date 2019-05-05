import sys
import random
from tkinter import *
import time
import math, queue
from Init_map import *
import threading


map_dimension = 20
Robot_map = [[0 for i in range(map_dimension)] for j in range(map_dimension)]
gxBest = 0
gyBest = 0
fitness_function = 3000000#16200.0
x_dest = 808
y_dest = 403
safe_done = threading.Lock()


class robot(threading.Thread):
    def __init__(self, robotID, xpos, ypos, color, w):
        threading.Thread.__init__(self)
        self.robotID = robotID
        self.xcen = self.pxBest = xpos
        self.ycen = self.pyBest = ypos
        self.xvel = self.yvel = 0
        self.obstacle = False
        #self.sec_xvel = self.sec_yvel = self.last_sec_xvel = self.last_sec_yvel = 0
        self.current_cell = [(xpos-43)//45, (ypos-43)//45]     
        self.last_cell = [0,0]
        self.mini_xvel = 0      # velocity of a single step of robot move
        self.mini_yvel = 0      # velocity of a single step of robot move
        self.fit_func = 3000000
        self.w = w
        self.Data_tomove = self.DetermineWayQueue = queue.Queue()
        self.step = self.step_back = 0
        self.Robot = self.w.create_polygon(self.xcen-11, self.ycen-11, self.xcen, self.ycen, self.xcen+11, self.ycen-11, self.xcen+11, self.ycen+11, self.xcen-11, self.ycen+11, fill = color)
    
    # function to run
    def run(self):
        self.PSO_algorithm()
        self.Robot_move()

    #def Update_gBest_lBest(self):
    def PSO_algorithm(self):    
        global gxBest, gyBest, x_dest, y_dest, fitness_function
        dest_cell_x = (x_dest-43)//45
        dest_cell_y = (y_dest-43)//45
        fitness = pow((x_dest - self.xcen), 2) + pow((y_dest - self.ycen), 2)
        # compare the current fitness and global fitness
        if (fitness < fitness_function):
            gxBest = self.xcen
            gyBest = self.ycen
            fitness_function = fitness
        # compare the current fitness and local fitness
        if (fitness < self.fit_func):
            self.pxBest = self.xcen
            self.pyBest = self.ycen
            self.fit_func = fitness 

        #global gxBest, gyBest
        local_xvel = 0
        local_yvel = 0
        #Vel = [0, 0, 0]
        # calculate the velocity
        r1 = random.random()
        r2 = random.random()
        c1 = 2
        c2 = 2
        with safe_done:
            if (fitness == 0):
                self.xvel = 0
                self.yvel = 0
            elif (fitness < 2030):
               if (Robot_map[dest_cell_y][dest_cell_x] !=0) and (Robot_map[dest_cell_y][dest_cell_x]!=1):
                    self.xvel = 0
                    self.yvel = 0
            else:
                self.xvel = 0.9*self.xvel + c1*r1*(self.pxBest - self.xcen) + c2*r2*(gxBest - self.xcen)
                self.yvel = 0.9*self.yvel + c1*r1*(self.pyBest - self.ycen) + c2*r2*(gyBest - self.ycen)
               
        #print (self.robotID, self.xvel, self.yvel, self.xcen, self.ycen, self.pxBest, self.pyBest, gxBest, gyBest)
        #if (self.robotID == 2): print(self.robotID, self.xcen, self.ycen)
            
        # scale x velocity to only four velocity range (0,1,2)
        if abs(self.xvel) < 22.0:
            local_xvel = 0      
        elif (abs(self.xvel) >= 22.0):
            if self.xvel < 0.0:
                local_xvel = -1
            else:
                local_xvel = 1
        # scale y velocity to only four velocity range (0,1,2)
        if abs(self.yvel) < 22.0:
            local_yvel = 0 
        elif (abs(self.yvel) >= 22.0): 
            if self.yvel < 0.0:
                local_yvel = -1
            else:
                local_yvel = 1

        Velocity = [0, 0]

        x_current = self.current_cell[0]
        y_current = self.current_cell[1]
                                
        #with safe_done: ??????????????????????????????????????????
        # find the optimum movement
        Velocity[0], Velocity[1] = Next_position(local_xvel, local_yvel, x_current, y_current)        
        # update the next position of this robot on Robot_map
        Update_robot_map(2, Velocity[0], Velocity[1], 0, 0, x_current, y_current, self.robotID)           

        self.Data_tomove.put(Velocity)
        self.w.after(1620, self.PSO_algorithm)      # 45*30 = 1350
        
    
    
    # Manage GUI
    def Robot_move(self):        
        Obs_x = Obs_y = 0
        #direction_x = direction_y = 0
        data = [0,0]

        try:
            data = self.Data_tomove.get_nowait() 
        # if no data on the Queue
        except queue.Empty:
            self.step += 1
            #if (self.step >= 43): print(self.step, self.robotID)
            
            #with safe_done: ??????????????????????????????????????????
            if ((self.step+1)%11 == 0):
                self.current_cell[0] = round((self.xcen-43)/45)
                self.current_cell[1] = round((self.ycen-43)/45)
                Update_robot_map(1, 0, 0, self.last_cell[0], self.last_cell[1], self.current_cell[0], self.current_cell[1], self.robotID)
                      
        # with data on the queue -> do something
        else:
            # set new velocity of x and y direction                 
            self.mini_xvel = int(data[0])
            self.mini_yvel = int(data[1])
            # insure the robot already move a cell
            self.step = 0 
            self.step_back = 0                     
            self.last_cell[0] = self.current_cell[0]
            self.last_cell[1] = self.current_cell[1]
            self.obstacle = False
        
        if (self.step < 45):  
            #with safe_done: ???????????????????????????????????????????
            # Check Obstacle ??????
            Obs_x, Obs_y = UnknownObstacle(self.xcen-11, self.xcen+11, self.ycen-11, self.ycen+11)
            #print(Obs_x, Obs_y)
            if (Obs_x != 0) or (Obs_y != 0):        
                # update the new obstacle in the Robot and reset the intended move-in cell
                Update_robot_map(3, Obs_x, Obs_y, self.mini_xvel, self.mini_yvel, self.current_cell[0], self.current_cell[1], self.robotID)
                # if abstacle appears, robot should step back
                self.mini_xvel = -self.mini_xvel
                self.mini_yvel = -self.mini_yvel
                
                self.step_back = self.step
                self.obstacle = True
 
 
            
            #  Without obstacle
            if (self.obstacle == False):
                with safe_done:          
                    # make the move
                    self.w.move(self.Robot, self.mini_xvel, self.mini_yvel)
                    #update position
                    self.xcen += self.mini_xvel
                    self.ycen += self.mini_yvel
            else:                                          # Obstacle appears
                if (self.step_back != 0):
                    with safe_done:          
                        # make the move
                        self.w.move(self.Robot, self.mini_xvel, self.mini_yvel)
                        #update position
                        self.xcen += self.mini_xvel
                        self.ycen += self.mini_yvel
                    self.step_back -= 1
        self.w.update()       
        self.w.after(30, self.Robot_move)

# this function check the Robot_map and decide which is the next position    
def Next_position(x_vel, y_vel, x_cell, y_cell):        # x_cell, y_cell the coordinate of current cell
    # declare variable
    

    with safe_done:
        global map_dimension
        Possible_move = [[1, 1, 1], [1, 1, 1], [1, 1, 1]]  
        next_pos_x = next_pos_y = 0
        for i in range(y_cell-1, y_cell+2, 1):
            for j in range(x_cell-1, x_cell+2, 1):
                if ((i > map_dimension -1) or (j > map_dimension -1) or (i < 0) or (j < 0)):
                    Possible_move[i+1- y_cell][j+1- x_cell] = -1
                
                elif (Robot_map[i][j] != 0) and (Robot_map[i][j] != 1):
                    Possible_move[i+1- y_cell][j+1- x_cell] = 0      
        #for i in range(3):
            #print(Possible_move[i][0], Possible_move[i][1], Possible_move[i][2])
        #print('\n')
        # set 9 possible movement        
        if ((x_vel > 0) and (y_vel > 0)):               
            if (Possible_move[1][2] == 1):
                next_pos_x = 1
                next_pos_y = 0 
            if (Possible_move[2][1] == 1):
                next_pos_x = 0
                next_pos_y = 1 
            if (Possible_move[2][2] == 1):
                next_pos_x = 1
                next_pos_y = 1 
        
        elif ((x_vel < 0) and (y_vel < 0)):               
            if (Possible_move[1][0] == 1):
                next_pos_x = -1
                next_pos_y = 0 
            if (Possible_move[0][1] == 1):
                next_pos_x = 0
                next_pos_y = -1 
            if (Possible_move[0][0] == 1):
                next_pos_x = -1
                next_pos_y = -1                    
        
        elif ((x_vel > 0) and (y_vel < 0)):               
            if (Possible_move[1][2] == 1):
                next_pos_x = 1
                next_pos_y = 0 
            if (Possible_move[0][1] == 1):
                next_pos_x = 0
                next_pos_y = -1 
            if (Possible_move[0][2] == 1):
                next_pos_x = 1
                next_pos_y = -1 
        
        elif ((x_vel < 0) and (y_vel > 0)):               
            if (Possible_move[1][0] == 1):
                next_pos_x = -1
                next_pos_y = 0 
            if (Possible_move[2][1] == 1):
                next_pos_x = 0
                next_pos_y = 1 
            if (Possible_move[2][0] == 1):
                next_pos_x = -1
                next_pos_y = 1 

        elif ((x_vel > 0) and (y_vel == 0)):               
            if (Possible_move[2][2] == 1):
                next_pos_x = 1
                next_pos_y = 1 
            if (Possible_move[0][2] == 1):
                next_pos_x = 1
                next_pos_y = -1 
            if (Possible_move[1][2] == 1):
                next_pos_x = 1
                next_pos_y = 0 
               
        elif ((x_vel < 0) and (y_vel == 0)):               
            if (Possible_move[0][0] == 1):
                next_pos_x = -1
                next_pos_y = -1 
            if (Possible_move[2][0] == 1):
                next_pos_x = -1
                next_pos_y = 1 
            if (Possible_move[1][0] == 1):
                next_pos_x = -1
                next_pos_y = 0
                
        elif ((x_vel == 0) and (y_vel > 0)):               
            if (Possible_move[2][0]== 1):
                next_pos_x = -1
                next_pos_y = 1 
            if (Possible_move[2][2]== 1):
                next_pos_x = 1
                next_pos_y = 1 
            if (Possible_move[2][1]== 1):
                next_pos_x = 0
                next_pos_y = 1                                      

        elif ((x_vel == 0) and (y_vel < 0)):               
            if (Possible_move[0][0] == 1):
                next_pos_x = -1
                next_pos_y = -1 
            if (Possible_move[0][2] == 1):
                next_pos_x = 1
                next_pos_y = -1 
            if (Possible_move[0][1] == 1):
                next_pos_x = 0
                next_pos_y = -1 
        elif ((x_vel == 0) and (y_vel == 0)):
            next_pos_x = next_pos_y = 0
        
        # check whether a cross move happen?
        if ((next_pos_x == 1) and (next_pos_y == 1)):
            check1 = Robot_map[y_cell][x_cell+1]
            check2 = Robot_map[y_cell+1][x_cell]
            if (check1 != 0) and (check1 != 1):     # check cross move
                if (check1//10 == check2//10):
                    next_pos_x = 0
                    next_pos_y = 0
            if (check1 == 100): next_pos_x = 0      # check obstacle by side
            if (check2 == 100): next_pos_y = 0
        
        elif ((next_pos_x == 1) and (next_pos_y == -1)):
            check1 = Robot_map[y_cell][x_cell+1]
            check2 = Robot_map[y_cell-1][x_cell]
            if (check1 != 0)and (check1 != 1):
                if (check1//10 == check2//10):      # check cross move
                    next_pos_x = 0
                    next_pos_y = 0
            if (check1 == 100): next_pos_x = 0      # check obstacle by side
            if (check2 == 100): next_pos_y = 0     
        
        elif ((next_pos_x == -1) and (next_pos_y == -1)):
            check1 = Robot_map[y_cell][x_cell-1]
            check2 = Robot_map[y_cell-1][x_cell]
            if (check1 != 0)and (check1 != 1):
                if (check1//10 == check2//10):      # check cross move
                    next_pos_x = 0
                    next_pos_y = 0
            if (check1 == 100): next_pos_x = 0      # check obstacle by side
            if (check2 == 100): next_pos_y = 0           
        elif ((next_pos_x == -1) and (next_pos_y == 1)):
            check1 = Robot_map[y_cell][x_cell-1]
            check2 = Robot_map[y_cell+1][x_cell]
            if (check1 != 0) and (check1 != 1):      # check cross move
                if (check1//10 == check2//10):
                    next_pos_x = 0
                    next_pos_y = 0
            if (check1 == 100): next_pos_x = 0      # check obstacle by side
            if (check2 == 100): next_pos_y = 0        
        
        return next_pos_x, next_pos_y

def UnknownObstacle(x_left, x_right, y_up, y_down):
    with safe_done:
        # Robot is the center to coordinate direction
        global Obst 
        Obs_x = Obs_y = 0
        for i in range(4):
            if (x_right == Obst[i][0]):
                if ((y_up<=Obst[i][3])&(y_up>=Obst[i][2]))or((y_down>=Obst[i][2])&(y_down<=Obst[i][3])):
                    Obs_x = 1
                 
            if (x_left == Obst[i][1]): 
                if ((y_up<=Obst[i][3])&(y_up>=Obst[i][2]))or((y_down>=Obst[i][2])&(y_down<=Obst[i][3])):
                    Obs_x = -1

            if (y_down == Obst[i][2]): 
                if ((x_right>=Obst[i][0])&(x_right<=Obst[i][1]))or((x_left>=Obst[i][0])&(x_left<=Obst[i][1])):
                    Obs_y = 1

            if (y_up == Obst[i][3]): 
                if ((x_right>=Obst[i][0])&(x_right<=Obst[i][1]))or((x_left>=Obst[i][0])&(x_left<=Obst[i][1])):
                    Obs_y = -1

        return Obs_x, Obs_y
  
# update the information for the Robot_map
def Update_robot_map(index, pos_x, pos_y, x_mulpur, y_mulpur, x_current, y_current, robotID):
    with safe_done:
        if (index == 1):      
            # robot has not left the cell yet
            if ((x_mulpur == x_current) and (y_mulpur == y_current)):
                Robot_map[y_current][x_current] = robotID*10 + 1

            else: #if ((x_last != x_current)or(y_last != y_current)):
                Robot_map[y_current][x_current] = robotID*10 + 1
                Robot_map[y_mulpur][x_mulpur] = 1
        # set the intending move-in cell
        if (index == 2):
            # update the next position to go               
            x_next = x_current + pos_x
            y_next = y_current + pos_y
            Robot_map[y_next][x_next] = robotID*10 + 2

        if (index == 3):
            # update a new obstacle
            x_obs = x_current + pos_x
            y_obs = y_current + pos_y
            Robot_map[y_obs][x_obs] = 100
            # Reset the intened move-in cell
            x_intend = x_current + x_mulpur
            y_intend = y_current + y_mulpur
            check = robotID*10 + 2
            if (Robot_map[y_intend][x_intend] == check):
                Robot_map[y_intend][x_intend] = 0
                print('here everything is still correct', robotID)
        
 #   if (robotID == 4) or (robotID == 9):   
    #for i in range(5, 17, 1):
        #print('{:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d} {:3d}'. format(Robot_map[i][0], Robot_map[i][1], Robot_map[i][2], Robot_map[i][3], Robot_map[i][4], Robot_map[i][5], Robot_map[i][6], Robot_map[i][7], Robot_map[i][8], Robot_map[i][9], Robot_map[i][10], Robot_map[i][11], Robot_map[i][12], Robot_map[i][13], Robot_map[i][14], Robot_map[i][15], Robot_map[i][16], Robot_map[i][17], Robot_map[i][18], Robot_map[i][19]))
    #print('\n')
       



Robot = []      # list of Robot
Obst = []       # list of obstacles

if __name__ == "__main__":
  
    # create obstables
    Obst1 = [245, 290, 335, 470]    # obstacle coordinate: x_left,x_right,y_up,y_down
    Obst.append(Obst1)
    Obst2 = [335, 380, 425, 515]
    Obst.append(Obst2)     
    Obst3 = [425, 470, 290, 380] 
    Obst.append(Obst3)
    Obst4 = [515, 560, 380, 560]
    Obst.append(Obst4)
   
    
    fenster = Tk()  # Erzeugung eines Fensters
    fenster.title("Robot Path Finding Simulator")
    # use Canvas use to display, edit and update graphs and other drawings 
    w = Canvas(fenster, width = 1000, height = 950)
    w.pack()
    #direction = CW
    Simulator_map = Create_map(w)
    w.create_rectangle(793,388,823, 418, fill ='red')

    dx1, dy1 = Simulator_map.Convert_to_coordinate(1,2)
    dx2, dy2 = Simulator_map.Convert_to_coordinate(2,8)
    dx3, dy3 = Simulator_map.Convert_to_coordinate(3,5)
    dx4, dy4 = Simulator_map.Convert_to_coordinate(5,5)
    dx5, dy5 = Simulator_map.Convert_to_coordinate(5,8)
    dx6, dy6 = Simulator_map.Convert_to_coordinate(4,2)
    
    d_x = [dx1, dx2, dx3, dx4, dx5, dx6]
    d_y = [dy1, dy2, dy3, dy4, dy5, dy6]
    color = ['green','blue','yellow','orange','violet', 'pink']
    
    for i in range(5):
        Robot1 = robot(i+1, d_x[i], d_y[i], color[i], w)    
        Robot.append(Robot1)  
    
    for i in range(5):
        Robot[i].start()
    fenster.mainloop()
