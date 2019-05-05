import sys
import random
from tkinter import *
import time
import math

MAP_DIMENSION = 20
CELL_SIZE = 45


class Create_map(object):
    def __init__(self, w):
        self.nrows = MAP_DIMENSION
        self.ncolums = MAP_DIMENSION
        self.w = w
        self.Create_Obstacles()
        self.Draw_map()
    
    def Create_Obstacles(self):  
        
        rec1 = self.w.create_rectangle(245,335,290,470, fill = "black")
        
        rec2 = self.w.create_rectangle(335,425,380,515, fill = "black")
        
        rec3 = self.w.create_rectangle(425,290,470,380, fill = "black")
        
        rec4 = self.w.create_rectangle(515,380,560,560, fill = "black")       
    
    def Draw_map(self):      
        j = 20
        for i in range(self.nrows+1):
            self.w.create_line(20,j, CELL_SIZE*MAP_DIMENSION + 20,j, width = 2, fill = "black")
            j += CELL_SIZE
        j = 20
        for i in range(self.ncolums+1):
            self.w.create_line(j, 20, j, CELL_SIZE*MAP_DIMENSION+20, width = 2, fill = "black")
            j += CELL_SIZE
        #self.Create_Obstacles(w)  
    
    # convert the crosswork map to coorindate
    def Convert_to_coordinate(self, xloc, yloc):
        return (xloc-1)*CELL_SIZE +43, (yloc-1)*CELL_SIZE+43
