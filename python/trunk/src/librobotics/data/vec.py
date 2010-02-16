'''
Created on 2010/02/16

@author: Mahisorn
'''


import math


class Vec2(object):
    '''
    2D Vector class
    '''
    x = 0;
    y = 0;   
    
    def __str__(self):
        return '(' + self.x.__str__() + ', ' + self.y.__str__() + ')'  
    def __repr__(self):
        return '(' + self.x.__repr__() + ', ' + self.y.__repr__() + ')'
    

    def __init__(self, x_val=0, y_val=0):
        '''
        Constructor
        '''
        self.x = x_val
        self.y = y_val
        
    def size(self):
        return math.sqrt(self.x**2 + self.y**2)
    
    def is_zero(self):
        return math.sqrt(self.x**2 + self.y**2) < 1e-6
                       
    def __add__(self, other):        
        return Vec2(self.x + other.x, self.y + other.y)
    
    def __iadd__(self, other):        
        self.x += other.x
        self.y += other.y
        return self
            
    def __sub__(self, other):        
        return Vec2(self.x - other.x, self.y - other.y)
    
    def __isub__(self, other):        
        self.x -= other.x
        self.y -= other.y
        return self
      
    def __mul__(self, other):        
        return (self.x * other.y) - (self.y * other.x)    
    
    def __xor__(self, other):        
        return (self.x * other.x) + (self.y * other.y)
    

    
    
        
    
        