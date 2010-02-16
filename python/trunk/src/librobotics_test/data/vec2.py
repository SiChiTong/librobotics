'''
Created on 2010/02/16

@author: Mahisorn
'''

import librobotics.data.vec
import unittest
import random
import math

class Vec2Test(unittest.TestCase):    
    def setUp(self):
        self.x1 = random.random()
        self.y1 = random.random()
        self.x2 = random.random()
        self.y2 = random.random()         
        
    def test_size(self):
        x = librobotics.data.vec.Vec2(self.x1, self.y1)
        self.assertEquals(x.size(), math.sqrt(self.x1**2 + self.y1**2))
    
    def test_zero(self):
        x = librobotics.data.vec.Vec2(0.0, 0.0)
        y = librobotics.data.vec.Vec2(0.0, 1e-6)
        z = librobotics.data.vec.Vec2(0.0, 1e-7)        
        self.assertTrue(x.is_zero())
        self.assertFalse(y.is_zero())
        self.assertTrue(z.is_zero())
           
    def test_add(self):               
        x = librobotics.data.vec.Vec2(self.x1, self.y1)
        y = librobotics.data.vec.Vec2(self.x2, self.y2)
        z = x + y;     
        self.assertEqual(self.x1 + self.x2, z.x)
        self.assertEqual(self.y1 + self.y2, z.y)
        
    def test_sub(self):        
        x = librobotics.data.vec.Vec2(self.x1, self.y1)
        y = librobotics.data.vec.Vec2(self.x2, self.y2)
        z = x - y;     
        self.assertEqual(self.x1 - self.x2, z.x)
        self.assertEqual(self.y1 - self.y2, z.y)
              

if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(Vec2Test)
    unittest.TextTestRunner(verbosity=2).run(suite)
