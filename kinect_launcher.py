#!/usr/bin/env python
from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import cv
import time
import numpy as np
from numpy import array

import rocket_backend
from time import sleep

FAR = 700
from time import time
start = time()

class Controller(object):
    def __init__(self):
        manager = rocket_backend.RocketManager()
        manager.acquire_devices()
        self.launcher = manager.launchers[0]
        self.armed = False

    def fire(self):
        if self.armed:
            self.launcher.issue_command(4)
            self.armed = False

    def left(self):
        self.launcher.issue_command(2)

    def right(self):
        self.launcher.issue_command(3)

    def up(self):
        self.launcher.issue_command(1)

    def down(self):
        self.launcher.issue_command(0)

    def stop(self):
        self.launcher.issue_command(5)

    def tick(self):
        self.launcher.check_limits()

controller = Controller()

def extract_blob(depth, field, x, y):
    stack = [(x, y)]
    left = right = x
    top = bottom = y
    width = len(field[0])
    height = len(field)
    field[y][x] = 1
    while stack:
        x, y = stack.pop()
        if x < left:
            left = x
        elif x > right:
            right = x
        if y < top:
            top = y
        elif y > bottom:
            bottom = y
        for x, y in ((x-1, y), (x+1, y), (x, y-1), (x, y+1)):
            if 0 < x < width and 0 < y < height and depth[y][x] <= FAR and not field[y][x]:
                field[y][x] = 1
                stack.append((x, y))
    return (left, right, top, bottom)

def process(depth):
    height, width = depth.shape
    field = np.zeros_like(depth).tolist()
    depth = depth.tolist()
    r = np.zeros_like(depth).tolist()
    g = np.zeros_like(depth).tolist()
    b = np.zeros_like(depth).tolist()
    blobs = []
    for y in xrange(0,height):
        row = depth[y]
        for x in xrange(0, width):
            z = row[x]
            if z > FAR:
                b[y][x] = 255
                continue
            if field[y][x]:
                r[y][x] = 255
                continue
            blobs.append(extract_blob(depth, field, x, y))
            g[y][x] = 255
    DEAD = 100
    for blob in blobs:
        if time()-start < 10:
            break
        if blob[1]-blob[0] > 10 and blob[3]-blob[2] > 10:
            controller.armed = True
            print blob
            if blob[0] < DEAD:
                controller.left()
            elif blob[1] > width-DEAD:
                controller.right()
            elif blob[2] < DEAD*0.5:
                controller.up()
            elif blob[3] > height-DEAD*0.5:
                controller.down()
            else:
                controller.stop()
            break
    else:
        controller.fire()
    controller.tick()
    return r,g,b

def doloop():
    while True:
        # Get a fresh frame
        (depth,_), (rgb,_) = get_depth(), get_video()

        depth = depth[::2, ::2]
        r,g,b = process(depth)

        # Build a two panel color image
        d3 = np.dstack((r,g,depth/20)).astype(np.uint8)
        da = np.hstack((d3,rgb[::2, ::2]))

        # Simple Downsample
        cv.ShowImage('both',np.array(da[:,:,::-1]))
        cv.WaitKey(5)

doloop()

"""
IPython usage:
 ipython
 [1]: run -i demo_freenect
 #<ctrl -c>  (to interrupt the loop)
 [2]: %timeit -n100 get_depth(), get_rgb() # profile the kinect capture

"""

