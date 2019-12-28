from visual import *
from random import random
import time

def randomValue(low, high):
    return (high-low)*random() + low;

dt = 0.01
Fnet = vector(0.0, 0.0, 0.0)
hasVal = 0
ball1 = sphere(pos=(1,0,0), radius = .8) #.2
ball2 = sphere(pos=(3,1,0), radius = .4) #(3,1,0)

ball1.v = vector(1,0,0)
ball1.m = 2
ball1.p = ball1.m * ball1.v

ball2.v = vector(-1,-2,0) #(-1,-2,0)
ball2.m = 1
ball2.p = ball2.m * ball2.v

'''v1_hat_x = cos(0)
v1_hat_y = sin(0)
v_hat_ball1 = vector(v1_hat_x, v1_hat_y, 0);

v2_hat_x = cos(3)
v2_hat_y = sin(1)
v_hat_ball2 = vector(v2_hat_x, v2_hat_y, 0);

if (ball1.radius >= 0.7):
    v_mag_ball1 = vector(randomValue(0.1,5),randomValue(0.1,5),0);
    v_ball1 = mag(v_mag_ball1) * v_hat_ball1; #get the random velocity vector
    ball1.m = randomValue(6.5, 10); #random mass for appropriate radii
    #finally set the momentum
    ball1.p = v_ball1 * ball1.m;

elif (ball1.radius <= 0.39):
    v_mag_ball1 = vector(randomValue(3.5,7),randomValue(3.5,7),0);
    v_ball1 = mag(v_mag_ball1) * v_hat_ball1;
    ball1.m = randomValue(0.1,3.5);
    ball1.p = v_ball1 * ball1.m;
else:
    v_mag_ball1 = vector(randomValue(2.5,6),randomValue(2.5,6),0);
    v_ball1 = mag(v_mag_ball1) * v_hat_ball1;
    ball1.m = randomValue(3.6,6.4);
    ball1.p = v_ball1 * ball1.m;
    
if (ball2.radius >= 0.7):
    v_mag_ball2 = vector(randomValue(0.1,5),randomValue(0.1,5),0);
    v_ball2 = mag(v_mag_ball2) * v_hat_ball2; #get the random velocity vector
    ball2.m = randomValue(6.5, 10); #random mass for appropriate radii
    #finally set the momentum
    ball2.p = v_ball2 * ball2.m;

elif (ball2.radius <= 0.39):
    v_mag_ball2 = vector(randomValue(3.5,7),randomValue(3.5,7),0);
    v_ball2 = mag(v_mag_ball2) * v_hat_ball2;
    ball2.m = randomValue(0.1,3.5);
    ball2.p = v_ball2 * ball2.m;
else:
    v_mag_ball2 = vector(randomValue(2.5,6),randomValue(2.5,6),0);
    v_ball2 = mag(v_mag_ball2) * v_hat_ball2;
    ball2.m = randomValue(3.6,6.4);
    ball2.p = v_ball2 * ball2.m;'''

# r vector arrows of ball positions
ball1_pos = arrow(axis = ball1.pos, color = color.red, shaftwidth = 0.1)
ball2_pos = arrow(axis = ball2.pos, color = color.blue, shaftwidth = 0.1)

# v vector arrows of ball velocities
ball1_v = arrow(axis = ball1.v, color = color.green, shaftwidth = 0.1)
ball2_v = arrow(axis = ball2.v, color = color.magenta, shaftwidth = 0.1)

# relative position - aka the difference in their positions in vector form
r = ball1.pos-ball2.pos
r_rel = arrow(axis = (ball1.pos - ball2.pos), color = color.yellow, shaftwidth = 0.1)
r_rel.pos = ball2_pos.axis

#relative velocity
vRel = ball1.v - ball2.v
v_rel = arrow(axis = (ball1.v - ball2.v), color = color.white, shaftwidth = 0.1)
v_rel.pos = ball2_v.axis

#print('ball_1.v: ', ball1.v, 'v_Rel: ', vRel, 'dot: %f' % (dot(vrel,r)))

# center of mass position of the balls
r_cm = arrow(axis = (ball1.m*ball1.pos + ball2.m*ball2.pos) / (ball1.m + ball2.m), color = color.orange, shaftwidth = 0.1)

v_cm = arrow(axis = (ball1.p + ball2.p) / (ball1.m + ball2.m), color = color.cyan, shaftwidth = 0.1)

def checkCollision(ball1, ball2):
    r = ball1.pos-ball2.pos; #distance between their centers
    magr = mag(r);
    #print('magr: ', magr)
    #print('radius: ', ball1.radius + ball2.radius)
    vrel = (ball1.p/ball1.m) - (ball2.p/ball2.m); #tells you how one is moving in comparison to another
    #we need to know if the dot product is pos or neg to tell if they are moving toward each other
    #print('dot: ', dot(vrel,r))
    
    if (magr < ball1.radius + ball2.radius and dot(vrel,r) < 0) : #if its less than their radii, collision has happened
        #print('reeeeeeeeeeeeee')
        pause()
        vcm = (ball1.p + ball2.p) / (ball1.m + ball2.m);
        rhat = norm(ball1.pos - ball2.pos); #get the direction vector
        ball1.p = ball1.p - 2*ball1.m*rhat*dot((ball1.p/ball1.m) - vcm, rhat);
        ball2.p = ball2.p - 2*ball2.m*rhat*dot((ball2.p/ball2.m) - vcm, rhat);

def pause():
    while True:
        rate(50)
        if scene.mouse.events:
            m = scene.mouse.getevent()
            if m.click == 'left': return


while True:
    rate(1/dt);

    if (hasVal):
        pause()

    # calculate all forces acting on balls
    checkCollision(ball1, ball2)
    
    # update momentum of each ball using impulse formula
    ball1.p += Fnet*dt      #ball1.v*ball1.m
    ball2.p += Fnet*dt      #ball2.v*ball2.m

    # update velocities of balls
    ball1.v = ball1.p / ball1.m
    ball2.v = ball2.p / ball2.m

    # update positions of balls
    ball1.pos += ball1.v*dt
    ball2.pos += ball2.v*dt

    # update position arrows
    ball1_pos.axis = ball1.pos
    ball2_pos.axis = ball2.pos

    #update velocity arrows
    ball1_v.pos = ball1.pos
    ball1_v.axis = ball1.v
    ball2_v.pos = ball2.pos
    ball2_v.axis = ball2.v

    #update relative position vector
    r = ball1.pos-ball2.pos
    r_rel.axis = ball1.pos - ball2.pos
    r_rel.pos = ball2_pos.axis

    #update relative velocity vector
    vRel = ball1.v - ball2.v
    v_rel.axis = ball1.v - ball2.v
    v_rel.pos = ball2_v.axis

    #update center of mass position vector
    r_cm.axis = (ball1.m*ball1.pos + ball2.m*ball2.pos) / (ball1.m + ball2.m)

    #update center of mass velocity
    v_cm.axis = (ball1.p + ball2.p) / (ball1.m + ball2.m)

    #print('ball_1.v: ', ball1.v, 'v_rel: ', vRel, 'dot: %f' % (dot(vrel,r)))
    #pause()

    #print(ball1.v)
    #print(ball2.v)
    #print(ball1.p)
    #print(ball2.p)
    print(mag(ball1.p + ball2.p))

    #check boundaries
    # don't use velocity because we're updating velocity by momentum now
    # use momentum instead - it won't break co
    '''if (ball1.pos.x >= 5):
        ball1.v.x = -ball1.v.x
    if (ball2.pos.x >= 5):
        ball2.v.x = -ball2.v.x
    if (ball1.pos.x <= -5):
        ball1.v.x = ball1.v.x
    if (ball2.pos.x <= -5):
        ball2.v.x = ball2.v.x
    if (ball1.pos.y >= 5):
        ball1.v.y = -ball1.v.y
    if (ball2.pos.y >= 5):
        ball2.v.y = -ball2.v.y
    if (ball1.pos.y <= -5):
        ball1.v.y = ball1.v.y
    if (ball2.pos.y <= -5):
        ball2.v.y = ball2.v.y'''
    
    if (ball1.pos.x >= 5):
        ball1.p.x += -2*abs(ball1.p.x)
    if (ball2.pos.x >= 5):
        ball2.p.x += -2*abs(ball2.p.x)
    if (ball1.pos.x <= -5):
        ball1.p.x += 2*abs(ball1.p.x)
    if (ball2.pos.x <= -5):
        ball2.p.x += 2*abs(ball2.p.x)
    if (ball1.pos.y >= 5):
        ball1.p.y += -2*abs(ball1.p.y)
    if (ball2.pos.y >= 5):
        ball2.p.y += -2*abs(ball2.p.y)
    if (ball1.pos.y <= -5):
        ball1.p.y += 2*abs(ball1.p.y)
    if (ball2.pos.y <= -5):
        hasVal = 1
        ball2.p.y += 2*abs(ball2.p.y)

    

    

    
