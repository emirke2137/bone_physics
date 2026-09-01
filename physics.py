import bpy
from mathutils import Vector


gravity = bpy.data.scenes[0].gravity

def predict_positions(dt, points):
    global gravity
    
    for i in range (1, len(points)):
        
        points[i].prev_pos = points[i].pos.copy()
        #formula: new_pos = prev_pos + velocity*dt + 0.5*acceleration*dt^2
        #acc = gravity for now
        #drag_force = -drag_coefficient * sim_vel[i]
        #drag_acceleration = drag_force / mass
        acc = gravity
        points[i].pos += points[i].velocity*dt + 0.5*acc*dt**2
    

def solve_distance(con):

    #current state
    delta = con.point_b.pos - con.point_a.pos   #vector
    distance = delta.length                     #scalar value
    
    if distance != 0:
        direction = delta/distance #normalized vector
        #difference between curent state and intended state
        error = distance - con.distance
        
        #correction
        #how much each bone needs to move to satisfy the constraint
        #either both move half of the error length
        #or one moves the whole distance if the other is the ctr bone
        correction = direction * error / (con.point_a.weight + con.point_b.weight)
        con.point_a.pos += correction * con.point_a.weight
        con.point_b.pos -= correction * con.point_b.weight
        
        
    
#apply constraints like distance from neighbour, colisions. 
def apply_constraints(dt, constraints):
   
    iterations=10
    for iteration in range(iterations):

        for con in constraints:
            solve_distance(con)
    
        

#velocity based on the final position
def calculate_final_velocity(dt,points):
    for p in points:
        p.velocity = (p.pos - p.prev_pos)/dt
        