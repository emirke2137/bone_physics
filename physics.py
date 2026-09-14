import bpy
from mathutils import Vector
CollisionConstraint = bpy.data.texts["data.py"].as_module().CollisionConstraint


gravity = bpy.data.scenes[0].gravity
drag= 0.1

####### TEST AREA ########
test_cones=[
            bpy.data.objects['cone'],
            bpy.data.objects['cone.001'],
            bpy.data.objects['cone.002'],
            bpy.data.objects['cone.003'],
            bpy.data.objects['cone.004'],
]
test_positions2=[]
############################

def predict_positions(dt, points):
    global gravity, drag
    
    for i in range (1, len(points)):
        
        points[i].prev_pos = points[i].pos.copy()
        #formula: new_pos = prev_pos + velocity*dt + 0.5*acceleration*dt^2        
        drag_force = -drag* points[i].velocity
        #drag_acceleration = drag_force / mass
        acc = gravity + drag_force
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
        
def solve_colisions(con):
    #bounce_direction = (con.point.pos - con.location)*con.normal
    #new_pos = con.location + con.normal *con.point.colision_radius
    d = con.normal.dot(con.location-con.point.pos)
    if d > 0:
        #print(d)
        correction = con.normal * (d+con.point.colision_radius)
        new_pos = con.point.pos + correction
        con.point.pos = new_pos
     
    

def find_collisions(points, colliders):
    
    collision_constraints = []

    for point in points:
        movement = point.pos - point.prev_pos
        length = movement.length
        if length == 0:
            continue
        
        direction = movement.normalized()
        for obj in colliders.keys():
                      
            location, normal, face_index, distance = colliders[obj].ray_cast(point.prev_pos,direction,length)  
            #print(point.name,obj.name, location, normal, face_index,distance)
            
            if location:
                #print(point.name,obj.name, location, normal, face_index,distance )                      
                collision_constraints.append(CollisionConstraint(point,location,normal))
    return collision_constraints

    
#apply constraints like distance from neighbour, colisions. 
def apply_constraints(dt, distance_constraints, points, colliders):
    
    iterations=10
    for iteration in range(iterations):

        for con in distance_constraints:
            solve_distance(con)
        
        collision_constaints = find_collisions(points, colliders)
        for con in collision_constaints:
            solve_colisions(con)
    
        

#velocity based on the final position
def calculate_final_velocity(dt,points):
    for p in points:
        p.velocity = (p.pos - p.prev_pos)/dt
        