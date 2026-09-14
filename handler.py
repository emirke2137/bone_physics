import bpy
from mathutils import Vector, Matrix, Quaternion
from mathutils.bvhtree import BVHTree
import math

###### TEST AREA #########
test_balls=[
            bpy.data.objects['sphere'],
            bpy.data.objects['sphere.001'],
            bpy.data.objects['sphere.002'],
            bpy.data.objects['sphere.003'],
            bpy.data.objects['sphere.004'],
]

test_positions=[]

##########################

init_rig = bpy.data.texts["init_rig.py"].as_module().init_rig
predict_positions = bpy.data.texts["physics.py"].as_module().predict_positions
apply_constraints = bpy.data.texts["physics.py"].as_module().apply_constraints
calculate_final_velocity = bpy.data.texts["physics.py"].as_module().calculate_final_velocity

distance_constraints, points, roots, control_bones = init_rig()
#constraints    - representation of connections between bones
#points         - representation of each bone (tail)
#root           - collection of points of control bones, shared with points list
#control_bones  - real equivalent of roots

collision_group=[]
for obj in bpy.data.objects:
    if "collision" in obj and obj["collision"]==True:
        collision_group.append(obj)
colliders={} 
armature = bpy.data.objects["Armature"]
control_bone = armature.pose.bones[0]
 
def rotate_to_goal(prev_rotation,rest, direction):
    
    #only the y roation part, set the rest to 0
    old_y = prev_rotation @ Vector((0, 1, 0))
    #reverse transformation the would lead to y axis pointing along direction from old_y
    delta = old_y.rotation_difference(direction)
    rotation = delta @ prev_rotation # quaternion
    rest_global = armature.matrix_world.to_quaternion() @ rest
    #smooth using rest rotation
    rotation = rotation.slerp(rest_global, 0.05)
    
    return rotation
    

def apply_transform():
    global points, armature
    
    for i in range (0, len(points)): 
        if points[i].weight!=0: #0 is for control bone
            
            #vector from bone pivot to new tail position
            direction = points[i].pos - points[i].prev_point.pos
            direction.normalize()
            
            new_rotation = rotate_to_goal(points[i].orientation,points[i].rest_position,direction)
            points[i].orientation = new_rotation.copy()
        
            
            #put that transformation into a matrix
            world_matrix = new_rotation.to_matrix().to_4x4()
            #make transforamtion center at the previous bone tip
            world_matrix.translation = points[i].prev_point.pos
            # Convert world transform back into armature space
            #apply to bone
            armature.pose.bones[i].matrix = armature.matrix_world.inverted() @ world_matrix
        
           


def simulate_cloak(dt): 
    global distance_constraints, points, armature, roots, control_bones
    for i in range (len(control_bones)):
        
        #get new control bone position
        roots[i].pos = (armature.matrix_world @ control_bones[i].tail).copy()
        #override old velocity with curent velocity
        roots[i].velocity = (roots[i].pos - roots[i].prev_pos)/dt
        #override prev position with curent to be used in next step
        roots[i].prev_pos = roots[i].pos.copy()
    
    
    
    predict_positions(dt,points)
    
    ######## TEST AREA ########
    global test_balls,test_cones,test_positions
    for i in range(5):
        vec = points[i].pos.copy()
        test_positions.append(vec)
    ###########################
    
    colliders=build_bvh()
    #colision_constraints=find_collisions(points,colliders)
    #print()
    #print(colision_constraints)
    apply_constraints(dt,distance_constraints,points,colliders)
    calculate_final_velocity(dt,points)
    apply_transform()
    ######## TEST AREA ########
    for i in range(5):
  
        test_balls[i].location = test_positions[i] 
        #test_cones[i].location = test_positions2[i]
    test_positions=[]
    test_positions2=[]
    
    ###########################
    

def build_bvh():
    colliders={}
    
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for obj in collision_group:
        #obj with applied modifiers
        obj_eval = obj.evaluated_get(depsgraph)
     
        #verts = [Vector, Vector, Vector...]
        verts = [obj_eval.matrix_world @ v.co for v in obj_eval.data.vertices]
        #polygons = [[1,2,6,5],[5,6,4,3]...]
        polygons = [[v for v in poly.vertices] for poly in obj_eval.data.polygons]
        bvh = BVHTree.FromPolygons(verts, polygons)
        
        colliders[obj]=bvh
    return colliders


 
def is_playing():
    if bpy.context.screen and bpy.context.screen.is_animation_playing:
        return True
    
def update_real_time(scene,a):
    if is_playing():
        return # do nothing if animation is playing
    dt = 1.0 / scene.render.fps #make more acurate
    simulate_cloak(dt)
    
def update_playback(scene,a):
    
    dt = 1.0 / scene.render.fps #make more acurate
    simulate_cloak(dt)
    
def register():
    unregister()
    bpy.app.handlers.depsgraph_update_post.append(update_real_time)
    bpy.app.handlers.frame_change_post.append(update_playback)
    

def unregister():
    
    for h in list(bpy.app.handlers.depsgraph_update_post):
        if h.__name__ == "update_real_time":
            bpy.app.handlers.depsgraph_update_post.remove(h)
            
    for h in list(bpy.app.handlers.frame_change_post):
        if h.__name__ == "update_playback":
            bpy.app.handlers.frame_change_post.remove(h)

    