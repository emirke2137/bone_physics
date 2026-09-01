import bpy
from mathutils import Vector, Matrix, Quaternion
import math
init_rig = bpy.data.texts["init_rig.py"].as_module().init_rig
predict_positions = bpy.data.texts["physics.py"].as_module().predict_positions
apply_constraints = bpy.data.texts["physics.py"].as_module().apply_constraints
calculate_final_velocity = bpy.data.texts["physics.py"].as_module().calculate_final_velocity

constraints, points, roots, control_bones = init_rig()
#constraints    - representation of connections between bones
#points         - representation of each bone (tail)
#root           - collection of points of control bones, shared with points list
#control_bones  - real equivalent of roots
armature = bpy.data.objects["Armature"]
control_bone = armature.pose.bones[0]

def remove_twist(direction, reference_x):
    #stabilize bone twist
    
    x = reference_x - direction * reference_x.dot(direction)
    
    #reference_x.dot(direction) -> projection of direction (curent Y axis) on reference (old X axis)
    #scalar value the will be 0 if they are perpendicular
    
    #direction * reference_x.dot(direction) -> the direction vector scaled by the amount they "lie" on x_reference
    
    #x = reference_x - direction * reference_x.dot(direction) -> move away from the old x axis, so that the new x axis is perpendicular to the new y

    #how much the bone can twist
    if x.length < 0.001:
        return direction.to_track_quat('Y', 'Z').to_matrix()
    
    #make sure it has unit lengh 
    x.normalize()
    #create Z axis
    z = direction.cross(x)#cross product gives a vector perpendicular to both direction and x
    z.normalize()
    
    #calculate x again to eliminate errors from floating point acuracy
    x = z.cross(direction)
    x.normalize()

    
    return Matrix((x, direction, z)).transposed()
    

def apply_transform():
    global points, armature
    
    for i in range (0, len(points)): 
        if points[i].weight!=0: #0 is for control bone
            
            #vector from bone pivot to new tail position
            direction = points[i].pos - points[i].prev_point.pos
            direction.normalize()
            
            world_rotation = remove_twist(direction,
                                            points[i].orientation)
            #put that transformation into a matrix
            world_matrix = world_rotation.to_4x4()
            #make transforamtion center at the previous bone tip
            world_matrix.translation = points[i].prev_point.pos
            # Convert world transform back into armature space
            #apply to bone
            armature.pose.bones[i].matrix = armature.matrix_world.inverted() @ world_matrix
            #save orientation - it glitxesfd
            #points[i].orientation = (armature.matrix_world.to_3x3() @ armature.pose.bones[i].x_axis).normalized()
    

    

def simulate_cloak(dt): 
    global constraints, points, armature, roots, control_bones
    for i in range (len(control_bones)):
        
        #get new control bone position
        roots[i].pos = (armature.matrix_world @ control_bones[i].tail).copy()
        #override old velocity with curent velocity
        roots[i].velocity = (roots[i].pos - roots[i].prev_pos)/dt
        #override prev position with curent to be used in next step
        roots[i].prev_pos = roots[i].pos.copy()
    
    predict_positions(dt,points)
    apply_constraints(dt,constraints)
    calculate_final_velocity(dt,points)
    apply_transform()
    
 
 
def is_playing():
    if bpy.context.screen and bpy.context.screen.is_animation_playing:
        return True
    
def update_real_time(scene):
    if is_playing():
        return # do nothing if animation is playing

    print("real")
    dt = 1.0 / scene.render.fps #make more acurate
    simulate_cloak(dt)
    
def update_playback(scene):
    print("play")
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

    