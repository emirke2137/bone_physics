import bpy
from mathutils import Vector

enabled = True
gravity = Vector((0, 0, -9.81))

armature = bpy.data.objects["Armature"]
chain = armature.pose.bones

#use this to move the chain
control_bone = chain['Bone']

sim_pos=[]
prev_pos=[]
sim_vel=[]



for c in chain:
    sim_pos.append((armature.matrix_world @ c.tail).copy())
    prev_pos.append((armature.matrix_world @ c.tail).copy())
    sim_vel.append(Vector((0, 0, 0)))


#predict each point's next position based on it's current velocity (verlet integration) and external forces, ignoring the control bone
def predict_positions(dt):
    global sim_pos,sim_vel,gravity,chain
    
    for i in range (1, len(sim_pos)):
        prev_pos[i] = sim_pos[i].copy()
        
        #formula: new_pos = prev_pos + velocity*dt + 0.5*acceleration*dt^2
        #acc = gravity for now
        #drag_force = -drag_coefficient * sim_vel[i]
        #drag_acceleration = drag_force / mass
        acc = gravity
        sim_pos[i] += sim_vel[i]*dt + 0.5*acc*dt**2
    

def solve_distance(i):
    global sim_pos
    
    a = sim_pos[i-1]        #parent pos
    b = sim_pos[i]          #curent bone pos
    #current state
    delta = b - a           #vector
    distance = delta.length #scalar value
    
    if distance != 0:
        direction = delta/distance #normalized vector
        #deffrence between curent state and intended state
        error = distance - chain[i].length
        
        #weight -> how much the point is affected
        #asuming equal weight
        #control bone has weight=0 -> position not altered
        w_a = 0 if i == 1 else 1
        w_b = 1
        
        #correction
        #how much each bone needs to move to satisfy the constraint
        #either both move half of the error length
        #or one moves the whole distance if the other is the ctr bone
        correction = direction * error / (w_a + w_b)
        sim_pos[i-1] += correction * w_a
        sim_pos[i]   -= correction * w_b
        
        
    
#apply constraints like distance from neighbour, colisions. 
def apply_constraints(dt):
    global sim_pos,sim_vel,gravity,chain
    
    iterations=10
    for iteration in range(iterations):

        for i in range (1, len(sim_pos)):
            solve_distance(i)
    
        

#velocity based on the final position
def calculate_final_velocity(dt):
    global sim_vel, sim_pos, prev_pos
    for i in range (1, len(sim_pos)):
        sim_vel[i] = (sim_pos[i] - prev_pos[i])/dt
        

#change the pgysical positions of the bones in blender
def apply_transform():
    global sim_pos, chain
    
    for i in range (1, len(sim_pos)): 
        direction = sim_pos[i] - sim_pos[i-1]
        direction.normalize()
        #create rotation transform from that direction
        #from bones pose space (y as up axis) to world (z as up axis)
        world_rotation = direction.to_track_quat('Y', 'Z')

        #put that transformation into a matrix
        world_matrix = world_rotation.to_matrix().to_4x4()
        #make transforamtion center at the previous bone tip
        world_matrix.translation = sim_pos[i-1]
        # Convert world transform back into armature space
        #apply to bone
        chain[i].matrix = armature.matrix_world.inverted() @ world_matrix
    

    
    

def simulate_cloak(dt):
    global prev_pos, sim_pos,sim_vel
    
    #get new control bone position
    sim_pos[0] = (armature.matrix_world @ control_bone.tail).copy()
    #override old velocity with curent velocity
    sim_vel[0] = (sim_pos[0] - prev_pos[0])/dt
    #override prev position with curent to be used in next step
    prev_pos[0] = sim_pos[0].copy()
    
    predict_positions(dt)
    apply_constraints(dt)
    calculate_final_velocity(dt)
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

    

register()

print("script loaded")