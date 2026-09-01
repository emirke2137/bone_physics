import bpy
from mathutils import Vector

enabled = True

gravity = Vector((0, 0, -9.81))

armature = bpy.data.objects["Armature"]
chain = armature.pose.bones

#use this to move the chain
control_bone = chain['Bone']
#prev_pos = (armature.matrix_world @ control_bone.tail).copy()

sim_pos=[]
prev_pos=[]
sim_vel=[]
sim_acc=[]
for c in chain:
    sim_pos.append((armature.matrix_world @ c.tail).copy())
    prev_pos.append((armature.matrix_world @ c.tail).copy())
    sim_vel.append(Vector((0, 0, 0)))
    sim_acc.append(gravity.copy())


def simulate_bone(dt,n): #n->number of bone in the chain, 0->root/control 1-4-> simulated bones
    global sim_pos,sim_vel,sim_acc,chain
    parent_pos=sim_pos[n-1]
    parent_vel=sim_vel[n-1]
    parent_acc=sim_acc[n-1]
    
    #verlet integration
    new_pos = sim_pos[n] + parent_vel*dt + parent_acc*dt*dt/2
    #length constrain 
    direction = new_pos - parent_pos
    direction.normalize()
    #update vectors
    prev_pos = sim_pos[n]
    sim_pos[n] = parent_pos + direction*chain[n].length
    sim_acc[n] = ((prev_pos - sim_pos[n])/dt - sim_vel[n])/dt
    sim_vel[n] = (prev_pos - sim_pos[n])/dt
    print(sim_pos[n],sim_vel[n],sim_acc[n])
    print()
    
    ### APPLY TRANSFORM TO BONE ###
    #create rotation transform from that direction
    #from bones pose space (y as up axis) to world (z as up axis)
    world_rotation = direction.to_track_quat('Y', 'Z')

    #put that transformation into a matrix
    world_matrix = world_rotation.to_matrix().to_4x4()
    #make transforamtion center at the previous bone tip
    world_matrix.translation = parent_pos
    # Convert world transform back into armature space
    #apply to bone
    chain[n].matrix = armature.matrix_world.inverted() @ world_matrix
    
    
    
    

def simulate_cloak(dt):
    global prev_pos, sim_pos,sim_vel, sim_acc, gravity
    
    #get new control bone position
    sim_pos[0] = (armature.matrix_world @ control_bone.tail).copy()
    #get ctrl bone acceleration based on change in velocity
    sim_acc[0] = (((sim_pos[0] - prev_pos[0])/dt) - sim_vel[0])/dt
    #override old velocity with curent velocity
    sim_vel[0] = (sim_pos[0] - prev_pos[0])/dt
    #override prev position with curent to be used in next step
    prev_pos[0] = sim_pos[0].copy()
    
    for i in range(1,5):
        simulate_bone(dt,i)
    print()
   
    
def update(scene, depsgraph):
    for update in depsgraph.updates:
        if update.id.get("Dynamic", True):
            dt = 1.0 / scene.render.fps #make more acurate
            simulate_cloak(dt)
    

    
    

def register():
    unregister()
    bpy.app.handlers.depsgraph_update_post.append(update)

def unregister():
    for h in list(bpy.app.handlers.depsgraph_update_post):
        bpy.app.handlers.depsgraph_update_post.remove(h)
    

register()
print("script loaded")