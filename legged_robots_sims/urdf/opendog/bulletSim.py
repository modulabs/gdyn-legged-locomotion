import os
import pybullet
import pybullet_data
import numpy as np
import time
from random import randint

pybullet.connect(pybullet.GUI)
pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(),"plane.urdf"),0,0,0)

obj = pybullet.loadURDF("opendog.urdf",[0,0,4],flags=pybullet.URDF_USE_SELF_COLLISION)

maxForce = 500

numJoints = pybullet.getNumJoints(obj)

pybullet.setGravity(0,0,-0.01)

t_end = time.time() + 600
co = 0

targets= [(-0.2,0.2,0.2,   -0.2,0.2,0.2,   0.2,0.2,0.2,   0.2,0.2,0.2),
		 (-0.3,0.3,0.3,   -0.3,0.3,0.3,   0.3,0.3,0.3,   0.3,0.3,0.3)]

ti = 0

while time.time() < t_end:
	pybullet.stepSimulation()
	if co == 2000:
		print( co )
		ti = (ti+1)%len(targets)
		co=0
	for j in range(12):
		pybullet.setJointMotorControl2(bodyUniqueId=obj, 
				jointIndex=j, 
				controlMode=pybullet.POSITION_CONTROL,
				targetPosition =targets[ti][j],
		force = maxForce)
	
	co = co+1
	#time.sleep(0.1)
	
	
	
	

print ("finished")
#remove all objects
pybullet.resetSimulation()

#disconnect from the physics server
pybullet.disconnect()