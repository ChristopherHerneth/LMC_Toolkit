import os
os.add_dll_directory("C:/OpenSim4.4/bin") # otherwise module _sombody not found error!
import opensim as osim
import numpy as np

# Convert the input Opensim Vec3 to a numpy array
def osimVec3ToArray(p):
    ll = 3
    out = np.zeros(ll)

    for i in range(ll):
        out[i] = p.get(i)

    return out