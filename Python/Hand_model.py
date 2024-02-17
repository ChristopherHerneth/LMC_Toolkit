import os
os.add_dll_directory("C:/OpenSim4.4/bin") # otherwise module _sombody not found error!
import opensim as osim
import numpy as np
import pathlib
import scipy
from scipy.spatial.transform import Rotation as R
from OsimPython.plotUtil import addArrow
from OsimPython.osim_type_util import osimVec3ToArray
from Linalg.ray_tracing_util import get_regressionPlane
import plotly.graph_objects as go

def getHandModel():
    # make the hand Model
    OsimModel = osim.Model(str(pathlib.Path(__file__).parent.resolve()) + '/hand_model_with_markers.osim')
    OsimModel.set_gravity(osim.Vec3(0))

    return OsimModel

def setHandPose(OsimModel, q):
    # q are joint angles (20dim)

    # Load Set from Model
    bs_ori = OsimModel.getBodySet()  # BodySet
    js_ori = OsimModel.getJointSet()  # JointSet
    mas_ori = OsimModel.getMarkerSet()  # MarkerSet
    cs_ori = OsimModel.getCoordinateSet()  # CoordinateSet

    # get defaut range
    #js_ori.get('CMC1').get_coordinates(1).get_range(0) # minimalm range
    #js_ori.get('CMC1').get_coordinates(1).get_range(1) # maximal range

    # set value to model
    js_ori.get('CMC1').get_coordinates(0).setDefaultValue(q[0])  # CMC1 abd 
    js_ori.get('CMC1').get_coordinates(1).setDefaultValue(q[1])  # CMC1 flex
    js_ori.get('MCP1').get_coordinates(0).setDefaultValue(q[2])  # MCP1 flex
    js_ori.get('IP').get_coordinates(0).setDefaultValue(q[3])  # IP flex
    js_ori.get('MCP2').get_coordinates(0).setDefaultValue(q[4])  # MCP2 abd 
    js_ori.get('MCP2').get_coordinates(1).setDefaultValue(q[5])  # MCP2 Flex 
    js_ori.get('PIP2').get_coordinates(0).setDefaultValue(q[6])  # PIP2 Flex 
    js_ori.get('DIP2').get_coordinates(0).setDefaultValue(q[7])  # DIP2 Flex 
    js_ori.get('MCP3').get_coordinates(0).setDefaultValue(q[8])  # MCP2 abd 
    js_ori.get('MCP3').get_coordinates(1).setDefaultValue(q[9])  # MCP2 Flex 
    js_ori.get('PIP3').get_coordinates(0).setDefaultValue(q[10])  # PIP2 Flex 
    js_ori.get('DIP3').get_coordinates(0).setDefaultValue(q[11])  # DIP2 Flex 
    js_ori.get('MCP4').get_coordinates(0).setDefaultValue(q[12])  # MCP2 abd 
    js_ori.get('MCP4').get_coordinates(1).setDefaultValue(q[13])  # MCP2 Flex 
    js_ori.get('PIP4').get_coordinates(0).setDefaultValue(q[14])  # PIP2 Flex 
    js_ori.get('DIP4').get_coordinates(0).setDefaultValue(q[15])  # DIP2 Flex 
    js_ori.get('MCP5').get_coordinates(0).setDefaultValue(q[16])  # MCP2 abd 
    js_ori.get('MCP5').get_coordinates(1).setDefaultValue(q[17])  # MCP2 Flex 
    js_ori.get('PIP5').get_coordinates(0).setDefaultValue(q[18])  # PIP2 Flex 
    js_ori.get('DIP5').get_coordinates(0).setDefaultValue(q[19])  # DIP2 Flex 

    OsimModel.finalizeConnections()
    state = OsimModel.initSystem()

    return state

def getHandMarkers(OsimModel, state, pos, orient):
    '''
        OsimModel from getHandModel()
        pos = nd.array([x, y, z])
        orient = euler angles (x, y, z)
    '''

    mas_ori = OsimModel.getMarkerSet()  # MarkerSet

    #orient += np.array([0, -42*np.pi/180, 192*np.pi/180]) # this is to make the hand flat with the xy plane in the 0 configuration

    #  base setting
    w_R_b = R.from_euler('xyz', [*orient]).as_matrix().squeeze()

    US_pos = osimVec3ToArray(mas_ori.get('US').getLocationInGround(state))
    RS_pos = osimVec3ToArray(mas_ori.get('RS').getLocationInGround(state))
    w_p_b = (US_pos+RS_pos)/2

    pos_markers = np.array([osimVec3ToArray(mas_ori.get(i).getLocationInGround(state)).T - w_p_b for i in range(mas_ori.getSize())])
    
    vx = pos_markers[24]-pos_markers[25]
    vx /= np.linalg.norm(vx)
    vy = pos_markers[4]-pos_markers[25]
    vy /= np.linalg.norm(vy)
    vz = np.cross(vx, vy)
    vz /= np.linalg.norm(vz)
    target_frame = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    input_frame = np.array([vx, vy, vz])

    def fun(x, target_frame, input_frame):
        RR = R.from_euler(seq='XYZ', angles=x, degrees=True).as_matrix().squeeze()
        obj = target_frame - RR @ input_frame

        return scipy.linalg.norm(obj) # frobenius norm

    x0 = [0, 0, 0]
    res = scipy.optimize.minimize(fun, x0, args=(target_frame, input_frame))
    
    RR = R.from_euler(seq='XYZ', angles=res.x, degrees=True).as_matrix().squeeze()
    pos_markers = (RR.T @ pos_markers.T).T
    pos_markers = (w_R_b.T @ pos_markers.T).T 
    # scale hand to 200mm length
    pos_markers *= 1/np.linalg.norm(pos_markers[20]-pos_markers[24]) * 200
    pos_markers += pos
    #pos_markers = pos_markers[[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 24, 25]] # remove marker indices 22, 23 (far away elbow)

    return pos_markers

def generate_hand_poses(HandModel, state, num_poses, x_limits=[-1000, 1000], y_limits=[-1000, 1000], z_limits=[200, 600], limitsPS=[180, -180], limis_FE=[60, 60], limits_WD=[40, 20]):
    '''
        state is the return of setHandPose(HandModel, q)
        position limits are in mm
        wrist angle limits are in deg
    '''
    # x = + flexion, - extension
    # y = + pronation, - supination
    # z = + ulna devation, - radial deviation

    fingers_idx = np.array([[0,3,8,17], #thumb (root to tip)
                        [4,9,13,18], #index
                        [5,10,14,19], #middle
                        [6,11,15,20], #ring
                        [7,12,16,21]]) #pinky

    handMarkers = np.zeros([num_poses, 26, 3])
    palmMarkers = np.zeros([num_poses, 8, 3])
    palmPlnNormals = np.zeros([num_poses, 3])
    palmCentroids = np.zeros([num_poses, 3])
    fingers = []

    x = np.random.uniform(x_limits[0],x_limits[1],num_poses) 
    y = np.random.uniform(y_limits[0],y_limits[1],num_poses) 
    z = np.random.uniform(z_limits[0],z_limits[1],num_poses) 

    ax = np.random.uniform(limis_FE[0],limis_FE[1],num_poses) * np.pi/180
    ay = np.random.uniform(limitsPS[0],limitsPS[1],num_poses) * np.pi/180
    az = np.random.uniform(limits_WD[0],limits_WD[1],num_poses) * np.pi/180

    for i in range(num_poses):
        pos_markers = getHandMarkers(HandModel, state, pos=np.array([x[i], y[i], z[i]]), orient=np.array([ax[i], ay[i], az[i]]))
        pos_markers *= 1/np.linalg.norm(pos_markers[20]-pos_markers[24]) * 200 # scale hand to 200mm length
        handMarkers[i, :] = pos_markers
        pv__ = (pos_markers[2] + pos_markers[1]) / 2 - pos_markers[6] # vector pointing from the finger root to the wrist
        pv__ /= np.linalg.norm(pv__)
        palmMarkers[i, :] = np.array([pos_markers[7] + pv__*3, pos_markers[6] + pv__*3, pos_markers[5] + pv__*3, pos_markers[4] + pv__*3, # to make sure the finger root markers do not intersect with the palm
                            pos_markers[1], pos_markers[25], pos_markers[24], pos_markers[2]])

        palmPlnNormals[i, :] = get_regressionPlane(palmMarkers[i, :]) # normal plane to the palm and palm centroid
        palmCentroids[i, :] = np.mean(palmMarkers[i, :], axis=0) # centroid marker of the palm

        # FINGERS
        # the line chain of a finger starts at the finger root and clinbs marker by marker to the finger tip
        # the base of a line is always the origin of a vector pointing from towards the tip -> this means the only frogner marker not a line origin is the finger tip
        #finger_lines has the shape (num_fingers, num_phlanges, len_phlanges)
        fingers += [np.array([[(pos_markers[fingers_idx[j, i]], 
                                (pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]])/np.linalg.norm(pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]]), 
                                np.linalg.norm(pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]])) for i in range(fingers_idx.shape[1]-1)] for j in range(fingers_idx.shape[0])], dtype=object)]
    
    return handMarkers, palmMarkers, palmPlnNormals, palmCentroids, fingers

def generate_hand_pose(HandModel, state, pos, PS, FE, WD):
    '''
        state is the return of setHandPose(HandModel, q)
    '''
    # x = + flexion, - extension
    # y = + pronation, - supination
    # z = + ulna devation, - radial deviation

    fingers_idx = np.array([[0,3,8,17], #thumb (root to tip)
                        [4,9,13,18], #index
                        [5,10,14,19], #middle
                        [6,11,15,20], #ring
                        [7,12,16,21]]) #pinky

    handMarkers = np.zeros([1, 26, 3])
    palmMarkers = np.zeros([1, 8, 3])
    palmPlnNormals = np.zeros([1, 3])
    palmCentroids = np.zeros([1, 3])
    fingers = []

    i = 0
    pos_markers = getHandMarkers(HandModel, state, pos=pos, orient=np.array([FE, PS, WD])*np.pi/180)
    pos_markers *= 1/np.linalg.norm(pos_markers[20]-pos_markers[24]) * 200 # scale hand to 200mm length
    handMarkers[i, :] = pos_markers
    pv__ = (pos_markers[2] + pos_markers[1]) / 2 - pos_markers[6] # vector pointing from the finger root to the wrist
    pv__ /= np.linalg.norm(pv__)
    palmMarkers[i, :] = np.array([pos_markers[7] + pv__*3, pos_markers[6] + pv__*3, pos_markers[5] + pv__*3, pos_markers[4] + pv__*3, # to make sure the finger root markers do not intersect with the palm
                        pos_markers[1], pos_markers[25], pos_markers[24], pos_markers[2]])

    palmPlnNormals[i, :] = get_regressionPlane(palmMarkers[i, :]) # normal plane to the palm and palm centroid
    palmCentroids[i, :] = np.mean(palmMarkers[i, :], axis=0) # centroid marker of the palm

    # FINGERS
    # the line chain of a finger starts at the finger root and clinbs marker by marker to the finger tip
    # the base of a line is always the origin of a vector pointing from towards the tip -> this means the only frogner marker not a line origin is the finger tip
    #finger_lines has the shape (num_fingers, num_phlanges, len_phlanges)
    fingers += [np.array([[(pos_markers[fingers_idx[j, i]], 
                            (pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]])/np.linalg.norm(pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]]), 
                            np.linalg.norm(pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]])) for i in range(fingers_idx.shape[1]-1)] for j in range(fingers_idx.shape[0])], dtype=object)]
    
    return handMarkers, palmMarkers, palmPlnNormals, palmCentroids, fingers

def plot_hand(fig, pos_markers):
    if fig is None:
        fig = go.Figure()
    fig.add_trace(go.Scatter3d(
            x=pos_markers[:, 0],
            y=pos_markers[:, 1],
            z=pos_markers[:, 2],
            mode='markers+text',
            text=np.arange(0, 26),
            marker=dict(
                size=5,
                color='blue',
            )
    ))

    fig.update_layout(
        width=1024,
        height=1024,
        scene = dict(
            xaxis=dict(),
            yaxis=dict(),
            zaxis=dict(),
            aspectmode='data', #this string can be 'data', 'cube', 'auto', 'manual'
            #a custom aspectratio is defined as follows:
            aspectratio=dict(x=1, y=1, z=1)
        ), 
        scene_camera = dict(
            up=dict(x=0, y=-0, z=20),
            center=dict(x=0, y=0, z=0),
            eye=dict(x=-2, y=-0.5, z=3)
            )
    )

    return fig

def plot_hand_and_marker(pos_markers, marker, fig=None, color='red', name=''):
    if fig == None:
        fig = plot_hand(pos_markers)
    fig.add_trace(go.Scatter3d(
            x=[marker[0]],
            y=[marker[1]],
            z=[marker[2]],
            mode='markers+text',
            text=name,
            marker=dict(
                size=5,
                color=color,
            )
    ))

    return fig