import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R
from Python.plotUtil import addArrow, addMarker
from Python.ray_tracing_util import intersectionPoint_pln_line, in_hull, dist_lines, get_regressionPlane

fingers_idx_LMC = np.array([[1, 2, 3, 4], #thumb (root to tip)
                    [5, 6, 7, 8], #index
                    [9, 10, 11, 12], #middle
                    [13, 14, 15, 16], #ring
                    [18, 19, 20, 21]]) #pinky

def getLMC_plns(LMC_orient, LMC_alpha1, LMC_alpha2):
    '''
        here the LMC is characterized by an orientation (extrinsic euler angles), and 2 angles that descrie the visibility pyramid of the LMC (see LMC documentation)
        The LMC are directional (The hand can only enter from one sode)

        nvevs has the shape [4, dim]; the nvecs are order clockwise, with 0 describing the plane on the long side, when the usb connector is on the left. Careful n0 will then point away from the user
        dim are the vector x, y, z directions
    '''
    LMC_orient = R.from_euler('xyz', [*LMC_orient], degrees=True).as_matrix().squeeze()

    # these vectors follow the LMC FoV planes
    LMC_vec_x_ = np.cos(LMC_alpha1) 
    LMC_vec_y_ = np.cos(LMC_alpha2)
    LMC_vec_z1_ = np.sin(LMC_alpha1)
    LMC_vec_z2_ = np.sin(LMC_alpha2)

    LMC_pln_vecs = (LMC_orient @ np.array([[0, -LMC_vec_y_, LMC_vec_z2_], [LMC_vec_x_, 0, LMC_vec_z1_], [0, LMC_vec_y_, LMC_vec_z2_], [-LMC_vec_x_, 0, LMC_vec_z1_]]).T).T
    LMC_pln_vecs = (LMC_pln_vecs.T / np.linalg.norm(LMC_pln_vecs, axis=1)).T

    # these vectors are normal to the FoV planes
    LMC_vec_xn_ = np.cos(LMC_alpha1+np.pi/2) 
    LMC_vec_yn_ = np.cos(LMC_alpha2+np.pi/2)
    LMC_vec_z1n_ = np.sin(LMC_alpha1+np.pi/2)
    LMC_vec_z2n_ = np.sin(LMC_alpha2+np.pi/2)
    
    LMC_nvecs = (LMC_orient @ np.array([[0, -LMC_vec_yn_, LMC_vec_z2n_], [LMC_vec_xn_, 0, LMC_vec_z1n_], [0, LMC_vec_yn_, LMC_vec_z2n_], [-LMC_vec_xn_, 0, LMC_vec_z1n_]]).T).T
    LMC_nvecs = (LMC_nvecs.T / np.linalg.norm(LMC_nvecs, axis=1)).T

    # this vector points in the direction such that when elbow to hand vector points in the same direction, the LMC will recognize the hand
    LMC_fwd = LMC_orient @ np.array([0, -LMC_vec_yn_, 0])
    LMC_fwd /= np.linalg.norm(LMC_fwd)

    #print(LMC_vec_x_, LMC_vec_xn_, LMC_vec_y_, LMC_vec_yn_, LMC_vec_z1_, LMC_vec_z1n_, LMC_vec_z2_, LMC_vec_z2n_)

    return LMC_nvecs, LMC_pln_vecs, LMC_fwd

def make_LMC(LMC_loc, LMC_orient, LMC_H=600, LMC_alpha1 = (180-150) / 2 * np.pi / 180, LMC_alpha2 = (180-120) / 2 * np.pi / 180):
    '''
        LMC is an array with the following entries
        0 = np.array with the x, y, z coordinates of the LMc center
        1 = np.array with the x, y, z, extrinsic euler rotatio angles of the LMC
        2 = float [mm] LMC visibility range
        3 = np.array nvevs has the shape [4, dim]; the nvecs are ordered clockwise, with 0 describing the plane on the long side, when the usb connector is on the left. Careful n0 will then point away from the user
        dim are the vector x, y, z directions
        4 = np.array plnvevs has the shape [4, dim]; the plnvecs are ordered clockwise, with 0 in the plane, when the usb connector is on the left. Careful n0 will then point away from the user
        dim are the vector x, y, z directions
        5 = float [deg] LMC FoV plane angles for the pyramid sides on the short sides (USB connector sides)
        6 = flot [deg] LMC FoV plane angles for the pyramid sides on the lon sides
    '''
    LMC_nvecs, LMC_pln_vecs, LMC_fwd = getLMC_plns(LMC_orient, LMC_alpha1, LMC_alpha2)
    return [LMC_loc, LMC_orient, LMC_H, LMC_nvecs, LMC_pln_vecs, LMC_fwd, LMC_alpha1, LMC_alpha2]

def plotLMC(fig, LMC, color='orange', name='', scale=1):
    LMC_loc = LMC[0]
    LMC_nvecs = LMC[3]
    LMC_pln_vecs = LMC[4]
    LMC_fwd = LMC[5]

    ss1 = LMC[2] #* np.cos(LMC[7]*np.pi/180)
    ss2 = LMC[2] #* np.cos(LMC[6]*np.pi/180)

    fig = addArrow(fig, LMC_loc, LMC_pln_vecs[0, :]*ss1*scale, color=color)
    fig = addArrow(fig, LMC_loc, LMC_pln_vecs[1, :]*ss2*scale, color=color)
    fig = addArrow(fig, LMC_loc, LMC_pln_vecs[2, :]*ss1*scale, color=color)
    fig = addArrow(fig, LMC_loc, LMC_pln_vecs[3, :]*ss2*scale, color=color)

    fig = addMarker(fig, LMC_loc+LMC_pln_vecs[0, :]*ss1/2*scale, color=color, name='0d')
    fig = addMarker(fig, LMC_loc+LMC_pln_vecs[1, :]*ss2*scale, color=color, name='1d')
    fig = addMarker(fig, LMC_loc+LMC_pln_vecs[2, :]*ss1/2*scale, color=color, name='2d')
    fig = addMarker(fig, LMC_loc+LMC_pln_vecs[3, :]*ss2*scale, color=color, name='3d')

    # for testing -> they should have a rightr angle with the planes!
    # fig = addArrow(fig, LMC_loc, LMC_nvecs[0, :]*10*scale, color=color)
    # fig = addArrow(fig, LMC_loc, LMC_nvecs[1, :]*10*scale, color=color)
    # fig = addArrow(fig, LMC_loc, LMC_nvecs[2, :]*10*scale, color=color)
    # fig = addArrow(fig, LMC_loc, LMC_nvecs[3, :]*10*scale, color=color)

    # fig = addMarker(fig, LMC_loc+LMC_nvecs[0, :]*5*scale, color=color, name='0d')
    # fig = addMarker(fig, LMC_loc+LMC_nvecs[1, :]*5*scale, color=color, name='1d')
    # fig = addMarker(fig, LMC_loc+LMC_nvecs[2, :]*5*scale, color=color, name='2d')
    # fig = addMarker(fig, LMC_loc+LMC_nvecs[3, :]*5*scale, color=color, name='3d')

    fig = addArrow(fig, LMC_loc, LMC_fwd*ss1*scale, color='gray')

    fig = addMarker(fig, LMC_loc, color=color, name=name) # LMC origin marker

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
            up=dict(x=0, y=-30, z=40),
            center=dict(x=0, y=0, z=0),
            eye=dict(x=0, y=-1, z=3)
            )
    )
    return fig

def testPointCloudInLMCFoV(LMC, markers, verbose=1):
    '''
        LMC = the array descrivbing the LMC FoV
        markers have the shape[num_markers, dim]; dim=3 for euklidian space
    '''
    occ = np.zeros(markers.shape[0]) # entries == 1 are outside the FoV of the LMC
    for mn, m in enumerate(markers):
        if np.linalg.norm(m) > LMC[2]: # the marker is outside the LMC range
            occ[mn] = 1
            if verbose: print('Range: ', mn)
            break
        for n in LMC[3]: # LMC plane normal vectors
            if (n @ (m-LMC[0])) < 0: # the marker is outside
                occ[mn] = 1
                if verbose: print('Outside of FoV LMC: ', n, ' marker: ', mn)

    fig = go.Figure()
    fig = plotLMC(fig, LMC, color='orange', name='0', scale=50)
    fig.add_trace(go.Scatter3d(
        x = markers[:, 0],
        y = markers[:, 1],
        z = markers[:, 2],
        mode='markers+text',
        marker=dict(
            color='black', 
            size=5,
        )
    ))
    fig.show()

# def testHandApproachDirection(LMC, vec):
#     HandModel = getHandModel()
#     q = np.zeros(20)
#     # set the thumb in a flat position
#     q[0] = 0.4
#     q[1] = -0.6
#     q[2] = 0.1
#     state = setHandPose(HandModel, q)
#     pos_markers = getHandMarkers(HandModel, state, pos=[0, 0, 0], orient=[0, 0, np.pi/2-0.1]) 
#     pos_markers *= 1/np.linalg.norm(pos_markers[20]-pos_markers[24]) * 200 # scale hand to 200mm length
#     fig = go.Figure()
#     fig = plot_hand(fig, pos_markers)

#     LMC = make_LMC(LMC_loc=(pos_markers[10] + pos_markers[5])/2+[0, 0, -250], LMC_orient=[0, 0, 0], LMC_H=600)
#     fig = go.Figure()
#     fig = plot_hand(fig, pos_markers)
#     fig = plotLMC(fig, LMC, color='orange', name='LMC', scale=50)

#     vFa = (pos_markers[24] + pos_markers[25])/2 - (pos_markers[22] + pos_markers[23])/2 # vector from the elbow to the wrist center
#     if (vFa @ LMC[5] ) < 0: # the forearm vector does not point in the same direction as the LMW forward vector 
#         print('Occluded')

#     fig.show()

def check_intersection_finger(finger, LMC_loc, marker, finger_radius, fig=None, verbose=0):
    '''
        this assumes that the marker is in the LMC Field of View
        we want to check if the direct line from the LMC center to a marker intersects the finger geometry
        We do this by checking:
            1. is the distance betwen a vector from the LMC to the marker and the vector along the finger phlange smaller than the finger radius?
            2. is the distance between the LMC and the closest point between these 2 vectors smaller than the distance form the LMC to the Marker?
            3: is the closest point between the 2 vectors in the phlange intervall
        finger[i, 0] = base location of phlange , i = num phlanges in the finger
        finger[i, 1] = unit vector along phlange
        finger[i, 3] = length of phlange
        LMC_loc = x, y, z location fo the LMC center
        marker = x, y, z, location of the marker that we want to check if the line of sight intersects from the LMC to the marker with the finger, 
        finger_radius float in mm 
        fig = None or go.Figure, 
        verbose= 0 we stop at the first intersection, 1 = we continue and check all phlanges (for testing)
        return: if fig is not None: fic and bool: True if there is an intersection (the marker is occluded by the finger)
        return bool: True if there is an intersection (the marker is occluded by the finger)
        return bool array for all the intersection phlanges if verbose == 1
    '''


    v1 = marker - LMC_loc
    dist_to_marker = np.linalg.norm(v1) # ditance form LMC to marker
    v1 /= dist_to_marker # vector from LMC center to marker 
    occ = np.zeros(finger.shape[0])
    for phlange in range(finger.shape[0]): # for all phlanges
        d, t1, t2 = dist_lines(p1=LMC_loc, p2=finger[phlange][0], v1=v1, v2=finger[phlange][1]) # LMC center, base of phlange, vector LMC to marker, vector along finger phlange
        occlusion = d < finger_radius and dist_to_marker >= t1 and t1 > 0 and t2 > 0 and t2 < finger[phlange][2]
        occ[phlange] = occlusion
        if fig is None and occlusion:
            if verbose > 0:
                print('      Phlange: {} intersection'.format(phlange))
            return True, fig
        elif fig is not None:
            if occlusion: 
                fig = addArrow(fig, finger[phlange][0], finger[phlange][1]*finger[phlange][2], color='red')
                #print(d, t1, t2, d < finger_radius, dist_to_marker >= t1,  t2 > 0,  t2 < finger_lines[finger, i][2])
                # print markers at closest location between rays
                fig = addArrow(fig, LMC_loc, v1*(dist_to_marker-5), color='red') # arrow to the marker we are investigating
                #fig = addMarker(fig, LMC_loc+v1*t1, color='red', name=str(phlange))
                #fig = addMarker(fig, finger[phlange][0]+finger[phlange][1]*t2, color='red', name=str(phlange))
                if verbose > 0:
                    print('      Phlange: {} intersection'.format(phlange))
                else:
                    return True, fig
            #else:
                #fig = addArrow(fig, LMC_loc, v1*(dist_to_marker-5), color='green') # arrow to the marker we are investigating
                #fig = addMarker(fig, LMC_loc+v1*t1, color='green', name=str(phlange))
                #fig = addMarker(fig, finger[phlange][0]+finger[phlange][1]*t2, color='green', name=str(phlange))

    if verbose == 0:        
        return False, fig
    elif verbose == 2:
        return occ, fig
    else:
        return np.max(occ), fig
        
def check_intersection_palm(LMC_loc, marker, palm_plane_normal, palm_centroid, palm_markers, fig=None, verbose=0):
    '''check if the line of sight is blocked by the palm plane'''
    v = marker - LMC_loc # vector from LMC center to marker
    ll = np.linalg.norm(v)
    v /= ll
    intersection_point = intersectionPoint_pln_line(n=palm_plane_normal, v=v, c=palm_centroid, p=marker)
    LMC_IP = (intersection_point - LMC_loc) / np.linalg.norm(intersection_point - LMC_loc) # vector from LMC to palm intersection point
    occlusion = False
    if np.linalg.norm(intersection_point - LMC_loc) < ll and np.arccos(LMC_IP@v) == 0: # if the finger marker is closer to the LMC than the intersection point, there cant be a collision AND the vector from the LMC to the intersection point and the vector from the LMC to the marker point in the same direction
        occlusion = in_hull(palm_markers, intersection_point) # check if the intersection point lies in the conves hull of the palm markers
        
        if verbose > 0:
            print('    Palm intersection')

        if fig is not None:
            #fig = addArrow(fig, LMC_loc, v*100, color='black')
            if occlusion:  
                fig = addMarker(fig, intersection_point, color='red')#, name=str(m))
            # else:
            #     fig = addMarker(fig, intersection_point, color='green')#, name=str(m))
    return occlusion, fig

def check_LMC_Hand_visibility(LMC, pos_markers, fingers_idx, finger_lines, finger_radius, palm_plane_normal, palm_centroid, palm_markers, forearm_vec, mode='LMC', fig=None, verbose=0):
    '''
        CHECKS IF THE HAND IS VISIBLE 
        forearm_vec = vector from the elbow to the wrist center
        mode can be either of 'LMC' or 'OSIM', depending on the hand model used
    '''
    occlusions_ = np.ones(7)
    ###############################################################
    # 0 check if the approach angle is correct
    # the approach vector must pass through the LMC entrance plane
    if (forearm_vec @ LMC[5] ) < 0: # the forearm vector does not point in the same direction as the LMC forward vector 
        occlusions_ *= 0
        if verbose == 1:
            print('    Wrong approach direction')
        else:
            return occlusions_, fig
    ###############################################################
    # 1 check if all markers are in the LMC volume
    # the inproduct of all LMC plane normal vectors with all markers needs to be non-negative
    if mode == 'LMC':
        pp = pos_markers[1:]
    elif mode == 'OSIM':
        pp = pos_markers[[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 24, 25]]
    else:
        pp = pos_markers
    
    for mm, m in enumerate(pp): # remove marker indices 22, 23 (far away elbow) -> they can be outside
        if np.linalg.norm(m-LMC[0]) > LMC[2]: # the marker is outside the LMC range
            occlusions_ *= 0
            if verbose == 1:
                print('    Marker {} out of range'.format(mm))
            else:
                return occlusions_, fig
        for n in LMC[3]: # LMC plane normal vectors
            if (n @ (m-LMC[0])) < 0: # the marker is outside
                occlusions_ *= 0
                if verbose == 1:
                    print('    Marker {} out of LMC FoV'.format(mm))
                else:
                    return occlusions_, fig
    
    # if occlusions_[1] == 0:
    #     print('Outside LMC range')

    ###############################################################
    # 2 check if center marker is visible (occluded by fingers)
    # doesnt make sense now
    # check with all fingers
    # cv = False # is the palm centroid occluded by fingers?
    # for f in range(fingers_idx.shape[0]):
    #     if check_intersection_finger(finger_lines[f], LMC_loc=LMC0[0], marker=palm_centroid, finger_radius=finger_radius):
    #         cv = True
    #         break
    # print(cv)

    ##############################################################
    # 3 check all finger markers
    # check if marker passes through palm plane to be seen
    # check against all other fingers (we ignore self occlusion)
    for f in range(fingers_idx.shape[0]): # finger
        if verbose > 0:
            print('___________________________________')
            print('Finger ', f)
        for m in fingers_idx[f]: # for all finger markers if a single finger marker is occluded, the entire finger is marked as not visible
            marker = pos_markers[m] # finger marker
            fv, fig = check_intersection_palm(LMC[0], marker, palm_plane_normal, palm_centroid, palm_markers, fig=fig, verbose=0) #check if the line of sight is blocked by the palm plane
            if fv:
                occlusions_[2+f] = 0
                if verbose == 1:
                    print('    Finger {}, marker {}; occluded by Palm'.format(f, m))
                else:
                    break
            for of in range(fingers_idx.shape[0]): # check if the line of sight is blocked by a other fingers of the handr
                if of == f:
                    continue
                fv, fig = check_intersection_finger(finger_lines[of], LMC_loc=LMC[0], marker=marker, finger_radius=finger_radius, fig=fig, verbose=0)
                if fv:
                    occlusions_[2+f] = 0
                    if verbose == 1:
                        print('    Finger {}, marker {}: occluded by Finger {}'.format(f, m, of))
                    else:
                        break    
        # if occlusions_[2+f] == 0: # when a single finger is not visible
        #     break

    return occlusions_, fig

def make_hand_poses(frames):
    '''
        makes the input to check_LMC_Hand_visibility()
        CAREFUL HERE!! Make sure to onl yinclude the markers in frames that are visible in the documentation!
        frames have the shape (num_frames, num_markers, 3)
    '''
    fingers_idx = fingers_idx_LMC
    num_poses = frames.shape[0]
    handMarkers = np.zeros([num_poses, frames.shape[1], 3])
    palmMarkers = np.zeros([num_poses, 6, 3])
    palmPlnNormals = np.zeros([num_poses, 3])
    palmCentroids = np.zeros([num_poses, 3])
    forearm_vecs = np.zeros([num_poses, 3])
    fingers = []

    for i in range(num_poses):
        pos_markers = frames[i, :]
        handMarkers[i, :] = pos_markers
        pv1__ =  pos_markers[1] - pos_markers[9] # vector pointing from the finger root to the wrist
        pv1__ /= np.linalg.norm(pv1__)
        palmMarkers[i, :] = np.array([pos_markers[5] + pv1__*3, pos_markers[9] + # to make sure the finger root markers do not intersect with the palm
                                      pv1__*3, pos_markers[13] + pv1__*3, pos_markers[18] + pv1__*3, pos_markers[1] - pv1__*5, pos_markers[17]])

        palmPlnNormals[i, :] = get_regressionPlane(palmMarkers[i, :]) # normal plane to the palm and palm centroid
        palmCentroids[i, :] = np.mean(palmMarkers[i, :], axis=0) # centroid marker of the palm
        forearm_vecs[i, :] = (pos_markers[1] + pos_markers[17])/2 - pos_markers[0] # vector from the elbow to the wrist center

        # FINGERS
        # the line chain of a finger starts at the finger root and clinbs marker by marker to the finger tip
        # the base of a line is always the origin of a vector pointing from towards the tip -> this means the only frogner marker not a line origin is the finger tip
        #finger_lines has the shape (num_fingers, num_phlanges, len_phlanges)
        fingers += [np.array([[(pos_markers[fingers_idx[j, i]], 
                                (pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]])/np.linalg.norm(pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]]), 
                                np.linalg.norm(pos_markers[fingers_idx[j, i+1]]-pos_markers[fingers_idx[j, i]])) for i in range(fingers_idx.shape[1]-1)] for j in range(fingers_idx.shape[0])], dtype=object)]
    
    return handMarkers, palmMarkers, palmPlnNormals, palmCentroids, fingers, forearm_vecs