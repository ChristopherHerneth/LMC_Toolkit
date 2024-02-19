import numpy as np
import plotly.graph_objects as go
from Python.LMC_util import make_LMC, check_LMC_Hand_visibility
from Python.Hand_model import plot_hand
from Python.LMC_util import plotLMC

# optimization
def PSO_Objective(x, args):
    '''
        cost funtion for particle swarm optmization
        x has the shape
    '''
    data = args[0]
    finger_radius = args[1]
    fingers_idx = args[2]
    num_LMC = args[3]
    LMC_H = args[4]
    LMC_alpha1 = args[5]
    LMC_alpha2 = args[6]
    parameter_mult = args[7]

    pos = x*parameter_mult
    cost = []
    num_poses = data[0].shape[0]
    for xx in pos: # for each particle
        LMCs = [make_LMC(LMC_loc=np.array([xx[0+4*l], xx[1+4*l], 0]), LMC_orient=[xx[2+4*l], 0, xx[3+4*l]], 
                         LMC_H=LMC_H, LMC_alpha1=LMC_alpha1, LMC_alpha2=LMC_alpha2) for l in range(num_LMC)]
        occlusions = np.zeros([num_poses, num_LMC, 7])
        for lm, LMC in enumerate(LMCs):
            for i in range(num_poses):
                occlusions[i, lm], _= check_LMC_Hand_visibility(LMC, pos_markers=data[0][i], fingers_idx=fingers_idx, finger_lines=data[4][i], 
                                                              finger_radius=finger_radius, palm_plane_normal=data[2][i], palm_centroid=data[3][i], 
                                                              palm_markers=data[1][i], forearm_vec=data[5][i], fig=None, verbose=0)

        occlusions = np.array(occlusions).astype(np.int16)

        '''
            this metrics functions in the following
            the number of poses having a finger that is not seen by and LMC is punished the most 
            a change in the number of poses that have 0 visibility must be noticeable in the metric compared to 1, 2, 3, 4, 5 fingers, and so on
            this is why the denumerator grows with the exponential
        '''
        #print(occlusions.shape) # num_poses, num_LMC, 7 (output dim of check_LMC_Hand_visibility)
        vis = np.sum(occlusions, axis = 1)[:, 2:] # how many LMCs see each finger for each pose
        #print(vis.shape)
        vis = np.min(vis, axis=1) # take the minimum of each pose: e.g in pose i there is a finger that is only seen by 1 LMC -> this pose will get the value 1
        #print(vis)
        vis_LMC = np.zeros(num_LMC+1) # counts how many poses have 1, 2, 3, 4, 5  fngers not seen by any LMC
        unique, counts = np.unique(vis, return_counts=True)
        #print(vis_LMC, unique, counts)
        vis_LMC[unique] = counts

        met = 0
        for lm in range(num_LMC):  
            met += vis_LMC[lm] / num_poses ** (lm+1) # maximize the global visibility of all fingers

        # this doesnt work if there are not enough unique poses.
        # maximize the local visibility of fingers in an LMC
        # for lm, LMC in enumerate(LMCs):
        #     # met_ = 0
        #     # vis = np.sum(occlusions[:, lm, 2:], axis = 1) # for each pose how many fingers does this LMC see
        #     # vis_LMC = np.zeros(fingers_idx.shape[0]+1) # counts how many poses have 1, 2, 3, 4, 5  fngers not seen by any LMC
        #     # unique, counts = np.unique(vis, return_counts=True)
        #     # vis_LMC[unique] = counts

        #     # for f in range(fingers_idx.shape[0]+1):
        #     #     met_ += vis_LMC[f] / num_poses ** (f+1)
            
        #     met_ = np.sum(occlusions[:, lm, 2:]) / (num_poses * fingers_idx.shape[0]) + 1

        #     # penalize large LMC elevations 
        #     a_met = np.sum(np.abs(x[0, [2+4*lm, 3+4*lm]])) / (num_LMC*2*10*met_)
        #     met += a_met
            
        #     # maximize distance between LMCs (volume... to some extent)
        #     d_met = 0
        #     for lmm in range(num_LMC):
        #         if lmm == lm:
        #             continue
        #         d_met += np.linalg.norm(LMCs[lm][0]-LMCs[lmm][0]) / (1000*num_LMC*100) * met_
        #     met -= d_met
        cost.append(met) # a cost for each particle
    return cost

# this one has bugs!!
def PSO_Objective1(x, args):
    '''
        cost funtion for particle swarm optmization
        x has the shape
    '''
    data = args[0]
    finger_radius = args[1]
    fingers_idx = args[2]
    num_LMC = args[3]
    LMC_H = args[4]
    LMC_alpha1 = args[5]
    LMC_alpha2 = args[6]
    parameter_mult = args[7]

    pos = x*parameter_mult
    cost = []
    num_poses = data[0].shape[0]
    for xx in pos: # for each particle
        LMCs = [make_LMC(LMC_loc=np.array([xx[0+4*l], xx[1+4*l], 0]), LMC_orient=[xx[2+4*l], 0, xx[3+4*l]], 
                         LMC_H=LMC_H, LMC_alpha1=LMC_alpha1, LMC_alpha2=LMC_alpha2) for l in range(num_LMC)]
        occlusions = np.zeros([num_poses, num_LMC, 7])
        for lm, LMC in enumerate(LMCs):
            for i in range(num_poses):
                occlusions[i, lm], _ = check_LMC_Hand_visibility(LMC, pos_markers=data[0][i], fingers_idx=fingers_idx, finger_lines=data[4][i], 
                                                              finger_radius=finger_radius, palm_plane_normal=data[2][i], palm_centroid=data[3][i], 
                                                              palm_markers=data[1][i], forearm_vec=data[5][i])

        occlusions = np.array(occlusions).astype(np.int16)
        #print(occlusions.shape) # num_poses, num_LMC, num
        vis = np.sum(occlusions, axis = 1)[:, 2:] # how many LMCs see each finger for each pose
        #print(vis.shape)
        vis = np.min(vis, axis=1) # take the minimum of each pose: e.g in pose i there is a finger that is only seen by 1 LMC -> this pose will get the value 1
        #print(vis)
        vis_LMC = np.zeros(fingers_idx.shape[0]) # counts how many poses have 1, 2, 3, 4, 5  fngers not seen byy any LMC
        unique, counts = np.unique(vis, return_counts=True)
        #print(vis_LMC, unique, counts)
        vis_LMC[unique] = counts

        met = 0
        '''
            this metrics functions in the following
            the number of poses having a finger that is not seen by and LMC is punished the most 
            a change in the number of poses that have 0 visibility must be noticeable in the metric compared to 1, 2, 3, 4, 5 fingers, and so on
            this is why the denumerator grows with the exponential
        '''
        # for f in range(fingers_idx.shape[0]-1): # for each finger:
        #     print(f, f+1)
        #     met += (vis_LMC[f] + 1) / (num_poses ** (f+1))
        # met /= (num_poses ** (fingers_idx.shape[0]-2)) 

        # maximize the global visibility of all fingers
        met += vis_LMC[0] / num_poses
        met += vis_LMC[1] / num_poses ** 2
        met += vis_LMC[2] / num_poses ** 3
        met += vis_LMC[3] / num_poses ** 4
        #met *= num_poses

        # maximize the local visibility of fingers in an LMC
        for lm, LMC in enumerate(LMCs):
            met_ = 0
            vis = np.sum(occlusions, axis = 1)[lm, 2:] # how many LMCs see each finger for each pose
            vis_LMC = np.zeros(fingers_idx.shape[0]) # counts how many poses have 1, 2, 3, 4, 5  fngers not seen byy any LMC
            unique, counts = np.unique(vis, return_counts=True)
            vis_LMC[unique] = counts

            met_ += vis_LMC[0] / num_poses
            met_ += vis_LMC[1] / num_poses ** 2
            met_ += vis_LMC[2] / num_poses ** 3
            met_ += vis_LMC[3] / num_poses ** 4
            #met_ *= num_poses ** 4
            #met += met_
            
            # penalize large LMC elevations 
            a_met = np.sum(np.abs(x[0, [2+4*lm, 3+4*lm]])) / (num_LMC*2*10) * met_
            met += a_met
            
            # maximize distance between LMCs (volume... to some extent)
            d_met = 0
            for lmm in range(num_LMC):
                if lmm == lm:
                    continue
                d_met += np.linalg.norm(LMCs[lm][0]-LMCs[lmm][0]) / (1000*num_LMC*100) * met_
            met -= d_met
            
        # penalize large LMC elevations 
        #a_met = np.sum(np.abs(x*np.array([0, 0, 1, 1]*num_LMC))) / (num_LMC*2*10)
        
        # # maximize distance between LMCs (volume... to some extent)
        # ds = 0
        # for lm in range(num_LMC):
        #     for lmm in range(lm, num_LMC):
        #         ds += np.linalg.norm(LMCs[lm][0]-LMCs[lmm][0]) / 1000
        # met -= ds * 10

        # try to maximize the distance from the LMCs to the center   
        # for lm, LMC in enumerate(LMCs):
        #     met -= np.linalg.norm(LMC[0]) / 1000 
        cost.append(met) # a cost for each particle
    return cost

def plotPSOResult(pos, args):
    data = args[0]
    finger_radius = args[1]
    fingers_idx = args[2]
    num_LMC = args[3]
    LMC_H = args[4]
    LMC_alpha1 = args[5]
    LMC_alpha2 = args[6]
    parameter_mult = args[7]
    num_poses = data[0].shape[0]

    xx = pos*parameter_mult
    LMCs = [make_LMC(LMC_loc=[xx[0+4*l], xx[1+4*l], 0], LMC_orient=[xx[2+4*l], 0, xx[3+4*l]], 
                     LMC_H=LMC_H, LMC_alpha1=LMC_alpha1, LMC_alpha2=LMC_alpha2) for l in range(num_LMC)]
    occlusions = np.zeros([num_poses, num_LMC, 7])
    for lm, LMC in enumerate(LMCs):
        for i in range(num_poses):
            occlusions[i, lm], _ = check_LMC_Hand_visibility(LMC, pos_markers=data[0][i], fingers_idx=fingers_idx, 
                                                          finger_lines=data[4][0], finger_radius=finger_radius, 
                                                          palm_plane_normal=data[2][i], palm_centroid=data[3][i], 
                                                          palm_markers=data[1][i], forearm_vec=data[5][i])

    bestLMCs = np.zeros(num_LMC)
    for i in range(num_poses):
        print('_________________________________')
        print('POS: ', i)
        for lm, LMC in enumerate(LMCs):
            print(occlusions[i, lm, 2:], np.sum(occlusions[i, lm, 2:]))
        k = np.sum(occlusions[i, :, 2:], axis = 1)
        kk = np.flatnonzero(k == np.max(k))
        bestLMCs[kk] += 1
        print('Best LMC: ', kk)

    print(bestLMCs)

    occlusions = np.array(occlusions).astype(np.int16)
    vis = np.sum(occlusions, axis = 1)[:, 2:] # how many LMCs see each finger for each pose
    print(vis)
    vis = np.min(vis, axis=1) # take the minimum of each pose: e.g in pose i there is a finger that is only seen by 1 LMC -> this pose will get the value 1
    vis_LMC = np.zeros(fingers_idx.shape[0]) # counts how many poses have 1, 2, 3, 4, 5  fngers not seen byy any LMC
    unique, counts = np.unique(vis, return_counts=True)
    vis_LMC[unique] = counts
    
    fig = go.Figure()
    for lm, LMC in enumerate(LMCs):
        fig = plotLMC(fig, LMC, name='LMC_{}'.format(lm), scale = 1)

    #fig = go.Figure()
    for pos_markers in data[0]:
        fig = plot_hand(fig, pos_markers)
    #fig = plot_hand(fig, data[0][0])
    fig.show()