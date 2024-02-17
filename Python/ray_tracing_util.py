import numpy as np
from scipy.optimize import linprog

def intersectionPoint_pln_line(n, v, c, p):
    # https://math.stackexchange.com/questions/100439/determine-where-a-vector-will-intersect-a-plane
    ''''finds the point of intersection of a plane and a line (if it exists)
        n = plane normal vector
        v = vector along line
        c = a point on the plane
        p = point on line

        return np.array([x, y, z]) of intersection point or -1 if line and plane are parallel
    '''
    if np.cross(v, n).all() == 0:
        return -1 # the plane and the line are parallel and there are infinitely many solutions
    
    t = - (n @ (p - c)) / (n @ v)
    return p + t * v

def in_hull(points, x):
    '''
        check if point x lies within the convex hull of points
        return true if the point is in the hull
    '''
    # https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
    n_points = len(points)
    n_dim = len(x)
    c = np.zeros(n_points)
    A = np.r_[points.T,np.ones((1,n_points))]
    b = np.r_[x, np.ones(1)]
    lp = linprog(c, A_eq=A, b_eq=b)
    return lp.success

def dist_lines(p1, p2, v1, v2):
    '''calculates the shortest distance between lines and the position on the lines
        p1 = point on line 1
        v1 = vector along line 1 (unit lenth)
        p2 = point on line 2
        v2 = vector along line 2 (unit length)

        return d = shortest distance between lines, t1: p1 + v1 * t1  = point on line 1, where the distsnce to line 2 is closest, t2: vice versa 
    '''
    #https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
    nl = np.cross(v1, v2)
    d = abs(nl @ (p1 - p2)) / np.linalg.norm(nl)
    t1 = ((np.cross(v2, nl)) @ (p2 - p1)) / (nl @ nl)
    t2 = ((np.cross(v1, nl)) @ (p2 - p1)) / (nl @ nl)

    return d, t1, t2

def get_regressionPlane(markers):
    ''' 
        calculates the regression plane, best fitting the markers
        the markers should have th efollowing shape: (num_markers, dimensions)

        return: ndarray(size=dimensions) normal vector the plane (the direction is not determines)
    '''

    svd = np.linalg.svd(markers.T - np.mean(markers, axis=0, keepdims=True).T) 
    right = svd[0]
    n = right[:, -1].squeeze() # vector along prosthesis pronation axis
    n /= np.linalg.norm(n)

    return n
