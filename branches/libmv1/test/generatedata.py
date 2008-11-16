import inspect 
from numpy.random import rand, randn
from numpy import *

def norm(x):
    return sqrt(sum(x**2))

def make_lookat_rotation(center, up=(0,1,0)):
    """Make a rotation matrix such that center becomes the direction of the
    positive z-axis, and y is oriented close to up"""
    zc = center / sqrt(sum(center**2))
    xc = cross(up, zc)
    xc /= sqrt(sum(xc**2))
    yc = cross(zc, xc)
    return matrix((xc,yc,zc))

# Guarentee repeatable random data
random.seed(1)

def emit_variable_as_c(name, value):
    if type(value) == int:
        return 'unsigned int %s = %s;\n' % (name, value)
    ret = []
    if len(value.shape) == 1:
        ret.append('unsigned int %s_size = %s' % (name, len(value)))
    else:
        ret.append('unsigned int %s_size1 = %s' % (name, value.shape[0]))
        ret.append('unsigned int %s_size2 = %s' % (name, value.shape[1]))
    ret.append('double %s_raw[] = {%s}' % (name, repr(list(ravel(value)))[1:-1]))
    ret.append('') # trailing semi
    return ';\n'.join(ret)

def export(variables):
    "Emit a string of string separated variables as literal c code"
    print 'namespace {\n'
    frame = inspect.currentframe().f_back
    d = dict(frame.f_globals)
    d.update(frame.f_locals)
    for var in variables.split():
        print emit_variable_as_c(var, d[var])
    print '} // end data anonymous namespace'

class NViewTestScene(object):
    """A N-View test scene, with lines and points. The scene in 3-space along
    with it's projection in each of the N views is computed. Lines in the world
    are represented by their two threespace end points, and lines in each image
    are projected stored as normals to the plane formed by the two endpoints in
    the image and the camera center."""
    def __init__(self, nviews=5, npoints=50, nlines=0, fx=650, fy=650, ax=300, ay=240,
                 s=0, sigma=0.00, dist=10, uniform_center_jitter=0.00, X=None):
        """
        Generate a new N-View scene.
        @param nviews   Number of views
        @param npoints  Number of views
        @param nlines   Number of views
        @param fx       Focal length x
        @param fy       Focal length y
        @param ax       Camera center x
        @param ay       Camera center y
        @param s        Camera CCD skew
        @param sigma    Amount of noise added to image points
        """

        self.nviews = nviews
        self.npoints = npoints

        # Make points centered on the origin.
        if X != None:
            self.X = X
        else:
            X = self.X = matrix(rand(4, npoints))
            X[:3,:] -= 0.5 
            X[3,:] = 1
        x = self.x = zeros((nviews, 3, npoints))
        # Projective depths
        z = zeros((nviews, npoints)) # private for now
        P = self.P = zeros((nviews, 3, 4))

        # C is around a ring centered at the origin, on the xz plane
        C = self.C = matrix(zeros((3, nviews)))
        R = self.R = zeros((nviews, 3, 3))
        # Same camera for each view
        K = self.K = matrix([[fx,  s,  ax],
                             [0,  fy,  ay],
                             [0,   0,   1]])

        for i in range(nviews):
            theta = rand()*2*pi
            theta = i*2*pi/nviews
            C[0,i] = cos(theta)
            C[1,i] = sin(theta)
            C[2,i] = 0.0
            C[:,i] *= dist
            if i < nviews/2:
                C[:,i] *= 2
            # Add some noise to the optical center of each camera
            # to prevent issues when all camera optical centers
            # intersect exactly.
            lookdir = -array(C)[:,i]
            lookdir /= norm(lookdir)
            lookdir += uniform_center_jitter*(rand(3)-0.5)
            R[i] = make_lookat_rotation(lookdir)
            t = -R[i]*C[:,i]
            P[i] = K * hstack((R[i], t))
            x[i] = P[i]*X 
            z[i] = x[i,2,:]
            # Normalizing breaks points at infinity
            x[i] /= x[i,2,:]
            x[i,:2,:] += randn(2,npoints)*sigma 

    def show_view(self, n):
        from pylab import *
#        figure()
        # Since the x direction is reversed, we have to flip x
#        xn = mat('-1 0 0; 0 1 0') * self.x[n]
        xn = self.x[n]
        plot(xn[0,:], xn[1,:], 'r.')
        show()

    def show(self):
        import matplotlib.axes3d as p3

        fig = figure()
        ax = p3.Axes3D(fig)
        X = numpy.array(self.X)
        ax.scatter3D(X[0,:],X[1,:],X[2,:])
        for i in range(self.nviews):
            k1x = dot(linalg.inv(self.K), self.x[i])
            x = dot(self.R[i].T, k1x) + numpy.array(self.C[:,i])
            minx = min(k1x[0,:])
            miny = min(k1x[1,:])
            maxx = max(k1x[0,:])
            maxy = max(k1x[1,:])
            corners = array(((minx, maxx, maxx, minx, minx,),
                             (miny, miny, maxy, maxy, miny,),
                             (1.0,  1.0,  1.0,  1.0,  1.0, )))
            frame = dot(self.R[i].T, corners) + numpy.array(self.C[:,i])
            hold(True)
            ax.scatter3D(x[0,:],x[1,:],x[2,:], c='g')
            ax.scatter3D(array([self.C[0,i]]),
                         array([self.C[1,i]]),
                         array([self.C[2,i]]), c='g')
            # Each point on the line is on it's own row; the columns are x,y,z
            ax.add_lines([frame.T], colors=['g'])
            for j in range(4):
                ax.add_lines([vstack((frame[:,j].T, self.C[:,i].T)).A],
                             colors=['g'])

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
        ax.scatter3D(array([0]),
                     array([0]),
                     array([0]), c='r')
        show()

    def canonicalize_camera(self, canonical):
        """
        Transform all image points, world points, and camera matricies such
        that P[i] = [I|0]
        """
        # FIXME insert reference to thesis appendix here
        # FIXME write thesis appendix with this derivation
        Rcan = self.R[canonical].copy()   # Oops! views can hurt
        Ccan = self.C[:,canonical].copy()
        M = vstack((hstack((Rcan.T,       Ccan)),
                    hstack((zeros((1,3)), ones((1,1))))))
        Minv = linalg.inv(M)
        Kinv = linalg.inv(self.K)

        # Transform the views
        for cam in range(self.nviews):
            Rp = dot(self.R[cam], Rcan.T)
            Cp = dot(Rcan, self.C[:,cam]-Ccan)
            Pp = hstack((Rp, -dot(Rp, Cp)))
            xp = dot(Kinv, self.x[cam])
            self.R[cam] = Rp
            self.C[:,cam] = Cp
            self.P[cam] = Pp
            self.x[cam] = xp

        self.X = dot(Minv, self.X)

