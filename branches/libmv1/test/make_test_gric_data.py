# this generates test data for GRIC model selection.
# the data is used by test_gric.cc
from pylab import *
from numpy import *

# looking down positive z axis
# scene is at x,y in 1,1
# at distance z=3
n = 20
X = 2*rand(4,n)
X[2,:] += 3
X[3,:] = 1.0

rgb='rgb'
hold(True)
def make_projected(X, scale=0.1, outlier_fraction=0.2):
    n = X.shape[1]
    num_outliers = n*outlier_fraction;
    xs = []
    for t in 0,1,2:
        P = eye(4)
        P[:,3] = [scale*t,0,0,1]
        x = dot(P,X)
        x /= x[2,:]
        x = x[:2,:]
        outliers = rand(2,num_outliers)
        outliers *= c_[amax(x,1) - amin(x,1)]
        outliers -= c_[amin(x,1)]
        xs.append(hstack((x,outliers)))
    return xs

# 3-view general scene
xs_ppp = make_projected(X)

# flatten scene; two H's work as well as 2 P's
X[2,:] = 3
xs_hh = make_projected(X)

xs = xs_hh
xs = xs_ppp

# only for debugging
#for i in 0,1,2:
#    plot(xs[i][0,:],xs[i][1,:],rgb[i]+'.')
#show()

def to_list(x):
    if type(x) == type([]) or len(x.shape) == 2 :
        return [to_list(y) for y in x]
    if len(x.shape) == 1:
        return list(x)

json_out = dict(hh=to_list(xs_hh), ppp=to_list(xs_ppp))
c_out = repr(json_out).replace("'",'\\"')
print 'static const char *gric_json = "%s";' % c_out
