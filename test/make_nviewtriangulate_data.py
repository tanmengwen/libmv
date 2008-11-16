from generatedata import *
scene = NViewTestScene(
    nviews=4, npoints=1, dist=1.5,
    fx=3000, fy=3000,
    ax=600, ay=600, sigma=0.00)
#Ps = scene.P.reshape(10*3,4)
P0 = scene.P[0]
P1 = scene.P[1]
P2 = scene.P[2]
P3 = scene.P[3]
x = hstack(scene.x)[:2,:].T
X = array(ravel(scene.X[:,0]))
export('x X P0 P1 P2 P3')
