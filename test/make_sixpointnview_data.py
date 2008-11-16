from generatedata import *
scene = NViewTestScene(
    nviews=18, npoints=6, dist=1.5,
    fx=3000, fy=3000,
    ax=600, ay=600,
    sigma=0.00)
x = hstack(scene.x)
#scene.show_view(0)
export('x')
