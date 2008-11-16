from generatedata import *
nviews = 18
scene = NViewTestScene(
    nviews=nviews, npoints=50, dist=1.5,
    fx=3000, fy=3000,
    ax=600, ay=600,
    sigma=0.00)

# 2D projections are roughly in [0,1200]
x = vstack([xx[:2,:] for xx in scene.x]).T

# Make the first 20 of the 50 points outliers
noise_magnitude = 10
for i in range(nviews):
    scene.x[i,:2,:20] += noise_magnitude*(randn(2,20)-.5)

x_noisy = vstack([xx[:2,:] for xx in scene.x]).T

#scene.show_view(0)
export('x x_noisy')
