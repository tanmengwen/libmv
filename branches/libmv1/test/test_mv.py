
from pylab import *
import numpy
import _mv
x = _mv.TrackedSequence()
fmt = "/home/keir/thesis/movies/blender_test1/%04d.pgm"
fmt_png = "/home/keir/thesis/movies/blender_test1/%04d.png"
x.enable_debug_output()
sframe = 28
eframe = 32
x.track_pgm_sequence(fmt, sframe, eframe, 600)
r = _mv.ProjectiveReconstruction();
r.reconstruct(x);

def line(v1,v2, color='b', style=None, linewidth=None, alpha=None, zorder=None):
    "Display a line on the current figure"
    l = Line2D(array([v1[0],v2[0]]), array([v1[1],v2[1]]), color=color)
    if style:
        l.set_linestyle(style) # -, --, -., :
    if linewidth:
        l.set_linewidth(linewidth)
    if alpha:
        l.set_alpha(alpha)
    if zorder:
        l.set_zorder(zorder)
    gca().add_line(l)

def line(v1, v2, **kwargs):
    plot((v1[0],v2[0]),
         (v1[1],v2[1]), **kwargs)

def circle(center,radius, **kwargs):
    m = 8.
    theta = arange(0,2*pi+pi/m,pi/m)
    x=radius*cos(theta) + center[0]
    y=radius*sin(theta) + center[1]
    plot(x,y, **kwargs)

def plot_frame(n):
#    figure()
    im = imread(fmt_png % (sframe+n))
    matshow(im, interpolation='nearest')
    hold(True)
    meas, repro = r.get_frame_inliers(n)
    meas = array(meas)
    repro = array(repro)
    plot(meas[:,0],meas[:,1],'ys',mfc=None,mec='w',mew=1,ms=6,aa=False)
    plot(repro[:,0],repro[:,1],'r+',alpha=1.0, ms=6,aa=False)
#    for mm, rr in zip(meas, repro):
#        line(mm,rr,color='b',linewidth=2)
#        circle(mm, norm(mm-rr), color='r')
    tracks = r.get_tracks()
    for t in tracks:
        if not t: continue
        tt = array(t)
        plot(tt[:,0],tt[:,1], 'r-')
#    hold(True)
#    for k in range(len(meas)):
#        rr = repro[k,:]
#        mm = meas[k,:]
#        line(mm,rr,color='r',linewidth=2)
#        line((0,0),rr,color='r',linewidth=2)
#        hold(True)
#        print 'plotted line from',mm,'to',rr
#    show()

if 1:
    plot_frame(1);

if 0:
    figure()
    i = imread(fmt_png % 28)
    imshow(i)
    hold(True)
    y = x.get_point_features(0)
    plot(y[:,0],600-y[:,1],'r.')
    hold(True)
    y = x.get_point_features(1)
    plot(y[:,0],y[:,1],'g.')
    y = x.get_point_features(2)
    plot(y[:,0],y[:,1],'b.')
    show()

if 0:
    figure()
    k = x.get_track_matrix()
    pcolor(k,shading='flat')
    show()
