import Blender
from Blender import *
from Blender.Scene import Render

scn = Scene.GetCurrent()
context = scn.getRenderingContext()

print context.__class__, type(context)
Render.EnableDispWin()
#context.extensions = True
#context.renderPath = "//myRenderdir/"
#context.sizePreset(Render.PC)
#context.imageType = Render.AVIRAW
for i in range(10):
	print "--------- rendering frame:",i
	context.startFrame(i)
	context.endFrame(i)
#context.renderAnim()
	context.render()

#context.imageType = Render.TARGA
#context.fps = 15
#context.sFrame = 15
#context.eFrame = 22
#context.renderAnim()

#Render.CloseRenderWindow()
print context.fps
print context.cFrame