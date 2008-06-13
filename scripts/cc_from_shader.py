#!/usr/bin/env python

import sys

shader_name = sys.argv[1]

fvert = open( shader_name+'.vert' )
ffrag = open( shader_name+'.frag' )

fout = open( shader_name+'.h','w' )

text = '\nchar %s_vert[] = "\\\n'%shader_name

for line in fvert:
	line = line.rstrip('\n').rstrip('\r')
	text += line + '\t\t\\\n'
text += '";\n\n'

text += '\nchar %s_frag[] = "\\\n'%shader_name
for line in ffrag:
	line = line.rstrip('\n').rstrip('\r')
	text += line + '\t\t\\\n'
text += '";\n\n'


fout.write( text )

fout.close()
