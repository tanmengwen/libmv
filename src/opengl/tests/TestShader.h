
char TestShader_vert[] = "\
		\
void main()		\
{		\
	gl_TexCoord[0] = gl_MultiTexCoord0;		\
	gl_Position = ftransform();		\
}		\
		\
";


char TestShader_frag[] = "\
		\
uniform sampler2D templeTex;		\
		\
void main()		\
{		\
	vec2 coords = gl_TexCoord[0].st;		\
	coords.y = coords.y + sin(30*coords.x) / 10;		\
	vec3 c = vec3(texture2D(templeTex, coords));		\
	gl_FragColor = vec4(c, 1.0);		\
}		\
		\
";

