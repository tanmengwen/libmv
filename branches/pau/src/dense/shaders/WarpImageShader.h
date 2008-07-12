
char WarpImageShader_vert[] = "\
		\
		\
void main()		\
{		\
	gl_TexCoord[0] = gl_MultiTexCoord0;		\
		\
	gl_Position = ftransform();		\
}		\
		\
";


char WarpImageShader_frag[] = "\
		\
uniform sampler2D key_texture;		\
uniform sampler2D ref_texture;		\
		\
/* homography to the reference imagess */		\
uniform mat3 H;		\
		\
		\
void main()		\
{		\
	vec3 keyc = texture2D(key_texture, gl_TexCoord[0].st).rgb;		\
		\
	vec3 q = H * vec3(gl_TexCoord[0].st,1);		\
	vec3 refc = texture2DProj(ref_texture, q).rgb;		\
		\
	gl_FragColor = vec4(refc,1.);		\
}		\
		\
		\
";
