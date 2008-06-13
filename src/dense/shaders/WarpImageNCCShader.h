
char WarpImageNCCShader_vert[] = "\
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


char WarpImageNCCShader_frag[] = "\
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
	float I = texture2D(key_texture, gl_TexCoord[0].st).b;		\
		\
	vec3 q = H * vec3(gl_TexCoord[0].st,1);		\
	vec3 colorI = texture2DProj(ref_texture, q).rgb;		\
		\
	/* convert to grayscale */		\
	float J = dot(vec3(0.2125, 0.7154, 0.0721), colorI);		\
		\
	gl_FragColor = vec4( J, J*J, I*J, 1. );		\
}		\
		\
		\
";

