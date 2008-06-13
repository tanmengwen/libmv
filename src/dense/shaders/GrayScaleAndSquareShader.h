
char GrayScaleAndSquareShader_vert[] = "\
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


char GrayScaleAndSquareShader_frag[] = "\
		\
uniform sampler2D I_texture;		\
		\
void main()		\
{		\
	vec3 colorI = texture2D(I_texture, gl_TexCoord[0].st).rgb;		\
		\
	/* convert to grayscale */		\
	float I = dot(vec3(0.2125, 0.7154, 0.0721), colorI);		\
		\
	gl_FragColor = vec4( I, I*I, I, 1 );		\
}		\
		\
";

