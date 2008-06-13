
char PlaneSweepShader_vert[] = "\
		\
		\
void main()		\
{		\
	gl_Position = ftransform();		\
}		\
		\
";


char PlaneSweepShader_frag[] = "\
		\
/* the number of reference images */		\
uniform int N;		\
/* the minimum number of required inliers */		\
uniform int K;		\
		\
const int maxN=6;		\
		\
uniform sampler2D key_texture;		\
uniform sampler2D ref_texture0;		\
uniform sampler2D ref_texture1;		\
uniform sampler2D ref_texture2;		\
uniform sampler2D ref_texture3;		\
uniform sampler2D ref_texture4;		\
uniform sampler2D ref_texture5;		\
		\
/* homographies to the reference imagess */		\
uniform mat3 H0;  /* TODO: use built-in projective coordinates ql_TexCoord[i] */		\
uniform mat3 H1;		\
uniform mat3 H2;		\
uniform mat3 H3;		\
uniform mat3 H4;		\
uniform mat3 H5;		\
		\
/* the inverse of the key image sizes */		\
uniform float key_width1, key_height1;		\
		\
void main()		\
{		\
	/* texture coordinates of the key image */		\
	float keyx = (gl_FragCoord.x + 0.5) * key_width1;		\
	float keyy = (gl_FragCoord.y + 0.5) * key_height1;		\
	/* color of the key image */		\
	vec3 keyc = texture2D(key_texture, vec2(keyx,keyy)).rgb;		\
		\
	float sumcost=0.;		\
		\
	if(0<N)	{		\
		vec3 q = H0 * vec3(gl_FragCoord.xy,1);		\
		vec3 refc = texture2DProj(ref_texture0, q).rgb;		\
		sumcost += distance(keyc,refc);		\
	}		\
	if(1<N)	{		\
		vec3 q = H1 * vec3(gl_FragCoord.xy,1);		\
		vec3 refc = texture2DProj(ref_texture1, q).rgb;		\
		sumcost += distance(keyc,refc);		\
	}		\
		\
	gl_FragColor = vec4(keyc,sumcost);		\
}		\
		\
";

