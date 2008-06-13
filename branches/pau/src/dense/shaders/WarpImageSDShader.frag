
uniform sampler2D key_texture;
uniform sampler2D ref_texture;

/* homography to the reference imagess */
uniform mat3 H;


void main()
{
	vec3 keyc = texture2D(key_texture, gl_TexCoord[0].st).rgb;

	vec3 q = H * vec3(gl_TexCoord[0].st,1);
	vec3 refc = texture2DProj(ref_texture, q).rgb;

	float score = distance(keyc,refc);

	gl_FragColor = vec4(score);
}


