
const float beta2 = 10./255.;

uniform sampler2D I_texture;
  /* ( conv(I), conv(I*I), I, conv(1) ) */
uniform sampler2D J_texture;
  /* ( conv(J), conv(J*J), conv(I*J), conv(1) ) */


void main()
{
	vec4 Itex = texture2D(I_texture, gl_TexCoord[0].st);
	vec4 Jtex = texture2D(J_texture, gl_TexCoord[0].st);

	float w = 1.;
	/* Jtex[3]; */

	float muI = Itex[0] / w;
	float vI  = Itex[1] / w - muI*muI + beta2;
	float muJ = Jtex[0] / w;
	float vJ  = Jtex[1] / w - muJ*muJ + beta2;

	float vIJ = Jtex[2] / w - muI*muJ;

	float ZNCC = vIJ / sqrt(vI*vJ);
	float score = 1. - ZNCC;

	gl_FragColor = vec4(score,score,score,score);
}

