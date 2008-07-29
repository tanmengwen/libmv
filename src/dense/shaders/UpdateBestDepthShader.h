
char UpdateBestDepthShader_vert[] = "\
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


char UpdateBestDepthShader_frag[] = "\
		\
		\
uniform sampler2D depth_score_texture;		\
uniform sampler2D best_depth_texture;		\
		\
uniform float depth;		\
uniform float nplanes;		\
		\
		\
void main() {		\
  /* TODO should be texture2D(...)[channel_of_interest] */		\
  float score = texture2D(depth_score_texture, gl_TexCoord[0].st)[1];		\
		\
  vec4 old = texture2D(best_depth_texture, gl_TexCoord[0].st);		\
		\
  if(score < old.g) {		\
    gl_FragColor.r = depth;		\
    gl_FragColor.g = score;		\
  }		\
  else {		\
    gl_FragColor.r = old.r;		\
    gl_FragColor.g = old.g;		\
  }		\
  gl_FragColor.b = old.b + (1. - score) / nplanes;		\
}		\
		\
";

