

uniform sampler2D to_push_texture;
uniform sampler2D sorted_list_texture;


void main()
{
	float score = texture2D(to_push_texture, gl_TexCoord[0].st).r;

	vec4 olds = texture2D(sorted_list_texture, gl_TexCoord[0].st);

	/* push score in the sorted vector olds */
	if(score < olds.a)
	{
		if(score < olds.b)
		{
			olds.a = olds.b;
			if(score < olds.g)
			{
				olds.b = olds.g;
				if(score < olds.r)
				{
					olds.g = olds.r;
					olds.r = score;
				}
				else
					olds.g = score;
			}
			else
				olds.b = score;
		}
		else
			olds.a = score;
	}

	gl_FragColor = olds;
}

