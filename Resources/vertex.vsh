uniform mediump mat4 in_mvpmat;
attribute mediump vec4 in_posit;
attribute mediump vec3 in_normal;
attribute mediump float in_ptsize;
attribute mediump vec3 in_color;
varying mediump vec4 out_color;

void main(void)
{
	out_color.xyz = in_color;
	out_color.w = 1.f;
    gl_Position = in_mvpmat * in_posit;
    gl_PointSize = in_ptsize;
}

