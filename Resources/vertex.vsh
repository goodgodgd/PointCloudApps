precision mediump float;

uniform mat4 u_mvpmat;

attribute vec4 a_posit;
attribute vec3 a_normal;
attribute vec3 a_color;
attribute float a_ptsize;

varying lowp vec4 v_color;

void main(void)
{
    vec3 lightpos = vec3(0,0,3);
    float dp = abs(dot(lightpos - vec3(a_posit[0], a_posit[1], a_posit[2]), a_normal));
    float intensity = max(1.0 - (1.0 - dp)/2.0, 0.5);
    v_color = vec4(a_color[0]*intensity, a_color[1]*intensity, a_color[2]*intensity, 1);
    gl_Position = u_mvpmat * a_posit;
    gl_PointSize = a_ptsize;
}
