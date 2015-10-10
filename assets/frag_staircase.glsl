#version 130

uniform float time;
in vec4 color;
in float brightness;
in vec2 uv;

void main()
{
	gl_FragColor = vec4(1., 0.5,0., 1.);
	gl_FragColor = color;
	float b = float(brightness > 0.5);
	// b = float(brightness>0.7) * brightness / 0.7;
	gl_FragColor = 0.4 * vec4(b+sin(uv.x+sin(0.25*time)*.5*.3)*.1,b+sin(0.1*uv.y+0.3+0.1*sin(time*0.1))*.175,b,0.85);
}
