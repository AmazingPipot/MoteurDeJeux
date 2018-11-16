#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform sampler2D texture;

varying vec2 v_texcoord;

//out vec4 color;
//uniform vec4 ambiant_color;

varying float color_factor;
//uniform vec4 color;
uniform vec4 ambiant_color;

//! [0]
void main()
{
    // Set fragment color from texture

    //color2 = texture2D(texture, v_texcoord);
    gl_FragColor = texture2D(texture, v_texcoord) * ambiant_color;
}
//! [0]

