#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif
uniform mat4 mvp_matrix;

//attribute vec3 a_normal;
attribute vec4 a_position;
attribute vec2 a_texcoord;
uniform vec4 color;
attribute vec4 a_normal;
//varying vec3 color_factor;
varying float color_factor;
uniform vec4 light_direction;
varying vec4 ambiant_color;

varying vec2 v_texcoord;


//! [0]
void main()
{

    //color_factor = max(dot(normal, light_direction), 0.0);

    // Calculate vertex position in screen space
    //color_factor = max(dot(normal, light_direction), 0.0);
    gl_Position = mvp_matrix * a_position;
    color_factor = max(dot(a_normal,light_direction),0.0);


    // Pass texture coordinate to fragment shader
    // Value will be automatically interpolated to fragments inside polygon faces
    v_texcoord = a_texcoord;
    ambiant_color = color;
}
//! [0]
