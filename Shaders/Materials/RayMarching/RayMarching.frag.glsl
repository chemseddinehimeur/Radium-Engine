struct Material
{
    sampler3D buffer;
};

uniform Material material;

layout (location = 0) out vec4 fragColor;
// These are used for "traditional" depth prepass. Use it for debug here ...
layout (location = 1) out vec4 out_normal;
layout (location = 2) out vec4 out_diffuse;
layout (location = 3) out vec4 out_specular;


layout (location = 0) in vec3 in_position;
layout (location = 1) in vec3 in_texcoord;
//layout (location = 2) in vec3 in_viewVector;
layout (location = 3) in vec3 in_cameraInModelSpace;

float stepsize = 0.01;


vec4 shade(vec3 position, in float value) {
//    if ( value > 1)
//        return (vec4(0.1, 0., 0., 0.1));
        return vec4(vec3(value), 1.0);
}

void main(void) {
	float value;
	vec4 color;
	vec4 frgColor = vec4(0.0);
    bool hit = false;

	// le rayon
	vec3 raypos = in_texcoord;//in_position;
    vec3 camera = in_cameraInModelSpace;
    vec3 raydir =in_position - camera;
    raydir.x /= 4;
    raydir = normalize(raydir) * stepsize;

    for(int i = 0;
        i<1000;
        i++) {
        value = texture(material.buffer, raypos).r;
        if ( value != 0. )
        {
            frgColor += vec4(value);
            hit = true;
        }

        raypos = raypos + raydir;
        if ( any(greaterThan(raypos, vec3(1.0))) ||
             any(lessThan(raypos, vec3(0.0))) ||
             all(greaterThan(frgColor.xyz, vec3(1.0)))
           )
           break;
    }

    if ( ! hit ) discard;

    fragColor = frgColor;

    out_normal = vec4(normalize(raydir), 1.0);
    out_diffuse = vec4(normalize(in_position), 1.0);
    out_specular = vec4(raypos, 1.0);

}
