#version 120
#extension GL_ARB_shading_language_420pack : require

// RViz's native indexed map shader with sampler bindings assigned at link time.
// Avoids Mesa/Ogre validating sampler2D and sampler1D on the same default unit.
uniform float alpha;
layout(binding = 0) uniform sampler2D eight_bit_image;
layout(binding = 1) uniform sampler1D palette;
varying vec2 UV;

void main()
{
  vec4 color = texture1D(palette, 0.999 * texture2D(eight_bit_image, UV).x);
  gl_FragColor = vec4(color.rgb, color.a * alpha);
}
