3.2.3

Fixed:
- Error regarding "SetRenderAttachmentDepth" when depth priming mode was Disabled in Unity 6.1

3.2.2

Fixed:
- Rendering taking effect even when camera is outside of an Underwater Area for components created before v3.2.0

Changed:
- Underwater Area is no longer rendering if the assigned Box Collider is disabled

3.2.1

Fixed:
- Some shading features (eg. Caustics/Translucency) being disabled in a build if not originally enabled in the editor (regression since v3.2.0)
- Memory leak when using Skybox lighting (0.9kb)

3.2.0
Requires v3.2.0 of the Stylized Water 3 base asset!

Added:
- "Render Method" option on the render feature. Specifies if rendering takes place before transparent materials, or during.
- Support for multiple simultaneous camera's (eg. split-screen)

Changed:
- Various improvements to underwater shading, surface clarity decreases as the camera goes deeper
- Improved camera submersion detection, now also factors in the screen corners.

3.1.1

Added:
- Underwater Area now supports rotated box colliders

Changed:
- Detection of the active Underwater Area now also factors which is closest, to better handle two or more overlapping areas

Fixed:
- Lake volume in demo not having the same material assigned as the water mesh
- Removed missing prefabs from demo scene (remnant of showcasing materials)
- Waterline Lens Offset parameter also appearing to clip the underwater fog near the camera

3.1.0

Added:
- Particle Effect controller, makes effects follow the camera underwater up to a specific depth (eg. sunshafts)
- Support for mobile hardware.

Changed:
- Rewritten rendering for Unity's Render Graph
- Optimized technical design, no longer uses full-screen post-processing.
- Redesigned to work exclusively with defined underwater areas (box collider triggers)
- Revised shader for transparency, now only blend the alpha value.

Removed:
- Blur/distortion effects, incurred maintenance overhead as URP's rendering code kept changing. Considered niche as AAA games do not use distortion effects underwater.
- Volume-based settings blending functionality (deemed unused).