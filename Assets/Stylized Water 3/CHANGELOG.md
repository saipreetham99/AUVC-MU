3.2.1 (July 7th 2025)

Added:
- Align Transform To Water component, "Follow Target" transform field. Has the transform follow another on the XZ plane
- Set Custom Water Time component, added an option to use Coordinated Universal Time
- Render feature settings, added option to disable Height pre-pass for the scene view when playing.

Changed:
- Align Transform To Water, 'smoothing' is now a scalable float value, rather than a toggle

Fixed:
- GPU height queries not always firing a callback if the water didn't happen to render in the very first frame

3.2.0 (June 12th 2025)

Added:
- Support for Rendering Layers (aka Light Layers)
- Surface Foam clipping parameter, for dynamic effects foam
- Light reflections now have a "Sharp" option
- Water Decal shader, option to enable vertex color alpha fading (eg. particle effects)
- Nintendo Style + Arcade Ocean materials

Changed:
- The "Receive Dynamic Effects Foam" toggle is now a float value. Allows to scale up the foam per-material.
- Material UI, separated reflections into Light and Environment sections

Fixed:
- GPU Height Queries returning incorrect Wave animation heights if the Scene-view tab was open during Play mode
- Distance Normals flowing backwards for rivers when Advanced Shading was used

3.1.1 (June 3rd 2025)

Fixed:
- Underwater Rendering not rendering correctly when Deferred rendering was in use
- Demo scene loading in some scenes twice in Unity 6.1 when entering play mode

Changed:
- Render feature SSR settings, the "Reflect Skybox" parameter was renamed to "Reflect Everything" for clarity

3.1.0 (May 30th 2025)
This update includes some core changes, to add support for the Underwater Rendering extension, but has no functional impact on existing projects.

Added:
- Demo scene, night-time lighting scenario
- Global Wave Origin Offset component, add a Transform's position to any wave layers set to the "Radial" mode
- Screen-Space Reflections, added option to also accept skybox reflections
- Option on render feature to disable GPU Height Queries in edit-mode (avoids the scene-view always redrawing)

Changed:
- Planar Reflections Renderer, reflection will now flip if the camera goes below the plane (support for underwater rendering).

Fixed:
- Reflection probes not affecting the water surface in Unity 6.1 (requires 6000.1.3f+)
- Potential situations where the Ocean surface may be incorrectly culled.

Removed:
- Planar Reflections Renderer, "Render Range" parameter. The functionality behind this was deprecated in Unity 6.0.

3.0.5 (February 17th 2025)

Fixed:
- Camera projection being overriden in minimal rendering setups when using Dynamic Effects

Added:
- Planar Reflections Renderer, option to disable rendering in the scene-view
- Set Water Position Offset component, added "negate" option

Removed:
- Curved World 2020 support (can still be manually incorporated)

3.0.4 (January 15th 2025)

Added:
- Fog integration for Buto (v7+)
- Wave profile procedural editor, "base direction" parameter

Changed:
- Optimization to height readback GPU processing
- Improved error handling when using unsupported features on WebGL

Fixed:
- Fixed incorrect shading on slopes in some scenarios when using Dynamic Effects (requires v3.0.4 as well)

3.0.3 (December 2nd 2024)

Added:
- Installation section in asset window, provides setup warnings/errors with quick-fix buttons
- Lowpoly-style normal map
- Puddle prefabs
- Option to specify the source for the underwater fog: Depth Texture or Vertex Color
- Menu item: Window/Stylized Water 3/Create default reflection probe
- Options to toggle Dynamic Effects Height/Foam/Normals per material
- Set Custom Time component: added option for dynamic speed
- Distance Foam feature, blends in a 2nd layer of Surface Foam within a configurable range

Changed:
- "Vertex Color Depth" option has been replaced by "Vertex Color Transparency" (value automatically upgrades)
- Having Directional Caustics or Screen-Space Reflections enabled no longer forces the Depth texture to render (though still requires it)
- Water Decals now revert to their original height if not water surface was detected below them

Fixed:
- Occlusion Culling causing incorrect Planar Reflections
- Possible null-ref error stemming from render feature
- Resolved a GC-allocation
- Memory leak in destroyed Align To Water components if the CPU-method was used.
- Flat shading not having any effect if Dynamic Effects is disabled
- Water Grid, not creating any geometry if the Vertex Distance value was larger than an individual tile
- Foam Bubbles: dynamic effects foam not contributing
- Ocean prefab potentially showing as changed in a VCS, despite no apparent changes.
- Material UI: min/max slider not allowing keyboard input

3.0.2 (November 12th 2024)

Changed:
- Material UI will now show a notification if extensions or integration aren't active installed.
- Wave tint color is now applied after color absorption

Fixed:
- Enviro 3 fog not taking effect (now requires v3.2.0+)
- Shader error when using the Gamma color space
- Workaround for editor crash when using DirectX 12 and first adding the render feature (IN-88755)

3.0.1 (November 4th 2024)
This version also requires updating the Dynamic Effects extension to v3.0.1

Added:
- Intersection Foam, parameter to control the ripple speed separately

Changed:
- Render target inspector is now functional again

Fixed:
- Dynamic Effects extension not installed automatically
- Point/spot lights no longer affecting the water after a certain distance
- Incorrect configuration on Waterfall particle prefabs

3.0.0 (October 31st 2024)

What's new?
• Rewritten rendering code for Render Graph support
• Revamped wave animations, allowing for various types of waves
• Height Pre-pass, allows other shaders to read out the water surface height.
• GPU-based height query system, making rivers and Dynamic Effects readable
• Water decals, snaps textures onto the water (oil spills, weeds, targeting reticules)
• Improved wave crest foam shading (min/max range + bubbles)
• Ocean mesh component, 8x8km mesh with gradual vertex density
• Improved support for RigidBodies for the Align To Water component
• Waterfall prefabs (3 sizes)

Added:
- Option on shader to disable point/spot light: caustics & translucency
- Waterfall prefabs (mesh, material + particles)
- Support for the Waves feature on rivers
- Align Transform To Water component now better handles RigidBodies

Changed:
- Directional Caustics is now a per-material option
- Screen-space Reflections is now a per-material option
- Sharp and Smooth intersection foam styles are now merged into one feature
- "Align Transform To Waves" is now called "Align Transform To Water"

Removed:
- Integration for Dynamic Water Physics 2 (now deferred to author)
- Non-exponential Vertical Depth (deemed unused) option