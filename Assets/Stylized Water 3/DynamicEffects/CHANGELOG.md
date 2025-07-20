3.1.0
Updated to support Stylized Water 3 v3.1.0

Added:
- Ramp prefab effect

3.0.5

Changed:
- Rendering is now completely skipped if there are no effects enabled in the scene

3.0.4

Fixed:
- Incorrect shading on slopes in some scenarios when using Dynamic Effects (requires v3.0.4 as well)

3.0.3

Fixed:
- Prefabs potentially showing as changed in a VCS, despite no apparent changes.

Changed:
- Improved shading performance of foam application to materials.

3.0.2

Fixed:
- Shading appearing broken when using DirectX 12 and having the "Normal mipmaps" option enabled

3.0.1

• Updated to reflect changes in Stylized Water 3 v3.0.1

Added:
- Generic height modifier prefab effect

Changed:
- Shoreline Wave Spawner, added the option to hide the spawned objects

Fixed:
- Effect not updating while manipulating an object when Scale By Transform was enabled
- Scale By Transform option not respective negative scale values

3.0.0

What’s new with Stylized Water 3?

• Rewritten rendering code for Render Graph
• Effects are now fully compatible with the SRP Batcher and new GPU Resident Drawer, this minimizes drawcalls
• New shoreline wave spawner component, spawns the shoreline wave prefab along a spline + snaps an audio emitter to the spline, following the camera.
• The render feature is now embedded into the core SW3 render feature
• Effects are now readable through C#

Changed:
- Dynamic Effects component now works based off a "template" material
- Effect displacement can now scale with the Transform
- Normal strength scalable by height