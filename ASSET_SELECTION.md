# PyBullet facility asset selection

## Findings

- **iGibson:** static indoor scenes load mesh geometry into PyBullet, while iGibson's high quality materials are rendered by its separate MeshRenderer. The full model and scene datasets are separately downloaded and are not part of this repository. That renderer and dataset path do not meet the all-PyBullet requirement, so no iGibson runtime or scene asset was imported.
- **Bullet3:** PyBullet supports Wavefront OBJ visual meshes and reads adjacent MTL diffuse materials/textures. Its documentation distinguishes visual meshes from collision geometry and warns that concave triangle meshes are for static bodies. This is the compatible route for future authored architectural meshes. The simulator here remains on PyBullet GUI/OpenGL and TinyRenderer for DIRECT camera captures.
- **pybullet_robots / BotLab:** useful reference for PyBullet robot and scene setup, but this repository's public top-level material inspected here does not establish redistribution terms for a BotLab facility mesh. No BotLab geometry was copied.
- **pybullet-object-models:** useful manipulation reference with separate textured visual meshes and simplified collision meshes. Its repository declares LGPL-2.1; its bundled YCB content also points to upstream dataset/toolbox sources, so no YCB or third-party task object was copied. The six task objects remain the project's own original primitives.

## Integrated materials and assets

The four small PNG maps in `assets/textures/` are deterministic, project-authored diffuse textures (graphite concrete, warm wall panel, brushed aluminium, acoustic ceiling). They add surface variation without a new dependency or external license. Existing room detailing uses separate visual-only trims, seams, ribs, lights, and ceiling members. The room still has four collidable walls, one collidable flat floor, exactly six task objects, and one marked zone; the corner crates, charging pads, and posts have been removed from the fetch scene.

This pass does **not** claim photorealism or claim to have reused a downloaded architectural mesh. Blender is not installed in the current environment, and the inspected repositories did not provide a clearly redistributable industrial facility mesh that could be integrated without their separate datasets or additional licensing review. The current materials use diffuse textures and supported specular tint; they do not depend on unsupported PBR metallic/roughness features.
