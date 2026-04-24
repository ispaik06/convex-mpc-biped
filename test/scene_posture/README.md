# Scene Posture Utility

This directory contains a small MuJoCo utility for checking the posture encoded in an arbitrary scene XML.

The executable loads a scene, applies the named pose if the XML defines one, runs `mj_forward`, and reports:

- body origin `z`
- each selected site `z`
- each body-to-site `z` offset
- the body `z` value that would place all selected sites on the ground plane at `z = 0`

By default the utility looks for:

- body: `torso`
- key: `nominal_stance`
- sites: `left_foot_contact_site`, `right_foot_contact_site`

For other scenes, override those names with `--body` and `--site`.

## Build

```bash
cmake --build build -j 8 --target scene_posture_clearance
```

## Run

The scene XML must be passed explicitly. Relative paths are resolved against the current working directory first and then against the repository root.

```bash
./build/test/scene_posture/scene_posture_clearance --xml models/mit_humanoid/scene.xml --key nominal_stance
```

Example for another scene once it defines the relevant body/site names and a `nominal_stance` key:

```bash
./build/test/scene_posture/scene_posture_clearance --xml models/unitree_robots/g1/scene_23dof.xml --body torso_link --site left_foot_contact_site --site right_foot_contact_site --key nominal_stance
```

If the scene has no keyframes yet, the utility falls back to `qpos0`.

## Output meaning

- `body_origin_z` is the world `z` of the selected body origin.
- `site[i].z` is the world `z` of the selected site.
- `site[i].body_minus_site_z` is the body height needed to bring that site to ground while keeping the same relative pose.
- `body_z_for_all_sites_on_ground` is the maximum of those values across all selected sites.

## Notes

- This target is for manual inspection only and is not added to CTest.
- The utility is meant to support posture tuning, not runtime control.
