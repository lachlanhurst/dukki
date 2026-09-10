# mjcf_to_f360

Rebuilds a MuJoCo MJCF robot (for example the microduck model in
`microduck_rl`) inside Autodesk Fusion 360 as a positioned, jointed assembly
of mesh bodies. Nothing is converted to solid CAD geometry. The result is a
reference assembly to remodel parts against.

What you get in Fusion:

- One component per MJCF body, placed at the body's world pose in the zero
  configuration, all directly under the root. Bodies attached to the world
  (the trunk) are grounded.
- Each visual mesh geom imported as a mesh body inside its body's component,
  named after the geom or mesh, coloured from the MJCF material.
- An as-built revolute joint for every hinge (slider for every slide) between
  child and parent components, with MJCF joint ranges applied as limits. A
  construction sketch line named `axis_<joint>` marks each axis.

## Files

| File | Runs where | Purpose |
| --- | --- | --- |
| `mjcf_to_f360.py` | Fusion 360 | The script. Entry point is `run(context)`. |
| `mjcf_to_f360.manifest` | Fusion 360 | Required by Fusion to recognise the folder as a script. |
| `mjcf_model.py` | both | MJCF reader, standard library only. |
| `stl_tools.py` | both | STL read/transform/write, used to bake geom poses into mesh copies. |
| `dump_assembly.py` | any Python 3 | Prints the tree and joints as the script will build them. `--bake` writes positioned STLs and `assembly.json`. |
| `check_against_mujoco.py` | needs `mujoco` | Verifies the reader against MuJoCo's own kinematics. |

## Running in Fusion 360

1. Copy this whole folder to the Fusion machine. Keep the folder name
   `mjcf_to_f360`; Fusion requires the folder, script and manifest to share it.
2. Also copy the MJCF file and its mesh folder. Mesh paths are resolved
   relative to the MJCF file through `meshdir`, so the easiest option is to
   check out `microduck_rl` on that machine.
3. In Fusion create a new empty design and save it.
4. Utilities > Add-Ins > Scripts tab > the green `+` next to "My Scripts",
   choose this folder. It appears as `mjcf_to_f360`.
5. Select it and press Run. A file dialog asks for the MJCF file. Pick
   `src/mjlab_microduck/robot/microduck/robot_walk.xml` or another variant.
6. Wait for the progress dialog. A summary lists components, mesh bodies,
   joints and warnings. The full log and the baked STL copies are written to
   the system temp folder under `mjcf_to_f360/<model name>/` unless
   `BAKE_DIR` is set.

Settings live at the top of `mjcf_to_f360.py`:

- `MJCF_PATH`: hard-code the file to skip the dialog.
- `INCLUDE_GEOM_GROUPS`: `(0, 1, 2)` imports visual meshes only. Add `3` to
  include collision meshes (in this model those duplicate visual parts).
- `SKIP_MESHES`: mesh asset names to leave out, for example `("xl330",)` when
  you would rather place a J288 model yourself.
- `CREATE_JOINTS`, `GROUND_ROOT_BODIES`, `APPLY_COLOURS`: switch features off
  if they cause trouble on your Fusion version.
- `SWITCH_TO_DIRECT_MODELLING`: the API only allows mesh bodies in a
  parametric design inside a base feature, so the script creates one base
  feature per body (named `meshes_<body>`). Set this to `True` to convert the
  empty design to direct modelling instead, which drops the timeline and
  avoids the base features altogether.

## Checking the model before you go to the Fusion machine

```sh
python3 scripts/mjcf_to_f360/dump_assembly.py \
    ../microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml

cd ../microduck_rl && uv run python \
    ../microduck-unitree/scripts/mjcf_to_f360/check_against_mujoco.py \
    src/mjlab_microduck/robot/microduck/robot_walk.xml
```

The first command prints every body with its world position and quaternion,
every joint with its world origin and axis, and every geom that will be
imported or skipped. The second compares those numbers with MuJoCo and
reports the worst error.

## Fallback without the script

`dump_assembly.py --bake OUT_DIR --units mm` writes one STL per geom with the
body-relative pose baked in, plus `assembly.json` listing each body's world
matrix and each joint's world origin and axis. Those files can be inserted
into Fusion manually if the API route misbehaves.

## Units and conventions

- MJCF lengths are metres. The Fusion API works in centimetres. Baked STLs
  are written in millimetres by default and imported as millimetres.
- MuJoCo quaternions are `(w, x, y, z)`; the reader also accepts `euler`,
  `axisangle`, `xyaxes` and `zaxis` and follows the `<compiler>` settings for
  angle units and Euler sequence.
- Components sit flat under the root rather than nested, because as-built
  joints are simplest to create between occurrences in the same context.
  The MJCF parent is recorded in each component's description.

## Known limits

- Only mesh geoms are imported. Primitive geoms (box, capsule, sphere) are
  listed as skipped by `dump_assembly.py`.
- Ball joints, equality constraints and sites are not created. Sites appear
  in `dump_assembly.py` output only through their parent body.
- Mesh `refpos`/`refquat` attributes are ignored.
- The Fusion API calls (mesh import, mesh body appearance, joint limits) are
  wrapped so a failure produces a warning in the log rather than aborting the
  import. Check the warnings list if something looks off.
- The script has not yet been run inside Fusion. The MJCF reading and mesh
  baking are verified against MuJoCo on this side; the Fusion API calls are
  written from the API reference and samples and may need a fix or two on
  first run. The log file records the exact call that failed.
