# f360_to_mjcf

Puts the Dukki geometry from the Fusion 360 model (`claude_test` in the Microduck project) into the
MuJoCo models in `microduck_rl` (`src/mjlab_microduck/robot/microduck/robot_*.xml`). It is the
reverse of `mjcf_to_f360`, and it keeps everything the RL tasks depend on: body tree, joints,
actuators, sensors, sites, body names and the two `*_foot_collision` geom names.

What it changes in each of the seven robot variants:

- Visual meshes: one low-resolution STL per Fusion part (at most 2,500 faces, metres), placed
  in the MJCF body it belongs to. Two exceptions:
  - The soles keep the full Fusion export. MuJoCo collides against a mesh's convex hull, and
    decimation left the flat undersides bumpy, which cut the area the robot stands on.
  - The J288 is built from primitives (`j288_mesh` in `build_mjcf.py`): a box for the housing
    and a cylinder for the driven horn on the flat end. The non-driven horn sits in the inset end,
    inside the box's envelope, and is left out, as are the mounting holes. Decimation had
    mangled it.
- Collision geoms: each variant keeps its own choice of colliding parts. An old Onshape part's
  collision role passes to the Fusion part that replaced it (`OLD2NEW` in `build_mjcf.py`), so
  meshes stay meshes. The one exception is the battery, which collides as a box where the old
  battery collided. `robot_allcollisions` gives every part a collision geom.
- Inertials: every body's inertial is the sum of its parts, from Fusion's physical properties
  (`fusion_phys.py`). The printed parts take theirs from their Fusion materials: PA12 SLS (EOS
  PA 2200, 990 kg/m^3, for JLC3DP's 3301PA, whose density is not published) and TPU for the
  soles (1,200 kg/m^3, provisional). `masses.json` overrides the rest by component name: the
  J288s, bearings, battery and electronics, weighed, with the inertia scaled from the CAD shape
  and an optional centre of mass. It also excludes parts not on the robot (the soft jaw parts):
  no mass and no geometry. The roller-skate feet have no Fusion parts and keep their Onshape
  inertials.
- `imu` site: moved to the LSM6DSV16X's position in the CAD. It keeps the trunk's axes, because
  the host rotates chip data into the trunk frame and `infer_policy.py` and the
  `raw_accelerometer` observation read the site that way. A new `imu_chip` site carries the
  chip's own axes (x left, y down, z rear). Its quaternion is the host's mount rotation.

How it places the parts. `claude_test` came from the STEP file, not from the MJCF, so it has
its own pose: the trunk is offset by (-26, 0, +3.47) mm and the legs are bent. `fit_links.py`
finds, for each link, the rigid transform that takes it to the MJCF zero pose:

- the trunk is a translation fitted to the three trunk-mounted servo axes;
- each child is a rotation about its joint's axis that lines up the next joint's axis;
- the head has no next joint and keeps claude_test's world z up;
- each foot is turned about its ankle axis until its sole's underside is level, heel to toe,
  with the Onshape sole's underside, which sits flat at the `STAND` keyframe. claude_test's feet
  are about 1.5 degrees toe-down from that, so its world z is no guide. The new sole's underside
  is a very shallow curve made of facets rather than one plane, so the fit levels a line through
  its lowest points (`profile_slope` in `fit_links.py`).

The Fusion servo axes land within 0.09 mm and 0.005 degrees of the MJCF joint axes, so the
kinematics did not change. If that residual grows, the bodies themselves have moved and need
new positions in the MJCF, which this tool does not do.

## Running it

Needs Python with `mujoco`, `numpy`, `scipy`, `trimesh` and `fast-simplification`. The
`microduck_rl` environment does not install on an Intel Mac, so a small virtual environment is
enough.

1. In Fusion, with `claude_test` active, run `fusion_export.py` (through the Fusion MCP server or
   as a script). It writes `work/fusion_data.json` and `work/raw/*.stl`. Check `OUT_DIR` and
   `NAMES_JSON` at its top. Any part that is new to the model must be added to `names.json`
   first, and to its link's rigid group in Fusion. Then run `fusion_phys.py` the same way: it
   writes `work/fusion_phys.json`. A new bought part needs a weighed entry in `masses.json`;
   `null` stops the build until it has one (`F360_MJCF_PENDING_OK=1` keeps the Fusion mass).
2. Restore the Onshape-era robot folder: `build_mjcf.py` rewrites from that version, not from its
   own output, so run `git checkout 53b8971 -- src/mjlab_microduck/robot/microduck` in
   `microduck_rl` first.
3. `python fit_links.py`, then `python build_mjcf.py`. Set `MICRODUCK_RL_ROBOT` to point at
   another copy of the robot folder, and `F360_MJCF_WORK` to use another work folder.
4. Delete the STLs and `.part` files nothing references any more (the old Onshape parts, apart
   from the roller-skate parts).
5. Copy the Onshape-era folder to `work/old_model` (with its assets) and run `python validate.py`.
   It compares every scene: mass, sole height at the `STAND` keyframe, and self-contacts.
6. On the `dukki-j288` branch, regenerate `robot_allcollisions_backlash.xml`, which only that
   branch has: copy `robot_allcollisions.xml` over it and run `python add_backlash.py
   robot_allcollisions_backlash.xml` (default arguments).
