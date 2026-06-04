"""
Scan configuration. TWO demo objects are defined here; pick one with
`run.py --object <name>` (default: db5). Tune a number here, not in the code.

  db5  (default, RECOMMENDED)  LEGO Speed Champions "007 Aston Martin DB5" #76911
       ~17 x 7 x 5 cm matte car. Solid + chunky + ASYMMETRIC (front != back, distinct
       sides) -> best of the candidates for BOTH a clean point cloud AND a watertight
       mesh (no symmetry/thin-part problems). It's light-grey/silver, so the *geometric*
       texture (panel lines, wheels, gaps) carries the stereo match more than colour —
       light it DIFFUSELY to avoid glare on the flat-silver parts.

  teemo  Funko Pop "Teemo with Mushroom" #1138, ~9 cm figure (+ ~2.5 cm buddy). Glossy
         vinyl, so expect some depth holes, but solid + asymmetric + colourful. Kept as
         the original example.

Both feed the identical pipeline: segment -> object-centred merge (multiway ICP) -> mesh.
"""

DB5 = dict(
    name="007 Aston Martin DB5 (LEGO 76911)",
    key="db5",
    shape="car",                 # selects the synthetic stand-in in _synth_test.py
    radius=0.35,                 # camera-to-car distance (m); car is ~17 cm so 0.35 frames it well
    shots=36,                    # views around the car (10 deg steps). MORE shots = SMALLER step
                                 # = more overlap between consecutive views = more of them
                                 # feature-register = a fuller car (the smooth silver sides need
                                 # the overlap). Was 24 (15 deg); raise to 48 for even finer.
    zmin=0.22, zmax=0.50,        # depth gate: brackets a 17-cm car at ~35 cm (end-on it's deeper)
    crop=0.13,                   # half-size (m) of the keep-box around the car centroid
    voxel=0.0015,                # 1.5 mm — fine enough for panel/wheel detail
    poisson_depth=10, mesh=True, # a car body meshes cleanly -> Poisson on
    # synthetic stand-in dims (m): ~17 x 7 x 5 cm
    car_length=0.165, car_width=0.07, car_height=0.05,
)

TEEMO = dict(
    name="Teemo with Mushroom (Funko 1138)",
    key="teemo",
    shape="figure",
    radius=0.40, shots=24,
    zmin=0.30, zmax=0.52, crop=0.12,
    voxel=0.002, poisson_depth=9, mesh=True,
    fig_height=0.09, fig_width=0.07, fig_depth=0.06, buddy=0.025,
)

OBJECTS = {"db5": DB5, "teemo": TEEMO}
DEFAULT = DB5


def select(name):
    """Set the active object by key (see OBJECTS). Returns the chosen config dict."""
    global DEFAULT
    if name not in OBJECTS:
        raise SystemExit(f"unknown object '{name}'. choose from: {', '.join(OBJECTS)}")
    DEFAULT = OBJECTS[name]
    return DEFAULT
