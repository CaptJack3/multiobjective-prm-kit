# Research examples you can inspect and reuse

Start with the **kitchen**, **workbench (`office`)** and **multilevel shelves**.
Each has an original static Isaac/USD scene, a saved animated scene, its editable
geometry specification and a portable procedural generator. The collection also
includes E4/E5/E7 and six-wall environments.

| Scene | Preview | USD |
|---|---|---|
| Kitchen | [Multiview image](../examples/research_showcase/scenes/kitchen/preview.jpg) | [Static](../examples/research_showcase/scenes/kitchen/scene.usdc) · [Animated](../examples/research_showcase/scenes/kitchen/motion_scene.usdc) |
| Workbench | [Multiview image](../examples/research_showcase/scenes/office/preview.jpg) | [Static](../examples/research_showcase/scenes/office/scene.usdc) · [Animated](../examples/research_showcase/scenes/office/motion_scene.usdc) |
| Shelves | [Multiview image](../examples/research_showcase/scenes/shelves/preview.jpg) | [Static](../examples/research_showcase/scenes/shelves/scene.usdc) · [Animated](../examples/research_showcase/scenes/shelves/motion_scene.usdc) |

After cloning/downloading, open **`examples/research_showcase/index.html`** in your
browser. It links every scene, all 21 preference videos and saved routes, and
one E4 smoothing comparison. No PRM run or simulator installation is needed to
view the gallery. GitHub does not execute the HTML gallery in its ordinary file view.

## Create and modify environments

From the repository root with the `usd` extra installed:

```bash
python examples/research_showcase/generate_household_specs.py --output runs/household_specs
python examples/research_showcase/build_scenes.py --scene kitchen \
  --spec-root runs/household_specs --output runs/rebuilt_kitchen
```

Edit `runs/household_specs/kitchen/scene_spec.json` before the second command to
change objects or dimensions. Open `runs/rebuilt_kitchen/kitchen.usdc` using
Isaac Sim's **File → Open**. The generator uses the standard library; the USD
builder uses NumPy/OpenUSD and does not run PRM or require Isaac to author the file.

## Inspect historical preference routes

```bash
python examples/research_showcase/plot_routes.py \
  --scene kitchen --output runs/kitchen_routes.png
```

This plots saved TCP paths and their recorded costs using the `render` extra.
The [CSV](../examples/research_showcase/metrics.csv) contains all 21 objective
vectors. No new planning is performed.

## Match the hardware and the experiment

The archived USD scenes use the earlier camera/bracket assembly and a bare tool
flange. The later 2026-11-21 preference videos use the Robotiq/PickNik/D435 assembly.
These are documented separately in the gallery. A route from one revision is not
a validated motion for another. Household USD uses world coordinates, while its
planning JSON uses the robot-base frame; the translation is documented.

The household results used task-biased 500K-node lazy PRMs and selected preference
routes, not a complete Pareto frontier. The wall examples reused large historical
PRM results with local repair/smoothing. The smoothing example comes from an
older E4 study. These are kinematic demonstrations, not grasp or controller execution.

The [detailed inventory and commands](../examples/research_showcase/README.md)
state exactly what is bundled, what can be rebuilt and which historical pipelines
are outside the portable distribution.
