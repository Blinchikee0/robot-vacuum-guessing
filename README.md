# Your robot vacuum is guessing

An interactive browser simulator of the one mechanism that lets a robot vacuum clean in straight stripes: **Monte Carlo localization**.

A robot vacuum has no GPS, and its wheels slip. It never knows exactly where it is. Instead it keeps hundreds of guesses about its position, moves all of them the way its wheels say, and keeps the guesses from which the walls look the way its spinning laser sees them.

**Live:** https://blinchikee0.github.io/robot-vacuum-guessing/

**Code:** https://github.com/Blinchikee0/robot-vacuum-guessing

## The idea in two minutes

Open the page. The robot is already cleaning a small flat. Then try the three things in the right panel:

1. **Blind it.** Switch the laser off. The orange cloud of guesses spreads out, the error number climbs, and the clean stripes in the dust stop lining up.
2. **Make the floor slippery.** Raise wheel slip. The wheels count more distance than the robot really covers ("The wheels counted 12.4 m. It really moved 11.1 m.").
3. **Kidnap it.** Drag the robot into another room, or press *Kidnap*. Random guesses appear all over the flat, and within seconds one of them fits the laser scan and the cloud snaps onto the real robot.

After all three, the page states the insight in one paragraph.

## What is computed (nothing is pre-recorded)

| Part | File | What it does |
|---|---|---|
| Apartment | `js/world.js` | Walls and furniture as rectangles and circles. Ray casting, collision tests, and a 2.5 cm distance field. |
| Real robot | `js/robot.js` | Differential drive. Each wheel slips by a slowly wandering random amount (more on the rug and in turns). Encoders count 1000 ticks per turn. Optional gyroscope. A 360-beam lidar with noise and dropouts. |
| Cleaning plan | `js/planner.js` | Back-and-forth lanes per room, joined with A* paths through the doorways. Planned on the robot's map, not on the real room. |
| Driving | `js/controller.js` | Follows the plan using the robot's **believed** position. If the belief is wrong, the real robot goes to the wrong place. |
| **Localization** | `js/filter.js` | The particle filter. **Move**: every guess follows the odometry plus noise. **Weigh**: likelihood-field model, each guess projects the laser endpoints and looks up the distance to the nearest wall in the map. **Resample**: low-variance resampling. Random guesses are injected when the fit gets worse than usual (augmented MCL), which is what lets a kidnapped robot recover. |
| Coverage | `js/coverage.js` | Which part of the real floor was actually cleaned. |
| Photo studio | `js/studio.js` | The "Robot up close" page: its own scene with softbox reflections, a key light with soft shadows and a blurred contact shadow. It draws only when something changes: a quick frame while things move, then one refined frame (4x multisampling and GTAO ambient occlusion) once the picture is still. The robot there uses physical materials (clear-coated plastic, metal, rubber, translucent bin), clipping planes for the cutaway, a lid on a hinge, and extra parts: a circuit board with glowing traces, wires, suspension springs, a fan and a pleated filter. |
| Drawing | `js/scene.js`, `js/robot-model.js`, `js/floor.js` | Two WebGL renderers, each with its own canvas inside its own element, so the 3D scrolls with the page. The dust is erased on the GPU: the real robot stamps circles into a mask texture every frame. The robot camera has its own small sun so its shadow stays sharp. If frames get slow, the page turns off the glass blur and then lowers its pixel density by itself. |
| Glue and UI | `js/main.js`, `js/i18n.js` | Simulation loop, tasks, readouts, step mode, inspector. |

The filter runs five times per simulated second with up to 3000 guesses. One cycle with 1000 guesses and 36 beams takes about 1.5 ms.

## Going deeper

* **Inside** tab: pause and run the cycle one step at a time (Move, Sense, Weigh, Resample). The camera drops to a top view near the robot, and arrow size shows each guess's weight.
* Click any orange arrow on the floor to see where the laser hits would land if the robot were there, and how many land on a wall.
* Drag furniture. The robot's map does not change, so the laser sees things the map does not have. *Update map* fixes it.
* Switch the gyroscope off and put the robot on a thick rug: now even the laser cannot hold it, because the heading drifts faster than the guesses can follow.
* Toggles: laser rays, a wheels-only ghost (pure dead reckoning), the robot's distance map, the planned route, the gyroscope.
* **Robot camera** (top right): a follow camera on the real robot. The lidar retracts when switched off and the wheels drop when you lift it.

## Two pages

* **Simulation**: the flat, the robot at work, the three experiments.
* **Robot up close**: the same robot standing still in a photo studio, shown as realistically as we could. The simulation pauses while this page is open.
  * Four views: **Whole**, **Open** (the lid swings up on its hinge with the laser turret, the dust bin slides out), **Cutaway** (the shell is sliced away so the parts show in place) and **Apart** (every part floats in stacking order).
  * Click a part, on the robot or in the list, and the camera flies to it while it does its job: the laser turret spins and fans out its beams, the wheels turn and count ticks, the circuit board's traces light up while the gyroscope reads the robot's turn, the bumper clicks, the fan spins. A live readout follows the part.
  * **Take the tour** walks through eight steps with an explanation for each.
  * The four parts the robot uses to find its way are marked in orange.

## What this model simplifies

* The map is known from the start. Real robots build it while they localize (SLAM).
* A whole lidar turn is taken at one instant.
* The world is flat, furniture does not rotate.
* Wheel slip is a random process, not tyre physics.

## Run locally

Any static file server works. From the project folder:

```
python -m http.server 5173
```

Then open `http://localhost:5173`. Opening `index.html` directly from disk will not work, because browsers block ES modules on `file://`.

Headless test of the simulation core (no browser needed):

```
node tests/sim.test.js
node tests/kidnap.test.mjs
node tests/slip.test.mjs
```

## Deploy (GitHub Pages)

1. Create a public repository and push this folder to the `main` branch.
2. Repository *Settings* > *Pages* > *Build and deployment* > *Deploy from a branch* > `main` / `(root)`.
3. The site appears at `https://<your-github-name>.github.io/<repo-name>/` after a minute.

No build step, no API keys, no sign-in.

## Design

The interface follows the glass look of iOS 26: floating translucent panels over the live 3D scene, segmented controls whose selection stretches across like liquid, and spring animations (CSS `linear()` easing where the browser supports it).

## Components we did not write, and why

* **Three.js 0.170** (MIT), loaded from jsDelivr: WebGL rendering only. From its examples folder we use `RoundedBoxGeometry`, `RoomEnvironment`, and for the photo studio `EffectComposer`, `RenderPass`, `GTAOPass`, `OutputPass` and the two blur shaders. None of the localization runs in Three.js.
* **Figtree** (SIL Open Font License) from Google Fonts: one solid, rounded sans-serif for all text and numbers.

Everything else, including the simulation, the particle filter, the planner and the robot model, is plain JavaScript in this repository.

## References

* S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005. Chapter 5 (odometry motion model), chapter 6 (likelihood field), chapter 8 (Monte Carlo localization and augmented MCL).
