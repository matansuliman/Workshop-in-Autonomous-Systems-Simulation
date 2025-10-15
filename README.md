# Drone Precision Landing

A MuJoCo + PySide6 desktop app for **precision landing** of a quadrotor on a moving circular pad.  
The drone uses a bottom camera → ArUco detection → simple predictor → PID controller to align and descend.

> **Platform**: Windows • **Python**: 3.11

---

## Quick Start

1. **Create venv (recommended) & install deps**
   ```bash
   pip install -r requirements.txt
   ```

2. **Configure** (edit `config.yaml`)
   - Set the MuJoCo scene path (example below uses the repo root):
     ```yaml
     path_to_xml: "skydio_x2/scene.xml"
     ```
   - (Optional) Tweak plotting and camera:
     ```yaml
     plotter:
       ext: png
     
     camera_streamer:
       resolutions:
         high: [640, 480]
         low:  [320, 240]
       frequency: 30
     ```
     
3. **Run**
   ```bash
   python app.py
   ```

---

## What You’ll See

- A GUI with **status readouts** and a **live preview** (low‑res).  
- The app runs the **simulation** and **camera streamer** on background threads.  
- When centered and stable above the pad, the controller **descends in phases** and touches down; plots are saved on exit (e.g., `Quadrotor-plot.png`).

---

## Repo Map

### Entry Point
- **`app.py`** – Bootstraps the app: initializes `QApplication`, `SimulationRunner`, `CameraStreamer`, and `GUI`. Connects signals and starts background threads.

### Core Simulation
- **`simulation.py`** – Main simulation loop. Handles pause/resume/terminate, state updates, and interactions with physical objects.
- **`environment.py`** – MuJoCo wrapper: loads XML scenes, steps simulation, currently coupled to MuJoCo api.
- **`models.py`** – Physical objects: `Quadrotor`, `Pad` with sensors and loggings.

### Control & Orchestration
- **`controllers.py`** – Non-physical controllers; `QuadrotorController` implements PID control, alignment, and phased descent.
- **`orchestrators.py`** – Scene logic orchestrators (e.g., **Follow**).

### Sensing & Detection
- **`sensors.py`** – Basic MuJoCo sensors and advanced sensor abstractions.
- **`detectors.py`** – ArUco marker detection from camera stream.
- **`predictors.py`** – Simple motion predictor using past positions.
- **`noises.py`** – Synthetic sensor noise for realism.

### Streaming & GUI
- **`streamers.py`** – Offscreen renderer: streams images to GUI and detector pipeline.
- **`gui.py`** – PySide6 GUI: status readouts, live camera preview, control buttons ans sliders.

### Utilities
- **`fps.py`** – Maintains loop frequency.
- **`timer.py`** – Pausable timer utility.
- **`logger.py`** – Logging class, used across simulation and controllers.
- **`helpers.py`** – Miscellaneous global helper functions.

### Configuration
- **`config.py`** – Loads settings from `config.yaml`.
- **`config.yaml`** – YAML configuration: scene paths, camera settings, plotting, frequency, etc.
- **`globals.py`** – Global variables: config, logger, environment reference.

### Output & Plots
- **`plots.py`** – Generates plots (PNG) from simulation logs upon exit.

### Assets & Scenes
- **`/skydio_x2`** – MuJoCo XML files (world and objects), textures, and other assets.

### Documentation
- **`Presentation.pdf`** – Overview of the project.
- **`ClassDiagram.pdf`** – UML class diagrams for reference.
- **`self_evaluation.txt`** – Making this project great.

---

## Troubleshooting

- **OpenGL/GLFW errors**: update GPU drivers; ensure a desktop OpenGL context is available.
