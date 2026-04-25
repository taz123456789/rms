# Letter Organizer Robot — Webots Simulation

A fully self-contained Webots R2023b simulation of a **5-DOF robotic arm** that autonomously sorts letters (A/B/C/D) into a pigeonhole shelf using computer vision.

## Quick Start

```bash
# 1. Install Python dependencies
pip install -r requirements.txt

# 2. (Optional) Regenerate textures
python3 generate_textures.py

# 3. Open in Webots
#    File → Open World → worlds/letter_organizer.wbt
#    Press the Play button
```

## Project Structure

```
webots-letter-organizer/
  worlds/
    letter_organizer.wbt          # Webots world file (main entry)
  controllers/
    letter_sorter/                 # Robot controller (Python)
      letter_sorter.py             # Main FSM controller
      forward_kinematics.py        # 5-DOF FK (direct transforms)
      inverse_kinematics.py        # DLS IK with multi-start search
      letter_recognizer.py         # OpenCV template matching
      config/
        arm_params.yaml            # Arm kinematic parameters
        pigeonhole_map.yaml        # Target positions
        templates/                 # Recognition templates (A/B/C/D)
  protos/                          # Local PROTO files (offline)
    backgrounds/
    floors/
    textures/                      # Letter textures + slot labels
  generate_textures.py             # Script to regenerate all textures
  requirements.txt                 # Python dependencies
```

## Architecture

### Arm Kinematics (5-DOF)
- **Joint chain**: shoulder_yaw (Z) → shoulder_pitch (Y) → elbow_pitch (Y) → wrist_pitch (Y) → wrist_roll (Z)
- **Link lengths**: base=0.065m, upper_arm=0.2m, forearm=0.17m, wrist=0.02m, gripper=0.06m
- **Total reach**: ~0.45m horizontal, ~0.515m vertical
- **FK**: Direct homogeneous transforms matching Webots geometry exactly
- **IK**: Damped Least Squares (position-only), multi-start search, <1mm accuracy

### FSM Controller (23 states)
```
INIT → HOME → CAMERA_POSE → CAPTURE → RECOGNIZE
  → APPROACH_PICKUP → LOWER_PICKUP → GRASP → LIFT
  → APPROACH_SLOT → PLACE → RELEASE → RETREAT → HOME
  → (next letter or DONE)
```

### Letter Recognition
- OpenCV multi-scale template matching
- Templates: 96x96 red-on-white letter stamps (A/B/C/D)
- Confidence threshold: 0.5 (fallback to expected order)

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| `supervisor FALSE` | No supervisor API needed; standard Robot controller |
| Local PROTO files | Fully offline operation, no network dependency |
| Direct transforms (not DH) | Exactly matches Webots .wbt joint/endPoint structure |
| Position-only IK (3-DOF) | 5-DOF arm cannot control full 6-DOF pose |
| Multi-start IK | Ensures convergence for all reachable targets |
| Template matching (not OCR) | No Tesseract dependency; works with generated textures |


## Requirements

- **Webots R2023b** (or compatible)
- **Python 3.8+**
- numpy, opencv-python, Pillow, PyYAML
