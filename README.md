# sharework_python

Lightweight Python utilities and assets for working with the **ShareWork** robot/cell description (URDF + meshes) **outside of ROS** or **alongside it**.

This repository is intentionally minimal. It provides:
- A URDF model of the ShareWork setup  
- The corresponding mesh assets  
- A small Python loader to make the model easy to consume in kinematics and dynamics workflows

---

## Installation

Install directly from GitHub via `pip`:

```bash
pip install git+https://github.com/JRL-CARI-CNR-UNIBS/sharework_python
```

---

## Repository contents

- **`sharework.urdf`**  
  URDF model describing the ShareWork setup.

- **`assets/`**  
  Meshes and additional resources referenced by the URDF.

- **`load_sharework.py`**  
  Loader utilities for resolving URDF and asset paths and constructing a usable robot model from Python.

- **`__init__.py`**  
  Package entry point.

---

## Usage

### Import the loader

```python
from sharework import loadSharework
```

### Load the model

Define the robot joint list and create the model wrapper:

```python
# --- UR10e joint names ---
UR10E_JOINTS = [
    "ur10e_shoulder_pan_joint",
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint",
]

prefix = "ur10e_"

model_wrapper = loadSharework(UR10E_JOINTS)
model = model_wrapper.model
data = model.createData()
```

### Access a frame

For example, retrieve the tool frame:

```python
tool_frame_id = model.getFrameId(prefix + "tool0")
```

---

## Common Pinocchio operations

Typical kinematics and dynamics computations using Pinocchio:

```python
import pinocchio as pin

pin.framesForwardKinematics(model, data, q)
pin.computeForwardKinematicsDerivatives(model, data, q, dq, ddq)

# Homogeneous transformation of the tool frame
Tbt = data.oMf[tool_frame_id]

# Frame velocity (LOCAL_WORLD_ALIGNED)
twist = pin.getFrameVelocity(
    model,
    data,
    tool_frame_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
)

vel_linear = twist.linear
vel_angular = twist.angular

# Frame Jacobian
J = pin.computeFrameJacobian(
    model,
    data,
    q,
    tool_frame_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
)
```

---

## Requirements

This package does not enforce a specific robotics stack. It is typically used with one of the following Python libraries:

- **Pinocchio** — URDF parsing, kinematics, and dynamics  
- **urdfpy** — URDF parsing and basic geometry utilities  

Install the backend appropriate for your workflow.

---

## Notes

- Mesh paths are handled by the loader to ensure correct resolution of the `assets/` directory.
- The package is designed to remain lightweight and focused on model loading, rather than full simulation or control.
