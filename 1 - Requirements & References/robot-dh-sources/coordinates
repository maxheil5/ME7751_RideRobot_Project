# Coordinate Frame & Notation Conventions

## Frames
- {0}: base frame
- {i}: link i frame (DH)
- {6}: flange frame
- {T}: tool frame (fixed transform from flange)

## Transform Direction
- Use: ^0T_i (transform from frame i to base) OR consistent alternative.
- Rotation representation:
  - Prefer rotation matrix R and position p for core math.
  - Optional: roll-pitch-yaw for UI display.

## Units
- Length: meters
- Angles: radians internally (degrees only for display)

## Pose Error Metrics (for verification)
- Position error: ||p_ref - p||_2
- Rotation error: angle of R_ref^T R (via acos((trace()-1)/2))
