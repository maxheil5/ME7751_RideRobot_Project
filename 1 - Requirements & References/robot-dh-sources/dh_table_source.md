# DH Table Source

## DH Convention
- [ ] Standard DH
- [x] Modified DH
Chosen convention: **Modified DH**
Notes: Convention must match the transform order used in code and derivation.

## Source
- Citation: robotkinematicscatalogue/inversekinematics/__6DOF/__industrialRobots/KUKA_KR500_R2830.py
- Link / PDF location: https://github.com/SaltworkerMLU/RobotKinematicsCatalogue/blob/main/robotkinematicscatalogue/inversekinematics/__6DOF/__industrialRobots/KUKA_KR500_R2830.py
- Page number(s): N/A (Python source file)
- Units: alpha and thetaoffset are in radians; a and d are in millimetres.
- Any caveats (e.g., sign conventions, base/tool offsets, joint zero definitions): joint 4 has a = -55 mm; joint 6 has an additional thetaoffset = +pi.

## DH Parameters (paste table here)
| α [rad] | a [mm] | d [mm] | θoffset [rad] |
|---------|--------|--------|---------------|
| 0       | 0      | 1045   | 0             |
| -pi/2   | 500    | 0      | 0             |
| 0       | 1300   | 0      | -pi/2         |
| -pi/2   | -55    | 1025   | 0             |
| +pi/2   | 0      | 0      | 0             |
| -pi/2   | 0      | 290    | +pi           |
