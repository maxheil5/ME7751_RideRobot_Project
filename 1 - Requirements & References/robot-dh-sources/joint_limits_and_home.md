# Joint Limits & Home Pose

## Joint Limits
Provide limits in radians (and degrees in parentheses if useful).

| Joint | min (rad) | max (rad) | min (deg) | max (deg) |
|------:|----------:|----------:|----------:|----------:|
| q1    | -3.22886  | 3.22886   | -185      | 185       |
| q2    | -2.26893  | 0.34907   | -130      | 20        |
| q3    | -1.74533  | 2.51327   | -100      | 144       |
| q4    | -6.10865  | 6.10865   | -350      | 350       |
| q5    | -2.09440  | 2.09440   | -120      | 120       |
| q6    | -6.10865  | 6.10865   | -350      | 350       |

## Home Pose Definition
- q_home = [0, 0, 0, 0, 0, 0]
- Note: We choose the zero configuration as home because the catalogue doesn't specify a non-zero zero-pose and a zero pose is convenient for visualisation/analysis.

## Notes
- If the DH table uses offsets, document them explicitly.
- If the physical demo uses different limits, document that separately in Section 6/7.
