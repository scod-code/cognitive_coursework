# Dataset Plan — Person B Custom YOLO Model
## cognitive_coursework | COMP40761

---

## Target Classes

These classes are derived directly from the coursework feature dependency chain.
Each class supports one or more assessed MoSCoW features.

| Class Name       | Coursework Feature It Supports          | MoSCoW Level  |
|---|---|---|
| `stop_sign`      | Traffic sign recognition + response (#6) | Should        |
| `fast_sign`      | Traffic sign recognition + response (#6) | Should        |
| `slow_sign`      | Traffic sign recognition + response (#6) | Should        |
| `goal_marker`    | Goal position detection (#5)             | Should        |
| `landmark_A`     | Landmark database / memory (#8)          | Would         |

> Note: `landmark_A` is a placeholder name. Replace with the actual poster
> type agreed in the maze environment. Confirm with the lab demonstrator
> what physical markers are available.

---

## Positive Examples — What Each Class Looks Like

| Class         | What counts as a positive                            |
|---|---|
| `stop_sign`   | The stop poster clearly visible, any angle, any distance > 0.3m |
| `fast_sign`   | The fast poster clearly visible                      |
| `slow_sign`   | The slow poster clearly visible                      |
| `goal_marker` | The goal indicator visible, partial occlusion allowed |
| `landmark_A`  | Any identifiable landmark poster — tight box required |

---

## Negative Examples — What Must NOT Be Detected

These are the cases the model must learn to reject.
Red walls are the primary false-positive risk identified in Lab 5.

- Red maze wall with no sign
- Partial sign out of frame (only the edge visible, < 20% of sign in view)
- Signs at extreme blur (robot moving fast)
- Overexposed or underexposed frames
- Robot arm or fixture partially blocking sign

Include at least 20–30 negative images in the dataset.

---

## Ignored / Out-of-Scope Classes

These classes appeared in Lab 5 examples but are NOT coursework requirements.
Do not annotate them unless a specific coursework feature depends on them.

- `orange`, `tree`, `vehicle` — Lab 5 example classes only
- General indoor objects (chairs, tables, people)

---

## Dataset Size Target

| Class         | Minimum images | Notes                              |
|---|---|---|
| `stop_sign`   | 30             | Multiple angles, distances         |
| `fast_sign`   | 30             | Same                               |
| `slow_sign`   | 30             | Same                               |
| `goal_marker` | 30             | Include partial views              |
| `landmark_A`  | 20             | Can expand if time allows          |
| Negatives     | 30             | Red walls, blank walls, clutter    |
| **Total**     | **~170**       | Minimum viable training set        |

---

## Annotation Rules

- Use **Roboflow** free tier for annotation and export
- Draw **tight bounding boxes** — box should touch the sign edges, not include surrounding wall
- If a sign is partially visible (> 50% in frame), annotate it
- If a sign is < 50% in frame, skip it or add to negatives
- Export format: **YOLOv8** (not COCO, not Pascal VOC)

---

## Training Target Metrics

From coursework description (file: Final Coursework project description):
- mAP@0.5 > **0.60**
- Precision > **0.70**
- Recall > **0.60**

Export the best checkpoint as `models/best.pt` after training.

---

## Status

- [ ] Classes frozen and agreed
- [ ] Images collected (physical maze / similar environment)
- [ ] Annotated in Roboflow
- [ ] Exported in YOLOv8 format
- [ ] Training complete
- [ ] Metrics verified
- [ ] `best.pt` deployed in ROS2 node
