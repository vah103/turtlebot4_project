# MX072 Run04 G2-A: five qualitative viewpoint picks (causal)

Five points selected manually from Room Node panel (third panel) at LOCAL entry decisions **D02 D08 D14 D18 D21** of optimized New Room Run04.

## Points, raw/native P1 coordinates

| Decision | Room | Padded native (row,col) | Cropped P1 (x,y) m | A* on current observed FREE grid |
|---|---:|---|---|---:|
|D02|1|(662,610)|(11.0,16.2)|1.54 m|
|D08|2|(528,678)|(17.8,2.8)|2.01 m|
|D14|3|(530,609)|(10.9,3.0)|2.98 m|
|D18|4|(536,525)|(2.5,3.6)|2.32 m|
|D21|5|(683,535)|(3.5,18.3)|3.05 m|

## Evidence contract
- Uses exact archived third-panel Room Node images embedded in MX072_NEWROOM_RUN04_vs_RUN05_REAL_25_DECISIONS.html; only five named sections opened.
- Other current inputs: room_r41/decisions decision JSON; inference input and semantic NPZ only for the five selected decisions.
- No GT, future ticks, future decisions, postdecision odometry or terminal outcomes used in selection.
- All points are observed=FREE, semantic=ROOM, mean<0.5 and reachable by 8-neighbor A* on observed FREE cells with diagonal corner cutting forbidden. Robot footprint/safety not guaranteed.
- Qualitative human choice, not automatic or proven-optimal G2-A, and not empirical navigation result.
- Maps use 0.1m P1 cropping and 500-cell native padding; not ROS world coordinates.

Open the HTML for high-resolution images and annotations; contact PNG shows all five.
