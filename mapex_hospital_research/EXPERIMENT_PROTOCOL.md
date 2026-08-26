# Experiment Protocol

Mọi phương pháp so sánh phải dùng cùng protocol, trừ khi thay đổi đó chính là biến thí nghiệm và được ghi rõ.

## Environment

- World: `Hospital`
- Simulator: TODO
- Robot: TurtleBot4
- Spawn x: TODO
- Spawn y: TODO
- Spawn yaw: TODO

## Mapping / SLAM

- SLAM package/config: TODO
- Map resolution: TODO
- Map frame: TODO
- Fixed canvas used for logging: TODO

## Navigation

- Nav2 config: TODO
- Goal timeout: TODO
- Recovery behavior: TODO
- Goal acceptance radius: TODO

## Sensor

- LiDAR topic: TODO
- Max range: TODO
- Sensor update rate: TODO

## Exploration

- Frontier detector: TODO
- Stopping condition: TODO
- No-frontier timeout: TODO
- Minimum frontier size: TODO
- Random seed policy: TODO

## Repetition

- Nearest target runs: 10
- MapEx target runs: 10
- Minimum acceptable runs before preliminary analysis: 5 per method

## Metrics required per run

- coverage vs time
- coverage vs distance
- total distance
- total exploration time
- number of frontier goals
- successful goals
- failed goals
- success rate
- termination reason

MapEx additionally logs all decision-level data defined in `docs/DATA_SCHEMA.md`.

## Fair-comparison rule

Không được thay spawn, Nav2, SLAM, sensor, timeout hoặc stopping condition giữa Nearest và MapEx mà không ghi rõ lý do và chạy lại baseline tương ứng.
