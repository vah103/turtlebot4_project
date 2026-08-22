"""Hospital snapshot recorder with conservative SLAM map-jump detection."""

import json
from datetime import datetime, timezone

import rclpy

from frontier_exploration.map_integrity import analyze_known_mask_transition
from frontier_exploration.map_snapshot_core import write_json_atomic
from frontier_exploration.map_snapshot_recorder import MapSnapshotRecorder


class HospitalMapSnapshotRecorder(MapSnapshotRecorder):
    """Record Hospital snapshots and mark runs invalid after a large map jump."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('map_jump_detection_enabled', True)
        self.declare_parameter('map_jump_min_previous_known_cells', 50000)
        self.declare_parameter('map_jump_min_changed_cells', 20000)
        self.declare_parameter('map_jump_min_changed_fraction', 0.15)
        self.declare_parameter('map_jump_min_known_lost_cells', 5000)
        self._initialize_integrity_metadata()

    def _integrity_config(self) -> dict:
        return {
            'enabled': bool(
                self.get_parameter('map_jump_detection_enabled').value
            ),
            'min_previous_known_cells': int(
                self.get_parameter('map_jump_min_previous_known_cells').value
            ),
            'min_changed_cells': int(
                self.get_parameter('map_jump_min_changed_cells').value
            ),
            'min_changed_fraction': float(
                self.get_parameter('map_jump_min_changed_fraction').value
            ),
            'min_known_lost_cells': int(
                self.get_parameter('map_jump_min_known_lost_cells').value
            ),
            'method': (
                'known-mask transition; requires large total change and '
                'known-to-unknown regression'
            ),
        }

    def _read_run_metadata(self) -> dict:
        path = self._run_dir / 'run.json'
        try:
            return json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(
                f'Cannot read run metadata for integrity update: {exc}'
            )
            return {}

    def _initialize_integrity_metadata(self) -> None:
        payload = self._read_run_metadata()
        payload['run_invalid'] = bool(payload.get('run_invalid', False))
        payload.setdefault('invalid_reason', None)
        payload.setdefault('first_invalid_sequence', None)
        payload.setdefault('map_jump_events', [])
        payload['map_jump_detection'] = self._integrity_config()
        write_json_atomic(self._run_dir / 'run.json', payload)

    def _mark_run_invalid(
        self,
        *,
        previous_sequence: int,
        current_sequence: int,
        trigger: str,
        map_msg,
        diagnostics: dict,
    ) -> None:
        payload = self._read_run_metadata()
        events = list(payload.get('map_jump_events', []))
        event = {
            'detected_utc': datetime.now(timezone.utc).isoformat(),
            'previous_sequence': previous_sequence,
            'sequence': current_sequence,
            'trigger': trigger,
            'map_stamp': {
                'sec': int(map_msg.header.stamp.sec),
                'nanosec': int(map_msg.header.stamp.nanosec),
            },
            'source_map': {
                'width': int(map_msg.info.width),
                'height': int(map_msg.info.height),
                'resolution': float(map_msg.info.resolution),
                'origin_x': float(map_msg.info.origin.position.x),
                'origin_y': float(map_msg.info.origin.position.y),
            },
            'diagnostics': diagnostics,
        }
        events.append(event)

        payload['run_invalid'] = True
        payload['invalid_reason'] = 'suspected_map_jump'
        payload.setdefault('first_invalid_sequence', current_sequence)
        if payload['first_invalid_sequence'] is None:
            payload['first_invalid_sequence'] = current_sequence
        payload['map_jump_events'] = events
        payload['map_jump_detection'] = self._integrity_config()
        write_json_atomic(self._run_dir / 'run.json', payload)

        self.get_logger().error(
            'MAP JUMP SUSPECTED; run marked invalid: '
            f'{previous_sequence:06d}->{current_sequence:06d}, '
            f'changed={diagnostics["changed_known_mask_cells"]}, '
            f'lost={diagnostics["known_lost_cells"]}, '
            'fraction='
            f'{diagnostics["changed_fraction_of_previous_known"]:.3f}'
        )

    def _capture_pair(
        self,
        map_msg,
        odom_msg,
        sync_delta_sec,
        trigger: str,
        *,
        force: bool = False,
    ) -> bool:
        previous_mask = self._last_saved_known_mask
        previous_sequence = self._sequence - 1

        saved = super()._capture_pair(
            map_msg,
            odom_msg,
            sync_delta_sec,
            trigger,
            force=force,
        )
        if (
            not saved
            or previous_mask is None
            or not bool(self.get_parameter('map_jump_detection_enabled').value)
        ):
            return saved

        current_mask = self._last_saved_known_mask
        if current_mask is None:
            return saved

        diagnostics = analyze_known_mask_transition(
            previous_mask,
            current_mask,
            min_previous_known_cells=int(
                self.get_parameter('map_jump_min_previous_known_cells').value
            ),
            min_changed_cells=int(
                self.get_parameter('map_jump_min_changed_cells').value
            ),
            min_changed_fraction=float(
                self.get_parameter('map_jump_min_changed_fraction').value
            ),
            min_known_lost_cells=int(
                self.get_parameter('map_jump_min_known_lost_cells').value
            ),
        )
        if diagnostics['suspected_map_jump']:
            self._mark_run_invalid(
                previous_sequence=previous_sequence,
                current_sequence=self._sequence - 1,
                trigger=trigger,
                map_msg=map_msg,
                diagnostics=diagnostics,
            )
        return saved


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HospitalMapSnapshotRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
