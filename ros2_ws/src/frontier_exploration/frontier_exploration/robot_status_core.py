"""ROS-independent health classification helpers for the robot status monitor."""

from __future__ import annotations

from dataclasses import dataclass


STATUS_OK = 'OK'
STATUS_WARNING = 'WARNING'
STATUS_NOT_OK = 'NOT_OK'


@dataclass(frozen=True)
class HealthThresholds:
    odom_warning_sec: float = 2.0
    odom_critical_sec: float = 5.0
    scan_warning_sec: float = 2.0
    scan_critical_sec: float = 5.0
    map_warning_sec: float = 8.0
    map_critical_sec: float = 20.0
    tf_critical_sec: float = 5.0
    navigation_stall_warning_sec: float = 20.0
    consecutive_nav_failures_warning: int = 3


def topic_rate_hz(
    receipt_times: list[float],
    now: float,
    window_sec: float,
) -> float | None:
    """Return the observed receive rate over a recent monotonic-time window."""
    window = max(0.1, float(window_sec))
    recent = [stamp for stamp in receipt_times if now - stamp <= window]
    if len(recent) < 2:
        return None
    elapsed = recent[-1] - recent[0]
    if elapsed <= 0.0:
        return None
    return float(len(recent) - 1) / elapsed


def _classify_age(
    name: str,
    age_sec: float | None,
    warning_sec: float,
    critical_sec: float,
    *,
    startup_complete: bool,
) -> tuple[list[str], list[str]]:
    critical: list[str] = []
    warnings: list[str] = []
    if age_sec is None:
        if startup_complete:
            critical.append(f'Không nhận được dữ liệu {name}.')
        else:
            warnings.append(f'Đang chờ dữ liệu {name}.')
        return critical, warnings

    if age_sec >= max(0.0, critical_sec):
        critical.append(f'{name} đã mất dữ liệu {age_sec:.1f} s.')
    elif age_sec >= max(0.0, warning_sec):
        warnings.append(f'{name} chậm/stale {age_sec:.1f} s.')
    return critical, warnings


def classify_health(
    *,
    startup_complete: bool,
    run_invalid: bool,
    run_invalid_reason: str | None,
    odom_age_sec: float | None,
    scan_age_sec: float | None,
    map_age_sec: float | None,
    tf_missing_durations_sec: dict[str, float],
    nav_active: bool,
    stationary_duration_sec: float | None,
    consecutive_nav_failures: int,
    exploration_complete: bool,
    thresholds: HealthThresholds,
) -> dict:
    """Classify the monitored system into OK, WARNING, or NOT_OK."""
    critical: list[str] = []
    warnings: list[str] = []

    if run_invalid:
        detail = run_invalid_reason or 'unknown'
        critical.append(
            'Dataset/run đã được đánh dấu INVALID '
            f'(reason={detail}).'
        )

    streams = (
        (
            '/odom',
            odom_age_sec,
            thresholds.odom_warning_sec,
            thresholds.odom_critical_sec,
        ),
        (
            '/scan',
            scan_age_sec,
            thresholds.scan_warning_sec,
            thresholds.scan_critical_sec,
        ),
        (
            '/map',
            map_age_sec,
            thresholds.map_warning_sec,
            thresholds.map_critical_sec,
        ),
    )
    for name, age, warning, critical_age in streams:
        age_critical, age_warnings = _classify_age(
            name,
            age,
            warning,
            critical_age,
            startup_complete=startup_complete,
        )
        critical.extend(age_critical)
        warnings.extend(age_warnings)

    for transform_name, missing_sec in sorted(
        tf_missing_durations_sec.items()
    ):
        if missing_sec <= 0.0:
            continue
        if startup_complete and missing_sec >= thresholds.tf_critical_sec:
            critical.append(
                f'TF {transform_name} mất liên tục {missing_sec:.1f} s.'
            )
        else:
            warnings.append(
                f'TF {transform_name} hiện chưa khả dụng '
                f'({missing_sec:.1f} s).'
            )

    if (
        nav_active
        and stationary_duration_sec is not None
        and stationary_duration_sec
        >= thresholds.navigation_stall_warning_sec
    ):
        warnings.append(
            'Robot đang có goal Nav2 nhưng gần như đứng yên '
            f'{stationary_duration_sec:.1f} s.'
        )

    if (
        consecutive_nav_failures
        >= max(1, thresholds.consecutive_nav_failures_warning)
    ):
        warnings.append(
            f'Có {consecutive_nav_failures} frontier goal thất bại liên tiếp.'
        )

    if critical:
        return {
            'status': STATUS_NOT_OK,
            'headline': '🔴 NOT OK - HỆ THỐNG KHÔNG ỔN, NÊN DỪNG RUN',
            'summary': (
                'Có lỗi nghiêm trọng có thể làm dữ liệu hoặc navigation '
                'không còn đáng tin.'
            ),
            'critical_reasons': critical,
            'warnings': warnings,
        }

    if warnings:
        return {
            'status': STATUS_WARNING,
            'headline': (
                '⚠️ WARNING - ROBOT VẪN CHẠY NHƯNG CẦN THEO DÕI'
            ),
            'summary': (
                'Chưa thấy lỗi nghiêm trọng, nhưng có tín hiệu cần chú ý.'
            ),
            'critical_reasons': [],
            'warnings': warnings,
        }

    if exploration_complete:
        summary = (
            'Exploration đã hoàn tất và các tín hiệu đang giám sát '
            'đều bình thường.'
        )
    elif nav_active:
        summary = (
            'Robot đang navigation; không phát hiện lỗi nghiêm trọng '
            'từ các tín hiệu đang giám sát.'
        )
    else:
        summary = (
            'Robot đang ổn; không phát hiện lỗi nghiêm trọng từ các '
            'tín hiệu đang giám sát.'
        )

    return {
        'status': STATUS_OK,
        'headline': '✅ OK - ROBOT ĐANG HOẠT ĐỘNG BÌNH THƯỜNG',
        'summary': summary,
        'critical_reasons': [],
        'warnings': [],
    }
