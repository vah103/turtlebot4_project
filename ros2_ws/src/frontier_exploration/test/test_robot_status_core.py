from frontier_exploration.robot_status_core import (
    HealthThresholds,
    STATUS_NOT_OK,
    STATUS_OK,
    STATUS_WARNING,
    classify_health,
    topic_rate_hz,
)


def _health(**overrides):
    values = {
        'startup_complete': True,
        'run_invalid': False,
        'run_invalid_reason': None,
        'odom_age_sec': 0.02,
        'scan_age_sec': 0.05,
        'map_age_sec': 1.0,
        'tf_missing_durations_sec': {},
        'nav_active': False,
        'stationary_duration_sec': None,
        'consecutive_nav_failures': 0,
        'exploration_complete': False,
        'thresholds': HealthThresholds(),
    }
    values.update(overrides)
    return classify_health(**values)


def test_healthy_robot_has_explicit_ok_headline():
    result = _health(nav_active=True)

    assert result['status'] == STATUS_OK
    assert 'ROBOT ĐANG HOẠT ĐỘNG BÌNH THƯỜNG' in result['headline']
    assert not result['critical_reasons']


def test_invalid_run_is_not_ok_and_recommends_stopping():
    result = _health(
        run_invalid=True,
        run_invalid_reason='suspected_map_jump',
    )

    assert result['status'] == STATUS_NOT_OK
    assert 'NÊN DỪNG RUN' in result['headline']
    assert 'INVALID' in result['critical_reasons'][0]


def test_missing_scan_after_startup_is_not_ok():
    result = _health(scan_age_sec=None)

    assert result['status'] == STATUS_NOT_OK
    assert any('/scan' in reason for reason in result['critical_reasons'])


def test_startup_waiting_for_topics_is_only_warning():
    result = _health(
        startup_complete=False,
        odom_age_sec=None,
        scan_age_sec=None,
        map_age_sec=None,
    )

    assert result['status'] == STATUS_WARNING
    assert not result['critical_reasons']


def test_repeated_navigation_failures_raise_warning_not_critical():
    result = _health(consecutive_nav_failures=3)

    assert result['status'] == STATUS_WARNING
    assert any('3 frontier goal' in warning for warning in result['warnings'])


def test_stationary_active_navigation_raises_warning():
    result = _health(
        nav_active=True,
        stationary_duration_sec=21.0,
    )

    assert result['status'] == STATUS_WARNING
    assert any('đứng yên' in warning for warning in result['warnings'])


def test_sustained_tf_failure_is_not_ok():
    result = _health(tf_missing_durations_sec={'map -> odom': 6.0})

    assert result['status'] == STATUS_NOT_OK
    assert any('map -> odom' in reason for reason in result['critical_reasons'])


def test_topic_rate_uses_recent_window():
    rate = topic_rate_hz(
        [0.0, 1.0, 2.0, 8.0, 9.0, 10.0],
        now=10.0,
        window_sec=3.0,
    )

    assert rate == 1.0
