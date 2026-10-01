"""デバイスなしで時間分解、上限、エラー行、既存CSV保護を検証する。"""
import csv
import importlib.util
from pathlib import Path
import pytest
from hwt905_rs485_driver.timing_diagnostics import TimingDiagnostics


def test_timing_csv(tmp_path):
    path = tmp_path/'timing.csv'
    diag = TimingDiagnostics(str(path), 2)
    diag.add(1.0, 1.0, .01, 1.002, 1.003, 1.004, 1.004, 'ok')
    diag.add(1.01, 1.01, .01, 1.012, 1.013, 1.021, 1.021, 'ok')
    diag.add(1.03, 1.021, .01, 1.04, None, None, None, 'read_error')
    diag.save()
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2
    assert float(rows[0]['read_ms']) == pytest.approx(2)
    assert float(rows[0]['conversion_ms']) == pytest.approx(1)
    assert float(rows[0]['publish_ms']) == pytest.approx(8)
    assert float(rows[0]['deadline_overrun_ms']) == pytest.approx(1)
    assert rows[0]['deadline_missed'] == '1'
    assert rows[1]['publish_ms'] == ''
    assert rows[1]['evicted_total'] == '1'
    with pytest.raises(FileExistsError):
        diag.save()
    spec = importlib.util.spec_from_file_location('analyze_timing', Path(__file__).parents[1]/'tools/analyze_imu_timing.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.summarize(path)
    assert result['read_errors'] == 1
    assert result['deadline_misses'] == 1
    assert result['evicted_samples'] == 1


def test_invalid_capacity():
    with pytest.raises(ValueError):
        TimingDiagnostics('', 0)


def test_startup_retention_and_windows(tmp_path):
    """先頭保存と末尾ringの中央欠落をHz低下と誤認しない。"""
    path = tmp_path/'startup.csv'
    diag = TimingDiagnostics(str(path), 2, startup_seconds=5, origin=100)
    for start in (101,102,106,107,108):
        diag.add(start,start,.01,start+.002,start+.003,start+.004,start+.004,'ok')
    diag.save()
    spec = importlib.util.spec_from_file_location('analysis', Path(__file__).parents[1]/'tools/analyze_imu_timing.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.summarize(path)
    assert result['samples']==4
    assert result['metadata']['lost_samples']==1
    assert result['windows'][0]['samples']==2
    assert result['successful_cycles_hz'] is None
    assert result['startup_events_ms']['first_success_s']==pytest.approx(1004)
    assert result['long_intervals'][0]['elapsed_s']==2


def test_trial_command_safety():
    """デバイスは起動せず、bringupコマンドのactuator無効指定を検証する。"""
    spec = importlib.util.spec_from_file_location('trials', Path(__file__).parents[1]/'tools/run_imu_startup_trials.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cmd = module.command('bringup','trial','unused','/dev/ttyUSB0',230400,100)
    assert 'use_vehicle_interface:=false' in cmd
    assert 'use_teleop:=false' in cmd
    # Foxyでは整数表記100だとdouble_valueが0になるため、小数表記で渡す。
    assert 'poll_hz:=100.0' in module.command('imu','trial','unused','/dev/ttyUSB0',230400,100)
