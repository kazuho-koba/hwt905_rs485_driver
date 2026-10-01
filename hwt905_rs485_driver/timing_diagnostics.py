"""周期診断の有界メモリ記録。センサ読み取り中のファイルI/Oを避ける。"""
import csv
import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path


class TimingDiagnostics:
    """最後のmax_samples件を保持する。時刻は全て単調時計の秒。"""

    columns = ['sequence', 'status', 'start_s', 'scheduled_s', 'period_ms',
               'start_interval_ms', 'wake_lateness_ms', 'read_ms',
               'conversion_ms', 'publish_ms', 'processing_ms',
               'deadline_overrun_ms', 'deadline_missed', 'evicted_total']

    def __init__(self, path, max_samples, startup_seconds=0, origin=None):
        if max_samples < 1:
            raise ValueError('timing_max_samplesは1以上にしてください')
        self.path = Path(path).expanduser() if path else Path('/tmp') / (
            'wit_timing_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
        self.rows = deque(maxlen=max_samples)
        self.sequence = 0
        self.previous_start = None
        self.evicted = 0
        self.origin = time.perf_counter() if origin is None else origin
        self.startup_seconds = startup_seconds
        # 起動直後の先頭20000件は別枠で固定保持し、後続ring bufferで消さない。
        # 件数にも上限を設け、異常なpoll_hzでもメモリが無制限に増えないようにする。
        self.startup_rows = []
        self.events = {'node_start_s': self.origin}

    def mark(self, name):
        """接続完了などの節目を単調時計で記録する。"""
        self.events[name] = time.perf_counter()

    def add(self, start, scheduled, period, read_end, conversion_end,
            publish_end, deadline_check, status):
        """読み取り・変換・publishを分離し、次のdeadline超過を保存する。"""
        interval = '' if self.previous_start is None else (start-self.previous_start)*1000
        self.previous_start = start
        if len(self.rows) == self.rows.maxlen:
            self.evicted += 1
        overrun = '' if deadline_check is None else max(0, deadline_check-scheduled-period)*1000
        # read_errorではpublishに到達しないため、未実行区間は空欄とする。
        self.rows.append((self.sequence, status, start, scheduled, period*1000,
                          interval, max(0, start-scheduled)*1000, (read_end-start)*1000,
                          '' if conversion_end is None else (conversion_end-read_end)*1000,
                          '' if publish_end is None else (publish_end-conversion_end)*1000,
                          '' if publish_end is None else (publish_end-start)*1000,
                          overrun, '' if overrun == '' else int(overrun > 0), self.evicted))
        self.sequence += 1
        if 0 <= start-self.origin < self.startup_seconds and len(self.startup_rows) < 20000:
            self.startup_rows.append(self.rows[-1])
        if status == 'ok' and 'first_success_s' not in self.events:
            self.events['first_success_s'] = publish_end

    def save(self):
        """worker停止後にCSVを排他的に作成する。既存ファイルは上書きしない。"""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(self.columns)
            # 先頭固定区間と末尾区間の重複はsequenceで除き、時間順で出力する。
            retained = {r[0]: r for r in self.startup_rows}
            retained.update({r[0]: r for r in self.rows})
            writer.writerows(retained[k] for k in sorted(retained))
        metadata = dict(self.events)
        metadata.update(total_samples=self.sequence, retained_samples=len(retained),
                        lost_samples=self.sequence-len(retained),
                        startup_seconds=self.startup_seconds,
                        startup_capacity=20000, startup_samples=len(self.startup_rows))
        with self.path.with_suffix('.json').open('x') as stream:
            json.dump(metadata, stream, indent=2)
