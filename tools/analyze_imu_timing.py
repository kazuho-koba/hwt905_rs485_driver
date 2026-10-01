#!/usr/bin/env python3
"""Wit周期診断CSVを集計する。ROSやセンサ接続は不要。"""
import argparse
import csv
import json
import statistics
from pathlib import Path


def summarize(path, boundaries=(5.0, 10.0, 20.0), gap_ms=15.0):
    """保持された区間だけを解析し、エラーと記録脱落を明示する。"""
    with Path(path).open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    metadata_path = Path(path).with_suffix('.json')
    metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    origin = metadata.get('node_start_s', float(rows[0]['start_s']) if rows else 0)
    result = {'samples': len(rows),
              'read_errors': sum(r['status'] != 'ok' for r in rows),
              'deadline_misses': sum(r['deadline_missed'] == '1' for r in rows),
              'evicted_samples': int(rows[-1]['evicted_total']) if rows else 0}
    for key in ('start_interval_ms', 'wake_lateness_ms', 'read_ms',
                'conversion_ms', 'publish_ms', 'processing_ms', 'deadline_overrun_ms'):
        values = sorted(float(r[key]) for r in rows if r[key] != '')
        if values:
            # 分位点はnearest-rankを使用。平均Hzは読み取り開始間隔でありpublish Hzではない。
            import math
            result[key] = {'mean': statistics.mean(values), 'median': statistics.median(values),
                           'p95': values[max(0, math.ceil(len(values)*0.95)-1)],
                           'p99': values[max(0, math.ceil(len(values)*0.99)-1)], 'max': values[-1]}
    ok = [r for r in rows if r['status'] == 'ok']
    if len(ok) > 1:
        duration = float(ok[-1]['start_s']) - float(ok[0]['start_s'])
        # 保持上限による中央区間の脱落を、センサ停止と誤認してHz計算しない。
        contiguous = all(int(b['sequence']) == int(a['sequence'])+1 for a,b in zip(rows,rows[1:]))
        result['successful_cycles_hz'] = (len(ok)-1)/duration if duration > 0 and contiguous else None
    result['metadata'] = metadata
    result['time_origin'] = 'node_start' if metadata else 'first_record_legacy'
    result['startup_events_ms'] = {k: (v-origin)*1000 for k,v in metadata.items()
                                   if k.endswith('_s') and isinstance(v,(float,int))}
    result['windows'] = []
    edges = [0.0] + list(boundaries) + [float('inf')]
    for lo,hi in zip(edges,edges[1:]):
        selected = [r for r in rows if lo <= float(r['start_s'])-origin < hi]
        successful = [r for r in selected if r['status']=='ok']
        intervals = [float(r['start_interval_ms']) for r in selected if r['start_interval_ms']]
        contiguous = all(int(b['sequence'])==int(a['sequence'])+1 for a,b in zip(selected,selected[1:]))
        duration = float(successful[-1]['start_s'])-float(successful[0]['start_s']) if len(successful)>1 else 0
        result['windows'].append({'from_s':lo, 'to_s':None if hi==float('inf') else hi,
            'samples':len(selected), 'read_errors':sum(r['status']!='ok' for r in selected),
            'successful_cycles_hz':(len(successful)-1)/duration if duration>0 and contiguous else None,
            'gap_count':sum(v>gap_ms for v in intervals),
            'max_interval_ms':max(intervals) if intervals else None,
            'retention_gap':not contiguous})
    # 間隔は終端のcycleに割り当てる。起動時刻基準で場所を確認できる。
    result['long_intervals'] = [{'elapsed_s':float(r['start_s'])-origin,
        'interval_ms':float(r['start_interval_ms']), 'sequence':int(r['sequence'])}
        for r in rows if r['start_interval_ms'] and float(r['start_interval_ms'])>gap_ms]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, nargs='+', help='複数試行のCSVを指定可能')
    parser.add_argument('--boundaries', type=float, nargs='+', default=[5,10,20])
    parser.add_argument('--gap-ms', type=float, default=15)
    args = parser.parse_args()
    if args.gap_ms <= 0 or any(v <= 0 for v in args.boundaries) or args.boundaries != sorted(set(args.boundaries)):
        parser.error('閾値は正、区間境界は昇順で重複なしにしてください')
    print(json.dumps({str(p): summarize(p,args.boundaries,args.gap_ms) for p in args.csv},
                     indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
