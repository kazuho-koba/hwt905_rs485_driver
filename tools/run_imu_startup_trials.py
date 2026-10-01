#!/usr/bin/env python3
"""IMU単独または安全なpm_bringupで、起動・正常停止を反復する。"""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def live_group(pgid):
    """今回のprocess groupだけを列挙する。無関係なノードには触れない。"""
    rows = subprocess.check_output(['ps','-eo','pid,pgid,stat,args'], text=True).splitlines()[1:]
    return [r for r in rows if int(r.split()[1])==pgid and not r.split()[2].startswith('Z')]


def stop_trial(proc):
    """launch親をSIGINT停止し、残った子にも同じ試行groupでSIGINTを送る。"""
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        # teeなどのラッパーで親だけでは終了できない場合、専用groupへ転送する。
        pass
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    for _ in range(30):
        proc.poll()  # 終了した親を回収し、zombieを残さない。
        if not live_group(proc.pid):
            return
        time.sleep(1)
    # SIGKILLでCSVを失うことを避け、次の試行を開始せず異常を報告する。
    raise RuntimeError('試行processが終了していません。PGID=' + str(proc.pid) + '\n' + '\n'.join(live_group(proc.pid)))


def command(mode, name, csv, port, baud, poll_hz):
    """pm_bringup試験ではactuatorと操縦ノードを必ず無効にする。"""
    if mode == 'imu':
        return ['ros2','run','hwt905_rs485_driver','hwt905_imu_node','--ros-args',
                '-p','timing_diagnostics:=true','-p','timing_csv:='+str(csv),
                '-p','port:='+port,'-p','baud:='+str(baud),'-p','poll_hz:='+str(float(poll_hz))]
    return ['ros2','launch','pm_bringup','pm_bag_global_localization.launch.py',
            'use_vehicle_interface:=false','use_teleop:=false','use_gnss:=false','use_ntrip:=false',
            'wit_timing_diagnostics:=true','bag_name:='+name]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['imu','bringup'], default='imu')
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--duration', type=float, default=40, help='process起動からの秒数')
    parser.add_argument('--pause', type=float, default=5)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=230400)
    parser.add_argument('--poll-hz', type=float, default=100)
    args = parser.parse_args()
    if args.trials < 1 or args.duration <= 0 or args.pause < 0 or args.poll_hz <= 0:
        parser.error('試行数・時間・周波数は正、pauseは非負にしてください')
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for i in range(args.trials):
        name = 'rosbag2_' + datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S_%f') + '_wit_startup_' + str(i)
        folder = args.output/name if args.mode=='imu' else Path.home()/'patasmonkey_ws/bags'/name
        csv = folder/'wit_timing.csv'
        # bringupではrosbag自身がfolderを作る。事前作成するとrecordが失敗する。
        if args.mode=='imu':
            folder.mkdir()
        cmd = command(args.mode,name,csv,args.port,args.baud,args.poll_hz)
        with (args.output/(name+'.log')).open('x') as log:
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            print('trial',i+1,'PID',proc.pid,'CSV',csv,flush=True)
            try:
                end = time.monotonic()+args.duration
                while proc.poll() is None and time.monotonic()<end:
                    time.sleep(.2)
            finally:
                stop_trial(proc)
        if not csv.is_file() or not csv.with_suffix('.json').is_file():
            raise RuntimeError('CSV/起動metadataが保存されていません: '+str(csv))
        if args.mode=='bringup':
            if not (folder/'metadata.yaml').is_file():
                raise RuntimeError('bagのmetadataがありません: '+str(folder))
            with (args.output/(name+'_bag_info.txt')).open('x') as info:
                subprocess.run(['ros2','bag','info',str(folder)],stdout=info,stderr=subprocess.STDOUT,check=True)
        results.append({'csv':str(csv), 'command':cmd,'exit_code':proc.returncode})
        (args.output/'trials.json').write_text(json.dumps(results,indent=2))
        if i+1<args.trials:
            time.sleep(args.pause)
    analyzer = Path(__file__).with_name('analyze_imu_timing.py')
    with (args.output/'comparison.json').open('x') as output:
        subprocess.run(['python3',str(analyzer)]+[r['csv'] for r in results],stdout=output,check=True)


if __name__=='__main__':
    main()
