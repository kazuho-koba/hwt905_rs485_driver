# hwt905_rs485_driver

WitMotion HWT905/HWT905-485 IMU を **RS485（Modbus-RTU）** 経由で読み取り、  
ROS 2 Foxy互換の `sensor_msgs/msg/Imu` と `sensor_msgs/msg/MagneticField` をpublishするドライバです。

## 周期診断（任意有効化）

### 起動直後の反復試験

診断ON時はnode初期化開始、Modbus初期化完了、読み取りループ開始、最初の正常publishの単調時計時刻をCSVと同名のJSONへ保存します。OSのprocess起動からPython importまでの時間やセンサ内部の初期化完了を直接測るものではありません。
`timing_startup_seconds`（既定20秒）の先頭最大20000件を別枠に保持するため、末尾ring bufferが上限に達しても起動区間は残ります。実際に失われた行数はJSONの`lost_samples`に示します。

以下は各40秒を3回実施するコマンドです。既存launch/service/IMU nodeを停止し、同じポートへ二重アクセスしない状態にしてください。JetsonではFoxy→ros2_ws→patasmonkey_wsをsourceし、SSH制御接続を試験終了まで保持します。

```bash
python3 ~/ros2_ws/src/hwt905_rs485_driver/tools/run_imu_startup_trials.py --mode imu --output /tmp/wit_startup_imu
python3 ~/ros2_ws/src/hwt905_rs485_driver/tools/run_imu_startup_trials.py --mode bringup --output /tmp/wit_startup_bringup
```

出力先は存在しないディレクトリを指定してください。IMU単独ではその下にCSV/JSONを保存し、bringupでは`~/patasmonkey_ws/bags`の各試行へ保存します。bringupではvehicle_interface/teleop/GNSS/NTRIPを無効化します。IMU単独のport/baud/poll-hzはオプション指定可、bringupでは既存configを使います。
`--duration`、`--trials`、`--pause`で試験時間・回数・試行間待機を調整できます。bringupは画像bagも保存するため容量に注意してください。勝手にbagを削除することはありません。

runnerは試行専用process groupを作り、SIGINT後も子が残れば同じgroupへSIGINTを転送して終了を待ちます。終了できなければ次の試行を開始せず、PID groupを報告します。SIGKILLは使いません。OpenVINSラッパー自身の終了不備を修正する機能ではありません。

出力先の`comparison.json`には各試行の0〜5秒、5〜10秒、10〜20秒、20秒以降のHz/エラー/長い間隔の件数・最大値が入ります。区間に十分な成功サンプルがない場合や保持欠落で連続性が失われた場合、Hzはnullです。長い間隔の既定閾値は15 msで、間隔の終端を区間に割り当てます。

```bash
python3 ~/ros2_ws/src/hwt905_rs485_driver/tools/analyze_imu_timing.py trial1.csv trial2.csv --boundaries 5 10 20 --gap-ms 20
```

旧CSVも解析できますが、JSONのない旧記録では最初の読み取り時刻を基準にするためnode起動前後の待機時間は復元できません。

通常は`timing_diagnostics=false`で、診断CSVも追加の時刻計測も行いません。
100 Hz未達の原因を調べるときだけ次のように起動します。既存のIMUノードを停止してから実行してください。同じserial portへ二重接続しないでください。

```bash
ros2 launch hwt905_rs485_driver hwt905_imu.launch.py timing_diagnostics:=true timing_csv:=/tmp/wit_timing_trial.csv
```

Ctrl+Cで正常終了するとCSVを保存します。空の保存先では`/tmp/wit_timing_<日時>.csv`を自動採番します。既存ファイルは上書きしません。直接nodeを起動する場合も`--ros-args -p timing_diagnostics:=true -p timing_csv:=/tmp/wit_timing_trial.csv`が使えます。

```bash
python3 ~/ros2_ws/src/hwt905_rs485_driver/tools/analyze_imu_timing.py /tmp/wit_timing_trial.csv
```

診断中は最後の`timing_max_samples`件（既定60000件、100 Hzで約10分）をメモリ保持します。上限を超えた古い行は破棄し、CSVと集計の`evicted_samples`で件数を明示します。読み取り中のディスクI/Oを避けますが、時刻取得・行生成自体の小さな負荷は残ります。SIGKILLや電源断では保存されません。

CSVの時刻はROS時刻ではなく単調時計の秒です。以下の時間値はmsです。

- `start_interval_ms`: 読み取り開始の間隔。センサ内部の測定周期やbag受信間隔とは異なります。
- `wake_lateness_ms`: 予定された読み取り開始からの遅れ。OSスケジューリングやsleepの遅延などを含みます。
- `read_ms`: Modbus executeの所要時間。USB/serial送受信、センサ応答待ち、Python処理を含み、個別要因を分離する値ではありません。
- `conversion_ms`: 読み取ったレジスタの単位変換、quaternion、ROSメッセージ設定まで。
- `publish_ms`: IMUと磁気のpublish呼び出し時間。購読側での受信完了は測定しません。
- `processing_ms`: 読み取り開始からpublish完了まで。
- `deadline_overrun_ms` / `deadline_missed`: 次周期の予定時刻を超えた時間／超過判定。ドライバが周期基準をリセットした条件と対応します。

読み取りエラーの行は`status=read_error`で、未実行の変換・publish・deadline判定は空欄です。エラー時の100 ms待機は次の開始間隔へ反映されます。集計は平均・中央値・p95・p99・最大値を出し、保持区間内の成功サイクル周波数も示します。

今回は周期制御やpoll_hz、通信設定を変更せず、原因の計測だけを追加しています。通常OSで厳密な10 ms周期を保証する機能ではありません。
