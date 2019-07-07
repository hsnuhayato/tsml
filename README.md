# tsml
二足歩行プログラム開発環境を晒す！

二足歩行の勉強や自分の制御則実装して検証したい時、最初の環境構築において挫折しないか？俺はしていた。シミュレータの使い方やミドルウェアの仕様(rosとかopenrtmとか)を把握するだけで大変だよね...makefileも書かなぎゃいけない正直めんどい。人の環境を拝借してコントローラの中身だけ弄りたい！！とか思ったことない？自分の開発環境を共有する。二足歩行プログラム開発したいの人の手助けになると幸いです。
自分の開発環境は以下のものが含まれている：

* choreonoidの実行環境、openrtmコントローラビルド環境のdockerイメージ
* ロボットモデルファイル
* コントローラソースコード一式
* ロボットをオンラインで操作するpythonのui

ホストのエディタでコントローラのソースコードを編集し、ディスクをdocker環境へマウントしdocker内でビルドし、docker環境内のchoreonoid使って物理シミュレーションにより動作確認するという開発し方。コントローラの中身を弄るだけで自分の制御則を検証できる環境にするつもりである。ロボットコントローラはopenrtm使用しいる。開発環境はdockerで完結するのでdockerさえ使えばホストOS環境は何でも大丈夫なはず。自分の環境はubuntu16.04である。
まだまだ未熟なシステムなので、コメント、PR大歓迎。なおこの記事読んで皆さんが次々と自分の環境を公開してお互いに学び合うといいなあと思う。これもこの記事の狙いである。

と、勉強ではなく実用的な歩行システムいますぐに使いたい方はhrpsys-baseを使った方が良い。自分のプログラムはまだめちゃコケる。

# 環境の導入
[こちらの記事](https://qiita.com/wukunn/items/07f7fa38e3d1e013973e)を参考にして私が作ったchoreonoidのdocker imageをpullして動作確認してみてください。OpenRTMの歩行サンプルがうまく動作するのを確認(これ重要)。

# ソースコードの取得

githubからpull:

```bash
$ git clone https://github.com/hsnuhayato/tsml -b wu_branch
```

以下に中身を簡単に説明する。本記事が触れていないファイルの説明を省略する。


```bash
tsml
├── CMakeLists.txt // メインmakeファイル
├── bin
│   ├── hrpsysjy
│   ├── jvrc.sh // choreonoidのproject起動するスクリプト
│   ├── rtc.conf // openrtm設定ファイル
│   ├── rtc.conf.choreonoid -> rtc.conf
│   └── setup_jvrc_tsml.sh 
├── customBush // 足首関節のゴムブッシュ要素を模擬するボディカスタマイザ
│   ├── CMakeLists.txt
│   └── src
├── etc
│   ├── QrCodeTest.cnoid
│   ├── RobotJVRC.conf // コントローラパラメータ設定ファイル
│   ├── RobotState.cnoid
│   ├── WalkTestBush.cnoid //この記事使用するchoreonoidのprojectファイル
│   ├── bridgeJVRC.conf // 仮想ロボットの入力出力ポート設定
│   └── bridgeRobotState.conf 
├── login_docker.sh // docker環境にログインするスクリプト
├── model // ロボットモデルファイル
│   ├── JVRC-1
│   ├── JVRC-TSML // 本記事はこれを使う
│   ├── misc
│   └── tasks
├── rtc // opemrtmのコントローラ
│   ├── ArmControlCartesian.cnoid
│   ├── CMakeLists.txt
│   ├── CameraViewer
│   ├── PdServo // 関節PDサーボ制御RTC
│   ├── PointCloudViewer.cnoid
│   ├── ReferenceHolder.cnoid // コントローラの出力を保持するRTC
│   ├── RobotState
│   ├── SequencePlayer.cnoid
│   ├── StateEstimator // 姿勢推定RTC
│   ├── clean.sh
│   ├── creeklib
│   ├── gamepad
│   ├── hrpsys_st // 歩行安定化制御RTC
│   ├── install.sh
│   ├── make.sh
│   └── sony_cnoid // 歩行パターン生成RTC
├── run_docker.sh // docker環境起動スクリプト
├── script
│   ├── dualpc
│   ├── jvrc
│   ├── python_ui // ipythonのインターフェイス
│   └── test
└── share
    ├── hrpsys // cmakeモジュール
    ├── jar
    └── jython


```

# ビルド

docker環境内でビルドしよう。

```bash
$ cd tsml;./run_docker.sh
```
でローカルのtsmlフォルダをdoekcer環境の/mntにマウントする。次にdocker環境で


```bash
# cmake -H. -Bbuild;cd build;make install
```

でビルドとdocker内にインストールする。中にJVRC1Customizer.so(ゴムブッシュを模擬するbody customizer)のインストール先がdocker環境内の/usr/local/lib/choreonoid-1.7/customizer/なので、docker終了後に消える。docker起動するたびにmake installするか、docker環境をcommitして保持させておくと良い。

# choreonoidプロジェクトファイル起動と動作確認
docker環境で

```bash
# cd bin
# ./jvrc.sh
```

で以下スクショのchoreonid GUI環境が起動するはず。
![Screenshot from 2019-06-30 23-18-39.png](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/159774/4d63d90b-b5b9-c814-69a1-9fc285849850.png)
ここでchoreonoidコアダンプで落ちた人、omniNamesが起動しているかをチェックしてください。起動していなければrtm-namingで起動しよう。

次にホスト環境でもう一個ターミナルを起動し、tsml/login_docker.shを起動しdocker環境に入る。コンソールに相当するpythonスクリプトを起動。


```bash
# cd script/python_ui
# ipython -i python_ui.py
```

でロボットが足踏み始めるはず。ipythonに

```bash
In [1]:  hcf.stepping()

```

入れると足踏み停止する。これで動作確認完了。

# コントローラの構成
コントローラの構成は以下の図で示す。

![Screenshot from 2019-07-01 22-38-03.png](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/159774/237ca978-2ef8-433b-1d24-6ec53990eee0.png)

* kf：姿勢推定RTC
* wpg: 歩行パターン生成RTC。パターン生成の説明はこちらの記事[自作二足歩行パターン生成](https://qiita.com/wukunn/items/05c1e94b547191cb8e83)を参照されたい。
* sh: 最後の指令値を保持するRTC。指令値を出力するRTCを切り替える前に最後の指令を取得用。今回指令値を出力するRTCはwpg一個だけの場合はいらないが、一応残しておいた。
* st: 歩行安定化制御RTC。stの説明はこちらの記事[歩行安定化制御について](https://qiita.com/wukunn/items/0cef41a6206dc8abc6fa)を参照されたい。

これらのRTCでは、起動時にonInitialize関数が一回呼ばれて、その後onExecute関数が毎制御周期に呼ばれる。メインロジックもonExecute内に実装されている。
なお、コントローラが以下の順番で同期実行している。
(kf->wpg->sh->st->)creekPdServo->JVRC-TSML
ただし、creekPdServoとJVRC-TSMLの制御周期が1[ms]で、他のRTCが5[ms]。
それぞれはchoreonoid GUI上のタイムバーの内部フレームレートとBodyRTCの実行周波数(恐らく実行周期の誤字...)で設定できる。


# コンソールのpythonスクリプトについて
./jvrc.shを起動直後の時点にはpdservoとrobotのRTC(JVRC-TSML)しか存在しない。python_ui.pyは他RTCの生成や起動、portを接続したり、service port(corba RPC)を呼んだりする。このpythonスクリプトはhrpsys-baseのrtm.pyやhrpsys_config.pyを依存している。要はRTCをオンラインで操作するインターフェイスを提供する。ipythonを起動し、各RTCのサービスをipython上で叩くことでロボットに指示を渡す。qtとかでGUIの皮をかぶせることも可能。なお、 getRTCListで定義されたRTCはactivateCompsの時点でロボットRTC(JVRC-TSML)の実行コンテキストをアタッチし、同期実行になる。RTCList内のRTC順番は実行順になる。

例えば、今自分の歩行パターンRTCは以下のメソッドを用意している：

* hcf.set_vel(10,0,0): 歩行速度(x, y, yaw旋回)
* hcf.stepping(): 足踏み開始/または終了

# コントローラパラメータ設定ファイル
RTC毎に読み込むパラメータ設定ファイルは全部別々と設定できるが、管理しやすくため、すべてのRTCは以下の設定パラメータファイルを読み込むようにした。

- tsml/etc/RobotJVRC.conf

各RTCの実装内に以下のおまじないでパラメータを取得する。例えば"dt"というパラメータは以下の通りで取得：

```c++
double dt;
RTC::Properties& prop = getProperties();
coil::stringTo(dt, prop["dt"].c_str());
```

歩行パターン生成用のパラメータ：

* wpg.dt: サンプリング時間
* initBasePos: ロボット全身関節角度が0[deg]の時のrootリンク位置
* initBaseRpy: ロボット全身関節角度が0[deg]の時のrootリンク姿勢
* RLEG_END: ロボットモデルの右足末端リンク名
* LLEG_END: ロボットモデルの左足末端リンク名
* RARM_END: ロボットモデルの右手末端リンク名
* LARM_END: ロボットモデルの左手末端リンク名
* BASE_LINK: ロボットモデルのrootリンク名
* HEAD_P: ロボットモデルの頭部ピッチ軸リンク名
* HEAD_Y: ロボットモデルの頭部ヨー軸リンク名
* halfpos: 中腰姿勢の全軸角度
* cm_offset_x: 足首リンク座標系から足幾何形状の中心までのx方向距離
* Tsup: 片足支持期期間
* Tsup_stepping: その場足踏み時の片足支持期期間
* Tdbl: 両足支持期期間
* offsetZMPy: ZMP計画時に足幾何形状の中心からのy方向のオフセット
* offsetZMPy_stepping: ZMP計画時に足幾何形状の中心からのy方向のオフセット(その場足踏み時)
* offsetZMPx: ZMP計画時に足幾何形状の中心からのx方向のオフセット
* Zup: 遊脚高さ
* pitch_angle: 遊脚のつま先離陸かかと着陸角度

* link_b_front: 足首リンク座標系から足幾何形状の前方エッジまでのvector
* link_b_rear: 足首リンク座標系から足幾何形状の後方エッジまでのvector
* link_b_ee: 足首リンク座標系から足幾何形状の中心のvector
* ankle_height: 足首関節高さ

歩行安定化制御パラメータ(殆どpython_ui内でstのservice port呼んで代入している)
* end_effectors: [link_path名, link_path末端リンク名, link_path根元リンク名, link_path末端リンク座標系からend effecorまでの位置ベクトル、 link_path末端リンク座標系からend effecorまでのaxis angle] * link_path数
* cop_offset: link_path末端リンク座標系から足の圧力中心点までのベクトル


#  customBushについて
choreonoidには足首関節に設置するゴムブッシュ(ダンバーやバネなどの機械要素)を模擬することができる。この機能は[ボディカスタマイザの実装](https://github.com/hsnuhayato/tsml/tree/wu_branch/customBush/src)により実現する。基本的にはブッシュ化したい関節名(ロボットのモデルファイル内の関節名に対応する)を定義して、バネとダンパー定数を定義し反発力を計算する。硬さを調整したい人はソース内の

```c++
customizer->springT  = 1.3e6; // N/m
customizer->dampingT = 1.0e3; // N/(m/s)
customizer->springR  = 3e3; // Nm / rad
customizer->dampingR = 2.5; // Nm / (rad/s)
```
この辺を弄ると良い。
# モデルファイル
オリジナルのJVRC-1ロボットを少々弄った:

* 足をJAXONの足で入れ替えた。ゴムブッシュを追加した。
* ハンドカメラとハンドにライト追加。
* 頭部にレンジセンサ追加
* ハンドwrist関節構成をyaw-roll-yawからpitch-roll-yawに変更(逆運動学の都合で)
* テキスチャ全体張り替えた。もとのデザインはいまいち覇気を感じないのでモノアイにした(ごめんなさい園山さん...)


# 制御則だけ弄りたい方へ
ここからが本記事のポイント。私は二足歩行開発用の土台をある程度作ったつもりで、これをベースにして自分なりの制御則を作成方法を説明する。歩行生成作りたい時は、sony RTCをいじると良い。(むかしsonyの論文を読んで実装してたのでコンポーネント名をsonyにした。とくに意味がない)sonyは以下の入出力ポートを用意した(一部現在使用しないポート省略)。

* +DataInPort: q 関節角度現在地
* +DataInPort: rhsensor 右手フォースセンサ値
* +DataInPort: lhsensor 左手フォースセンサ値
* +DataInPort: rfsensor 右足フォースセンサ値
* +DataInPort: lfsensor 左足フォースセンサ値
* +DataInPort: mc 関節角度指令値(一周期前)
* +DataInPort: basePosInit ロボットrootリンク位置(他のRTCがロボット現在位置更新用)
* +DataInPort: baseRpyInit ロボットrootリンク姿勢(他のRTCがロボット現在姿勢更新用)
* +DataOutPort: rzmp rootリンク座標系から見た目標ZMP位置
* +DataOutPort: refq 関節角度指令値
* +DataOutPort: basePosOut ロボットrootリンク位置指令値
* +DataOutPort: baseRpyOut ロボットrootリンク姿勢指令値
* +DataOutPort: contactStates 二足の場合長さ2のデータ。それぞれ右足、左足の接地状態を表す。空中にある場合は0で、それ以外の場合は1
* +DataOutPort: toeheelRatio 二足の場合長さ2のデータ。それぞれ右足、左足の足の回転状態を表す。つま先かかと接地期の足には0，それ以外の場合は１。
* +DataOutPort: controlSwingSupportTime 二足の場合長さ2のデータ。それぞれ右足、左足の支持脚->遊脚もしくは遊脚->支持脚 のように接地状態が変わるまでの時間。
 
これでロボットの歩行動作を計画するための情報が揃っていると思う。InPortの情報を用いてsony RTCのonExeucte関数内で制御則を実装し、outportの情報を生成し出力するとよい。また、ST使いたくない方はSTをoffのままにして、そうするとsonyのoutPort  refq がshを経由してそのままservoに流す。

次は歩行安定化制御をいじりたい人はtsml/rtc/hrpsys_st/Stabilizer.cpp をいじると良い。Stabilizerは以下の入出力ポートを用意している(私が使ってないポートの説明を省略する)。

* +DataInPort: qCurrent ロボット関節角度現在値
* +DataInPort: qRef　ロボット関節角度指令値
* +DataInPort: rpy　カルマンフィルタRTCから出力された姿勢
* +DataInPort: zmpRef　rootリンク座標系からみたzmp
* +DataInPort: basePosIn　ロボットrootリンク位置指令値
* +DataInPort: baseRpyIn　ロボットrootリンク姿勢指令値
* +DataInPort: contactStates　二足の場合長さ2のデータ。それぞれ右足、左足の接地状態を表す。空中にある場合は0で、それ以外の場合は1
* +DataInPort: toeheelRatio　二足の場合長さ2のデータ。それぞれ右足、左足の足の回転状態を表す。つま先かかと接地期の足には0，それ以外の場合は１。
* +DataInPort: controlSwingSupportTime 二足の場合長さ2のデータ。それぞれ右足、左足の支持脚->遊脚もしくは遊脚->支持脚 のように接地状態が変わるまでの時間。
* +DataInPort: qRefSeq 
* +DataInPort: walkingStates
* +DataInPort: sbpCogOffset
* +DataInPort: rfsensor　右足力センサ値
* +DataInPort: rfsensorRef
* +DataInPort: lfsensor　左足力センサ値
* +DataInPort: lfsensorRef
* +DataInPort: rhsensor
* +DataInPort: rhsensorRef
* +DataInPort: lhsensor
* +DataInPort: lhsensorRef
* +DataInPort: limbCOPOffset_rfsensor
* +DataInPort: limbCOPOffset_lfsensor
* +DataInPort: limbCOPOffset_rhsensor
* +DataInPort: limbCOPOffset_lhsensor
* +DataOutPort: q 修正後の関節角度指令値。servo RTCへ流れる。
* +DataOutPort: tau
* +DataOutPort: zmp
* +DataOutPort: refCapturePoint
* +DataOutPort: actCapturePoint
* +DataOutPort: diffCapturePoint
* +DataOutPort: diffFootOriginExtMoment
* +DataOutPort: actContactStates
* +DataOutPort: COPInfo
* +DataOutPort: emergencySignal
* +DataOutPort: originRefZmp
* +DataOutPort: originRefCog
* +DataOutPort: originRefCogVel
* +DataOutPort: originNewZmp
* +DataOutPort: originActZmp
* +DataOutPort: originActCog
* +DataOutPort: originActCogVel
* +DataOutPort: actBaseRpy
* +DataOutPort: currentBasePos
* +DataOutPort: currentBaseRpy
* +DataOutPort: allRefWrench
* +DataOutPort: allEEComp
* +DataOutPort: debugData

安定化制御の制御則弄る場合、Stabilizer RTCのonExeucte関数内で制御則を実装し、出力ポートqにて修正後の目標角度を出力すると良い。

# コントローラのメソッド追加(RTC service portのメソッド)
ipythonから呼び出すservice portのメソッドの追加方法をここで説明する。例えばsony RTCにメソッドを追加する場合、

1. sonyService.idlにメソッドを定義
1. メソッドはsony.hにて定義し、sony.cppにて実装する。
1. sonyService_impl.hとsonyService_impl.cppに手順2で追加したメソッドを呼ぶ実装を追加
1. 再度にcmakeを実行してからソースをビルドする。
1. python_ui.pyにself.wpg_svc.追加されたメソッド名にて呼び出す

sonyService_impl.hとsonyService_impl.cppすでにある実装を参考にして追加してみてください。

# TODO ROS pluginの導入：
docker imageにros pluginを導入する予定。

# 謝辞
以前JVRCに一緒に参加してくれたcreekさんに感謝。彼がいないとこのシステムはできないだろう。
