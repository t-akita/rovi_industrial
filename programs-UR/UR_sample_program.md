#UR用サンプルプログラム

## ロボットキャリブ用〜test1
- X0.script

	シーンクリア

- X1.script

	撮影〜撮影ポジション送信

- X2-rcalib.script

	ソルバー呼出し


## ビジュアルティーチング用〜mt
- X0.script

	シーンクリア

- X1.script

	撮影〜撮影ポジション送信

- X2.script

	ソルバー呼出し
	
## RoVIとの相性について
プログラミングの際には以下の単位系について、留意が必要です。

- 単位系

	UR: rad, RoVI: deg
	UR: m, RoVI: mm(m?)

- 回転表記

	UR: オイラー角
	RoVI: roll,pitch,yaw
