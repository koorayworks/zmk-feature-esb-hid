# zmk-feature-esb-hid

ZMK の HID レポート（キーボード／コンシューマ／マウス）を Nordic ESB で送る
ZMK モジュール。キーボード側でキーマップを処理し、完成した HID レポートを
レシーバーへ送って USB HID に中継する（BLE HOGP の ESB 版）。

split のポジション情報を送る方式ではないため、キーボードごとに自分の
キーマップを持てる。ポインティング（トラックボール等）のマウスレポートも
同じ経路で送れる。

## 状態

Step 1（実験段階）: 固定アドレス・暗号化なし・1対1。
**未ビルド検証**。ビルドには対応するフックを持つ ZMK フォーク
（`vwbugowner1976/zmk` の `esb-hid` ブランチ）が必要。

## 使い方

west manifest にモジュールを追加し、ボード設定で有効化する。

キーボード側:

    CONFIG_ZMK_ESB_HID=y
    CONFIG_ESB_MAX_PAYLOAD_LENGTH=64

レシーバー側（マトリクスのない ZMK ビルド＋USB 接続）:

    CONFIG_ZMK_ESB_HID_RECEIVER=y
    CONFIG_ESB_MAX_PAYLOAD_LENGTH=64

テスト用 config: `vwbugowner1976/zmk-config-esb-hid-test`

## ワイヤ形式

1パケット=1レポート: `[種別][seq][HIDレポート本体]`。
種別 0=キーボード、1=コンシューマ、2=マウス。本体は ZMK のレポート
ボディそのまま（BLE HOG が送るものと同一）。

## ライセンス

モジュール本体は MIT。`esb/` 以下の Nordic ESB スタックは
nRF Connect SDK から取り込んだもので、`LicenseRef-Nordic-5-Clause`
（各ファイルのヘッダ参照）。
