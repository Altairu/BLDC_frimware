# BLDC モータ制御ファームウェア

## 概要
STM32F303K8Tx マイコンを使用したブラシレスDCモータ / BLDC 駆動制御用ファームウェアです。
3相インバータの相補PWM駆動、シャント電流センサの読み取り、CAN通信インターフェースなどの機能を備えています。

## ハードウェア構成
* マイコン (U2): STM32F303K8Tx (ARM Cortex-M4)
* 3相ゲートドライバ (U3): DRV8300DRGER
* スイッチングMOSFET (Q1からQ6): BSC040N10NS5
* 電流センスアンプ (U4, U5): INA240A1D
* CANトランシーバ (U1): SN65HVD230

## ピンアサイン

| ピン名称 | ネット名 | ペリフェラル設定 | 用途 |
| --- | --- | --- | --- |
| PA0 | LED_B | TIM2_CH1 | フルカラーLED 青制御 |
| PA1 | LED_R | TIM2_CH2 | フルカラーLED 赤制御 |
| PA2 | LED_G | TIM2_CH3 | フルカラーLED 緑制御 |
| PA3 | U_AMP | ADC1_IN3 / ADC2_IN3 | U相電流センサ入力 |
| PA4 | V_AMP | ADC2_IN1 | V相電流センサ入力 |
| PA5 | HALL_A | ADC2_IN2 / GPIO_Input | ホールセンサA入力 |
| PA6 | HALL_B | ADC2_IN3 / GPIO_Input | ホールセンサB入力 |
| PA7 | UL_pwm | TIM1_CH1N | U相 ローサイドPWM出力 |
| PA8 | UH_pwm | TIM1_CH1 | U相 ハイサイドPWM出力 |
| PA9 | VH_pwm | TIM1_CH2 | V相 ハイサイドPWM出力 |
| PA10 | WH_pwm | TIM1_CH3 | W相 ハイサイドPWM出力 |
| PA11 | CAN_rx | CAN_RX | CAN通信 受信 |
| PA12 | CAN_tx | CAN_TX | CAN通信 送信 |
| PA13 | swdio | SYS_JTMS-SWDIO | SWDデバッグデータ |
| PA14 | swclk | SYS_JTCK-SWCLK | SWDデバッグクロック |
| PB0 | VL_pwm | TIM1_CH2N | V相 ローサイドPWM出力 |
| PB1 | WL_pwm | TIM1_CH3N | W相 ローサイドPWM出力 |
| PB4 | SW1 | GPIO_Input | プッシュスイッチ入力 |
| PB7 | - | GPIO_Input | 汎用入力 |

## 開発環境
* ツールチェーン: STM32CubeCLT (v1.21.0)
* ビルドツール: CMake, Ninja
* エディタ: Visual Studio Code
* デバッガ: ST-LINK (SWD接続)
* プログラマ: STM32CubeProgrammer

## ビルド方法
VS Code の CMake Tools 拡張機能を使用するか、コマンドラインから以下のコマンドでビルドします。

```powershell
cmake --build build/Debug
```

## マイコンへの書き込み方法
STM32CubeProgrammer CLI を使用してファームウェアをフラッシュメモリに書き込みます。

PowerShell での実行コマンド：

```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w "build\Debug\BLDC_frimware.elf" -v -rst
```

オプションの解説：
* -c port=SWD : ST-LINK 経由で SWD 接続します。
* -w <ファイルパス> : 指定したバイナリをフラッシュメモリに書き込みます。
* -v : 書き込みデータのベリファイ照合を実施します。
* -rst : 書き込み完了後にマイコンをソフトウェアリセットして実行を開始します。

接続エラー (Unable to get core ID など) が発生した場合は、配線の接触を確認するか、マイコンのリセットボタンを押しながらコマンドを実行してください。

## 制御仕様
* PWM制御: タイマ1 (TIM1) によるセンターアラインPWM生成、デッドタイム32クロック挿入
* アナログ計測: ADC1 および ADC2 によるU相 / V相電流値とホールセンサ信号のサンプリング
* インジケータ: タイマ2 (TIM2) PWM 出力によるLED調光制御
* 外部通信: CAN ペリフェラルによる外部制御装置との通信