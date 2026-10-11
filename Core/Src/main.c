/* USER CODE BEGIN Header */
/*
 * BLDC モータ制御ファームウェア メイン処理
 *
 * オープンループの正弦波駆動で正転5秒と逆転5秒を交互に繰り返す。
 * 発熱を抑えるため、以下の対策を入れている。
 *   - 周波数に比例して電圧を上げる V/f 制御で低速時の電圧を下げる
 *   - 回転方向の切り替え前後で加減速して突入電流を防ぐ
 *   - 電流センサの値を見て振幅を絞る電流制限 (しきい値を決めるまでは無効)
 *   - エラー発生時は PWM 出力を止めて MOSFET をすべて OFF にする
 */
/* USER CODE END Header */
#include "main.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* TIM1 の自動リロード値 (64MHz センターアライン で 20kHz) */
#define PWM_ARR 1599U

#define PI_F 3.14159265f
#define TWO_PI_F (2.0f * PI_F)



/* クローズドループ定常時の変調振幅 (0.5 で電源電圧いっぱい) */
#define AMP_RUN 0.090f

/* 停止付近で使う最低振幅 */
#define AMP_BOOST 0.030f

/* 片方向あたりの回転時間 [ms] */
#define DIR_PERIOD_MS 5000U

/* 加速と減速にかける時間 [ms] */
#define RAMP_MS 800U

/* 電流制限を使うときは 1 にする current_peak_max
 * を見てしきい値を決めてから有効にする */
#define ENABLE_CURRENT_LIMIT 0

/* 電流制限のしきい値 [ADC カウント] オフセットからの変化量で判定する */
#define CUR_LIMIT_COUNTS 400

/* 電流制限で絞れる振幅倍率の下限 */
#define LIMIT_GAIN_MIN 0.3f

/* 電流オフセット校正のサンプル数 */
#define OFFSET_SAMPLES 64U

/* LED の明るさ (TIM2 の ARR は 999) */
#define LED_DUTY 500U
/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* デバッガで確認するための ADC 生値 */
volatile uint16_t amp_u = 0;
volatile uint16_t amp_v = 0;
volatile uint16_t hall_a = 0;
volatile uint16_t hall_b = 0;

/* 電流ゼロ時の ADC 値 起動時に校正する */
volatile uint16_t offset_u = 2048;
volatile uint16_t offset_v = 2048;

/* 3相のうち最大の電流 [ADC カウント] */
volatile int32_t current_peak = 0;

/* 起動してからの current_peak の最大値 しきい値決めに使う */
volatile int32_t current_peak_max = 0;

/* 電流制限による振幅倍率 1.0 で制限なし */
volatile float limit_gain = 1.0f;

/* センサ校正パラメータ */
volatile float hall_a_center = 1750.0f;
volatile float hall_b_center = 1750.0f;
volatile float sensor_zero_offset = 0.0f;
volatile float sensor_dir = 1.0f;

/* リアルタイムに観測するローター電気角 [rad] */
volatile float rotor_elec_angle = 0.0f;

/* 印加する電圧振幅 */
volatile float cmd_amp = 0.0f;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t ADC1_Read(void);
static uint16_t ADC2_ReadChannel(uint32_t channel);
static void Calibrate_Current_Offset(void);
static float Speed_Ratio(uint32_t elapsed_ms);
static void Set_Phase_Duty(float theta, float amp);
static void Set_Direction_Led(int8_t dir);
static void Update_Current_Limit(void);
static void DWT_Init(void);
static void Delay_Us(uint32_t us);
static void Play_Beep(uint32_t freq_hz, uint32_t duration_ms, float volume);
static void Play_Startup_Tune(void);
static void Calibrate_Alignment(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* ADC の自己校正 ADC を有効にする前に行う必要がある */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  /* PWM を出す前の電流ゼロの状態でオフセットを測る */
  Calibrate_Current_Offset();

  /* LED 用 PWM の開始 CH1 が青 CH2 が赤 CH3 が緑 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  /* 3相とも 50% から始めて線間電圧ゼロの状態で出力を開始する */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_ARR / 2U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_ARR / 2U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_ARR / 2U);

  /* ハイサイドとローサイドの相補 PWM を開始 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  /* 高精度ウェイト用 DWT カウンタの初期化 */
  DWT_Init();

  /* モーターから起動メロディを鳴らす */
  Play_Startup_Tune();

  /* ローターの初期位置キャリブレーションとセンサ極性自動判定 */
  Calibrate_Alignment();
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  int8_t dir = 1;     /* 回転方向 1 が正転 -1 が逆転 */
  uint32_t phase_start = HAL_GetTick();
  uint32_t last_tick = phase_start;

  Set_Direction_Led(dir);

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1. エンコーダ (HALL_A, HALL_B) からローターの現在電気角を計算 */
    hall_a = ADC2_ReadChannel(ADC_CHANNEL_2);
    hall_b = ADC2_ReadChannel(ADC_CHANNEL_3);

    float sa = (float)hall_a - hall_a_center;
    float sb = (float)hall_b - hall_b_center;
    float raw_angle = atan2f(sb, sa);

    /* ゼロ点補正と回転方向を反映したローター電気角 */
    float theta_rotor = (raw_angle - sensor_zero_offset) * sensor_dir;
    rotor_elec_angle = theta_rotor;

    /* 2. 1ms ごとのタイマ処理 (加減速・5秒ごとの反転・電流計測) */
    uint32_t now = HAL_GetTick();
    if (now != last_tick) {
      last_tick = now;

      /* 5秒たったら回転方向を反転する */
      uint32_t elapsed = now - phase_start;
      if (elapsed >= DIR_PERIOD_MS) {
        dir = (int8_t)-dir;
        phase_start = now;
        elapsed = 0;
        Set_Direction_Led(dir);
      }

      /* 加減速を含めた速度比から印加電圧振幅を決定 */
      float ratio = Speed_Ratio(elapsed);
      cmd_amp = (AMP_BOOST + (AMP_RUN - AMP_BOOST) * ratio) * limit_gain;

      /* 電流の読み取りと過電流監視 */
      amp_u = ADC1_Read();
      amp_v = ADC2_ReadChannel(ADC_CHANNEL_1);
      Update_Current_Limit();
    }

    /* 3. ローター磁極に対して 90 度進んだ位相に電圧を印加 (クローズドループ駆動) */
    float theta_v;
    if (dir > 0) {
      theta_v = theta_rotor + (PI_F / 2.0f);
    } else {
      theta_v = theta_rotor - (PI_F / 2.0f);
    }

    /* 3相 PWM を即時更新 */
    Set_Phase_Duty(theta_v, cmd_amp);
  }
  /* USER CODE END 3 */
}

/* システムクロック設定 HSI と PLL で 64MHz */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV4;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* ADC1 初期化 PA3 の U相電流を読む */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/* ADC2 初期化 V相電流とホールセンサを切り替えて読む */
static void MX_ADC2_Init(void) {

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/* CAN 初期化 */
static void MX_CAN_Init(void) {

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
}

/* TIM1 初期化 モータ駆動用の3相相補 PWM */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 32;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/* TIM2 初期化 フルカラー LED 用 PWM */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/* GPIO 初期化 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ADC1 で U相電流を1回読む */
static uint16_t ADC1_Read(void) {
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
    Error_Handler();
  }
  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return value;
}

/* ADC2 のチャンネルを切り替えて1回読む */
static uint16_t ADC2_ReadChannel(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};

  HAL_ADC_Stop(&hadc2);

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADC_Start(&hadc2);

  if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK) {
    Error_Handler();
  }

  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc2);

  HAL_ADC_Stop(&hadc2);

  return value;
}

/* モータに電流が流れていない状態で電流センサのゼロ点を測る */
static void Calibrate_Current_Offset(void) {
  uint32_t sum_u = 0;
  uint32_t sum_v = 0;

  for (uint32_t i = 0; i < OFFSET_SAMPLES; i++) {
    sum_u += ADC1_Read();
    sum_v += ADC2_ReadChannel(ADC_CHANNEL_1);
  }

  offset_u = (uint16_t)(sum_u / OFFSET_SAMPLES);
  offset_v = (uint16_t)(sum_v / OFFSET_SAMPLES);
}

/*
 * 回転開始からの経過時間に応じた速度比を返す
 * 最初の RAMP_MS で 0 から 1 まで加速し、最後の RAMP_MS で 0 まで減速する
 */
static float Speed_Ratio(uint32_t elapsed_ms) {
  if (elapsed_ms < RAMP_MS) {
    return (float)elapsed_ms / (float)RAMP_MS;
  }
  if (elapsed_ms > DIR_PERIOD_MS - RAMP_MS) {
    return (float)(DIR_PERIOD_MS - elapsed_ms) / (float)RAMP_MS;
  }
  return 1.0f;
}



/* 電気角と振幅から3相の PWM デューティを設定する */
static void Set_Phase_Duty(float theta, float amp) {
  float du = 0.5f + amp * sinf(theta);
  float dv = 0.5f + amp * sinf(theta - TWO_PI_F / 3.0f);
  float dw = 0.5f + amp * sinf(theta + TWO_PI_F / 3.0f);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(du * PWM_ARR));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * PWM_ARR));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dw * PWM_ARR));
}

/* 回転方向を LED で表示する 正転は赤 逆転は青 */
static void Set_Direction_Led(int8_t dir) {
  if (dir > 0) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_DUTY);
  } else {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_DUTY);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  }
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}

/*
 * 電流が流れすぎていたら振幅倍率を下げ、収まっていればゆっくり戻す
 * W相は U相と V相の和の符号反転で求める
 */
static void Update_Current_Limit(void) {
  int32_t iu = (int32_t)amp_u - (int32_t)offset_u;
  int32_t iv = (int32_t)amp_v - (int32_t)offset_v;
  int32_t iw = -(iu + iv);

  int32_t peak = (iu < 0) ? -iu : iu;
  int32_t abs_v = (iv < 0) ? -iv : iv;
  int32_t abs_w = (iw < 0) ? -iw : iw;
  if (abs_v > peak) {
    peak = abs_v;
  }
  if (abs_w > peak) {
    peak = abs_w;
  }
  current_peak = peak;
  if (peak > current_peak_max) {
    current_peak_max = peak;
  }

#if ENABLE_CURRENT_LIMIT
  float gain = limit_gain;
  if (peak > CUR_LIMIT_COUNTS) {
    gain -= 0.02f;
    if (gain < LIMIT_GAIN_MIN) {
      gain = LIMIT_GAIN_MIN;
    }
  } else if (gain < 1.0f) {
    gain += 0.002f;
    if (gain > 1.0f) {
      gain = 1.0f;
    }
  }
  limit_gain = gain;
#endif
}

/* DWT サイクルカウンタを有効にする */
static void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* マイクロ秒単位の待機 */
static void Delay_Us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < cycles) {
  }
}

/* モーターのコイルを高周波で振動させて音を鳴らす */
static void Play_Beep(uint32_t freq_hz, uint32_t duration_ms, float volume) {
  if (freq_hz == 0 || volume <= 0.0f) {
    Set_Phase_Duty(0.0f, 0.0f);
    HAL_Delay(duration_ms);
    return;
  }
  uint32_t half_period_us = 500000U / freq_hz;
  uint32_t cycles = (duration_ms * 1000U) / (half_period_us * 2U);
  for (uint32_t i = 0; i < cycles; i++) {
    Set_Phase_Duty(0.0f, volume);
    Delay_Us(half_period_us);
    Set_Phase_Duty(PI_F, volume);
    Delay_Us(half_period_us);
  }
  Set_Phase_Duty(0.0f, 0.0f);
}

/* 起動時の確認メロディ (ド・ミ・ソ・高ド) */
static void Play_Startup_Tune(void) {
  Play_Beep(1046, 100, 0.02f);
  HAL_Delay(20);
  Play_Beep(1318, 100, 0.02f);
  HAL_Delay(20);
  Play_Beep(1568, 100, 0.02f);
  HAL_Delay(20);
  Play_Beep(2093, 180, 0.025f);
  HAL_Delay(100);
}

/* ローターの初期位置キャリブレーションとセンサ極性自動判定 */
static void Calibrate_Alignment(void) {
  /* LED を緑色に点灯してキャリブレーション中を示す */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_DUTY);

  /* 1. 電気角 0 度に電圧を印加して静止位置に引き込む (600ms) */
  Set_Phase_Duty(0.0f, AMP_BOOST);
  HAL_Delay(600);

  /* 電気角 0 度でのセンサ角度を測定 */
  uint32_t sum_a = 0;
  uint32_t sum_b = 0;
  for (int i = 0; i < 32; i++) {
    sum_a += ADC2_ReadChannel(ADC_CHANNEL_2);
    sum_b += ADC2_ReadChannel(ADC_CHANNEL_3);
  }
  float a0 = (float)(sum_a / 32U) - hall_a_center;
  float b0 = (float)(sum_b / 32U) - hall_b_center;
  float ang0 = atan2f(b0, a0);

  /* 2. 電気角 60 度 (PI/3) に動かしてセンサの回転方向を自動判定 (400ms) */
  Set_Phase_Duty(PI_F / 3.0f, AMP_BOOST);
  HAL_Delay(400);

  sum_a = 0;
  sum_b = 0;
  for (int i = 0; i < 32; i++) {
    sum_a += ADC2_ReadChannel(ADC_CHANNEL_2);
    sum_b += ADC2_ReadChannel(ADC_CHANNEL_3);
  }
  float a1 = (float)(sum_a / 32U) - hall_a_center;
  float b1 = (float)(sum_b / 32U) - hall_b_center;
  float ang1 = atan2f(b1, a1);

  /* 角度変化の差分からセンサ極性を判定 */
  float diff = ang1 - ang0;
  if (diff < -PI_F) diff += TWO_PI_F;
  if (diff > PI_F)  diff -= TWO_PI_F;
  sensor_dir = (diff >= 0.0f) ? 1.0f : -1.0f;

  /* 3. 再び電気角 0 度に戻して基準原点を確定 */
  Set_Phase_Duty(0.0f, AMP_BOOST);
  HAL_Delay(300);

  sum_a = 0;
  sum_b = 0;
  for (int i = 0; i < 32; i++) {
    sum_a += ADC2_ReadChannel(ADC_CHANNEL_2);
    sum_b += ADC2_ReadChannel(ADC_CHANNEL_3);
  }
  a0 = (float)(sum_a / 32U) - hall_a_center;
  b0 = (float)(sum_b / 32U) - hall_b_center;
  sensor_zero_offset = atan2f(b0, a0);

  /* アライメント完了音 (ピッ、ピッ) */
  Play_Beep(2500, 80, 0.02f);
  HAL_Delay(50);
  Play_Beep(2500, 80, 0.02f);
  HAL_Delay(150);
}

/* USER CODE END 4 */

/* エラー発生時の処理 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* PWM 出力を止めて MOSFET をすべて OFF にし、過熱を防ぐ */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}
#endif