/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "max7219_Yncrea2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* LSM6DSO (accelerometre) - adresse I2C 8 bits (deja decalee pour HAL) */
#define LSM6DSO_ADDR      0xD6
#define LSM6DSO_WHO_AM_I  0x0F   /* doit valoir 0x6C */
#define LSM6DSO_CTRL1_XL  0x10   /* config accelero (ODR + plage) */
#define LSM6DSO_CTRL3_C   0x12   /* auto-increment registres */
#define LSM6DSO_OUTX_L_A  0x28   /* debut des 6 octets X/Y/Z */
#define LSM6DSO_WAKE_UP_THS 0x5B /* seuil wake-up (impact) */
#define LSM6DSO_WAKE_UP_DUR 0x5C
#define LSM6DSO_FREE_FALL   0x5D /* seuil + duree chute libre */
#define LSM6DSO_MD1_CFG     0x5E /* routage vers INT1 */
#define LSM6DSO_MD2_CFG     0x5F /* routage vers INT2 */
#define LSM6DSO_ALL_INT_SRC 0x1A /* lecture pour acquitter les IT */

/* --- Detection de saut par seuils logiciels sur |a| (en mg) --- */
#define FREEFALL_MG     300   /* |a| sous ce seuil = chute libre => decollage */
#define IMPACT_MG      1800   /* |a| au dessus de ce seuil = impact => atterrissage */
#define AIRTIME_MIN_MS   80   /* rejette les faux declenchements (bruit) */
#define AIRTIME_MAX_MS 2000   /* garde-fou : au dela on annule la mesure */
#define GRAVITY_MG     1000   /* 1 g de reference */
#define FF_CONFIRM_MS    15   /* chute libre confirmee si |a| reste bas >= ce temps */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile AppState_t state   = APP_IDLE;
volatile uint32_t   t_start = 0;
volatile uint32_t   hang_time = 0;

/* Phase B1 : flags leves par les ISR LSM6DSO (validation detection) */
volatile uint8_t flag_takeoff = 0;   /* INT1 : chute libre (decollage) */
volatile uint8_t flag_landing = 0;   /* INT2 : impact (atterrissage) */

/* DIAG : compteurs d'IT pour detecter un emballement (PB3 = SWO partage ?) */
volatile uint32_t exti3_irq_count = 0;   /* INT2 / PB3 */
volatile uint32_t exti4_irq_count = 0;   /* INT1 / PB4 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
    return ch;
}

/* DIAG + RECUPERATION du bus I2C1 (PB8=SCL, PB9=SDA).
   A appeler AVANT toute transaction I2C. Lit l'etat des lignes au repos :
   une ligne a 0 au repos => bus bloque (esclave coince, pull-up absent, ou
   carte non alimentee). Puis genere jusqu'a 9 impulsions d'horloge sur SCL
   pour liberer un esclave qui tient SDA bas, suivies d'un STOP manuel. */
static void I2C_BusRecover(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Coupe le peripherique I2C pour reprendre la main sur les broches */
    HAL_I2C_DeInit(&hi2c1);

    /* PB8/PB9 en open-drain GPIO avec pull-up, niveau haut (relache) */
    g.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    int scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    int sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    printf("[I2C] Etat repos : SCL=%d SDA=%d (attendu 1/1)\r\n", scl, sda);

    if (sda == 0) {
        printf("[I2C] SDA bloque a 0 -> tentative de deblocage (9 clocks)\r\n");
        for (int i = 0; i < 9; i++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_Delay(1);
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 1) break;  /* SDA relache */
        }
        /* Condition de STOP : SDA passe de 0 a 1 pendant que SCL est haut */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* Remet le peripherique I2C en service (reconfigure PB8/PB9 en AF) */
    MX_I2C1_Init();
    printf("[I2C] Bus reinitialise\r\n");
}

static void I2C_Scan(void) {
    printf("--- Scan I2C ---\r\n");
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
            printf("Trouve : 0x%02X\r\n", addr << 1);
        }
    }
    printf("--- Fin scan ---\r\n");
}

volatile uint32_t tim_ms = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        tim_ms++;
    }
}

/* ===== LSM6DSO : acces I2C ===== */
static void LSM6DSO_Write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DSO_ADDR, buf, 2, 10);
}

static uint8_t LSM6DSO_Read(uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DSO_ADDR, &reg, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, LSM6DSO_ADDR, &val, 1, 10);
    return val;
}

/* Lit les 3 axes accelero (raw, LSB). Auto-increment active via CTRL3_C. */
static void LSM6DSO_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = LSM6DSO_OUTX_L_A;
    uint8_t d[6] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};  /* DIAG : sentinelle si jamais ecrasee */
    HAL_StatusTypeDef st_tx = HAL_I2C_Master_Transmit(&hi2c1, LSM6DSO_ADDR, &reg, 1, 10);
    HAL_StatusTypeDef st_rx = HAL_I2C_Master_Receive(&hi2c1, LSM6DSO_ADDR, d, 6, 10);
    if (st_tx != HAL_OK || st_rx != HAL_OK) {
        printf("[I2C] ERREUR lecture accel : tx=%d rx=%d err=0x%lx\r\n",
               st_tx, st_rx, (unsigned long)HAL_I2C_GetError(&hi2c1));
    }
    *ax = (int16_t)(d[1] << 8 | d[0]);
    *ay = (int16_t)(d[3] << 8 | d[2]);
    *az = (int16_t)(d[5] << 8 | d[4]);
}

/* Affiche un temps en millisecondes sur le MAX7219 au format S.mmm (secondes).
   Digit 1 = gauche ... digit 4 = droite sur ce module. */
static void Display_Time(uint32_t ms) {
    uint32_t sec  = ms / 1000;   // partie secondes
    uint32_t frac = ms % 1000;   // partie millisecondes (000-999)

    MAX7219_DisplayCharDP(1, '0' + (sec % 10), 1);      // secondes (unites) + point, a GAUCHE
    MAX7219_DisplayChar(2, '0' + (frac / 100) % 10);    // millisecondes (centaines)
    MAX7219_DisplayChar(3, '0' + (frac / 10) % 10);     // millisecondes (dizaines)
    MAX7219_DisplayChar(4, '0' + (frac % 10));          // millisecondes (unites), a DROITE
}

/* Affiche une hauteur de saut en cm sur le MAX7219.
   < 100 cm : format XX.X (point decimal sur le digit des unites de cm).
   >= 100 cm : format XXX (cm entiers, cas rare). */
static void Display_Height(float h_cm) {
    MAX7219_Clear();                       // digits inutilises => vraiment eteints
    if (h_cm < 0.0f) h_cm = 0.0f;

    if (h_cm >= 100.0f) {
        uint32_t cm = (uint32_t)(h_cm + 0.5f);
        if (cm > 999) cm = 999;
        MAX7219_DisplayChar(2, '0' + (cm / 100) % 10);  // centaines de cm
        MAX7219_DisplayChar(3, '0' + (cm / 10)  % 10);  // dizaines
        MAX7219_DisplayChar(4, '0' +  cm        % 10);  // unites
    } else {
        uint32_t t = (uint32_t)(h_cm * 10.0f + 0.5f);   // hauteur en dixiemes de cm
        MAX7219_DisplayChar  (2, '0' + (t / 100) % 10);     // dizaines de cm
        MAX7219_DisplayCharDP(3, '0' + (t / 10)  % 10, 1);  // unites de cm + point
        MAX7219_DisplayChar  (4, '0' +  t        % 10);     // dixiemes de cm
    }
}

/* Affiche "----" (indicateur de vol en cours). Ecrit une seule fois a l'entree
   de RUNNING : aucune ecriture SPI pendant le vol -> pas de jitter sur la detection. */
static void Display_Dashes(void) {
    MAX7219_DisplayChar(1, '-');
    MAX7219_DisplayChar(2, '-');
    MAX7219_DisplayChar(3, '-');
    MAX7219_DisplayChar(4, '-');
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  printf("=== VertiSense Ready ===\r\n");
  I2C_BusRecover();     /* diagnostic + deblocage du bus avant tout acces */
  I2C_Scan();

  MAX7219_Init();
  Display_Time(0);          // affiche 0.000 au demarrage

  HAL_TIM_Base_Start_IT(&htim6);
  printf("TIM6 OK\r\n");

  /* --- LSM6DSO : accelero + detection chute libre (INT1) / impact (INT2) --- */
  uint8_t who = LSM6DSO_Read(LSM6DSO_WHO_AM_I);
  printf("[LSM6DSO] WHO_AM_I = 0x%02X (attendu 0x6C)\r\n", who);
  LSM6DSO_Write(LSM6DSO_CTRL1_XL, 0x60);    // ODR 416 Hz, plage +/-2g (echantillonnage rapide)
  LSM6DSO_Write(LSM6DSO_CTRL3_C, 0x04);     // auto-increment des registres
  LSM6DSO_Write(LSM6DSO_FREE_FALL, 0x33);   // seuil chute libre ~312 mg, duree ~58 ms
  LSM6DSO_Write(LSM6DSO_WAKE_UP_THS, 0x08); // seuil impact = 8 x 62.5 mg = 500 mg
  LSM6DSO_Write(LSM6DSO_WAKE_UP_DUR, 0x00);
  LSM6DSO_Write(LSM6DSO_MD1_CFG, 0x10);     // chute libre -> INT1 (PB4)
  LSM6DSO_Write(LSM6DSO_MD2_CFG, 0x20);     // wake-up (impact) -> INT2 (PB3)
  LSM6DSO_Read(LSM6DSO_ALL_INT_SRC);        // purge des IT residuelles
  printf("[LSM6DSO] Detection chute libre / impact armee\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static AppState_t prev_state = APP_IDLE;

	  /* --- Echantillonnage accelero a CHAQUE tour de boucle (resolution maxi) :
	     avec ODR 416 Hz + I2C 400 kHz, une lecture prend ~0.2 ms -> on ne rate
	     plus ni le creux de chute libre ni le pic d'impact. --- */
	  static uint32_t last_log = 0;
	  static uint32_t mag = GRAVITY_MG;       // |a| courant en mg
	  static uint32_t ff_start = 0;           // debut de la fenetre de chute libre (0 = pas en chute libre)
	  {
	      int16_t ax, ay, az;
	      LSM6DSO_ReadAccel(&ax, &ay, &az);
	      /* conversion en mg : 0.061 mg/LSB a +/-2g */
	      int32_t mx = (int32_t)ax * 61 / 1000;
	      int32_t my = (int32_t)ay * 61 / 1000;
	      int32_t mz = (int32_t)az * 61 / 1000;
	      mag = (uint32_t)sqrtf((float)(mx*mx + my*my + mz*mz));
	  }

	  /* Log UART throttle 200 ms, UNIQUEMENT au repos : pendant le vol le printf
	     (~3 ms bloquant) jitterait la detection d'impact. */
	  if (state == APP_IDLE && HAL_GetTick() - last_log >= 200) {
	      last_log = HAL_GetTick();
	      printf("A: |a|=%lu mg\r\n", (unsigned long)mag);
	  }

	  /* Etat reellement traite a cette iteration : sert a detecter l'entree
	     dans un nouvel etat SANS se faire pieger par un changement de `state`
	     opere a l'interieur du switch (sinon le corps de RESULT est saute). */
	  AppState_t handling = state;

	  switch (state) {

	  case APP_IDLE:
	      if (prev_state != APP_IDLE) {
	          printf("IDLE - Saute pour demarrer\r\n");
	          Display_Time(0);
	          ff_start = 0;
	      }
	      /* Detection DECOLLAGE : chute libre confirmee sur >= FF_CONFIRM_MS.
	         t_start = instant ou |a| est PASSE sous le seuil (backdate) -> pas
	         de retard sur la mesure, et un simple pic de bruit ne declenche pas. */
	      if (mag < FREEFALL_MG) {
	          if (ff_start == 0) ff_start = tim_ms;          // debut du creux
	          if (tim_ms - ff_start >= FF_CONFIRM_MS) {
	              t_start = ff_start;                        // vrai instant de decollage
	              state = APP_RUNNING;
	              printf(">> TAKEOFF t_start=%lu\r\n", (unsigned long)t_start);
	          }
	      } else {
	          ff_start = 0;                                  // pas (ou plus) en chute libre
	      }
	      break;

	  case APP_RUNNING:
	      if (prev_state != APP_RUNNING) {
	          printf("RUNNING - en l'air...\r\n");
	          Display_Dashes();      // affichage fige "----", zero SPI pendant le vol
	      }
	      {
	          uint32_t elapsed = tim_ms - t_start;   // temps ecoule en ms
	          /* Detection ATTERRISSAGE : pic d'acceleration (impact) */
	          if (mag > IMPACT_MG) {
	              uint32_t air = tim_ms - t_start;
	              if (air >= AIRTIME_MIN_MS && air <= AIRTIME_MAX_MS) {
	                  hang_time = air;               // temps en l'air valide
	                  state = APP_RESULT;
	                  printf(">> LANDING air=%lu ms\r\n", (unsigned long)air);
	              } else {
	                  /* trop court/trop long => faux declenchement, on annule */
	                  printf("(faux declenchement : %lu ms ignore)\r\n",
	                         (unsigned long)air);
	                  state = APP_IDLE;
	              }
	          } else if (elapsed > AIRTIME_MAX_MS) {
	              /* jamais d'impact detecte => on abandonne la mesure */
	              printf("(timeout : pas d'impact detecte)\r\n");
	              state = APP_IDLE;
	          }
	      }
	      break;

	  case APP_RESULT:
	      if (prev_state != APP_RESULT) {
	          /* Hauteur du saut a partir du temps de vol :
	             temps montee = temps descente = t/2, donc
	             h = 1/2 * g * (t/2)^2 = g * t^2 / 8  (t en s, g = 9.81 m/s^2).
	             En cm : h_cm = 9.81 * (t_ms/1000)^2 / 8 * 100 */
	          float t_s  = hang_time / 1000.0f;
	          float h_cm = 9.81f * t_s * t_s / 8.0f * 100.0f;
	          printf("=== Temps en l'air : %lu ms  ->  hauteur : %d.%d cm ===\r\n",
	                 (unsigned long)hang_time,
	                 (int)h_cm, (int)(h_cm * 10.0f + 0.5f) % 10);
	          /* Alterne HAUTEUR (resultat principal) et TEMPS en l'air, en
	             terminant sur la hauteur. Formats distincts a l'ecran :
	             hauteur "42.7" (digit gauche eteint), temps "0.590". */
	          Display_Height(h_cm);   HAL_Delay(2000);
	          Display_Time(hang_time); HAL_Delay(2000);
	          Display_Height(h_cm);   HAL_Delay(2000);
	          state = APP_IDLE;
	      }
	      break;
	  }

	  prev_state = handling;   // etat traite, pas l'etat (eventuellement) modifie
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31;      /* 32 MHz / (31+1) = 1 MHz */
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;        /* 1 MHz / (999+1) = 1 kHz -> IRQ toutes les 1 ms */
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED3_Pin
                           LED4_Pin LED5_Pin LED6_Pin LED7_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BP1_Pin BP2_Pin */
  GPIO_InitStruct.Pin = BP1_Pin|BP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Boutons BP1/BP2 : actifs haut sur cette carte -> front MONTANT, pas de pull
     interne (pull-down externe present). Aligne sur le TP2_INTERRUPTIONS. */
  GPIO_InitStruct.Pin  = BP1_Pin | BP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* LSM6DSO INT1 (PB4, decollage) et INT2 (PB3, atterrissage) : front montant.
     PB3 etait le SWO -> reaffecte en EXTI (on garde le debug SWD PA13/PA14). */
  GPIO_InitStruct.Pin  = LSM_INT1_Pin | LSM_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
