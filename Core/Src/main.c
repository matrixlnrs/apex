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

/* HTS221 (temperature + humidite) - 2e capteur, adresse I2C 8 bits (0x5F<<1) */
#define HTS221_ADDR        0xBE
#define HTS221_WHO_AM_I    0x0F   /* doit valoir 0xBC */
#define HTS221_AV_CONF     0x10   /* moyennage T/H */
#define HTS221_CTRL_REG1   0x20   /* PD (actif), BDU, ODR */
#define HTS221_HUM_OUT_L   0x28   /* debut H_L,H_H,T_L,T_H (0x28..0x2B) */
#define HTS221_CALIB_START 0x30   /* 16 octets de calibration (0x30..0x3F) */
#define HTS221_AUTOINC     0x80   /* MSB du sous-registre = auto-increment I2C */

/* --- Detection de saut : on mesure la duree de la fenetre d'apesanteur,
   avec confirmation (anti-rebond) sur les deux bords pour ignorer les
   a-coups parasites (appareil tenu a la main). Seuils en mg. --- */
#define FREEFALL_MG     350   /* |a| sous ce seuil = apesanteur (en vol) */
#define LANDED_MG       700   /* |a| au dessus = revenu au sol (hysteresis) */
#define CONFIRM_MS       50   /* duree mini pour valider decollage/atterrissage */
#define AIRTIME_MIN_MS  120   /* rejette les micro-declenchements */
#define AIRTIME_MAX_MS 2000   /* garde-fou : au dela on annule la mesure */
#define GRAVITY_MG     1000   /* 1 g de reference */
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
volatile AppState_t state   = APP_IDLE;   /* etat de la mesure de saut */
volatile uint32_t   t_start = 0;          /* tim_ms au decollage */
volatile uint32_t   hang_time = 0;        /* temps en l'air mesure (ms) */
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

    int sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    if (sda == 0) {
        printf("[I2C] SDA bloque -> deblocage du bus\r\n");
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
    uint8_t d[6];
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DSO_ADDR, &reg, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, LSM6DSO_ADDR, d, 6, 10);
    *ax = (int16_t)(d[1] << 8 | d[0]);
    *ay = (int16_t)(d[3] << 8 | d[2]);
    *az = (int16_t)(d[5] << 8 | d[4]);
}

/* ===== HTS221 : temperature + humidite (2e capteur) ===== */
/* Coefficients de calibration usine, lus une fois a l'init. */
static float   hts_T0_degC, hts_T1_degC, hts_H0_rH, hts_H1_rH;
static int16_t hts_T0_OUT,  hts_T1_OUT,  hts_H0_OUT, hts_H1_OUT;

static void HTS221_Write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR, buf, 2, 10);
}

static uint8_t HTS221_Read(uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR, &reg, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, HTS221_ADDR, &val, 1, 10);
    return val;
}

/* Lecture multi-octets : le HTS221 exige le bit MSB du sous-registre pour
   l'auto-increment (sinon on relit toujours le meme registre). */
static void HTS221_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t n) {
    uint8_t r = reg | HTS221_AUTOINC;
    HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR, &r, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, HTS221_ADDR, buf, n, 10);
}

static void HTS221_Init(void) {
    uint8_t who = HTS221_Read(HTS221_WHO_AM_I);
    printf("Capteur 2 - Temp/Humidite HTS221 : %s\r\n",
           (who == 0xBC) ? "OK" : "ABSENT");

    HTS221_Write(HTS221_AV_CONF,   0x1B);   // moyennage par defaut (T=16, H=32)
    HTS221_Write(HTS221_CTRL_REG1, 0x85);   // PD=1 (actif), BDU=1, ODR=1 Hz

    /* Lecture des 16 octets de calibration (0x30..0x3F) */
    uint8_t c[16];
    HTS221_ReadMulti(HTS221_CALIB_START, c, 16);

    uint16_t T0_x8 = c[2] | ((uint16_t)(c[5] & 0x03) << 8);  // 10 bits
    uint16_t T1_x8 = c[3] | ((uint16_t)(c[5] & 0x0C) << 6);
    hts_H0_rH  = c[0] / 2.0f;
    hts_H1_rH  = c[1] / 2.0f;
    hts_T0_degC = T0_x8 / 8.0f;
    hts_T1_degC = T1_x8 / 8.0f;
    hts_H0_OUT = (int16_t)(c[6]  | (c[7]  << 8));
    hts_H1_OUT = (int16_t)(c[10] | (c[11] << 8));
    hts_T0_OUT = (int16_t)(c[12] | (c[13] << 8));
    hts_T1_OUT = (int16_t)(c[14] | (c[15] << 8));
}

/* Renvoie la temperature (degC) et l'humidite relative (%) calibrees. */
static void HTS221_Read_TH(float *temp_c, float *hum_pct) {
    uint8_t d[4];
    HTS221_ReadMulti(HTS221_HUM_OUT_L, d, 4);   // H_L,H_H,T_L,T_H
    int16_t H_OUT = (int16_t)(d[0] | (d[1] << 8));
    int16_t T_OUT = (int16_t)(d[2] | (d[3] << 8));

    /* Interpolation lineaire entre les 2 points de calibration usine */
    *hum_pct = hts_H0_rH + (float)(H_OUT - hts_H0_OUT) *
               (hts_H1_rH - hts_H0_rH) / (float)(hts_H1_OUT - hts_H0_OUT);
    *temp_c  = hts_T0_degC + (float)(T_OUT - hts_T0_OUT) *
               (hts_T1_degC - hts_T0_degC) / (float)(hts_T1_OUT - hts_T0_OUT);
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
  printf("\r\n=== VertiSense - mesureur de saut ===\r\n");

  I2C_BusRecover();     /* deblocage du bus I2C avant tout acces */

  MAX7219_Init();
  Display_Time(0);          // affiche 0.000 au demarrage
  HAL_TIM_Base_Start_IT(&htim6);

  /* --- Capteur 1 : LSM6DSO (accelerometre, detection de saut) --- */
  uint8_t who = LSM6DSO_Read(LSM6DSO_WHO_AM_I);
  printf("Capteur 1 - Accelerometre LSM6DSO : %s\r\n",
         (who == 0x6C) ? "OK" : "ABSENT");
  LSM6DSO_Write(LSM6DSO_CTRL1_XL, 0x40);    // ODR 104 Hz, plage +/-2g
  LSM6DSO_Write(LSM6DSO_CTRL3_C, 0x04);     // auto-increment des registres

  /* --- Capteur 2 : HTS221 (temperature + humidite) --- */
  HTS221_Init();

  printf("Pret - saute !\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static AppState_t prev_state = APP_IDLE;
	  static uint32_t   last_shown = 0xFFFFFFFF;

	  /* --- Echantillonnage accelero ~10 ms (aligne sur l'ODR 104 Hz),
	     utilise par la detection de saut (|a| courant dans `mag`). --- */
	  static uint32_t last_acc = 0;
	  static uint32_t mag = GRAVITY_MG;
	  if (HAL_GetTick() - last_acc >= 10) {
	      last_acc = HAL_GetTick();
	      int16_t ax, ay, az;
	      LSM6DSO_ReadAccel(&ax, &ay, &az);
	      /* conversion en mg : 0.061 mg/LSB a +/-2g */
	      int32_t mx = (int32_t)ax * 61 / 1000;
	      int32_t my = (int32_t)ay * 61 / 1000;
	      int32_t mz = (int32_t)az * 61 / 1000;
	      mag = (uint32_t)sqrtf((float)(mx*mx + my*my + mz*mz));

	      /* === Detection de saut (fenetre d'apesanteur + anti-rebond) ===
	         low_since / high_since = instant (tim_ms) ou |a| est passe en
	         continu sous FREEFALL / au dessus de LANDED. Remis a 0 des que la
	         condition n'est plus vraie => un a-coup bref ne valide rien. */
	      static uint32_t low_since  = 0;
	      static uint32_t high_since = 0;
	      if (mag < FREEFALL_MG) { if (!low_since)  low_since  = tim_ms; } else low_since  = 0;
	      if (mag > LANDED_MG)   { if (!high_since) high_since = tim_ms; } else high_since = 0;

	      if (state == APP_IDLE) {
	          /* DECOLLAGE : apesanteur maintenue >= CONFIRM_MS.
	             Le vol commence au debut de l'apesanteur (low_since). */
	          if (low_since && (tim_ms - low_since) >= CONFIRM_MS) {
	              t_start = low_since;
	              state = APP_RUNNING;
	          }
	      } else if (state == APP_RUNNING) {
	          uint32_t elapsed = tim_ms - t_start;
	          /* ATTERRISSAGE : retour au sol maintenu >= CONFIRM_MS, apres un
	             vrai vol. Le vol se termine au debut du retour au sol. */
	          if (high_since && (tim_ms - high_since) >= CONFIRM_MS
	                         && (high_since - t_start) >= AIRTIME_MIN_MS) {
	              uint32_t air = high_since - t_start;
	              if (air <= AIRTIME_MAX_MS) { hang_time = air; state = APP_RESULT; }
	              else                        state = APP_IDLE;
	          } else if (elapsed > AIRTIME_MAX_MS) {
	              state = APP_IDLE;          /* jamais d'atterrissage franc */
	          }
	      }
	  }

	  /* --- 2e capteur : HTS221 temperature + humidite, toutes les 5 s.
	     Uniquement au repos : ne jamais voler de temps I2C pendant un saut. --- */
	  static uint32_t last_th = 0;
	  if (state == APP_IDLE && HAL_GetTick() - last_th >= 5000) {
	      last_th = HAL_GetTick();
	      float t_c, h_p;
	      HTS221_Read_TH(&t_c, &h_p);
	      int t10 = (int)(t_c * 10.0f + 0.5f);   // dixiemes de degre
	      int h10 = (int)(h_p * 10.0f + 0.5f);   // dixiemes de %
	      printf("[HTS221] T=%d.%d C   H=%d.%d %%rH\r\n",
	             t10 / 10, t10 % 10, h10 / 10, h10 % 10);
	  }

	  /* Etat reellement traite a cette iteration : sert a detecter l'entree
	     dans un nouvel etat SANS se faire pieger par un changement de `state`
	     opere a l'interieur du switch (sinon le corps de RESULT est saute). */
	  AppState_t handling = state;

	  switch (state) {

	  case APP_IDLE:
	      if (prev_state != APP_IDLE) {
	          Display_Time(0);
	          last_shown = 0;
	      }
	      break;

	  case APP_RUNNING:
	      {
	          uint32_t elapsed = tim_ms - t_start;   // temps ecoule en ms
	          if (elapsed != last_shown) {
	              Display_Time(elapsed);             // mise a jour en direct
	              last_shown = elapsed;
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
	          int h10 = (int)(h_cm * 10.0f + 0.5f);   // hauteur en dixiemes de cm
	          printf("Saut : %lu ms en l'air  ->  %d.%d cm\r\n",
	                 (unsigned long)hang_time, h10 / 10, h10 % 10);
	          /* Alterne HAUTEUR (resultat principal) et TEMPS en l'air, en
	             terminant sur la hauteur. Formats distincts a l'ecran :
	             hauteur "42.7" (digit gauche eteint), temps "0.590". */
	          Display_Height(h_cm);   HAL_Delay(2000);
	          Display_Time(hang_time); HAL_Delay(2000);
	          Display_Height(h_cm);   HAL_Delay(2000);
	          printf("Pret - saute !\r\n");
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
  hi2c1.Init.ClockSpeed = 100000;
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
