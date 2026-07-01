# Config CubeMX — VertiSense v2
## Nouveau projet propre, architecture boutons + polling accéléromètre

---

## Créer le projet

`File` → `New STM32 Project`
Cherche **NUCLEO-L152RE** → sélectionne → `Next`
Donne un nom (ex: `VertiSense`) → `Finish`
CubeIDE ouvre automatiquement le `.ioc` dans CubeMX.

---

## ÉTAPE 1 — SYS

Panneau gauche → `System Core` → `SYS`

| Paramètre | Valeur |
|---|---|
| Debug | **Serial Wire** |
| Timebase Source | **SysTick** |

> ⚠️ Choisis bien **Serial Wire** et pas autre chose — sinon PB3 sera bloqué.

---

## ÉTAPE 2 — RCC

Panneau gauche → `System Core` → `RCC`

| Paramètre | Valeur |
|---|---|
| High Speed Clock (HSE) | **Disable** |
| Low Speed Clock (LSE) | **Crystal/Ceramic Resonator** |

Puis clique sur l'onglet **Clock Configuration** en haut.
Vérifie que **SYSCLK = 32 MHz**. Si ce n'est pas le cas, mets PLL comme source et règle pour obtenir 32 MHz.

---

## ÉTAPE 3 — USART2 (printf)

Panneau gauche → `Connectivity` → `USART2`

**Mode** → sélectionne **Asynchronous**
(les broches PA2/PA3 apparaissent automatiquement en vert)

Onglet **Parameter Settings** :

| Paramètre | Valeur |
|---|---|
| Baud Rate | **115200** |
| Word Length | **8 Bits** |
| Parity | **None** |
| Stop Bits | **1** |

---

## ÉTAPE 4 — SPI1 (MAX7219 afficheur)

Panneau gauche → `Connectivity` → `SPI1`

**Mode** → sélectionne **Full-Duplex Master**
**Hardware NSS Signal** → **Disable**

Onglet **Parameter Settings** :

| Paramètre | Valeur |
|---|---|
| Baud Rate Prescaler | **Prescaler 2** (→ 16 Mbit/s) |
| Data Size | **8 Bits** |
| First Bit | **MSB First** |
| Clock Polarity (CPOL) | **Low** |
| Clock Phase (CPHA) | **1 Edge** |

---

## ÉTAPE 5 — I2C1 (LSM6DSO + HTS221)

Panneau gauche → `Connectivity` → `I2C1`

**Mode** → sélectionne **I2C**

Onglet **Parameter Settings** :

| Paramètre | Valeur |
|---|---|
| I2C Speed Mode | **Standard Mode** |
| I2C Clock Speed | **100 000 Hz** |

> ⚠️ On choisit **Standard (100 kHz)** et pas Fast.
> C'est plus robuste pour l'empilement de 3 cartes.

---

## ÉTAPE 6 — TIM6 (chrono du saut)

Panneau gauche → `Timers` → `TIM6`

**Coche la case "Activated"** → les onglets apparaissent.

Onglet **Parameter Settings** :

| Paramètre | Valeur |
|---|---|
| Prescaler (PSC) | **31999** |
| Counter Mode | **Up** |
| Counter Period (ARR) | **0** |
| auto-reload preload | **Disable** |

> Explication : 32 MHz / 32000 = 1000 Hz → IRQ toutes les **1 ms**.
> TIM6 n'a pas de broche externe — il n'apparaîtra pas sur le schéma du chip, c'est normal.

Onglet **NVIC Settings** :
- Coche **TIM6 global interrupt** ✅

---

## ÉTAPE 7 — ADC (potentiomètre RV1)

Panneau gauche → `Analog` → `ADC`

Coche **IN0** uniquement (PA0 = RV1).

Onglet **Parameter Settings** :

| Paramètre | Valeur |
|---|---|
| Resolution | **12 Bits** |
| Data Alignment | **Right** |
| Continuous Conversion Mode | **Disabled** |
| End Of Conversion Selection | **Single Channel** |

---

## ÉTAPE 8 — GPIO Outputs (LEDs + CS MAX7219)

Pour chaque broche : clique dessus sur le schéma du chip
→ sélectionne `GPIO_Output`
→ clic droit → `Enter User Label` → tape le label.

| Broche | Label | Pull-up/down |
|---|---|---|
| PA8 | `SPI_CS` | **Pull-up** |
| PB1 | `LED0` | No pull |
| PB2 | `LED1` | No pull |
| PB10 | `LED2` | No pull |
| PB11 | `LED3` | No pull |
| PB12 | `LED4` | No pull |
| PB13 | `LED5` | No pull |
| PB14 | `LED6` | No pull |
| PB15 | `LED7` | No pull |

Pour régler le Pull-up de PA8 :
`System Core` → `GPIO` → trouve PA8 → colonne Pull-up/Pull-down → **Pull-up**

---

## ÉTAPE 9 — GPIO Inputs EXTI (boutons)

| Broche | Label | Mode | Pull | Trigger |
|---|---|---|---|---|
| PA11 | `BP1` | `GPIO_EXTI11` | **Pull-up** | **Falling Edge** |
| PA12 | `BP2` | `GPIO_EXTI12` | **Pull-up** | **Falling Edge** |

> BP1 = armer le système (IT 1)
> BP2 = remettre à zéro (IT 2)
> Falling Edge car les boutons ISEN32 sont actifs bas (appui = LOW)

Pour régler le mode et le trigger :
`System Core` → `GPIO` → trouve la broche
→ GPIO mode = `External Interrupt with Falling edge trigger detection`
→ GPIO Pull-up/Pull-down = `Pull-up`

---

## ÉTAPE 10 — NVIC (activer les interruptions)

Panneau gauche → `System Core` → `NVIC`

| Interruption | Cocher |
|---|---|
| TIM6 global interrupt | ✅ (déjà fait à l'étape 6) |
| EXTI line[15:10] interrupts | ✅ (couvre PA11 et PA12) |

---

## ÉTAPE 11 — Vérification finale

Contrôle visuel sur le schéma du chip (tout doit être jaune ou vert, rien en rouge) :

```
PA0  → ADC_IN0          (jaune)
PA2  → USART2_TX        (vert)
PA3  → USART2_RX        (vert)
PA5  → SPI1_SCK         (vert)
PA6  → SPI1_MISO        (vert)
PA7  → SPI1_MOSI        (vert)
PA8  → SPI_CS output    (jaune)
PA11 → BP1 EXTI11       (jaune)
PA12 → BP2 EXTI12       (jaune)
PA13 → SWD SWDIO        (vert, verrouillé)
PA14 → SWD SWCLK        (vert, verrouillé)
PB1  → LED0             (jaune)
PB2  → LED1             (jaune)
PB8  → I2C1_SCL         (vert)
PB9  → I2C1_SDA         (vert)
PB10 → LED2             (jaune)
PB11 → LED3             (jaune)
PB12 → LED4             (jaune)
PB13 → LED5             (jaune)
PB14 → LED6             (jaune)
PB15 → LED7             (jaune)
```

---

## ÉTAPE 12 — Générer le code

Clique **GENERATE CODE** (bouton bleu en haut à droite).

CubeIDE génère :
- `Core/Src/main.c` — avec toutes les `MX_XXX_Init()`
- `Core/Inc/main.h` — avec tous les `#define` de tes labels
- `Core/Src/stm32l1xx_it.c` — avec les squelettes d'ISR
- `Core/Src/stm32l1xx_hal_msp.c` — avec la config GPIO des périphériques

---

## ÉTAPE 13 — Modifier stm32l1xx_hal_msp.c (I2C pull-up)

Dans `Core/Src/stm32l1xx_hal_msp.c`, trouve `HAL_I2C_MspInit()`
et change **une seule ligne** :

```c
GPIO_InitStruct.Pull = GPIO_PULLUP;  // ← remplace GPIO_NOPULL
```

---

## Handles HAL générés (pour référence)

```c
ADC_HandleTypeDef  hadc;    // potentiomètre
I2C_HandleTypeDef  hi2c1;   // LSM6DSO + HTS221
SPI_HandleTypeDef  hspi1;   // MAX7219
TIM_HandleTypeDef  htim6;   // chrono 1ms
UART_HandleTypeDef huart2;  // printf
```

---

## Prochaine étape après génération

Envoie le `main.h` généré pour vérification,
puis on écrit le code **bloc par bloc**, en testant à chaque étape.
