# APEX — Notes techniques pour la soutenance

> Fiche de révision : chaque partie du code, comment elle marche, et les questions
> que le jury peut poser dessus. À relire la veille + 1 h avant.

---

## 1. Vue d'ensemble — qui fait quoi

```
Core/Src/main.c            → toute l'application (drivers capteurs, affichage, machine à états)
Core/Src/stm32l1xx_it.c    → les interruptions (TIM6 = tick 1 ms, EXTI = boutons BP1/BP2)
Core/Src/max7219_Yncrea2.c → driver de l'afficheur 7 segments (fourni en TP06, légèrement étendu)
Core/Inc/main.h            → broches + enum des 4 états
```

**Le fil rouge** : la boucle `while(1)` lit l'accéléromètre à chaque tour, calcule
`|a|` (la norme de l'accélération), et une machine à 4 états décide quoi en faire.
Le temps est fourni par TIM6 qui incrémente `tim_ms` toutes les millisecondes.

---

## 2. La machine à états (cœur du programme)

```
SETUP ──BTN3/BTN4──► IDLE ──|a|<300mg──► RUNNING ──|a|>1800mg──► RESULT ──► SETUP
```

| État | Ce qu'on y fait | Sortie |
|---|---|---|
| `APP_SETUP` | lit le potentiomètre (ADC), affiche l'objectif `-XX`, attend un bouton | appui BTN3/BTN4 |
| `APP_IDLE` | affiche `0.000`, surveille `|a|` | chute libre confirmée ≥ 15 ms |
| `APP_RUNNING` | affiche `----` figé, chrono en cours | impact, ou timeout 2 s |
| `APP_RESULT` | calcule h, lit l'humidité, affiche, vibre si victoire | automatique → SETUP |

**Le pattern `prev_state`** (question probable) : au début de chaque tour on copie
`state` dans `handling`, et à la fin on fait `prev_state = handling`. Le bloc
`if (prev_state != APP_X)` en tête de chaque état ne s'exécute donc **qu'une fois**,
à l'entrée dans l'état — c'est comme ça qu'on affiche le message une seule fois et
qu'on ne réécrit pas l'écran en boucle. On copie dans `handling` parce que `state`
peut changer *au milieu* du switch (par le code ou par une interruption bouton).

**Pourquoi `volatile` sur `state`, `tim_ms`, `t_start`, `hang_time`** : ces variables
sont modifiées dans des interruptions ET lues dans la boucle principale. `volatile`
interdit au compilateur de les mettre en cache dans un registre — sinon la boucle
pourrait ne jamais « voir » le changement fait par l'ISR.

---

## 3. La physique et la détection

### La formule
`h = g·t²/8`. D'où ça vient : montée = descente = t/2 chacune. La chute depuis le
sommet dure t/2 sans vitesse initiale : `h = ½·g·(t/2)² = g·t²/8`.
Dans le code (état RESULT) :
```c
float t_s  = hang_time / 1000.0f;               // ms → s
float h_cm = 9.81f * t_s * t_s / 8.0f * 100.0f; // m → cm
```
Ordres de grandeur à connaître : **500 ms → ~30 cm, 640 ms → ~50 cm**.

### La norme |a|
```c
mag = sqrt(mx² + my² + mz²)   // Pythagore en 3D
```
Pourquoi la norme et pas un seul axe ? Parce que le boîtier peut être tenu dans
n'importe quelle orientation — la norme est indépendante de l'orientation.
- Au repos : `|a| ≈ 1000 mg` (1 g, la gravité)
- En l'air (chute libre) : `|a| → ~0 mg`
- À l'impact : pic bref `> 1800 mg`

### La conversion en mg
Le capteur renvoie des entiers 16 bits signés. En pleine échelle ±2 g, la datasheet
donne **0.061 mg par LSB**. D'où `mx = ax * 61 / 1000` (astuce : entier ×61/1000
plutôt que flottant ×0.061).

### Les seuils et protections
| Constante | Valeur | Rôle |
|---|---|---|
| `FREEFALL_MG` | 300 | en dessous = chute libre |
| `IMPACT_MG` | 1800 | au dessus = impact |
| `FF_CONFIRM_MS` | 15 | la chute libre doit durer ≥ 15 ms (anti-bruit) |
| `AIRTIME_MIN_MS` | 80 | vol < 80 ms = faux déclenchement, ignoré |
| `AIRTIME_MAX_MS` | 2000 | vol > 2 s = pas d'impact détecté, abandon |

Subtilité à savoir expliquer : quand la chute libre est confirmée (au bout de 15 ms),
le chrono part de `ff_start` = l'instant où `|a|` est **passé** sous le seuil, pas
l'instant de la confirmation. Donc les 15 ms d'attente ne faussent pas la mesure.

---

## 4. Les périphériques, un par un

### TIM6 — le chronomètre (interruption n°1)
- Config : `PSC = 31`, `ARR = 999`. Horloge timer = 32 MHz.
  32 MHz / (31+1) = 1 MHz, puis 1 MHz / (999+1) = **1000 Hz → une IRQ par ms**.
- L'ISR appelle `HAL_TIM_IRQHandler` qui appelle notre callback :
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) tim_ms++;
}
```
- Démarré par `HAL_TIM_Base_Start_IT(&htim6)` dans `main()`.
- ⚠️ Anecdote de l'Affaire 1 : avec `ARR = 0`, le timer ne génère pas d'événement
  de mise à jour → `tim_ms` restait figé à 0 → tout ce qui dépendait du temps
  (anti-rebond des boutons inclus) semblait mort.

### Boutons BP1/BP2 — EXTI (interruption n°2)
- PA11/PA12, handler `EXTI15_10_IRQHandler` (les lignes EXTI 10 à 15 partagent
  un seul vecteur d'interruption).
- Dans l'ISR, on identifie quelle broche a déclenché avec
  `__HAL_GPIO_EXTI_GET_IT(pin)` **avant** que `HAL_GPIO_EXTI_IRQHandler()`
  n'efface le flag.
- Anti-rebond : on mémorise la date du dernier appui (`HAL_GetTick()`) et on
  ignore tout front à moins de 50 ms.
- Rôle : mode manuel de secours (BP1 démarre le chrono, BP2 l'arrête) — utile
  pour tester sans sauter.

### BTN3/BTN4 — GPIO en polling
- PC6/PC5, **actifs bas** (repos = 1 grâce au pull-up interne, appui = 0).
- Lus par scrutation dans l'état SETUP via `Button_Pressed()` qui détecte le
  **front** (passage relâché → appuyé) pour ne valider qu'une fois par appui.
- Pourquoi polling et pas interruption ? On n'en a besoin que dans SETUP, la
  boucle y passe des milliers de fois par seconde, c'est largement suffisant —
  et on a déjà nos 2 périphériques en IT (TIM6 + EXTI).

### ADC — le potentiomètre RV1 (PA0, canal 0)
```c
HAL_ADC_Start(&hadc);
HAL_ADC_PollForConversion(&hadc, 100);
raw = HAL_ADC_GetValue(&hadc);        // 0..4095 (12 bits)
HAL_ADC_Stop(&hadc);
objectif = raw * 99 / 4095;           // règle de trois → 0..99 cm
```
Conversion déclenchée par logiciel, une mesure à la fois (pas de mode continu).

### I2C1 — les capteurs (PB8 = SCL, PB9 = SDA, 400 kHz)
- Adresses **8 bits** (déjà décalées, format attendu par la HAL) :
  **LSM6DSO = 0xD6**, **HTS221 = 0xBE**. (En 7 bits ce serait 0x6B et 0x5F.)
- Lecture d'un registre = 2 transactions : `Transmit` de l'adresse du registre,
  puis `Receive` de la donnée.
- ⚠️ Affaire 2 : l'I2C était initialement sur PB6/PB7 (broches I2C valides du
  STM32) mais le shield est câblé sur PB8/PB9 via le connecteur Arduino. Le scan
  ne trouvait que les EEPROM de la carte ISEN32 (0xA0/0xA2), pas les capteurs.

### LSM6DSO — l'accéléromètre
Deux registres écrits à l'init :
- `CTRL1_XL (0x10) = 0x60` → **ODR 416 Hz**, pleine échelle ±2 g
  (les 4 bits de poids fort codent la fréquence : 0100=104 Hz, 0110=416 Hz)
- `CTRL3_C (0x12) = 0x04` → bit IF_INC : **auto-incrément** de l'adresse de
  registre, ce qui permet de lire X, Y, Z (6 octets) en une seule rafale depuis
  `OUTX_L_A (0x28)`.
Chaque axe arrive en 2 octets little-endian : `valeur = (haut << 8) | bas`.

### HTS221 — l'humidité (2e capteur du shield)
- `CTRL_REG1 (0x20) = 0x81` → bit PD (power-on) + ODR 1 Hz.
- Particularité : le capteur est **étalonné en usine**. On lit ses coefficients
  de calibration (H0_RH, H1_RH et les sorties brutes correspondantes H0_OUT,
  H1_OUT) une fois au démarrage, puis chaque mesure est une **interpolation
  linéaire** entre ces deux points de référence :
  `hum = H0_rh + (raw - H0_out) × (H1_rh - H0_rh) / (H1_out - H0_out)`
- Lu une fois par saut, dans RESULT ; affiché `H 45` et sur l'UART.

### SPI1 + MAX7219 — l'afficheur 7 segments
- SPI en maître, 8 bits, mode 0, CS logiciel sur PA8.
- Protocole MAX7219 : trames de 2 octets `[registre][donnée]`, verrouillées par
  le front du CS (LOAD). Registres digit 1..8 + registres de config (intensité,
  scan-limit, decode-mode, shutdown).
- Le driver (`max7219_Yncrea2.c`, fourni en TP06) travaille en mode « no decode » :
  chaque caractère passe par une table de police (`MAX7219_Font[]`) qui donne le
  masque des 7 segments. On a ajouté le caractère `-` à la table, et une fonction
  `MAX7219_DisplayCharDP()` qui allume en plus le **point décimal** (bit 7).
- Sur notre module, **digit 1 = à gauche** (on l'a découvert en voyant le temps
  s'afficher à l'envers).
- Formats affichés : objectif `-XX`, temps `S.mmm`, hauteur `XX.X`, humidité `H XX`.

### Moteur vibreur — GPIO simple (PB4)
`HAL_GPIO_WritePin` + `HAL_Delay` : 3 impulsions de 200 ms espacées de 150 ms.
(PB4 peut aussi faire du PWM via TIM3_CH1 — évolution possible pour moduler
l'intensité, mais pour une célébration binaire le GPIO suffit.)

### UART2 — le printf (115200 bauds, PA2/PA3)
La magie derrière `printf` : la libc appelle `_write()`, qui appelle notre
`__io_putchar()`, qui fait `HAL_UART_Transmit` caractère par caractère.
Câblé en interne vers le ST-Link → apparaît comme un port série USB sur le PC.

### I2C_BusRecover — la sécurité au boot
Si le MCU redémarre (flash, reset) au milieu d'une lecture I2C, l'esclave peut
rester bloqué en train de tenir SDA à 0 → le bus est mort. La procédure standard
de récupération : passer SCL en GPIO et envoyer jusqu'à 9 impulsions d'horloge
pour que l'esclave termine son octet et relâche SDA, puis générer une condition
de STOP, puis réinitialiser le périphérique I2C. On l'appelle une fois au boot.
(On l'a vécu en vrai : un log montrait `SDA=0` au démarrage.)

---

## 5. Les 3 affaires de debug — version technique précise

**Affaire 1 — ARR=0.** TIM6 configuré `PSC=31999, ARR=0`. Un timer STM32 compte
de 0 à ARR puis génère l'événement de mise à jour ; avec ARR=0 il ne génère rien
d'exploitable → `tim_ms` figé à 0. Or l'anti-rebond de BP1 testait
`(tim_ms - last) > 50` : toujours faux. Les boutons étaient électriquement
parfaits. **Fix** : `PSC=31, ARR=999` (32 MHz → 1 MHz → 1 kHz), et anti-rebond
rebasé sur `HAL_GetTick()` (SysTick, indépendant de TIM6).

**Affaire 2 — PB6/PB7 vs PB8/PB9.** Le scan I2C (boucle `HAL_I2C_IsDeviceReady`
sur les 127 adresses) trouvait 0xA0/0xA2 (EEPROM de l'ISEN32 sur PB6/PB7) mais
ni 0xD6 ni 0xBE. Le shield IKS01A3 passe par le connecteur Arduino = PB8/PB9.
**Fix** : changer les broches dans `HAL_I2C_MspInit()` (même AF4). Après : le scan
trouve 6 capteurs (0x32, 0x3C, 0x94, 0xBA, 0xBE, 0xD6).

**Affaire 3 — l'échantillonnage.** Version initiale : lecture accéléro toutes les
10 ms, ODR 104 Hz, affichage du chrono à chaque ms pendant le vol (4 écritures
SPI bloquantes). Le pic d'impact dure parfois < 10 ms → raté 1 fois sur 3, et
jusqu'à ±10 ms d'erreur sur t (h ∝ t² → ~14 % d'erreur sur h). **Fix** :
ODR 416 Hz + I2C 400 kHz + lecture à chaque tour de boucle (~0,2 ms) + afficheur
figé sur `----` pendant le vol + confirmation 15 ms de la chute libre.

---

## 6. Questions pièges possibles (au-delà de celles du discours)

**« Pourquoi 0xD6 alors que la datasheet dit 0x6B ? »**
0x6B est l'adresse 7 bits. La HAL attend l'adresse décalée d'un bit à gauche
(le bit 0 étant R/W) : 0x6B << 1 = 0xD6.

**« Pourquoi deux mesures de temps différentes (tim_ms et HAL_GetTick) ? »**
`tim_ms` (TIM6) est notre chronomètre de mesure. `HAL_GetTick()` (SysTick) sert
à l'anti-rebond des boutons — comme ça, même si TIM6 a un problème, les boutons
restent utilisables (leçon de l'Affaire 1).

**« Que se passe-t-il si on saute sans valider l'objectif ? »**
Rien : dans SETUP on ne surveille pas l'accéléromètre. Il faut valider pour
passer en IDLE. C'est un choix : pas de mesure sans objectif défini.

**« Pourquoi figer l'afficheur pendant le vol ? »**
Une écriture MAX7219 = 4 trames SPI + delays ≈ plusieurs centaines de µs à
quelques ms. Pendant ce temps la boucle ne lit pas l'accéléromètre → on peut
rater le pic d'impact. En vol, chaque microseconde de la boucle est consacrée
à la surveillance.

**« Votre anti-rebond de 50 ms, pourquoi 50 ? »**
Un rebond mécanique typique dure 1 à 10 ms ; 50 ms est une marge confortable
qui reste imperceptible pour l'utilisateur.

**« La HAL, c'est quoi exactement ? »**
Hardware Abstraction Layer de ST : des fonctions C qui écrivent dans les
registres du STM32 à notre place (`HAL_GPIO_WritePin` → registre BSRR, etc.).
Le squelette du projet est généré par STM32CubeMX depuis le fichier `.ioc` ;
notre code vit dans les blocs `/* USER CODE BEGIN/END */` pour survivre aux
régénérations.

**« Pourquoi le moteur ne vibre pas pendant le vol pour le feedback ? »**
Motor_Vibrate est bloquant (HAL_Delay) — pendant la vibration on ne mesure plus.
On ne l'utilise donc qu'en RESULT, quand la mesure est terminée.

**« Limite de votre système ? »**
Le seuil de chute libre suppose un vrai saut vertical. Un lancer du boîtier
donnerait aussi une "chute libre" → mesure absurde mais bornée par le garde-fou
des 2 s. Et la précision dépend de la netteté de l'impact à la réception.

---

## 7. Chiffres à connaître par cœur

| Quoi | Valeur |
|---|---|
| Horloge CPU | 32 MHz (HSI 16 MHz × PLL 6/3) |
| TIM6 | PSC 31, ARR 999 → 1 ms |
| ODR accéléromètre | 416 Hz, ±2 g, 0.061 mg/LSB |
| I2C | 400 kHz, LSM6DSO 0xD6, HTS221 0xBE |
| UART | 115200 bauds 8N1 |
| Seuils | 300 mg / 1800 mg, confirmation 15 ms, vol valide 80–2000 ms |
| Exemples h(t) | 500 ms → ~30 cm ; 640 ms → ~50 cm |
