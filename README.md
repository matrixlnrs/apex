# APEX - Mesureur de hauteur de saut

> Projet Système à Microprocesseur - ISEN Toulon (STM32)
> Carte NUCLEO-L152RE + shield capteurs X-NUCLEO-IKS01A3 + carte ISEN32

**APEX** (l'*apogée* = le point le plus haut d'un saut) mesure **à quelle hauteur tu sautes**.
On règle un **objectif** de hauteur avec un potentiomètre, on saute avec le boîtier, et
l'afficheur 7 segments indique la **hauteur atteinte** et le **temps de vol**. Si l'objectif
est dépassé, un **moteur vibre** pour célébrer la victoire.

---

## Principe

Impossible de mesurer une hauteur directement sans caméra ni GPS. APEX mesure à la place le
**temps de vol** (temps passé en l'air) :

- Dès que les pieds quittent le sol, le corps est en **chute libre** → l'accéléromètre ressent
  **~0 g** (apesanteur).
- À l'atterrissage, un **pic d'accélération** (impact) marque la fin du vol.
- La physique relie le temps de vol `t` à la hauteur :

```
h = g · t² / 8        (g = 9,81 m/s²,  t = temps de vol total en secondes)
```

*(Montée = descente = t/2, donc h = ½·g·(t/2)² = g·t²/8.)*

---

## Fonctionnement (machine à états)

```
SETUP ──[BTN3/BTN4]──► IDLE ──[chute libre]──► RUNNING ──[impact]──► RESULT ──► SETUP
```

1. **SETUP** - On tourne le potentiomètre **RV1** ; l'objectif (cm) s'affiche en direct
   (`-XX`). On valide avec **BTN3** ou **BTN4**.
2. **IDLE** - Prêt. L'accéléromètre est lu en continu ; le décollage est détecté quand
   `|a|` descend sous le seuil de chute libre (confirmé sur ≥ 15 ms).
3. **RUNNING** - En l'air. L'afficheur montre `----` (aucune écriture SPI pour ne pas
   perturber la détection). Le chrono (TIM6, 1 ms) tourne.
4. **RESULT** - L'impact arrête le chrono. On calcule la hauteur, on lit l'humidité
   ambiante (HTS221), on affiche **hauteur / temps / humidité** en alternance. Si
   `hauteur ≥ objectif` → **le moteur vibre 3×**.

---

## Périphériques utilisés

| Périphérique | Rôle | Broche(s) | Mode |
|---|---|---|---|
| **GPIO** (LEDs, boutons) | Boutons validation, LEDs | PC6 (BTN3), PC5 (BTN4), PA11/PA12 (BP1/BP2), PB1/2/10-15 (LEDs) | dont **interruption** (EXTI) |
| **TIMER TIM6** | Chrono du temps de vol (tick 1 ms) | - (timer interne) | **interruption** |
| **ADC** | Lecture du potentiomètre RV1 (objectif) | PA0 | polling |
| **UART2** | `printf` de debug vers le PC (115200 bauds) | PA2/PA3 | - |
| **I2C1** | Capteurs du shield | PB8 (SCL) / PB9 (SDA) | polling |
| **SPI1** | Afficheur 7 segments MAX7219 | PA5/PA6/PA7 + PA8 (CS) | polling |
| **GPIO sortie** | Moteur de vibration | PB4 | - |

**≥ 2 périphériques en interruption** : **TIM6** (chrono) + **EXTI boutons**.

### Capteurs du shield IKS01A3 (≥ 2 exigés)

| Capteur | Adresse I2C | Usage |
|---|---|---|
| **LSM6DSO** (accéléromètre) | `0xD6` | Détection décollage/atterrissage, calcul de hauteur |
| **HTS221** (humidité) | `0xBE` | Conditions ambiantes affichées au résultat |

---

## Détails techniques notables

- **Détection par seuils sur `|a|`** (magnitude de l'accélération en mg) :
  - Décollage : `|a| < 300 mg` maintenu ≥ 15 ms (anti-faux-déclenchement, `t_start` backdaté).
  - Atterrissage : `|a| > 1800 mg` (pic d'impact).
  - Validation du temps de vol (80–2000 ms) pour rejeter le bruit.
- **Échantillonnage rapide** : accéléromètre à **416 Hz**, I2C à **400 kHz**, lecture à
  chaque tour de boucle → on ne rate ni le creux de chute libre ni le pic d'impact.
- **TIM6** : `PSC = 31`, `ARR = 999` → IRQ toutes les **1 ms** ( ne jamais mettre ARR = 0).
- Anti-rebond des boutons basé sur `HAL_GetTick()` (SysTick, indépendant de TIM6).

### Réglages (dans `Core/Src/main.c`)

```c
#define FREEFALL_MG     300   // seuil de chute libre (décollage)
#define IMPACT_MG      1800   // seuil d'impact (atterrissage)
#define AIRTIME_MIN_MS   80   // temps de vol mini accepté
#define AIRTIME_MAX_MS 2000   // temps de vol maxi accepté
#define FF_CONFIRM_MS    15   // durée de confirmation de la chute libre
#define GOAL_MAX_CM      99   // objectif max (potentiomètre à fond)
```

---

## Compilation & flash

### Avec STM32CubeIDE (recommandé)
1. `File → Open Projects from File System…` → sélectionner ce dossier.
2. Build (marteau) puis Run/Debug pour flasher la NUCLEO via ST-Link (USB).
3. Terminal série : **115200 bauds, 8N1** sur le port COM ST-Link (VCP) pour voir les logs.

### En ligne de commande
```bash
# Toolchain ARM de CubeIDE dans le PATH, puis :
cd Debug
make -j4 all          # 'make all', pas 'make' seul
```
Le binaire produit est `Debug/APEX_PROJECT.elf`.

---

## Structure

```
Core/Src/main.c                 # application (machine à états, capteurs, détection, affichage)
Core/Src/stm32l1xx_it.c         # interruptions (TIM6, EXTI boutons)
Core/Src/max7219_Yncrea2.c/.h   # driver afficheur 7 segments (SPI)
Core/Inc/main.h                 # défs de broches + enum d'états
Config/CUBEMX.md                # notes de configuration CubeMX
```

---

## Utilisation rapide

1. Brancher la carte (USB ST-Link). Ouvrir un terminal série à 115200 bauds.
2. Au démarrage : `=== APEX ===` puis `Regle l'objectif avec RV1, puis valide (BTN3/BTN4).`
3. **Tourner RV1** → l'objectif s'affiche. **Appuyer BTN3/BTN4** pour valider.
4. **Sauter** avec le boîtier. Lire la hauteur / le temps sur l'afficheur.
5. Objectif atteint → le moteur vibre. Retour automatique au réglage.

---

## Auteurs

Projet réalisé dans le cadre du module Système à Microprocesseur - Julien GOMIS, Mathis LENORAIS--CLERE, Neil BEN-OTHMAN