# APEX — Texte de soutenance (15–20 minutes)

> **Mode d'emploi** : chaque section correspond à une slide. Les durées sont indicatives —
> répète une fois chrono en main pour caler ton rythme. Les passages entre [crochets]
> sont des indications scéniques, ne les lis pas. Parle lentement : un débit posé
> fait plus expert qu'un débit rapide.

---

## SLIDE 0 — Titre (1 min 30)

Bonjour à tous, merci de nous accueillir.

Notre projet s'appelle **APEX**. En anglais, l'apex, c'est le point le plus haut —
le sommet d'une trajectoire. Et c'est exactement ce que fait notre dispositif :
il mesure le point le plus haut de votre saut.

[Montre la trajectoire à l'écran]

Vous voyez cette courbe : un décollage, une montée, un sommet, une descente,
une réception. C'est la trajectoire de n'importe quel saut vertical. Notre boîtier,
lui, ne voit rien de tout ça. Il ne voit ni le sol, ni votre corps, ni la hauteur.
Et pourtant, à la fin du saut, il affiche votre détente en centimètres.

Comment ? C'est ce que nous allons vous expliquer dans les quinze prochaines minutes,
avec un détour par la physique, un peu d'architecture STM32, et surtout — parce que
c'est ça, la vraie vie d'un projet embarqué — une enquête de debug en trois actes.

[Transition] Commençons par le concept.

---

## SLIDE 1 — L'application (2 min 30)

L'idée d'APEX est simple : c'est un juge d'explosivité de poche.

Le scénario d'usage tient en quatre étapes. **Régler** : vous tournez un potentiomètre
pour fixer un objectif de hauteur — disons 40 centimètres — et l'afficheur le montre
en temps réel. **Sauter** : vous prenez le boîtier avec vous et vous sautez le plus
haut possible. **Mesurer** : à l'atterrissage, l'afficheur sept segments annonce
votre hauteur réelle. Et **vibrer** : si vous avez dépassé votre objectif, un moteur
vibre pour célébrer la victoire. C'est immédiat, c'est physique, c'est motivant.

Maintenant, la question que tout le monde se pose : comment mesurer une hauteur
de saut **sans GPS, sans caméra, sans capteur de distance** ?

La réponse tient dans une intuition physique magnifique. Pendant que vous êtes
en l'air — entre le moment où vos pieds quittent le sol et le moment où ils le
retouchent — vous êtes en **chute libre**. Vous montez, puis vous descendez, mais
du point de vue de la physique, c'est le même état : vous ne subissez que la gravité.

Et un accéléromètre embarqué, dans cet état, ressent quelque chose de très
particulier : **presque zéro g**. L'apesanteur. La même chose que ressentent
les astronautes dans la Station spatiale.

Notre pari, c'est donc de **chronométrer ce silence**. On mesure la durée pendant
laquelle l'accéléromètre ne ressent presque plus rien — c'est le temps de vol —
et la physique nous donne directement la hauteur. Plus vous restez longtemps
en l'air, plus vous avez sauté haut. C'est aussi simple, et aussi élégant, que ça.

À l'écran, vous voyez nos trois chiffres clés : environ zéro g en plein vol,
une résolution d'une milliseconde sur le chronomètre, et un verdict lisible
instantanément sur l'afficheur sept segments.

[Transition] Voyons maintenant comment on a construit ça concrètement.

---

## SLIDE 2 — Solution technique (3 min 30)

Notre plateforme, c'est une carte NUCLEO-L152RE de ST — un microcontrôleur STM32
avec un cœur ARM Cortex-M3 — surmontée de deux cartes d'extension : le shield
X-NUCLEO-IKS01A3 qui porte les capteurs, et la carte ISEN32 qui nous fournit
les boutons, les LEDs, l'afficheur et le potentiomètre.

Ce que vous voyez à l'écran, c'est notre orchestre de périphériques. Chacun a
un rôle unique, et je vais vous les présenter dans l'ordre de la chaîne de mesure.

**Le LSM6DSO**, c'est l'accéléromètre trois axes du shield. Il nous parle en
**I2C**. C'est lui, l'œil du système : c'est lui qui sent l'apesanteur du vol
et le choc de l'atterrissage.

**Le MAX7219** pilote l'afficheur sept segments. Il nous écoute en **SPI**.
Notez au passage un choix d'architecture : le capteur parle en I2C, l'afficheur
en SPI — deux bus différents, donc aucun conflit possible entre la lecture
et l'affichage.

**Le potentiomètre** est lu par l'**ADC**, le convertisseur analogique-numérique.
C'est notre interface de réglage : la tension du potentiomètre devient un objectif
en centimètres.

**Le timer TIM6**, configuré en interruption, est le cœur battant du système.
Toutes les millisecondes, exactement, il incrémente notre compteur de temps.
J'insiste : **toute la mesure repose sur lui**. Sans ses millisecondes, la formule
physique n'a rien à manger. Retenez ce point, il va devenir très important dans
quelques minutes.

**Les boutons**, en GPIO avec interruptions, servent à armer le système et à
valider l'objectif. Avec le timer, ce sont nos **deux périphériques en mode
interruption**, comme l'exige le cahier des charges.

**Le moteur vibreur**, en GPIO simple, célèbre la victoire.

Et enfin **l'UART**, avec printf, nous donne une fenêtre sur le firmware pendant
le développement. Vous verrez dans un instant qu'il a joué un rôle décisif.

Le tout est orchestré par le STM32, avec une machine à états que je vous
présenterai en détail dans la slide de résolution.

[Transition] Sur le papier, tout ça est propre et logique. Dans la réalité...
ça ne s'est pas exactement passé comme prévu. Et c'est tant mieux, parce que
c'est là qu'on a le plus appris.

---

## SLIDE 3 — Défi & enquête de debug (4 min 30)

[Change de ton — plus narratif, c'est le moment fort]

Le pari technique d'APEX — mesurer une hauteur sans capteur de distance —
est élégant sur le papier. Au banc de test, il est impitoyable. Nous avons
traversé trois pannes majeures, et chacune nous a appris quelque chose.
Laissez-moi vous les raconter comme l'enquête qu'elles ont été.

**Première affaire : le faux coupable.**

Symptôme : les boutons ne répondent pas. On appuie, rien ne se passe. Réflexe
naturel : chercher du côté des boutons. Configuration des broches, résistances
de pull-up, anti-rebond... on a tout vérifié. Les boutons étaient innocents.

Le vrai coupable était ailleurs, et bien caché : notre timer était configuré
avec un registre ARR — l'Auto-Reload Register — à zéro d'une manière qui figeait
le compteur de millisecondes. Or, souvenez-vous : **tout dépend de ce compteur**.
La logique d'anti-rebond des boutons utilisait le temps. Le temps étant gelé,
les boutons semblaient morts — alors qu'ils fonctionnaient parfaitement.

Leçon numéro un : dans un système embarqué, le symptôme apparaît rarement
là où se trouve la cause. Il faut remonter la chaîne de dépendances.

**Deuxième affaire : l'adresse fantôme.**

Symptôme : les capteurs I2C sont introuvables. On lance un scan complet du bus —
un balayage des cent vingt-sept adresses possibles — et le scan revient vide.
Aucun périphérique ne répond. Le silence total.

On vérifie l'alimentation du shield : correcte. Les connexions physiques :
correctes. La configuration logicielle du bus : correcte. Alors quoi ?

La réponse était dans le brochage. Notre firmware parlait sur les broches
PB6 et PB7 — qui sont effectivement des broches I2C du STM32. Mais le shield,
lui, écoutait sur **PB8 et PB9**, via le connecteur Arduino. Deux bus valides,
deux endroits différents. On criait dans une pièce vide.

Leçon numéro deux : toujours vérifier le routage physique carte par carte.
La datasheet du microcontrôleur ne suffit pas — il faut lire le schéma du shield.

**Troisième affaire : l'instant manqué.**

Symptôme, le plus frustrant des trois : la détection de saut fonctionne...
une fois sur trois. Parfois le décollage n'est pas vu. Parfois c'est l'impact
qui passe inaperçu. Impossible de faire une démo fiable dans ces conditions.

L'enquête, cette fois, s'est appuyée sur l'UART : en traçant les valeurs
d'accélération, on a compris que notre échantillonnage à dix millisecondes
était **trop lent**. Le creux d'apesanteur au décollage, et surtout le pic
d'impact à l'atterrissage, sont des événements brefs. À dix millisecondes
par lecture, on regardait ailleurs au moment où ça se passait.

La solution : passer l'accéléromètre à **416 hertz**, le lire à chaque tour
de boucle sans attente, et figer l'afficheur pendant le vol pour ne consacrer
le processeur qu'à la surveillance. Résultat : une détection fiable et
reproductible, saut après saut.

Leçon numéro trois — et c'est notre méthode générale : **isoler, mesurer,
prouver**. L'UART printf a été notre stéthoscope pendant tout le projet.
Chaque hypothèse a été vérifiée par une trace, jamais par intuition.

[Transition] Maintenant que le système est fiable, voyons comment il transforme
une accélération brute en hauteur de saut.

---

## SLIDE 4 — Résolution (4 min)

Tout repose sur cette formule : **h égale g t carré sur huit**.

D'où vient-elle ? C'est de la cinématique de terminale, appliquée intelligemment.
Pendant le saut, le seul mouvement est vertical, et la seule force est la gravité.
Le temps de vol total, t, se décompose en deux moitiés parfaitement symétriques :
t sur deux pour monter, t sur deux pour redescendre.

Prenons la descente : on tombe depuis le sommet, sans vitesse initiale, pendant
t sur deux. La hauteur de chute, c'est un demi de g fois le temps au carré —
donc un demi de g fois t sur deux au carré. Développez : g t carré sur huit.

Ce que je trouve remarquable, c'est que **la hauteur ne dépend que du temps**.
Pas de la masse du sauteur, pas de sa technique, pas de l'endroit. Un simple
chronomètre, s'il est précis, devient un instrument de mesure de hauteur.
Notre chronomètre, c'est le TIM6 et sa milliseconde.

Concrètement, pour un temps de vol de 500 millisecondes, la formule donne
environ 30 centimètres. Pour 640 millisecondes, on dépasse les 50 centimètres.
La précision d'une milliseconde sur t nous donne largement la précision
au centimètre sur h.

**Comment détecte-t-on le vol ?** Regardez la courbe à l'écran : c'est la norme
de l'accélération pendant un saut. Au repos, l'accéléromètre mesure un g —
la gravité. À l'impulsion, ça monte brièvement : vous poussez sur le sol.
Puis, dès que les pieds décollent : **chute vers zéro g**. C'est notre signal
de départ — le chronomètre démarre. Le plateau d'apesanteur dure tout le vol.
Et à la réception : **un pic violent**, bien au-dessus d'un g. Signal d'arrêt —
le chronomètre s'arrête.

Deux seuils, donc : un seuil bas pour le décollage, un seuil haut pour l'impact.
C'est simple, robuste, et sans dérive — pas d'intégration, pas d'accumulation
d'erreur.

**Et la machine à états** orchestre le tout. Quatre états. **SETUP** : on règle
l'objectif au potentiomètre, l'afficheur suit en temps réel. **PRÊT** : l'objectif
est validé par bouton, le système surveille l'accéléromètre. **EN L'AIR** :
l'apesanteur est détectée, le chronomètre tourne, l'afficheur est figé.
**RÉSULTAT** : l'impact est détecté, la hauteur est calculée et affichée —
et si l'objectif est dépassé, le moteur vibre. Puis retour au SETUP pour
le saut suivant.

Quatre états, deux seuils, une formule. Toute la complexité du projet tient
dans cette économie de moyens.

[Transition] Concluons.

---

## SLIDE 5 — Conclusion (2 min) + ouverture démo

Ce que nous retenons d'APEX tient en une phrase :
**on ne mesure pas le saut — on mesure le silence de la chute.**

Ce qui fonctionne aujourd'hui : le réglage d'objectif au potentiomètre avec
retour visuel immédiat ; la détection de vol fiable et reproductible, validée
saut après saut ; et le verdict complet — hauteur affichée, vibration de
victoire quand l'objectif est battu.

Ce projet nous a surtout appris une méthode. Face à une panne, ne jamais
accuser le symptôme : isoler, mesurer, prouver. Nos trois enquêtes de debug
nous ont donné plus de compétences que si tout avait fonctionné du premier coup.

Pour la suite, plusieurs évolutions sont naturelles : journaliser les sauts
en EEPROM pour suivre sa progression sur une saison d'entraînement ; streamer
la courbe d'accélération en direct par UART pour visualiser chaque saut ;
ou un mode multi-joueurs pour transformer APEX en jeu de détente entre amis.

Mais plutôt que d'en parler... le mieux, c'est de vous le montrer.

[Sors le boîtier — passage à la démo live]

Je vais régler un objectif... valider... et sauter devant vous.
Le verdict dans trois secondes.

[Après la démo]

Merci de votre attention. Nous sommes prêts pour vos questions.

---

# Annexe — Questions probables du jury et réponses courtes

**« Pourquoi ne pas intégrer l'accélération pour avoir la vitesse puis la hauteur ? »**
Parce que l'intégration accumule les erreurs de biais du capteur — en deux
intégrations successives, la dérive devient énorme en quelques centaines de
millisecondes. Notre méthode par temps de vol n'intègre rien : elle est immune
à la dérive.

**« Que se passe-t-il si l'utilisateur bouge le boîtier sans sauter ? »**
Le seuil de décollage exige une accélération proche de zéro g maintenue —
un simple geste ne produit pas d'apesanteur prolongée. Et le bouton d'armement
évite les déclenchements hors session.

**« Quelle est la précision de la mesure ? »**
La résolution du chronomètre est d'une milliseconde. Sur un saut typique de
500 ms, une erreur d'une milliseconde sur t donne environ un millimètre sur h.
La vraie limite, c'est la détection des seuils — de l'ordre de quelques
millisecondes — soit une précision réaliste de l'ordre du centimètre.

**« Pourquoi h = g·t²/8 et pas g·t²/2 ? »**
Parce que t est le temps de vol TOTAL. La chute depuis le sommet ne dure que
t/2. En substituant t/2 dans h = ½g·τ², on obtient g·t²/8.

**« Pourquoi le MAX7219 en SPI plutôt qu'en I2C ? »**
C'est son interface native. Et ça sépare physiquement le bus capteur du bus
affichage : pendant le vol, on peut marteler l'accéléromètre en I2C sans
jamais être ralenti par l'affichage.

**« Combien de temps le vol est-il chronométré, au maximum ? »**
Le compteur est en 32 bits sur une base d'une milliseconde : de quoi mesurer
49 jours de vol. Pour un saut humain, on a de la marge.
