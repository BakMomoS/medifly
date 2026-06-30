# Firmware MediFly — drone quadricoptère

Sketch Arduino pour l'ESP32 DevKit V1 qui pilote le prototype MediFly.
Conforme à l'architecture décrite dans le rapport final PLBD 19 (juin 2026), sections 8.2 à 8.3.

## Bibliothèques requises (Arduino IDE)

À installer via **Outils → Gérer les bibliothèques** :

- `Adafruit MPU6050` (par Adafruit)
- `Adafruit BMP280 Library` (par Adafruit)
- `Adafruit Unified Sensor` (par Adafruit) — dépendance des deux précédentes
- `TinyGPSPlus` (par Mikal Hart)
- `Bluepad32` (par Ricardo Quesada) — voir <https://github.com/ricardoquesada/bluepad32>

Et le **board manager ESP32** (Arduino Core 2.x — *pas* 3.x, sinon Bluepad32 ne compile pas) :
- Préférences → URL du gestionnaire de cartes : `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
- Outils → Type de carte → `ESP32 Dev Module`

## Câblage

| Composant      | Broche ESP32         | Notes                                 |
|----------------|----------------------|---------------------------------------|
| MPU6050 SDA    | GPIO 25              | I²C — partagé avec BMP280             |
| MPU6050 SCL    | GPIO 26              |                                       |
| BMP280 SDA/SCL | GPIO 25 / 26         | adresse 0x76 ou 0x77                  |
| NEO-6M TX      | GPIO 16 (RX ESP32)   | UART2 @ 9600 baud                     |
| NEO-6M RX      | GPIO 17 (TX ESP32)   |                                       |
| ESC M1 PWM     | GPIO 18              | avant-gauche (CCW)                    |
| ESC M2 PWM     | GPIO 19              | avant-droit (CW)                      |
| ESC M3 PWM     | GPIO 21              | arrière-gauche (CW)                   |
| ESC M4 PWM     | GPIO 22              | arrière-droit (CCW)                   |
| Servo MG90     | GPIO 27              | treuil de la box                      |

⚠️ Alimente bien le NEO-6M en 3.3 V (et **pas** en 5 V) — pareil pour le MPU6050.

## Téléversement

1. Connecte l'ESP32 à l'ordinateur en USB.
2. Outils → Port → choisis le COM de l'ESP32.
3. Vérifier → Téléverser (Ctrl+U).
4. Ouvrir le moniteur série à **115200 baud** pour voir la télémétrie.

## Format de la télémétrie Serial

Une ligne toutes les 200 ms (5 Hz) :

```
ARME  | R: 0.2° P: 0.1° | ALT:0.14m T:24.4°C | GPS:FIX SAT:7 LAT:33.573100 LNG:-7.589800 HDG:74.3 SPD:0.0
```

Ce format est **directement parsé** par le site MediFly (voir `_parseTelemetry()` dans `index.html`). Les regex côté web acceptent aussi les variantes `Sats:`, `Latitude:`, etc.

## Étapes de validation incrémentale (rappel)

Tester dans cet ordre, ne jamais sauter d'étape :

1. **Étape 1 — MPU seul** : commente l'armement des ESC dans `setup()`, observe la trame `R:` et `P:` qui bouge quand tu inclines.
2. **Étape 2 — moteurs + manette, sans PID** : commente l'appel à `pidStep` et envoie directement `throttleUs` aux 4 ESC. Vérifie les sens de rotation (CW/CCW).
3. **Étape 3 — PID sur banc, sans hélices** : remets les hélices retirées. Drone sur planche articulée. Règle Kp puis Kd puis Ki, axe par axe.
4. **Étape 4 — vol attaché** : 4 cordes aux coins. Premier vol contrôlé.
5. **Étape 5 — vol libre** : espace dégagé, hover, manœuvres légères.

## Failsafe

Le firmware coupe automatiquement les gaz (`armed = false`) si la liaison Bluetooth est perdue plus de 800 ms (`FAILSAFE_TIMEOUT_MS`). Désarmement manuel : bouton **Options/Select** de la manette.
