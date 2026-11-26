# Carte IMU « Neuro IMU » (rev A) → Raspberry Pi 4

Carte maison autour d'un **ST LSM6DSV16X** (accéléromètre + gyroscope 6 axes, fusion de
capteurs intégrée). Sources KiCad : [`IMU.zip`](IMU.zip) (schéma `IMU.kicad_sch`, circuit
`IMU.kicad_pcb`, fichiers JLCPCB dans `production/`).

Liaison choisie : **I2C** sur les GPIO de la Pi (pas d'Arduino ni d'USB en plus).

---

## 1. Ce qu'il y a sur la carte (lu dans le schéma)

| Repère | Rôle |
|---|---|
| IC1 | LSM6DSV16X (boîtier LGA-14) |
| C1, C2 | 100 nF de découplage sur `VDD_IO` et `VDD` |
| D1 + R1 | LED témoin de `VDD` (330 Ω) |
| D2 + R2 | LED témoin de `VDD_IO` (330 Ω) |
| R3 (10 kΩ) | `CS` tiré à `VDD_IO` → **mode I2C** (CS à 0 = mode SPI) |
| R5 (10 kΩ) | `SDO/SA0` tiré à la masse → **adresse I2C 0x6A** |
| R4 | `SDO/SA0` vers `VDD_IO` : **non monté (DNP)** — le monter (et retirer R5) donnerait l'adresse 0x6B |
| J1, J2 | 2 connecteurs 8 broches au pas de 2,54 mm |

Pas de résistances de tirage sur SDA / SCL : celles de la Pi (1,8 kΩ sur GPIO 2 / 3)
suffisent.

Les entrées capteur auxiliaire du LSM6DSV16X (`SDx`, `SCx`, broches 2 et 3) sont reliées
à la masse sur la carte : **les broches J1-4 `SCX` et J1-5 `SDX` ne sont reliées à rien**.

### Connecteurs

| Broche | J1 | J2 |
|---|---|---|
| 1 | GND | GND |
| 2 | INT2 | **SDA** |
| 3 | INT1 | **SCL** |
| 4 | SCX (non relié) | CS (tiré à VDD_IO) |
| 5 | SDX (non relié) | non relié |
| 6 | SD0 = SDO/SA0 (tiré à GND) | non relié |
| 7 | **VDD** | **VDD_IO** |
| 8 | **VDD** | **VDD_IO** |

---

## 2. Branchement sur la Pi

⚠️ **3,3 V uniquement** : le LSM6DSV16X supporte au plus 3,6 V sur `VDD` et `VDD_IO`.
**Jamais le 5 V** (broches 2 et 4 de la Pi).

| Carte IMU | Signal | Pi 4 : broche physique | Pi 4 : nom |
|---|---|---|---|
| J1-7 ou J1-8 | VDD (capteur) | **1** | 3V3 |
| J2-7 ou J2-8 | VDD_IO (entrées / sorties) | **17** | 3V3 |
| J1-1 et/ou J2-1 | GND | **6** (ou 9, 14, 20, 25…) | GND |
| J2-2 | SDA | **3** | GPIO 2 (SDA1) |
| J2-3 | SCL | **5** | GPIO 3 (SCL1) |
| J1-3 | INT1 *(facultatif : « donnée prête »)* | **7** | GPIO 4 |
| J1-2 | INT2 *(facultatif)* | **11** | GPIO 17 |
| J2-4 | CS | — | laisser en l'air (R3 → I2C) |
| J1-6 | SD0 / SA0 | — | laisser en l'air (R5 → adresse 0x6A) |
| J1-4, J1-5, J2-5, J2-6 | — | — | non reliés |

Minimum pour que ça marche : **3V3, GND, SDA, SCL** (4 fils, `VDD` et `VDD_IO` pouvant
venir de la même broche 3V3).

Conseils :

- fils **courts** (< 20 cm), SDA et SCL torsadés chacun avec une masse, **loin des câbles
  d'alimentation des servos** ;
- carte fixée **rigidement, à plat, au centre du corps**, flèche / axe X du composant vers
  l'**avant** du robot (le repère `imu_link` de l'URDF suit ce montage).

Repérage des broches sur la Pi : voir le schéma de la section 3 (Ethernet en haut, carte SD
en bas : broches 1, 3, 5, 7 tout en bas de la colonne de droite du connecteur).

---

## 3. Brochage de la Raspberry Pi 4 (connecteur 40 broches)

**Vue de dessus, prises Ethernet / USB vers le haut, lecteur de carte SD vers le bas** :
le connecteur GPIO est alors le long du **bord gauche** de la carte.

- colonne de **gauche** (au ras du bord) : broches **paires** 2 … 40 ;
- colonne de **droite** (vers l'intérieur) : broches **impaires** 1 … 39 ;
- **broche 1 = en bas à droite** du connecteur (côté carte SD, pastille carrée).

```
                      ┌─────── PRISES ETHERNET + USB (en haut) ───────┐
   bord de la carte   │
   (cote exterieur)   │   colonne PAIRE        colonne IMPAIRE
                      ▼   (bord)               (interieur)
                       GPIO 21 40 ● ● 39 GND
                       GPIO 20 38 ● ● 37 GPIO 26
                       GPIO 16 36 ● ● 35 GPIO 19
                           GND 34 ● ● 33 GPIO 13 PWM1
                  GPIO 12 PWM0 32 ● ● 31 GPIO 6
                           GND 30 ● ● 29 GPIO 5
              GPIO 1 (reserve) 28 ● ● 27 GPIO 0 (reserve)
                    GPIO 7 CE1 26 ● ● 25 GND
                    GPIO 8 CE0 24 ● ● 23 GPIO 11 SCLK
                       GPIO 25 22 ● ● 21 GPIO 9 MISO
                           GND 20 ● ● 19 GPIO 10 MOSI
                       GPIO 24 18 ● ● 17 3V3              <- VDD_IO (ou broche 1)
                       GPIO 23 16 ● ● 15 GPIO 22
                           GND 14 ● ● 13 GPIO 27
                  GPIO 18 PWM0 12 ● ● 11 GPIO 17          <- INT2 (option)
                   GPIO 15 RXD 10 ● ● 9  GND
                   GPIO 14 TXD  8 ● ● 7  GPIO 4           <- INT1 (option)
  GND IMU ->               GND  6 ● ● 5  GPIO 3 SCL       <- SCL (J2-3)
                            5V  4 ● ● 3  GPIO 2 SDA       <- SDA (J2-2)
                            5V  2 ● ● 1  3V3              <- VDD + VDD_IO
                      └──────── LECTEUR CARTE SD (en bas) ────────────┘
```

| Nom | Broche (bord) | Broche (intérieur) | Nom | IMU |
|---|--:|:--|---|---|
| GPIO 21 | 40 | 39 | **GND** |  |
| GPIO 20 | 38 | 37 | GPIO 26 |  |
| GPIO 16 | 36 | 35 | GPIO 19 |  |
| **GND** | 34 | 33 | GPIO 13 PWM1 |  |
| GPIO 12 PWM0 | 32 | 31 | GPIO 6 |  |
| **GND** | 30 | 29 | GPIO 5 |  |
| GPIO 1 (reserve) | 28 | 27 | GPIO 0 (reserve) |  |
| GPIO 7 CE1 | 26 | 25 | **GND** |  |
| GPIO 8 CE0 | 24 | 23 | GPIO 11 SCLK |  |
| GPIO 25 | 22 | 21 | GPIO 9 MISO |  |
| **GND** | 20 | 19 | GPIO 10 MOSI |  |
| GPIO 24 | 18 | 17 | **3V3** | VDD_IO (ou broche 1) |
| GPIO 23 | 16 | 15 | GPIO 22 |  |
| **GND** | 14 | 13 | GPIO 27 |  |
| GPIO 18 PWM0 | 12 | 11 | GPIO 17 | INT2 (option) |
| GPIO 15 RXD | 10 | 9 | **GND** |  |
| GPIO 14 TXD | 8 | 7 | GPIO 4 | INT1 (option) |
| **GND** | 6 | 5 | GPIO 3 SCL | SCL (J2-3) |
| **5V** | 4 | 3 | GPIO 2 SDA | SDA (J2-2) |
| **5V** | 2 | 1 | **3V3** | VDD + VDD_IO |

- Les GPIO sont en **3,3 V** : jamais de 5 V sur une broche GPIO.
- 3V3 : environ 500 mA au total pour tout ce qui y est branché (largement assez pour l'IMU).
- Broches 27 / 28 réservées à l'EEPROM des cartes HAT : ne pas les utiliser.
- Déjà utilisé sur ce robot : rien sur les GPIO (servos par USB / U2D2).

---

## 4. Vérifier une fois branché

Sur la Pi (`ssh hexapode`). L'I2C est **déjà activé** (`dtparam=i2c_arm=on` dans
`/boot/firmware/config.txt`, bus `/dev/i2c-1`) ; il manque seulement les outils :

```bash
sudo apt install i2c-tools
sudo i2cdetect -y 1          # doit afficher "6a"
sudo i2cget -y 1 0x6a 0x0f   # WHO_AM_I : doit repondre 0x70 (LSM6DSV16X)
```

Vérifié le 26/11/2026 : `6a` présent, `WHO_AM_I = 0x70` — **après une coupure d'alimentation
de la carte** (avant, le capteur ne répondait pas : I2C désactivé par un ancien réglage).
Nœud ROS et calibration : [ros2_ws/src/hexapod_imu/README.md](../../ros2_ws/src/hexapod_imu/README.md).
