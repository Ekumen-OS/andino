# andino_hardware

This package aims to provide the necessary information to the correct assembly of the robot.
> [!IMPORTANT]
> The hardware has been updated. If you are looking for the old design please go to [f7af556d00a099cf18a9d1e17eb7e725fa97e1da](https://github.com/Ekumen-OS/andino/tree/f7af556d00a099cf18a9d1e17eb7e725fa97e1da)


## Bill of Materials

| Part_number | Image | Part_name | Links | Comments |
|:--:|:--:|:-----------------------:|:--------------------:|:-------------------------------------------------------:|
| 1 | <img src="docs/parts/raspberry.jpg" width="400"> | Raspberry Pi 4 B (4 Gb) | [PiShop](https://www.pishop.us/product/raspberry-pi-4-model-b-2gb/), [TiendaTec](https://www.tiendatec.es/raspberry-pi/gama-raspberry-pi/1100-raspberry-pi-4-modelo-b-4gb-5056561800349.html), [Amazon](https://www.amazon.com/-/es/desarrollo-Multifuncional-Desarrollo-Aprendizaje-Programaci%C3%B3n/dp/B0B7RN5PPN/ref=sr_1_1?crid=2TP47BV2H7KR3&dib=eyJ2IjoiMSJ9.mGPLQSzGdo46mempglSxiyOOBw_6_bzApyMN9nBELUDhMvprgoi-t-zlc_pvFQPc0UNyPoKxyOc7x5dUVNo2yRiEc5D5H-qOEGPiy5Eyj2CATkz6OrMycJRDrRyIgrX_m3PWq5ZgZlfTX6iFeEzh42QTuLopPU1BAjkaUWcv9hbcXvg4bVRpybxgqssX1YJXzkt-sATzkmN5S7Rpe-7_-y-83lx2S6BdC5XR550Oews._M_Ne0Dh3wEIpSbYG6LzQIDHuXtoYlHuBKReSf9LPZg&dib_tag=se&keywords=raspberry+pi+4&qid=1771022722&sprefix=ras%2Caps%2C150&sr=8-1) | If you want better performance you could buy the 8GB model |
| 2 | <img src="docs/parts/base_chassis.jpg" width="400"> | [Base_chassis](./printing_model/chassis/base_chassis.stl) | 3D printed |
| 3 | <img src="docs/parts/lidar_chassis.jpg" width="400"> | [lidar_chassis](./printing_model/chassis/lidar_chassis.stl) | 3D printed |
| 4 | <img src="docs/parts/battery_chassis.jpg" width="400"> | [Battery_chassis](./printing_model/chassis/battery_chassis.stl) | 3D printed |
| 5 | <img src="docs/parts/arduino_lock_base.jpg" width="400" > | [Arduino_lock_base](./printing_model/chassis/arduino_lock_base.stl) | 3D printed |
| 6 | <img src="docs/parts/camera_case.jpg" width="400"> | [Camera_case](./printing_model/raspi_cam_mount/) | 3D printed |
| 7 | <img src="docs/parts/motors.jpg" width="400"> | 2 x Motor with Encoder | [Sparkfun](https://www.sparkfun.com/products/16413) | Motor name: Hobby Motor with Encoder - Metal Gear (DG01D-E) |
| 8 | <img src="docs/parts/wheels.jpg" width="400" > | Wheels | [Amazon](https://www.amazon.com/-/es/Paquete-neum%C3%A1ticos-repuesto-inteligentes-autom%C3%B3vil/dp/B0CG1C7T8J/ref=sr_1_1?crid=80ZH97C0XB41&dib=eyJ2IjoiMSJ9.ijIJncnmb5RTnhTUdHvLIkoRR9ETXL1yN80oP5o6pLl_M1gweGQv8Bc4ZzZXLwu3FHXErDb4N90k34FK7T4vksXcP6Ndq6uTXnHfDpvqZxqn9BXa1hpDxVFS2uIrh_UveXW9e-qeRadyM1IwvMVM2gOQaFt6fZ1ZacODgIiHDQSYorrqmwE3Ner99OtWPWkvMvqHuD75oDls9OZE99Ed5nNXlGzO7lOZyUIjCKtlmenpinQVPkfT-_6hTdBGrpdV7bwRpRlS7RuSV9nruQHakjroS5nTiKpq1TASIHR43QU.gsTbZywsqmgc6wPFsOY2lHRRYZnrQAcRF3Y9craGxIU&dib_tag=se&keywords=tt+motor+wheels&qid=1771023074&sprefix=motor+tt+wheel%2Caps%2C175&sr=8-1) | Wheels are generic wheels for TT motors.|
| 9 | <img src="docs/parts/arduino.jpg" width="400"> | Arduino Nano | [Amazon](https://www.amazon.es/Nano-V3-mega328P-Desarrollo-Binghe-Microcontrolador-Compatible/dp/B0DKJMJ147/ref=sr_1_1?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=2HELLSEW2KGDD&dib=eyJ2IjoiMSJ9.RFy2FmC9kl5hrXCOo1gt-654KLEWD37fvHkxBsa7xOP9l_IKF3_jJogbQYxkPyFckd_Uh_1YGRpoQZMp34mdmBllqzbql2jA9ZwJ_kWb4O_oFqPo7bysKSqpCIwfwmOEq6Zxoc8jlpc7feX0WCeefRYnuXkP6AphPl1xTY3BUQvj5IxJ1mYd0C4JSV-MmXBDkjuF8HY88HMehYgHBBbg-GbL3XSAcFS-tGMY4EgCNwFLRue1mCtStPcl5UWUtEsfb4fBXyZxKpZZlxLXVB1i7a3P3JzpVzqJW7fsrhLXVmA.Zb3P7-8aRzzJdhy0rvdbLZjkgG1zSptfoM4iaaJwvpE&dib_tag=se&keywords=arduino+nano+v3&qid=1771023225&sprefix=arduino+nano+v3%2Caps%2C474&sr=8-1) | Arduino pins can be soldered in any position, but it is recommended to solder them as shown in the picture.|
| 10 | <img src="docs/parts/IMU.jpg" width="400"> | IMU - BNO055 | [Amazon](https://www.amazon.es/Geomagnetismo-Aceleraci%C3%B3n-Dispositivos-Procesamiento-Adquisici%C3%B3n/dp/B0DSHWF2LT/ref=sr_1_10?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=NBCLYW8UDTCC&dib=eyJ2IjoiMSJ9.cDS6mGB-km8xQysSDalqV7UjQFmUmuGLd8qUWuBwsxBuKUAMfQnqP6zYPf6SgxLlX_Kb5PlV_bLRx_2x6xMXS5c9pZKDqw2WYRtxQyprJALBDl23XnGoIe7i-uDpGReg_-dT7JhMIiLsGUALIVkFI9bIre_f8eU-CrDNrm8ItUEWBk7KkJS5FWa5wBZR1uPyiWwFxw5Dr0gy6CqiR5ah-X9sSbJ9sORuzU65NzvM8zOdiWYpnmSbaY211Apj3Yjqd6Rs5c1rtBRQekgv_XGtqj-T-1GY6QbOAdz-GN4g1KY.OSbmnv3iJ_M_3bco3WVV9FVUZjUgd-EvLOOCehA7040&dib_tag=se&keywords=IMU+-+BNO055&qid=1771023269&sprefix=imu+-+bno055%2Caps%2C489&sr=8-10) | - |
| 11 | <img src="docs/parts/motor_driver.jpg" width="400"> | L298N Dual H Bridge | [Amazon](https://www.amazon.com/Bridge-Stepper-Driver-Module-Controller/dp/B09T6K9RFZ/ref=sr_1_4?crid=37YY7JO6C3WVE&keywords=l298&qid=1685740618&sprefix=l29%2Caps%2C277&sr=8-4) | - |
| 12 | <img src="docs/parts/lidar.jpg" width="400"> | RPLidar A1M8 | [RobotShop](https://www.robotshop.com/products/rplidar-a1m8-360-degree-laser-scanner-development-kit?_pos=3&_sid=b0aefcea1&_ss=r), [Amazon](https://www.amazon.com/RPLIDAR-A1M8-Evitaci%C3%B3n-Obst%C3%A1culos-Navegaci%C3%B3n/dp/B07TJW5SXF/ref=sr_1_1_sspa?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=6Z65FTP4XRZ&dib=eyJ2IjoiMSJ9.9rz2OfYpvAAEF8dUy6aYwqFhw6U8mCHA81HqCWfHiZMaPQO8jQOW-KuMmijXVXM4heaiaDCFsT9qvaTGa47y18B47pG_dGnbdgR9m_QX-UZ4WmVl0mU_EYa54QhKxx672F65mqmdz2lioURxc_7O3A.SFuniUk-6Gp0R_yL-P5bhzVNQNc-qPg9DkEKZPRvAvk&dib_tag=se&keywords=RPLidar+A1M8&qid=1771023378&sprefix=rplidar+a1m8%2Caps%2C250&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) | If no microUSB-USB cable is included, you will need to purchase one |
| 13 | <img src="docs/parts/camera.jpg" width="400"> | Raspi Camera Module V2, 8 MP | [Robotshop](https://www.robotshop.com/products/raspberry-pi-camera-module-v2), [Amazon](https://www.amazon.com/Raspberry-Pi-Camera-Module-Megapixel/dp/B01ER2SKFS?th=1), [Longer cable](https://www.amazon.es/AZDelivery-Repuesto-Raspberry-30cm-Flexkabel/dp/B01NAXKTDP/ref=sr_1_9?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=I1IK0FQVQCCU&dib=eyJ2IjoiMSJ9.2UYb-3a8M00iHZZiHT0xjp8vfzq-3BSmJSLxdtcCMV6WMj8g5T8T_j5DKX8cESxqnVN01YpV3nX28IuhewGTOsmJ4yF5st20TxU7kHNHftZE_aygB5vT-001wEvUfx70V0H0DZljw0YfC4R2wpjsTR_89pAA95C4F8LhJiGbbUGTEjBgmjnsRIFs6FYatZa9KVusNyv2cKxGZlav36gdoAkMXQUvX578c6frxSnH5DROCeK6bKSqrczA7R8OBVWC995fK1AxHbJLHZJEDQGXMVhUkHd2liBV0nq48mnyE6Q.nHty4TxSpldkH8FZAia_rx7sX6c2uIBpPRUEqwq5NiY&dib_tag=se&keywords=raspberry%2Bpi%2Bcamera%2Bcable&qid=1716878948&sprefix=raspberry%2Bpi%2Bcamera%2Bcable%2Caps%2C76&sr=8-9&th=1)  | A link for a longer cable (30 cm) is included, just in case the one included with the camera is too short. |
| 14 | <img src="docs/parts/battery_case.jpg" width="400"> | Battery Case | [Amazon](https://www.amazon.es/GTIWUNG-Soporte-Bater%C3%ADa-Almacenamiento-Pl%C3%A1stico/dp/B08CY5JKG4/ref=sr_1_5?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=14DBV6GMV9YIP&dib=eyJ2IjoiMSJ9.AiAmesc8h9MLdB6hmFQT4tzk_lWRWqH1soWP-mWBVQlusnlIziwGRyxc186s679Nd6KCvuJp5GEkinpVWIgSnQY_Y1CZaS0YD-KcJp73pDv4SKvSLKpKYQwfbtTV32_wsMqZrpZWJ_pJYOgDwif19DcMCtdmk5Xf3m4SrukOng3C-7gHiznzr0tLgOyv2EJdech-W6m0tJtl10II1eTnq1gz89W4GK3PW1Nb-Mc8fwr17el3KvvrkYBEvDrnhfdfdG-wwy0EH7xOAmrrDYiOSOlqEXP38Sj7ZYg6atB_axk.ZGQqYl_z8bP5Xg1xgjbRm7xW1ty1oA0oaW4B70268L0&dib_tag=se&keywords=18650%2Bbattery%2Bcase%2B3&qid=1771027178&sprefix=18650%2Bbattery%2Bcase%2B3%2Caps%2C423&sr=8-5&th=1) | Link is for a 6 pack cases but only one is needed. |
| 15 | <img src="docs/parts/batteries.jpg" width="400"> | Batteries | [Amazon](https://www.amazon.es/18650-Recargable-Unidades-Descarga-Soldadura/dp/B0FP6536TP/ref=sr_1_15?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=M4U5RCR55YN1&dib=eyJ2IjoiMSJ9.jFIzKfW_3oqvbUs3OnPuQilyGIEcqtutqxTkGORK6lpAovDhnXWTm5Yix_F5ydJWaX86sSlzT8BBfQOsQGgPMVOnfXTSl1C-t_v1OZgr25FyyN9yqrViLuXbwEJZKsnqP5MaFVp-PpBaUSxASMbSuXH11COgilp8xs0pKzblla_0Wul7YktN9H0kfAMqlVTxZ8A4KStBmtkZIwBf8BA9P0fruKy2a1xlIDb1VUJ9Av5Cy6YgfbI71n0WpSdn6UZDIObHhvPXmS51t3HrWSKplddyTB0cEXzQuxxBJ6MU3-I.GinFCd0nw6TBRYZFrZsIWg0D3qy2khbYxTYAeOUuHyM&dib_tag=se&keywords=18650+batteries&qid=1771026847&sprefix=18650+batter%2Caps%2C442&sr=8-15) | Any 18650 batteries should work |
| 16 | <img src="docs/parts/batteries_charger.jpg" width="400"> | Batteries charger | [Amazon](https://www.amazon.es/REACELL-Cargador-Inteligente-R%C3%A1pido-Recargable/dp/B0BNPCMNGZ/ref=sr_1_9?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=1145KJVG7QLLJ&dib=eyJ2IjoiMSJ9.AiAmesc8h9MLdB6hmFQT4tzk_lWRWqH1soWP-mWBVQlfWpxCQL2S-wweBItVcUZNpDFi-tDugEBzD4GyGI7cqxP91okI-rxfZCR1_v0BkR34SKvSLKpKYQwfbtTV32_wsMqZrpZWJ_pJYOgDwif19HJw9zLJGvUjCXpIuQXR2CFjl6RAixXLmmiybQAyPT2Fwxrf-vCF-psNvELBs_YVwprGp2PQZKVZvXfMSN9ceXv3GIu5kk3PzPMlTNaRtTPo8JWly-woqa3o7QXVK9kTj-lqEXP38Sj7ZYg6atB_axk.vKZ5XsA5t5sjahvQsLEJ1xbEZofTXhnjn66-dLbn0Mc&dib_tag=se&keywords=18650+battery+case+3&qid=1771026748&sprefix=18650+baterry+case+%2Caps%2C431&sr=8-9) | You can buy any 18650 battery charger e.g one for more than 2 batteries. |
| 17 | <img src="docs/parts/dc_dc_converter.jpg" width="400"> | DC - DC converter | [Amazon](https://www.amazon.com/-/es/Adaptador-corriente-compatible-Raspberry-autom%C3%B3vil/dp/B09DGDQ48H/ref=sr_1_3?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=2F9EDCW5A81YU&dib=eyJ2IjoiMSJ9.Dzh0FWbiiGTjQ8TuE9-5d7ve2ZNtAFn5kYrHT9HeK1R9n7cUSDWuYMXdJGyD9HcAv7Rb7tKCXZDTP4B5j9VSO4x_Hyg4oUOYsWDstc6WSS-w2dQh1dhSTEQ3ZdY0jJFBkADqXp2iTqFtGAbKFiR7F-dLwEIRR2y948q8oxcJO904XB91QB70CwJyA-8p1mZeictRqIEbb4m_5mZrXF4U8DJYquii8kKvEweyAN7Gd2s.oWCZlWJZVtrTbw4LU2zVIqdkDA6VStudh_zDdw_iwjs&dib_tag=se&keywords=dc%2Bdc%2Bstep%2Bdown%2Busb%2Bc&qid=1771024289&sprefix=dc%2Bdc%2Bstep%2Bdown%2Busb%2Bc%2Caps%2C194&sr=8-3&th=1) | You can use any stepdown DC-DC regulator that supports and input of 12V and give 5V output at at least 3A |
| 18 | <img src="docs/parts/kill_switch.jpg" width="400"> | Kill-Switch | [Amazon](https://www.amazon.com/interruptor-basculante-encendido-interruptores-cuadrados/dp/B0CS96LX3V/ref=sr_1_33_sspa?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=BKWXR2ZJGOJN&dib=eyJ2IjoiMSJ9.H2gIXvIBQb9Rm35su7Md9zwyr8Gptf1vTzMs4pM6pITGjHj071QN20LucGBJIEps.wS84pF1tUpmJZATTu-NYupgOL2yhl0xAL4PPmUfxyWI&dib_tag=se&keywords=interruptor%2B12V%2Bcon%2Bluz&qid=1771040005&sprefix=interruptor%2B12v%2Bcon%2Blu%2Caps%2C256&sr=8-33-spons&xpid=oohU7DE__5G1B&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGZfbmV4dA&th=1) | Mounting hole is 13 x 20 mm |
| 19 | <img src="docs/parts/caster_wheel.jpg" width="400"> | Caster wheel | [Amazon](https://www.amazon.es/Unidades-Peque%C3%B1as-Dispositivo-Transferencia-Transporte/dp/B098XHYW7F/ref=asc_df_B098XHYW7F/?tag=googshopes-21&linkCode=df0&hvadid=529604577974&hvpos=&hvnetw=g&hvrand=15132275207682237467&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9181150&hvtargid=pla-1396749454795&psc=1&mcid=b1df85a65d163e89b507de60e73f9e65) | - |
| 20 | SD Card | 64 GB SD Card | [Apokin](https://www.apokin.es/tarjeta-microsd-philips-64gb-class10.html), [Amazon](https://www.amazon.es/Kingston-Tarjeta-SDCS2-64GB-Adaptador/dp/B07YGZQ4H8/ref=sr_1_7?dib=eyJ2IjoiMSJ9.zE4PI6DCNK3d78rtl5ga1NQXGwJT1jC2iqi3mXNzbdJ4BosAUPCn9gc13Gc7pdHDx-7wTy4CDj0zIlgDpu9qXH-6GLgI--pJbfi3OvTBPhwwH-tfi1OzM9xqcAOJG6pJuTtkknsyFk6Ma2EHJ4UdheaziDC_KKaWNKgsf_DFbcA-ZxQSXlTtQqwHvCzgi8hq4vKGiEIY-LSZS_sXE9IUGroo0Isl59Po2IXhTBG5IHnnsVR_7lo0dVBVFYl-5GY2CvJbrixULuPl90TbFTTP6DoIeDcpFdDTcbvSK3Lecss.WXBgIZllFgQxx13Szl3q6WIlTOliwrN8V42J1SzNJ8o&dib_tag=se&qid=1714552555&refinements=p_n_feature_browse-bin%3A948155031&s=computers&sr=1-7) | The SD Card is used to host the OS for the Raspberry Pi |
| 21 | <img src="docs/parts/screws.jpg" width="400"> | screws kit | [Amazon](https://www.amazon.es/tornillos-avellanada-tuercas-surtido-roscados/dp/B0DQCSK3QR/ref=sr_1_5_sspa?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=AFF5F7FYBDUB&dib=eyJ2IjoiMSJ9.ZUD2S05GEuuoyW7sRYNpgDhWHJaADc8wpON9Sg2wys0v9XqOfz8oKGDaqaZmUBlSabjBiFTIzkPlEf7ON5uiBZQ6OB1aZ0UGrGfFlrLE_mLmdQdLaSQVmEVvBE995BLlvjqut8I6yvXme12RA2VJzG13WLi1qOVixQ4myjtnnA3B9wk505ZL4ZExdsJFdvVqtQLxRzIgXXnL9U53sOX0kBuBVFWmZH6vDgydNCoBzj-wa6JdoQE-axAzkKKTW0GYjZ0qi7ey6OC5UbLatuOEDGTK53V8xWL9xX1dhzlG4R4.8WkDmd8CW8NBbORnmVYv2NaSBqNEOT0Lb3NoxMatrQk&dib_tag=se&keywords=M3%2Bscrews%2Bkit&qid=1771027282&sprefix=m%2Bscrews%2Bkit%2Caps%2C325&sr=8-5-spons&aref=kiRFD1FbHJ&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1) | You don't need all the screws but here you can get all the screws that you need in the [screw-table](#screw-table)|
| 22 | <img src="docs/parts/zip_ties.jpeg" width="400" > | zip ties | [Amazon](https://www.amazon.com/ANOSON-tama%C3%B1os-pulgadas-resistentes-ultravioletas/dp/B0C2Z4L3S6/ref=sr_1_1_sspa?crid=2ZYM3RCU9OJ6E&dib=eyJ2IjoiMSJ9.yZ0eRA1tzbb2B37cITr4PLcStxxj1qdg5A10dy_E0ezu8RIc4Fsujpp2th3NXioZhTWgEQY-t4G5stldZ3mBP8nybzClHFN8dAmpdJGnX-DcVRUU3QcjpUohrfpgF7DLDsZSWdQxwK6C0eVrN31-BFKqK8takJlzA2qCquWuRI0QuBOm8RH4aCXwCk4RaKm_HFv91p-TSKEUWS9vCXwHJ9Q_w2RV0SOWn1Pc_ywMLXg.jU38936ZP5XD-iN9xdjLrik2UqqlSPLezCuR95I5Acg&dib_tag=se&keywords=zip%2Bties&qid=1771040286&sprefix=zip%2Caps%2C241&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1) | Link is a good option, but your only going to use 4 to 6 |

### Screw table
| Size | length | quantity needed |
|:--:|:------:|:--------------------:|
| 3m | 6mm | 0 |
| 3m | 8mm | 2 |
| 3m | 10mm | 4 |
| 3m | 12mm | 6 |
| 3m | 16mm | 5 |
| 3m | 20mm | 0 |
| 3m | 25mm | 2 |
| 3m | 30mm | 4 |
| 3m | Nuts | 19 |

### Tooling

| Number | Tool | Links | Comments |
|:--:|:------:|:--------------------:|:-------------------------------------------------------:|
| 1 | Set of screwdrivers | [Amazon]() | |

## Assembly Process

  ### 1. Rear wheel 
  #### Place the base chassis face down, install the  wheel in the base chassis.
  <img src="docs/assembly/1.base_and_casterwheel.jpg" width="600">

  #### Place the base chassis face up.

  <img src="docs/assembly/2.base_and_casterwheel_up.jpg" width="600">

  ### 2. Motors and IMU
  
  #### Install motors using 4 X 30mm screws and the IMU using a zip tide as shown in the image. 

  <img src="docs/assembly/3.motors_and_imu.jpg" width="600">

  ### 3. Camera 

  #### Pass the camera flex through the hole below the IMU base as shown in the image.

  <img src="docs/assembly/4.camera_and_base_case.jpg" width="600">

  ##### Place the camera inside the camera case as shown in the image.

  <img src="docs/assembly/5.camera_in_case.jpg" width="600">

  #### Place the camera case in its position as shown in the image, be careful when you spread the base, to avoid braking it.

  <img src="docs/assembly/6.camera_in_place.jpg" width="600">

  ### 4. Raspberry 

  #### Rotate the base 180 degrees and place the raspberry as shown in the image. And place the wheels by inserting them into the motors.

  <img src="docs/assembly/7.raspi_in_place.jpg" width="600">

  ### 5. Motor Driver
  
  #### Add power and 5V cables to the motor driver. at least 15cm for power cables.  

  <img src="docs/assembly/8.motor_driver_connections.jpg" width="600">

  #### Finish all the driver connections according to the [connection diagram](#motor-arduino)
  
  <img src="docs/assembly/9.motor_driver_in_place.jpg" width="600">

  ### 6. Arduino

  #### Place the arduino using the arduino lock base.

  <img src="docs/assembly/10.arduino_in_place.jpg" width="600">
  
  #### Make the remaining connection of the [connection diagram](#motor-arduino) and organize the cables as you prefer.

  <img src="docs/assembly/10.1.organize_cables.jpg" width="600">

  ### 7. Lidar Base

  #### Place the lidar in the lidar base as shown in the image.
  <img src="docs/assembly/11.lidar_base.jpg" width="600">

  #### Fix the lidar driver to the lidar base using zip tides as shown in the image.

  <img src="docs/assembly/12.lidar_in_place.jpg" width="600">

  #### Place the lidar base above the base cassis and use screw it in place. 

  <img src="docs/assembly/12.1.place_lidar_base_in_main_chassis.jpg" width="600">

  ### 8. Battery chassis. 
  
  #### Place the 5V step down driver and the killer switch in the battery chassis as shown in the image.

  <img src="docs/assembly/13.battery_case_switch_and_dcdc.jpg" width="600">

  #### Exterior should looks like this

  <img src="docs/assembly/14.battery_case_outside.jpg" width="600">
  
  #### Place the battery case inside the battery chassis as shown in the image.

  <img src="docs/assembly/15.battery_case_full.jpg" width="600">

  ### 9. Final Assembly

  #### Make all the power connections in the [power connections schema](#raspberry-power) and place the battery chassis in the base chassis as shown in the image.

  <img src="docs/assembly/16.place_battery_chassis_in_base_chassis.jpg" width="600">

  ### 10. Final Andino closed

  <img src="docs/assembly/17.final_version.jpg" width="600">

<a name="connection-diagram"></a>
## Connection Diagram

### Motor-Arduino

<img src="docs/layout/andino_diagram_arduino.jpg" />

Some frequent errors:
 - If one of the motors rotates in the opposite direction (think about the orientation of the motors in the chassis) probably the output(+ and -) of the L298N's output should be toggled.
 - When moving forward the encoder values should increase while moving backwards they should decrease. If it is happening the other way around probably the A and B encoder signals should be toggled.

### Raspberry-Power


<img src="docs/layout/full_power_layout.png" />

> [!NOTE]
> Ground cable to the switch is only necessary if it has a led indicator.

> [!NOTE]
> For the camera ensure the ribbon cable is properly connected with the blue or silver side facing the USB ports.

## Microcontroller Configuration

For uploading the microcontroller firmware please refer to [`andino_firmware`](../andino_firmware/README.md) package.

## Single Board Computer (SBC)

The SBC used in this project is a Raspberry Pi 4b so the guidelines here will refer particularly to this family of on-board computers, however extending its use to other families is possible as well.

This section details the required configuration that is needed in the SBC.
You can either follow these steps or **rely on community contribution (Recommended) for installing this via ansible playbooks**: See https://github.com/garyservin/andino_ansible_config

### Operative System

The recommended operative system depends on which ROS 2 distro you want to use:
- **Humble**: Ubuntu Mate 22.04 ARM64 - [Download](https://ubuntu-mate.org/download/arm64/)
- **Jazzy**: Ubuntu Server/Desktop 24.04 - [Download](https://ubuntu.com/download/raspberry-pi)

> [!IMPORTANT]
> It should work with either Desktop or Server versions. Desktop versions naturally are slower. Refer to https://ubuntu.com/download/raspberry-pi for general information on Ubuntu installation on Raspberry Pi.

For installing this OS in the Raspberry:
1. Download the image for your target ROS distro (see above).


2. Install OS to a microSD card using [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
   - No extra configuration should be necessary.

3. Boot your raspberry using the microSD and a HDMI connection. Some initial configuration is necessary. Follow the wizard for a proper set up. It is recommended to use simple User and Password combinations for the user. For example:
    - user: pi
    - password: admin

4. Once is done, run `sudo apt update && sudo apt upgrade` in a terminal for updating the system. Then reboot.

### Installing dependencies

Some packages are necessary to be installed towards a correct set up of the robot's on-board computer.

#### ssh-server

In general, you will want to access to the Raspberry remotely via `ssh` connection while being connected in the same network.
So we need to install `ssh-server`;
```
sudo apt-get install openssh-server
```
Enable it if it is not enabled yet:
```
sudo systemctl enable ssh --now
```

After this you will be able to access this device from a remote computer by doing:
```
ssh <user>@<ip>
```
For example if the user is `pi` and the ip is `192.168.0.102`
```
ssh pi@192.168.0.102
```


#### Common utilities

Install some common utilities that will be required later on.

```
sudo apt update
```

```
sudo apt install git net-tools software-properties-common build-essential -y
```
```
sudo apt install python3-rosdep2 python3-catkin-pkg python3-catkin-pkg-modules python3-rospkg-modules python3-rospkg  -y
```

#### Install ROS and Tools

 - [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
Follow suit the instructions for installing next dependencies from binaries:
 - [ROS 2 Humble(If Ubuntu 22.04)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
 - [ROS 2 Jazzy(If Ubuntu 24.04)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
      > [!IMPORTANT] It is worth checking out official instructions on installing ROS on Raspberry PI platforms, in particular for Jazzy onwards.
    > See [here](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html#ros-2-on-raspberry-pi): Relying on noble-backports in Ubuntu Noble (24.04) is necessary.

To automatically source ROS installation, it is recommended to add `source /opt/ros/humble/setup.bash` line to the `~/.bashrc` file.

#### Arduino

Arduino drivers are necessary for the SBC (Raspberry) <--> Microcontroller(Arduino) serial communication.

```
sudo apt install arduino
```

Configure it properly:
1. Add user to `dialout` and `plugdev` groups:
   ```
   sudo usermod -a -G dialout $USER
   ```
   ```
   sudo usermod -a -G plugdev $USER
   ```
   Note you will need a reboot after this to be effective.

> [!IMPORTANT]
> In Ubuntu 22.04 seems to be an issue with some chip drivers and the `brltty` daemon. To avoid ?> this conflict we remove `brltty` as suggested. See [this stackoverflow post](https://stackoverflow.com/>? > questions/70123431/why-would-ch341-uart-is-disconnected-from-ttyusb) for further information.
> - Remove `brltty` from the system
    ```
    sudo apt remove brltty
    ```

#### Raspberry Camera Module V2

After connecting the camera module to the Raspberry's camera port.
```
sudo apt install libraspberrypi-bin v4l-utils
```
```
sudo usermod -aG video $USER
```

Check camera status:
```
vcgencmd get_camera
```

If the output of the previous command is `supported=1 detected=1', everything is fine. If not, your camera won't work correctly, you need to perform some configuration first.

Modify the `config.txt` file for the boot:

```sh
 sudo nano /boot/firmware/config.txt
```

And add these lines under `[all]`:

```
# Autoload overlays for any recognized cameras or displays that are attached
# to the CSI/DSI ports. Please note this is for libcamera support, *not* for
# the legacy camera stack
start_x=1
gpu_mem=128
```

Save and close the file. Then we need to enable the camera support for the raspberry:

```sh
sudo apt install raspi-config
sudo raspi-config
```

Go to `Interface Options`, select `camera` and enable it.

Finally, you just need to reboot and the camera should be working fine.

#### RPLidar installation

The installation of the A1M8 RPLidar sensor is quite straight forward and a ros integration package will be installed later on via `rosdep`.

For now, after connecting it to the usb port:
 1. Verify USB connection: Green light in the usb conversor(A1M8 side board) should be turned on.
 2. Check the authority of RPLidar's serial-port:
    - `ls -l /dev |grep ttyUSB`
    - Add extra bits by doing `sudo chmod 666 /dev/ttyUSB<number_of_device>`

### USB Port name configuration

#### Fixed USB port names

As having multiple USB devices connected to the USB ports of the Raspberry Pi, the automatically assigned USB port numbers could unexpectedly change after a reboot.
To avoid assigning your device to a `tty_USBX` number that isn't the correct one we should assign fixed USB port name for each connected device.

The idea is to be able to generate a link between the real `ttyUSBX` port and an invented one. For this we will need to create rules, that every time the Raspberry Pi boots are executed, and therefore we
always point to the correct port name.

In order to create fixed names for the USB devices follow the instructions:

1. Check the devices you have connected:
    ```
    sudo dmesg | grep ttyUSB
    ```

    ```
    [  10.016170] usb 1-1.2: ch341-uart converter now attached to ttyUSB0
    [ 309.186487] usb 1-1.1: cp210x converter now attached to ttyUSB1
    ```
    In the setup where this was tested we have:
      -> Arduino Microcontroller -> _usb 1-1.2: ch341-uart converter now attached to ttyUSB0_
      -> A1M8 Lidar Scanner -> _usb 1-1.1: cp210x converter now attached to ttyUSB1_

    _Note: If you don't know how to identify each one you can simply connect them one by one and check this output._

2. Look for attributes for each device that we will use to anchor a particular device with a name.
  We will use the `idProduct` and `idVendor` of each device.
   - Arduino Microcontroller:
      ```
      udevadm info --name=/dev/ttyUSB0 --attribute-walk
      ```
      You should look for the `idProduct` and `idVendor` under the category that matches the usb number(1-1.X):
      In this case the `ttyUSB0` was referenced to the `usb 1-1.2`, so go to that section and find the ids:
      ```
        ATTRS{idProduct}=="7523"
        ATTRS{idVendor}=="1a86"
      ```
   - Lidar Scanner
      ```
      udevadm info --name=/dev/ttyUSB1 --attribute-walk
      ```
      In this case the `ttyUSB0` was referenced to the `usb 1-1.1`, so go to that section and find the ids:
      ```
        ATTRS{idProduct}=="ea60"
        ATTRS{idVendor}=="10c4"
      ```

3. Create the rules:

    Open the file:
    ```
    sudo nano /etc/udev/rules.d/10-usb-serial.rules
    ```

    Add the following:

    ```
    SUBSYSTEM=="tty", ATTRS{idProduct}=="7523", ATTRS{idVendor}=="1a86", SYMLINK+="ttyUSB_ARDUINO"
    SUBSYSTEM=="tty", ATTRS{idProduct}=="ea60", ATTRS{idVendor}=="10c4", SYMLINK+="ttyUSB_LIDAR"
    ```
    > [!IMPORTANT]
    You should be using your own information you obtained from previous commands, this is just an example.
    Note that in the `symlink` field a fixed name is indicated.

4. Re-trigger the device manager:
    ```
    sudo udevadm trigger
    ```

5. Verify
    ```
    ls -l /dev/ttyUSB*
    ```
    ```
    crw-rw---- 1 root dialout 188, 0 Sep  2 15:09 /dev/ttyUSB0
    crw-rw---- 1 root dialout 188, 1 Sep  2 15:09 /dev/ttyUSB1
    lrwxrwxrwx 1 root root         7 Sep  2 15:09 /dev/ttyUSB_ARDUINO -> ttyUSB0
    lrwxrwxrwx 1 root root         7 Sep  2 15:09 /dev/ttyUSB_LIDAR -> ttyUSB1
    ```

Done! You can always use your devices by the fixed names without using the port number.
Here, `ttyUSB_ARDUINO` and `ttyUSB_LIDAR` are fixed names for the Arduino Microcontroller and the Lidar Scanner respectively.

For more information you can take a look at this external tutorial: [Here](https://www.freva.com/assign-fixed-usb-port-names-to-your-raspberry-pi/)

### Create robot workspace

Let's create our workspace and build from source this repository.

```bash
cd ~
```
```bash
mkdir robot_ws/src -p
```
Clone this repository in the `src` folder
```bash
cd robot_ws/src
```
```bash
git clone <repository_address>
```
Install dependencies via rosdep:
```bash
cd ~/robot_ws
```
When it is the first time you run `rosdep`:
```bash
rosdep update
```
Make sure to export the `ROS_DISTRO` environment variable (use `humble` or `jazzy` depending on your installation):
```bash
export ROS_DISTRO=humble  # or jazzy
```
And then proceed to install the workspace dependencies:
```bash
rosdep install --from-paths src -i -y -r
```
Note that option `-r` has been added. For ARM based processors, there are missing packages, e.g. those related to simulation. We would not try to run the simulation in the compute platform of andino, however for convenience it is added here.

Let's source the ROS installation:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```
Let's build the packages:
```bash
colcon build
```
After building is completed:
```bash
source install/setup.bash
```

After this, you are good to go and use the robot!
Refer to [`usage`](../README.md#usage) section.

### Extra Recommendations & Tools

#### Network
Via terminal the wifi connection can be switched by doing:

List available wifi networks:
```
sudo nmcli dev wifi list
```
Connect to the desired one:
```
sudo nmcli --ask dev wifi connect <SSID>
```

#### Copy files remotely

Using `scp` is a useful tool when copying files remotely over `ssh`.

For copying a folder from host to remote unit:
```
scp -r <path/to/folder> <remote_user>@<remote_ip>:<remote_path_to_folder>
```

#### ROS Domain ID

The domain ID is used by DDS to compute the UDP ports that will be used for discovery and communication.

When using a "public" network using the domain id is a good technique to avoid extra noise with other ROS 2 system in the same network.

See [ROS_DOMAIN_ID](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html)

TLDR? Export an environment variable with the same ID in **all** ROS 2 clients in the network for a correct discovery.
```
export ROS_DOMAIN_ID=<a_number_between_0_and_101>
```

#### Using joystick for teleoperation

[`andino_bringup`](../andino_bringup/launch/teleop_joystick.launch.py) package provides a launch file for launching the corresponding `ROS 2` nodes for teleoperating the robot using a joystick.

It is worth mentioning that a set up might be needed depending on the gamepad you are using. Here some general guidelines:
 - In case you are using a _Xbox One Controller_ and you want use it wireless (via USB Wirless Dongle) installing [Xone](https://github.com/medusalix/xone) is recommended.
 - Verify that your joystick is actually working on Ubuntu:
    - Some tools that might be useful:
      - `sudo apt install joystick jstest-gtk evtest`
    - Run `evtest` to check if your pad is connected:
      ```
      $ evtest
        No device specified, trying to scan all of /dev/input/event*
        Not running as root, no devices may be available.
        Available devices:
          /dev/input/event22:	Microsoft X-Box One pad

      ```
    - Alternatively, you can use `jstest-gtk` to check the controller, you will find a pretty GUI to play with.
