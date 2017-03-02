EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:switches
LIBS:tx2step
LIBS:tx2step-cache
EELAYER 25 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA328P-MM IC1
U 1 1 58B4E37B
P 2450 4000
F 0 "IC1" H 1700 5250 50  0000 L BNN
F 1 "ATMEGA328P-MM" H 2800 2600 50  0000 L BNN
F 2 "MLF/QFN28" H 2450 4000 50  0000 C CIN
F 3 "" H 2450 4000 50  0000 C CNN
	1    2450 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B4E566
P 1200 3400
F 0 "#PWR?" H 1200 3150 50  0001 C CNN
F 1 "GND" H 1200 3250 50  0000 C CNN
F 2 "" H 1200 3400 50  0000 C CNN
F 3 "" H 1200 3400 50  0000 C CNN
	1    1200 3400
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58B4E57D
P 1200 3300
F 0 "C?" H 1210 3370 50  0000 L CNN
F 1 "10µF" H 1210 3220 50  0000 L CNN
F 2 "" H 1200 3300 50  0000 C CNN
F 3 "" H 1200 3300 50  0000 C CNN
	1    1200 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B4E779
P 1500 5200
F 0 "#PWR?" H 1500 4950 50  0001 C CNN
F 1 "GND" H 1500 5050 50  0000 C CNN
F 2 "" H 1500 5200 50  0000 C CNN
F 3 "" H 1500 5200 50  0000 C CNN
	1    1500 5200
	1    0    0    -1  
$EndComp
$Comp
L RJ45 J?
U 1 1 58B4F4CD
P 5850 3500
F 0 "J?" H 6050 4000 50  0000 C CNN
F 1 "RJ45" H 5700 4000 50  0000 C CNN
F 2 "" H 5850 3500 50  0000 C CNN
F 3 "" H 5850 3500 50  0000 C CNN
	1    5850 3500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 58B4FFB2
P 5150 3350
F 0 "#PWR?" H 5150 3100 50  0001 C CNN
F 1 "GND" H 5150 3200 50  0000 C CNN
F 2 "" H 5150 3350 50  0000 C CNN
F 3 "" H 5150 3350 50  0000 C CNN
	1    5150 3350
	1    0    0    -1  
$EndComp
$Comp
L SW_DPDT_x2 SW?
U 2 1 58B50C38
P 5100 4250
F 0 "SW?" H 5100 4420 50  0000 C CNN
F 1 "SW_DPDT_x2" H 5100 4050 50  0000 C CNN
F 2 "" H 5100 4250 50  0001 C CNN
F 3 "" H 5100 4250 50  0001 C CNN
	2    5100 4250
	-1   0    0    -1  
$EndComp
$Comp
L RJ12 J?
U 1 1 58B5172F
P 9850 1750
F 0 "J?" H 10050 2250 50  0000 C CNN
F 1 "RJ12" H 9700 2250 50  0000 C CNN
F 2 "" H 9850 1750 50  0000 C CNN
F 3 "" H 9850 1750 50  0000 C CNN
	1    9850 1750
	0    1    1    0   
$EndComp
$Comp
L D_TVS D?
U 1 1 58B52001
P 8750 1700
F 0 "D?" H 8750 1800 50  0000 C CNN
F 1 "D_TVS" H 8750 1600 50  0000 C CNN
F 2 "" H 8750 1700 50  0000 C CNN
F 3 "" H 8750 1700 50  0000 C CNN
	1    8750 1700
	0    1    1    0   
$EndComp
$Comp
L D_TVS D?
U 1 1 58B52074
P 9050 2200
F 0 "D?" H 9050 2300 50  0000 C CNN
F 1 "D_TVS" H 9050 2100 50  0000 C CNN
F 2 "" H 9050 2200 50  0000 C CNN
F 3 "" H 9050 2200 50  0000 C CNN
	1    9050 2200
	0    1    1    0   
$EndComp
$Comp
L RJ12 J?
U 1 1 58B53180
P 9850 4800
F 0 "J?" H 10050 5300 50  0000 C CNN
F 1 "RJ12" H 9700 5300 50  0000 C CNN
F 2 "" H 9850 4800 50  0000 C CNN
F 3 "" H 9850 4800 50  0000 C CNN
	1    9850 4800
	0    1    1    0   
$EndComp
$Comp
L D_TVS D?
U 1 1 58B53591
P 8750 4750
F 0 "D?" H 8750 4850 50  0000 C CNN
F 1 "D_TVS" H 8750 4650 50  0000 C CNN
F 2 "" H 8750 4750 50  0000 C CNN
F 3 "" H 8750 4750 50  0000 C CNN
	1    8750 4750
	0    1    1    0   
$EndComp
$Comp
L D_TVS D?
U 1 1 58B539C6
P 9050 5250
F 0 "D?" H 9050 5350 50  0000 C CNN
F 1 "D_TVS" H 9050 5150 50  0000 C CNN
F 2 "" H 9050 5250 50  0000 C CNN
F 3 "" H 9050 5250 50  0000 C CNN
	1    9050 5250
	0    1    1    0   
$EndComp
$Comp
L +5VL #PWR?
U 1 1 58B63E6D
P 1500 2900
F 0 "#PWR?" H 1500 2750 50  0001 C CNN
F 1 "+5VL" H 1500 3040 50  0000 C CNN
F 2 "" H 1500 2900 50  0000 C CNN
F 3 "" H 1500 2900 50  0000 C CNN
	1    1500 2900
	1    0    0    -1  
$EndComp
$Comp
L +5VL #PWR?
U 1 1 58B7953F
P 5150 3250
F 0 "#PWR?" H 5150 3100 50  0001 C CNN
F 1 "+5VL" H 5150 3390 50  0000 C CNN
F 2 "" H 5150 3250 50  0000 C CNN
F 3 "" H 5150 3250 50  0000 C CNN
	1    5150 3250
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 58B7B1BA
P 7500 900
F 0 "#PWR?" H 7500 750 50  0001 C CNN
F 1 "VCC" H 7500 1050 50  0000 C CNN
F 2 "" H 7500 900 50  0000 C CNN
F 3 "" H 7500 900 50  0000 C CNN
	1    7500 900 
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 58B7B246
P 7500 3950
F 0 "#PWR?" H 7500 3800 50  0001 C CNN
F 1 "VCC" H 7500 4100 50  0000 C CNN
F 2 "" H 7500 3950 50  0000 C CNN
F 3 "" H 7500 3950 50  0000 C CNN
	1    7500 3950
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58B7B49A
P 7700 1100
F 0 "C?" H 7710 1170 50  0000 L CNN
F 1 "10nF" H 7710 1020 50  0000 L CNN
F 2 "" H 7700 1100 50  0000 C CNN
F 3 "" H 7700 1100 50  0000 C CNN
	1    7700 1100
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C?
U 1 1 58B7B4F6
P 8000 1100
F 0 "C?" H 8010 1170 50  0000 L CNN
F 1 "10µF" H 8010 1020 50  0000 L CNN
F 2 "" H 8000 1100 50  0000 C CNN
F 3 "" H 8000 1100 50  0000 C CNN
	1    8000 1100
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58B7BBFB
P 7700 4150
F 0 "C?" H 7710 4220 50  0000 L CNN
F 1 "10nF" H 7710 4070 50  0000 L CNN
F 2 "" H 7700 4150 50  0000 C CNN
F 3 "" H 7700 4150 50  0000 C CNN
	1    7700 4150
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C?
U 1 1 58B7BF61
P 8000 4150
F 0 "C?" H 8010 4220 50  0000 L CNN
F 1 "10µF" H 8010 4070 50  0000 L CNN
F 2 "" H 8000 4150 50  0000 C CNN
F 3 "" H 8000 4150 50  0000 C CNN
	1    8000 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B7CB33
P 8000 4250
F 0 "#PWR?" H 8000 4000 50  0001 C CNN
F 1 "GND" H 8000 4100 50  0000 C CNN
F 2 "" H 8000 4250 50  0000 C CNN
F 3 "" H 8000 4250 50  0000 C CNN
	1    8000 4250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B7CE27
P 8000 1200
F 0 "#PWR?" H 8000 950 50  0001 C CNN
F 1 "GND" H 8000 1050 50  0000 C CNN
F 2 "" H 8000 1200 50  0000 C CNN
F 3 "" H 8000 1200 50  0000 C CNN
	1    8000 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B7D2BB
P 7500 3300
F 0 "#PWR?" H 7500 3050 50  0001 C CNN
F 1 "GND" H 7500 3150 50  0000 C CNN
F 2 "" H 7500 3300 50  0000 C CNN
F 3 "" H 7500 3300 50  0000 C CNN
	1    7500 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B7D41F
P 7500 6350
F 0 "#PWR?" H 7500 6100 50  0001 C CNN
F 1 "GND" H 7500 6200 50  0000 C CNN
F 2 "" H 7500 6350 50  0000 C CNN
F 3 "" H 7500 6350 50  0000 C CNN
	1    7500 6350
	1    0    0    -1  
$EndComp
$Comp
L DRV8834 IC?
U 1 1 58B80041
P 7500 2300
F 0 "IC?" H 6950 3250 60  0000 C CNN
F 1 "DRV8834" H 7900 1450 60  0000 C CNN
F 2 "" H 6950 3250 60  0001 C CNN
F 3 "" H 6950 3250 60  0001 C CNN
	1    7500 2300
	1    0    0    -1  
$EndComp
$Comp
L DRV8834 IC?
U 1 1 58B8147E
P 7500 5350
F 0 "IC?" H 6950 6300 60  0000 C CNN
F 1 "DRV8834" H 7900 4500 60  0000 C CNN
F 2 "" H 6950 6300 60  0001 C CNN
F 3 "" H 6950 6300 60  0001 C CNN
	1    7500 5350
	1    0    0    -1  
$EndComp
$Comp
L MCP73831 IC?
U 1 1 58B862EA
P 2650 6250
F 0 "IC?" H 2400 6550 60  0000 C CNN
F 1 "MCP73831" H 2750 5950 60  0000 C CNN
F 2 "" H 2400 6550 60  0001 C CNN
F 3 "" H 2400 6550 60  0001 C CNN
	1    2650 6250
	1    0    0    -1  
$EndComp
$Comp
L SW_SPDT_MSM SW?
U 1 1 58B86814
P 5100 1000
F 0 "SW?" H 5100 1200 50  0000 C CNN
F 1 "SW_SPDT_MSM" H 5100 800 50  0000 C CNN
F 2 "" H 5100 1000 50  0001 C CNN
F 3 "" H 5100 1000 50  0001 C CNN
	1    5100 1000
	1    0    0    -1  
$EndComp
$Comp
L POT RV?
U 1 1 58B868AD
P 5700 1050
F 0 "RV?" V 5525 1050 50  0000 C CNN
F 1 "POT" V 5600 1050 50  0000 C CNN
F 2 "" H 5700 1050 50  0000 C CNN
F 3 "" H 5700 1050 50  0000 C CNN
	1    5700 1050
	1    0    0    -1  
$EndComp
$Comp
L +5VL #PWR?
U 1 1 58B874D3
P 5700 900
F 0 "#PWR?" H 5700 750 50  0001 C CNN
F 1 "+5VL" H 5700 1040 50  0000 C CNN
F 2 "" H 5700 900 50  0000 C CNN
F 3 "" H 5700 900 50  0000 C CNN
	1    5700 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B87520
P 5700 1200
F 0 "#PWR?" H 5700 950 50  0001 C CNN
F 1 "GND" H 5700 1050 50  0000 C CNN
F 2 "" H 5700 1200 50  0000 C CNN
F 3 "" H 5700 1200 50  0000 C CNN
	1    5700 1200
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B876D6
P 4700 900
F 0 "R?" H 4730 920 50  0000 L CNN
F 1 "4.7KΩ" H 4730 860 50  0000 L CNN
F 2 "" H 4700 900 50  0000 C CNN
F 3 "" H 4700 900 50  0000 C CNN
	1    4700 900 
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B87737
P 4700 1100
F 0 "R?" H 4730 1120 50  0000 L CNN
F 1 "4.7KΩ" H 4730 1060 50  0000 L CNN
F 2 "" H 4700 1100 50  0000 C CNN
F 3 "" H 4700 1100 50  0000 C CNN
	1    4700 1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B895AD
P 4700 1200
F 0 "#PWR?" H 4700 950 50  0001 C CNN
F 1 "GND" H 4700 1050 50  0000 C CNN
F 2 "" H 4700 1200 50  0000 C CNN
F 3 "" H 4700 1200 50  0000 C CNN
	1    4700 1200
	1    0    0    -1  
$EndComp
$Comp
L +5VL #PWR?
U 1 1 58B89600
P 4700 800
F 0 "#PWR?" H 4700 650 50  0001 C CNN
F 1 "+5VL" H 4700 940 50  0000 C CNN
F 2 "" H 4700 800 50  0000 C CNN
F 3 "" H 4700 800 50  0000 C CNN
	1    4700 800 
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y?
U 1 1 58B8A677
P 5050 2500
F 0 "Y?" H 5050 2650 50  0000 C CNN
F 1 "16 MHz" H 5050 2350 50  0000 C CNN
F 2 "" H 5050 2500 50  0000 C CNN
F 3 "" H 5050 2500 50  0000 C CNN
	1    5050 2500
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C?
U 1 1 58B8A7FC
P 5400 2350
F 0 "C?" H 5410 2420 50  0000 L CNN
F 1 "C_Small" H 5410 2270 50  0000 L CNN
F 2 "" H 5400 2350 50  0000 C CNN
F 3 "" H 5400 2350 50  0000 C CNN
	1    5400 2350
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C?
U 1 1 58B8A9A7
P 5400 2650
F 0 "C?" H 5410 2720 50  0000 L CNN
F 1 "C_Small" H 5410 2570 50  0000 L CNN
F 2 "" H 5400 2650 50  0000 C CNN
F 3 "" H 5400 2650 50  0000 C CNN
	1    5400 2650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 58B8AC8A
P 5650 2500
F 0 "#PWR?" H 5650 2250 50  0001 C CNN
F 1 "GND" H 5650 2350 50  0000 C CNN
F 2 "" H 5650 2500 50  0000 C CNN
F 3 "" H 5650 2500 50  0000 C CNN
	1    5650 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 58B92C90
P 6450 1850
F 0 "#PWR?" H 6450 1600 50  0001 C CNN
F 1 "GND" H 6450 1700 50  0000 C CNN
F 2 "" H 6450 1850 50  0000 C CNN
F 3 "" H 6450 1850 50  0000 C CNN
	1    6450 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B95301
P 6450 4900
F 0 "#PWR?" H 6450 4650 50  0001 C CNN
F 1 "GND" H 6450 4750 50  0000 C CNN
F 2 "" H 6450 4900 50  0000 C CNN
F 3 "" H 6450 4900 50  0000 C CNN
	1    6450 4900
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58B95FE4
P 8300 4150
F 0 "C?" H 8310 4220 50  0000 L CNN
F 1 "10nF" H 8310 4070 50  0000 L CNN
F 2 "" H 8300 4150 50  0000 C CNN
F 3 "" H 8300 4150 50  0000 C CNN
	1    8300 4150
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58B96443
P 8300 1100
F 0 "C?" H 8310 1170 50  0000 L CNN
F 1 "10nF" H 8310 1020 50  0000 L CNN
F 2 "" H 8300 1100 50  0000 C CNN
F 3 "" H 8300 1100 50  0000 C CNN
	1    8300 1100
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B971C6
P 6450 2850
F 0 "R?" H 6480 2870 50  0000 L CNN
F 1 "47KΩ" H 6480 2810 50  0000 L CNN
F 2 "" H 6450 2850 50  0000 C CNN
F 3 "" H 6450 2850 50  0000 C CNN
	1    6450 2850
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R?
U 1 1 58B97329
P 6450 5900
F 0 "R?" H 6480 5920 50  0000 L CNN
F 1 "47KΩ" H 6480 5860 50  0000 L CNN
F 2 "" H 6450 5900 50  0000 C CNN
F 3 "" H 6450 5900 50  0000 C CNN
	1    6450 5900
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 58B97615
P 6200 2850
F 0 "#PWR?" H 6200 2600 50  0001 C CNN
F 1 "GND" H 6200 2700 50  0000 C CNN
F 2 "" H 6200 2850 50  0000 C CNN
F 3 "" H 6200 2850 50  0000 C CNN
	1    6200 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B980BD
P 6200 5900
F 0 "#PWR?" H 6200 5650 50  0001 C CNN
F 1 "GND" H 6200 5750 50  0000 C CNN
F 2 "" H 6200 5900 50  0000 C CNN
F 3 "" H 6200 5900 50  0000 C CNN
	1    6200 5900
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B98502
P 8550 2950
F 0 "R?" H 8580 2970 50  0000 L CNN
F 1 "0.1Ω" H 8580 2910 50  0000 L CNN
F 2 "" H 8550 2950 50  0000 C CNN
F 3 "" H 8550 2950 50  0000 C CNN
	1    8550 2950
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B98573
P 8850 2950
F 0 "R?" H 8880 2970 50  0000 L CNN
F 1 "0.1Ω" H 8880 2910 50  0000 L CNN
F 2 "" H 8850 2950 50  0000 C CNN
F 3 "" H 8850 2950 50  0000 C CNN
	1    8850 2950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B9897E
P 8850 3100
F 0 "#PWR?" H 8850 2850 50  0001 C CNN
F 1 "GND" H 8850 2950 50  0000 C CNN
F 2 "" H 8850 3100 50  0000 C CNN
F 3 "" H 8850 3100 50  0000 C CNN
	1    8850 3100
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B98E8F
P 8550 6000
F 0 "R?" H 8580 6020 50  0000 L CNN
F 1 "0.1Ω" H 8580 5960 50  0000 L CNN
F 2 "" H 8550 6000 50  0000 C CNN
F 3 "" H 8550 6000 50  0000 C CNN
	1    8550 6000
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B98F26
P 8850 6000
F 0 "R?" H 8880 6020 50  0000 L CNN
F 1 "0.1Ω" H 8880 5960 50  0000 L CNN
F 2 "" H 8850 6000 50  0000 C CNN
F 3 "" H 8850 6000 50  0000 C CNN
	1    8850 6000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58B993B1
P 8850 6200
F 0 "#PWR?" H 8850 5950 50  0001 C CNN
F 1 "GND" H 8850 6050 50  0000 C CNN
F 2 "" H 8850 6200 50  0000 C CNN
F 3 "" H 8850 6200 50  0000 C CNN
	1    8850 6200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58B996D7
P 9100 5700
F 0 "C?" H 9110 5770 50  0000 L CNN
F 1 "2.2µF" H 9110 5620 50  0000 L CNN
F 2 "" H 9100 5700 50  0000 C CNN
F 3 "" H 9100 5700 50  0000 C CNN
	1    9100 5700
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58B99B41
P 9100 2650
F 0 "C?" H 9110 2720 50  0000 L CNN
F 1 "2.2µF" H 9110 2570 50  0000 L CNN
F 2 "" H 9100 2650 50  0000 C CNN
F 3 "" H 9100 2650 50  0000 C CNN
	1    9100 2650
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B9B75C
P 6200 2450
F 0 "R?" H 6230 2470 50  0000 L CNN
F 1 "1KΩ" H 6230 2410 50  0000 L CNN
F 2 "" H 6200 2450 50  0000 C CNN
F 3 "" H 6200 2450 50  0000 C CNN
	1    6200 2450
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B9B7E8
P 6200 2650
F 0 "R?" H 6230 2670 50  0000 L CNN
F 1 "30Ω" H 6230 2610 50  0000 L CNN
F 2 "" H 6200 2650 50  0000 C CNN
F 3 "" H 6200 2650 50  0000 C CNN
	1    6200 2650
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B9BECE
P 6200 5700
F 0 "R?" H 6230 5720 50  0000 L CNN
F 1 "30Ω" H 6230 5660 50  0000 L CNN
F 2 "" H 6200 5700 50  0000 C CNN
F 3 "" H 6200 5700 50  0000 C CNN
	1    6200 5700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58B9C30F
P 6200 5500
F 0 "R?" H 6230 5520 50  0000 L CNN
F 1 "1KΩ" H 6230 5460 50  0000 L CNN
F 2 "" H 6200 5500 50  0000 C CNN
F 3 "" H 6200 5500 50  0000 C CNN
	1    6200 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2900 1550 2900
Wire Wire Line
	1500 3500 1550 3500
Wire Wire Line
	1500 5200 1550 5200
Wire Wire Line
	3450 4700 4300 4700
Wire Wire Line
	4300 4700 4300 1650
Wire Wire Line
	4300 1650 6700 1650
Wire Wire Line
	3450 4800 4400 4800
Wire Wire Line
	4400 4800 4400 1750
Wire Wire Line
	4400 1750 6700 1750
Wire Wire Line
	4700 3750 3450 3750
Wire Wire Line
	4700 3050 4700 3750
Wire Wire Line
	5400 3850 3450 3850
Wire Wire Line
	3450 3300 4900 3300
Wire Wire Line
	4900 3300 4900 3650
Wire Wire Line
	4900 3650 5400 3650
Wire Wire Line
	3450 3400 5000 3400
Wire Wire Line
	5000 3400 5000 3750
Wire Wire Line
	5000 3750 5400 3750
Wire Wire Line
	3450 3200 4800 3200
Wire Wire Line
	4800 3200 4800 3550
Wire Wire Line
	4800 3550 5400 3550
Wire Wire Line
	5150 3350 5400 3350
Wire Wire Line
	5150 3250 5400 3250
Wire Wire Line
	3450 5000 4700 5000
Wire Wire Line
	4700 5000 4700 4700
Wire Wire Line
	4700 4700 6700 4700
Wire Wire Line
	3450 3100 4200 3100
Wire Wire Line
	4200 3100 4200 4150
Wire Wire Line
	4200 4150 4900 4150
Wire Wire Line
	3450 4350 4900 4350
Wire Wire Line
	1550 5100 1500 5100
Wire Wire Line
	1500 5100 1500 5200
Wire Wire Line
	5400 3450 5300 3450
Wire Wire Line
	5300 3450 5300 4250
Wire Wire Line
	4700 3050 5400 3050
Wire Wire Line
	5400 3050 5400 3150
Wire Wire Line
	8500 1750 8300 1750
Wire Wire Line
	8500 1550 8500 1750
Wire Wire Line
	8300 1850 8750 1850
Wire Wire Line
	8750 1850 9400 1850
Wire Wire Line
	9150 1550 9150 1750
Wire Wire Line
	9150 1750 9400 1750
Connection ~ 8750 1850
Wire Wire Line
	8300 2050 9050 2050
Wire Wire Line
	9050 2050 9050 1650
Wire Wire Line
	9050 1650 9400 1650
Wire Wire Line
	9400 1950 9250 1950
Wire Wire Line
	9250 2350 9250 1950
Wire Wire Line
	8850 2350 9050 2350
Wire Wire Line
	9050 2350 9250 2350
Wire Wire Line
	8850 2350 8850 2150
Wire Wire Line
	8850 2150 8300 2150
Connection ~ 9050 2350
Wire Wire Line
	8300 4800 8450 4800
Wire Wire Line
	8450 4800 8450 4600
Wire Wire Line
	8450 4600 8750 4600
Wire Wire Line
	8750 4600 9150 4600
Wire Wire Line
	8300 4900 8750 4900
Wire Wire Line
	8750 4900 9400 4900
Wire Wire Line
	9150 4600 9150 4800
Wire Wire Line
	9150 4800 9400 4800
Connection ~ 8750 4600
Connection ~ 8750 4900
Wire Wire Line
	8300 5100 9050 5100
Wire Wire Line
	8300 5200 8850 5200
Wire Wire Line
	8850 5200 8850 5400
Wire Wire Line
	8850 5400 9050 5400
Wire Wire Line
	9050 5400 9250 5400
Wire Wire Line
	9050 5100 9050 4700
Wire Wire Line
	9050 4700 9400 4700
Wire Wire Line
	9250 5400 9250 5000
Wire Wire Line
	9250 5000 9400 5000
Connection ~ 9050 5400
Wire Wire Line
	3450 5100 4800 5100
Wire Wire Line
	4800 5100 4800 4800
Wire Wire Line
	4800 4800 6700 4800
Wire Wire Line
	1500 2900 1500 3200
Wire Wire Line
	1500 3200 1500 3500
Wire Wire Line
	1200 3200 1500 3200
Wire Wire Line
	1500 3200 1550 3200
Connection ~ 1500 3200
Wire Wire Line
	7500 1200 7600 1200
Wire Wire Line
	7500 4250 7600 4250
Wire Wire Line
	7500 1000 7700 1000
Wire Wire Line
	7700 1000 8000 1000
Wire Wire Line
	7500 900  7500 1000
Wire Wire Line
	7500 1000 7500 1200
Wire Wire Line
	7700 1200 8000 1200
Wire Wire Line
	7500 4050 7700 4050
Wire Wire Line
	7700 4050 8000 4050
Wire Wire Line
	8000 4250 7700 4250
Connection ~ 7700 1000
Connection ~ 7700 4050
Wire Wire Line
	7500 3950 7500 4050
Wire Wire Line
	7500 4050 7500 4250
Wire Wire Line
	8500 1550 8750 1550
Wire Wire Line
	8750 1550 9150 1550
Connection ~ 8750 1550
Wire Wire Line
	5300 900  5700 900 
Wire Wire Line
	5300 1100 5450 1100
Wire Wire Line
	5450 1100 5450 1200
Wire Wire Line
	5450 1200 5700 1200
Wire Wire Line
	4600 1000 4700 1000
Wire Wire Line
	4700 1000 4900 1000
Wire Wire Line
	4600 3950 3450 3950
Connection ~ 4700 1000
Wire Wire Line
	4100 4050 3450 4050
Wire Wire Line
	4100 1450 5850 1450
Wire Wire Line
	5850 1450 5850 1050
Wire Wire Line
	4100 1450 4100 4050
Wire Wire Line
	4600 1000 4600 3950
Wire Wire Line
	3900 2350 5050 2350
Wire Wire Line
	5050 2350 5300 2350
Wire Wire Line
	4000 2650 5050 2650
Wire Wire Line
	5050 2650 5300 2650
Wire Wire Line
	3900 2350 3900 3500
Wire Wire Line
	3900 3500 3450 3500
Connection ~ 5050 2350
Wire Wire Line
	3450 3600 4000 3600
Wire Wire Line
	4000 3600 4000 2650
Connection ~ 5050 2650
Wire Wire Line
	5500 2350 5650 2350
Wire Wire Line
	5650 2350 5650 2500
Wire Wire Line
	5650 2500 5650 2650
Wire Wire Line
	5650 2650 5500 2650
Connection ~ 5650 2500
Wire Wire Line
	3450 4900 4500 4900
Wire Wire Line
	6700 2050 4500 2050
Wire Wire Line
	4500 2050 4500 4900
Wire Wire Line
	3450 5200 4900 5200
Wire Wire Line
	4900 5200 4900 5100
Wire Wire Line
	4900 5100 6700 5100
Wire Wire Line
	6700 2250 6600 2250
Wire Wire Line
	6600 2250 6600 3550
Wire Wire Line
	6600 3550 8450 3550
Wire Wire Line
	8450 3550 8450 2550
Wire Wire Line
	8300 2550 8450 2550
Wire Wire Line
	8450 2550 9050 2550
Wire Wire Line
	6700 5300 6600 5300
Wire Wire Line
	6600 5300 6600 6600
Wire Wire Line
	6600 6600 8450 6600
Wire Wire Line
	8450 6600 8450 5600
Wire Wire Line
	8300 5600 8450 5600
Wire Wire Line
	8450 5600 9100 5600
Wire Wire Line
	6450 1850 6600 1850
Wire Wire Line
	6600 1850 6700 1850
Wire Wire Line
	6700 1950 6600 1950
Wire Wire Line
	6600 1950 6600 1850
Connection ~ 6600 1850
Wire Wire Line
	6450 4900 6600 4900
Wire Wire Line
	6600 4900 6700 4900
Wire Wire Line
	6700 5000 6600 5000
Wire Wire Line
	6600 5000 6600 4900
Connection ~ 6600 4900
Connection ~ 7500 1000
Connection ~ 7500 4050
Wire Wire Line
	8300 4250 8300 4600
Wire Wire Line
	8300 4050 8300 3950
Wire Wire Line
	8300 3950 7500 3950
Wire Wire Line
	8300 1200 8300 1550
Wire Wire Line
	8300 1000 8300 900 
Wire Wire Line
	8300 900  7500 900 
Wire Wire Line
	6550 2850 6700 2850
Wire Wire Line
	6350 2850 6200 2850
Wire Wire Line
	6550 5900 6700 5900
Wire Wire Line
	6200 5900 6350 5900
Wire Wire Line
	8550 2850 8300 2850
Wire Wire Line
	8300 2750 8850 2750
Wire Wire Line
	8850 3050 8850 3100
Wire Wire Line
	8550 3100 8850 3100
Wire Wire Line
	8850 3100 9100 3100
Wire Wire Line
	8850 5900 8850 5800
Wire Wire Line
	8850 5800 8300 5800
Wire Wire Line
	8300 5900 8550 5900
Wire Wire Line
	8550 3100 8550 3050
Wire Wire Line
	8550 6100 8550 6200
Wire Wire Line
	8550 6200 8850 6200
Wire Wire Line
	8850 6200 9100 6200
Wire Wire Line
	8850 6200 8850 6100
Connection ~ 8450 5600
Wire Wire Line
	9100 6200 9100 5800
Connection ~ 8850 6200
Connection ~ 8450 2550
Wire Wire Line
	9100 3100 9100 2750
Connection ~ 8850 3100
Wire Wire Line
	6700 2450 6500 2450
Wire Wire Line
	6500 2450 6500 2350
Wire Wire Line
	6500 2350 6200 2350
Wire Wire Line
	6200 2550 6700 2550
Wire Wire Line
	6700 2550 6700 2650
Wire Wire Line
	6200 2850 6200 2750
Wire Wire Line
	6200 5600 6700 5600
Wire Wire Line
	6700 5600 6700 5700
Wire Wire Line
	6700 5500 6500 5500
Wire Wire Line
	6500 5500 6500 5400
Wire Wire Line
	6500 5400 6200 5400
Wire Wire Line
	6200 5800 6200 5900
Wire Wire Line
	8850 2750 8850 2850
$Comp
L TPS61202 IC?
U 1 1 58BACDA2
P 4550 6350
F 0 "IC?" H 4250 6800 60  0000 C CNN
F 1 "TPS61202" H 4700 5900 60  0000 C CNN
F 2 "" H 4300 6850 60  0001 C CNN
F 3 "" H 4300 6850 60  0001 C CNN
	1    4550 6350
	1    0    0    -1  
$EndComp
$Comp
L TPS2113A IC?
U 1 1 58BAD11D
P 2650 7350
F 0 "IC?" H 2450 7650 60  0000 C CNN
F 1 "TPS2113A" H 2700 7050 60  0000 C CNN
F 2 "" H 2300 7800 60  0001 C CNN
F 3 "" H 2300 7800 60  0001 C CNN
	1    2650 7350
	1    0    0    -1  
$EndComp
$Comp
L BARREL_JACK CON?
U 1 1 58BAF2CD
P 1000 6200
F 0 "CON?" H 1000 6450 50  0000 C CNN
F 1 "BARREL_JACK" H 1000 6000 50  0000 C CNN
F 2 "" H 1000 6200 50  0000 C CNN
F 3 "" H 1000 6200 50  0000 C CNN
	1    1000 6200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
