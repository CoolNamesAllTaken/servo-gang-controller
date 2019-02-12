EESchema Schematic File Version 4
LIBS:controller-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
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
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 5C29A42F
P 4250 2400
F 0 "A1" H 3450 2500 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 3350 2400 50  0000 C CNN
F 2 "Modules:Arduino_Nano" H 4400 1450 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 4250 1400 50  0001 C CNN
	1    4250 2400
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U1
U 1 1 5C29A508
P 6150 2000
F 0 "U1" H 5700 1600 50  0000 C CNN
F 1 "74HC595" H 5600 1500 50  0000 C CNN
F 2 "Housings_SSOP:SOP-16_4.4x10.4mm_Pitch1.27mm" H 6150 2000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 6150 2000 50  0001 C CNN
	1    6150 2000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR010
U 1 1 5C29A585
P 6150 1400
F 0 "#PWR010" H 6150 1250 50  0001 C CNN
F 1 "+5V" H 6165 1573 50  0000 C CNN
F 2 "" H 6150 1400 50  0001 C CNN
F 3 "" H 6150 1400 50  0001 C CNN
	1    6150 1400
	1    0    0    -1  
$EndComp
NoConn ~ 4750 1900
NoConn ~ 4750 1800
NoConn ~ 4750 2200
NoConn ~ 4750 2400
NoConn ~ 4750 2500
NoConn ~ 4750 2600
NoConn ~ 4750 2700
NoConn ~ 4750 2800
NoConn ~ 4750 2900
NoConn ~ 4750 3000
NoConn ~ 4750 3100
$Comp
L power:GND #PWR03
U 1 1 5C29A76E
P 4400 3500
F 0 "#PWR03" H 4400 3250 50  0001 C CNN
F 1 "GND" H 4405 3327 50  0000 C CNN
F 2 "" H 4400 3500 50  0001 C CNN
F 3 "" H 4400 3500 50  0001 C CNN
	1    4400 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5C29A794
P 4200 3500
F 0 "#PWR02" H 4200 3250 50  0001 C CNN
F 1 "GND" H 4205 3327 50  0000 C CNN
F 2 "" H 4200 3500 50  0001 C CNN
F 3 "" H 4200 3500 50  0001 C CNN
	1    4200 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3400 4200 3400
Wire Wire Line
	4200 3400 4200 3500
Wire Wire Line
	4350 3400 4400 3400
Wire Wire Line
	4400 3400 4400 3500
$Comp
L power:GND #PWR011
U 1 1 5C29A7F6
P 6150 2700
F 0 "#PWR011" H 6150 2450 50  0001 C CNN
F 1 "GND" H 6155 2527 50  0000 C CNN
F 2 "" H 6150 2700 50  0001 C CNN
F 3 "" H 6150 2700 50  0001 C CNN
	1    6150 2700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male PWM1
U 1 1 5C29A990
P 7700 1050
F 0 "PWM1" H 7673 980 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7673 1071 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7700 1050 50  0001 C CNN
F 3 "~" H 7700 1050 50  0001 C CNN
	1    7700 1050
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male PWM2
U 1 1 5C29AA44
P 7700 1450
F 0 "PWM2" H 7673 1380 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7673 1471 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7700 1450 50  0001 C CNN
F 3 "~" H 7700 1450 50  0001 C CNN
	1    7700 1450
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male PWM3
U 1 1 5C29AA69
P 7700 1850
F 0 "PWM3" H 7673 1780 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7673 1871 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7700 1850 50  0001 C CNN
F 3 "~" H 7700 1850 50  0001 C CNN
	1    7700 1850
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male PWM4
U 1 1 5C29AB94
P 7700 2250
F 0 "PWM4" H 7673 2180 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7673 2271 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7700 2250 50  0001 C CNN
F 3 "~" H 7700 2250 50  0001 C CNN
	1    7700 2250
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male PWM5
U 1 1 5C29AB9B
P 7700 2650
F 0 "PWM5" H 7673 2580 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7673 2671 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7700 2650 50  0001 C CNN
F 3 "~" H 7700 2650 50  0001 C CNN
	1    7700 2650
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male PWM6
U 1 1 5C29ABA2
P 7700 3050
F 0 "PWM6" H 7673 2980 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7673 3071 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7700 3050 50  0001 C CNN
F 3 "~" H 7700 3050 50  0001 C CNN
	1    7700 3050
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male PWM7
U 1 1 5C29ACCD
P 7700 3450
F 0 "PWM7" H 7673 3380 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7673 3471 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7700 3450 50  0001 C CNN
F 3 "~" H 7700 3450 50  0001 C CNN
	1    7700 3450
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male PWM8
U 1 1 5C29ACD4
P 7700 3850
F 0 "PWM8" H 7673 3780 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7673 3871 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7700 3850 50  0001 C CNN
F 3 "~" H 7700 3850 50  0001 C CNN
	1    7700 3850
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR013
U 1 1 5C29AD72
P 7500 1050
F 0 "#PWR013" H 7500 900 50  0001 C CNN
F 1 "+5V" V 7515 1178 50  0000 L CNN
F 2 "" H 7500 1050 50  0001 C CNN
F 3 "" H 7500 1050 50  0001 C CNN
	1    7500 1050
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR015
U 1 1 5C29ADD3
P 7500 1450
F 0 "#PWR015" H 7500 1300 50  0001 C CNN
F 1 "+5V" V 7515 1578 50  0000 L CNN
F 2 "" H 7500 1450 50  0001 C CNN
F 3 "" H 7500 1450 50  0001 C CNN
	1    7500 1450
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR017
U 1 1 5C29ADE9
P 7500 1850
F 0 "#PWR017" H 7500 1700 50  0001 C CNN
F 1 "+5V" V 7515 1978 50  0000 L CNN
F 2 "" H 7500 1850 50  0001 C CNN
F 3 "" H 7500 1850 50  0001 C CNN
	1    7500 1850
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR019
U 1 1 5C29ADFF
P 7500 2250
F 0 "#PWR019" H 7500 2100 50  0001 C CNN
F 1 "+5V" V 7515 2378 50  0000 L CNN
F 2 "" H 7500 2250 50  0001 C CNN
F 3 "" H 7500 2250 50  0001 C CNN
	1    7500 2250
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR021
U 1 1 5C29AE29
P 7500 2650
F 0 "#PWR021" H 7500 2500 50  0001 C CNN
F 1 "+5V" V 7515 2778 50  0000 L CNN
F 2 "" H 7500 2650 50  0001 C CNN
F 3 "" H 7500 2650 50  0001 C CNN
	1    7500 2650
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR023
U 1 1 5C29AE3F
P 7500 3050
F 0 "#PWR023" H 7500 2900 50  0001 C CNN
F 1 "+5V" V 7515 3178 50  0000 L CNN
F 2 "" H 7500 3050 50  0001 C CNN
F 3 "" H 7500 3050 50  0001 C CNN
	1    7500 3050
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR025
U 1 1 5C29AE55
P 7500 3450
F 0 "#PWR025" H 7500 3300 50  0001 C CNN
F 1 "+5V" V 7515 3578 50  0000 L CNN
F 2 "" H 7500 3450 50  0001 C CNN
F 3 "" H 7500 3450 50  0001 C CNN
	1    7500 3450
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR027
U 1 1 5C29AEBB
P 7500 3850
F 0 "#PWR027" H 7500 3700 50  0001 C CNN
F 1 "+5V" V 7515 3978 50  0000 L CNN
F 2 "" H 7500 3850 50  0001 C CNN
F 3 "" H 7500 3850 50  0001 C CNN
	1    7500 3850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5C29AF6E
P 7500 950
F 0 "#PWR012" H 7500 700 50  0001 C CNN
F 1 "GND" V 7505 822 50  0000 R CNN
F 2 "" H 7500 950 50  0001 C CNN
F 3 "" H 7500 950 50  0001 C CNN
	1    7500 950 
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5C29AF9E
P 7500 1350
F 0 "#PWR014" H 7500 1100 50  0001 C CNN
F 1 "GND" V 7505 1222 50  0000 R CNN
F 2 "" H 7500 1350 50  0001 C CNN
F 3 "" H 7500 1350 50  0001 C CNN
	1    7500 1350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5C29AFC7
P 7500 1750
F 0 "#PWR016" H 7500 1500 50  0001 C CNN
F 1 "GND" V 7505 1622 50  0000 R CNN
F 2 "" H 7500 1750 50  0001 C CNN
F 3 "" H 7500 1750 50  0001 C CNN
	1    7500 1750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5C29AFE6
P 7500 2150
F 0 "#PWR018" H 7500 1900 50  0001 C CNN
F 1 "GND" V 7505 2022 50  0000 R CNN
F 2 "" H 7500 2150 50  0001 C CNN
F 3 "" H 7500 2150 50  0001 C CNN
	1    7500 2150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5C29B005
P 7500 2550
F 0 "#PWR020" H 7500 2300 50  0001 C CNN
F 1 "GND" V 7505 2422 50  0000 R CNN
F 2 "" H 7500 2550 50  0001 C CNN
F 3 "" H 7500 2550 50  0001 C CNN
	1    7500 2550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5C29B024
P 7500 2950
F 0 "#PWR022" H 7500 2700 50  0001 C CNN
F 1 "GND" V 7505 2822 50  0000 R CNN
F 2 "" H 7500 2950 50  0001 C CNN
F 3 "" H 7500 2950 50  0001 C CNN
	1    7500 2950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR024
U 1 1 5C29B043
P 7500 3350
F 0 "#PWR024" H 7500 3100 50  0001 C CNN
F 1 "GND" V 7505 3222 50  0000 R CNN
F 2 "" H 7500 3350 50  0001 C CNN
F 3 "" H 7500 3350 50  0001 C CNN
	1    7500 3350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5C29B062
P 7500 3750
F 0 "#PWR026" H 7500 3500 50  0001 C CNN
F 1 "GND" V 7505 3622 50  0000 R CNN
F 2 "" H 7500 3750 50  0001 C CNN
F 3 "" H 7500 3750 50  0001 C CNN
	1    7500 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 1600 6600 1600
Wire Wire Line
	6600 1600 6600 1150
Wire Wire Line
	6600 1150 7500 1150
Wire Wire Line
	6550 1700 6650 1700
Wire Wire Line
	6650 1700 6650 1550
Wire Wire Line
	6650 1550 7500 1550
Wire Wire Line
	6550 1800 7150 1800
Wire Wire Line
	7150 1800 7150 1950
Wire Wire Line
	7150 1950 7500 1950
Wire Wire Line
	6550 1900 7100 1900
Wire Wire Line
	7100 1900 7100 2350
Wire Wire Line
	7100 2350 7500 2350
Wire Wire Line
	6550 2000 7050 2000
Wire Wire Line
	7050 2000 7050 2750
Wire Wire Line
	7050 2750 7500 2750
Wire Wire Line
	6550 2100 7000 2100
Wire Wire Line
	7000 2100 7000 3150
Wire Wire Line
	7000 3150 7500 3150
Wire Wire Line
	6550 2200 6950 2200
Wire Wire Line
	6950 2200 6950 3550
Wire Wire Line
	6950 3550 7500 3550
Wire Wire Line
	6550 2300 6900 2300
Wire Wire Line
	6900 2300 6900 3950
Wire Wire Line
	6900 3950 7500 3950
NoConn ~ 6550 2500
NoConn ~ 3750 1800
NoConn ~ 3750 1900
NoConn ~ 4350 1400
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5C2A7FF0
P 700 750
F 0 "#FLG02" H 700 825 50  0001 C CNN
F 1 "PWR_FLAG" H 700 924 50  0000 C CNN
F 2 "" H 700 750 50  0001 C CNN
F 3 "~" H 700 750 50  0001 C CNN
	1    700  750 
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5C2A8052
P 1050 750
F 0 "#FLG03" H 1050 825 50  0001 C CNN
F 1 "PWR_FLAG" H 1050 924 50  0000 C CNN
F 2 "" H 1050 750 50  0001 C CNN
F 3 "~" H 1050 750 50  0001 C CNN
	1    1050 750 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 5C2A8094
P 700 750
F 0 "#PWR07" H 700 600 50  0001 C CNN
F 1 "+5V" H 715 923 50  0000 C CNN
F 2 "" H 700 750 50  0001 C CNN
F 3 "" H 700 750 50  0001 C CNN
	1    700  750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5C2A80EC
P 1050 750
F 0 "#PWR09" H 1050 500 50  0001 C CNN
F 1 "GND" H 1055 577 50  0000 C CNN
F 2 "" H 1050 750 50  0001 C CNN
F 3 "" H 1050 750 50  0001 C CNN
	1    1050 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2000 3650 2000
Wire Wire Line
	3650 2000 3650 1150
Wire Wire Line
	3650 1150 5700 1150
Wire Wire Line
	5700 1150 5700 1600
Wire Wire Line
	5700 1600 5750 1600
Wire Wire Line
	3750 2100 3600 2100
Wire Wire Line
	3600 2100 3600 1100
Wire Wire Line
	3600 1100 5650 1100
Wire Wire Line
	5650 1100 5650 1800
Wire Wire Line
	5650 1800 5750 1800
Wire Wire Line
	3750 2200 3550 2200
Wire Wire Line
	3550 2200 3550 1050
Wire Wire Line
	3550 1050 5600 1050
Wire Wire Line
	5600 1050 5600 2100
Wire Wire Line
	5600 2100 5750 2100
$Comp
L power:GND #PWR08
U 1 1 5C2AA0A1
P 5750 2200
F 0 "#PWR08" H 5750 1950 50  0001 C CNN
F 1 "GND" V 5755 2072 50  0000 R CNN
F 2 "" H 5750 2200 50  0001 C CNN
F 3 "" H 5750 2200 50  0001 C CNN
	1    5750 2200
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R1
U 1 1 5C2AABBA
P 5400 1900
F 0 "R1" V 5195 1900 50  0000 C CNN
F 1 "10K" V 5286 1900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5440 1890 50  0001 C CNN
F 3 "~" H 5400 1900 50  0001 C CNN
	1    5400 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 1900 5550 1900
$Comp
L power:+5V #PWR06
U 1 1 5C2AB722
P 5250 1900
F 0 "#PWR06" H 5250 1750 50  0001 C CNN
F 1 "+5V" V 5265 2028 50  0000 L CNN
F 2 "" H 5250 1900 50  0001 C CNN
F 3 "" H 5250 1900 50  0001 C CNN
	1    5250 1900
	0    -1   -1   0   
$EndComp
NoConn ~ 3750 2300
NoConn ~ 3750 2400
NoConn ~ 3750 2500
NoConn ~ 3750 2600
NoConn ~ 3750 2700
NoConn ~ 3750 3100
$Comp
L Connector:Conn_01x03_Male PWMIN2
U 1 1 5C5A836B
P 2000 3400
F 0 "PWMIN2" H 2106 3678 50  0000 C CNN
F 1 "Conn_01x03_Male" H 2106 3587 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 2000 3400 50  0001 C CNN
F 3 "~" H 2000 3400 50  0001 C CNN
	1    2000 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 3000 3750 3000
$Comp
L power:GND #PWR0101
U 1 1 5C5A9239
P 2200 3500
F 0 "#PWR0101" H 2200 3250 50  0001 C CNN
F 1 "GND" V 2205 3372 50  0000 R CNN
F 2 "" H 2200 3500 50  0001 C CNN
F 3 "" H 2200 3500 50  0001 C CNN
	1    2200 3500
	0    -1   -1   0   
$EndComp
NoConn ~ 2200 3400
$Comp
L Device:Q_NJFET_GDS Q1
U 1 1 5C5AC2C9
P 2250 1950
F 0 "Q1" H 2440 1904 50  0000 L CNN
F 1 "Q_NJFET_GDS" H 2440 1995 50  0000 L CNN
F 2 "Power_Integrations:TO-220" H 2450 2050 50  0001 C CNN
F 3 "~" H 2250 1950 50  0001 C CNN
	1    2250 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	3250 3000 3250 3300
Text Notes 1300 3400 0    50   ~ 0
PWM Input 2
$Comp
L Connector:Conn_01x02_Male SWPWR1
U 1 1 5C5B0603
P 750 1850
F 0 "SWPWR1" H 856 2028 50  0000 C CNN
F 1 "Conn_01x02_Male" H 856 1937 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 750 1850 50  0001 C CNN
F 3 "~" H 750 1850 50  0001 C CNN
	1    750  1850
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male PWMIN1
U 1 1 5C5B069B
P 2000 2900
F 0 "PWMIN1" H 2106 3178 50  0000 C CNN
F 1 "Conn_01x03_Male" H 2106 3087 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 2000 2900 50  0001 C CNN
F 3 "~" H 2000 2900 50  0001 C CNN
	1    2000 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 2800 3250 2800
Wire Wire Line
	3250 2800 3250 2900
Wire Wire Line
	3250 2900 3750 2900
NoConn ~ 2200 2900
$Comp
L power:GND #PWR0102
U 1 1 5C5B3954
P 2200 3000
F 0 "#PWR0102" H 2200 2750 50  0001 C CNN
F 1 "GND" V 2205 2872 50  0000 R CNN
F 2 "" H 2200 3000 50  0001 C CNN
F 3 "" H 2200 3000 50  0001 C CNN
	1    2200 3000
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x02_Male VIN1
U 1 1 5C5B3DAD
P 2000 4050
F 0 "VIN1" H 2106 4228 50  0000 C CNN
F 1 "Conn_01x02_Male" H 2106 4137 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 2000 4050 50  0001 C CNN
F 3 "~" H 2000 4050 50  0001 C CNN
	1    2000 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5C5B3E53
P 2200 4150
F 0 "#PWR0103" H 2200 3900 50  0001 C CNN
F 1 "GND" V 2205 4022 50  0000 R CNN
F 2 "" H 2200 4150 50  0001 C CNN
F 3 "" H 2200 4150 50  0001 C CNN
	1    2200 4150
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 5C5B3ED3
P 2200 4050
F 0 "#PWR0104" H 2200 3900 50  0001 C CNN
F 1 "+5V" V 2215 4178 50  0000 L CNN
F 2 "" H 2200 4050 50  0001 C CNN
F 3 "" H 2200 4050 50  0001 C CNN
	1    2200 4050
	0    1    1    0   
$EndComp
Text Notes 1300 4100 0    50   ~ 0
Power input
Text Notes 1300 2900 0    50   ~ 0
PWM Input 1
Text Notes 850  1600 0    50   ~ 0
Switchable 5V Power Output
Wire Wire Line
	2900 2700 3350 2700
Wire Wire Line
	3350 2700 3350 2800
Wire Wire Line
	3350 2800 3750 2800
$Comp
L power:GND #PWR0105
U 1 1 5C5B8F7D
P 2150 1750
F 0 "#PWR0105" H 2150 1500 50  0001 C CNN
F 1 "GND" H 2155 1577 50  0000 C CNN
F 2 "" H 2150 1750 50  0001 C CNN
F 3 "" H 2150 1750 50  0001 C CNN
	1    2150 1750
	-1   0    0    1   
$EndComp
Wire Wire Line
	2200 3300 3250 3300
Text Notes 7350 850  0    50   ~ 0
PWM Servo Outputs
Text Notes 5950 1150 0    50   ~ 0
Shift Register
$Comp
L power:GND #PWR0107
U 1 1 5C5C59D7
P 6150 3550
F 0 "#PWR0107" H 6150 3300 50  0001 C CNN
F 1 "GND" H 6155 3377 50  0000 C CNN
F 2 "" H 6150 3550 50  0001 C CNN
F 3 "" H 6150 3550 50  0001 C CNN
	1    6150 3550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0108
U 1 1 5C5C5A22
P 6150 3250
F 0 "#PWR0108" H 6150 3100 50  0001 C CNN
F 1 "+5V" H 6165 3423 50  0000 C CNN
F 2 "" H 6150 3250 50  0001 C CNN
F 3 "" H 6150 3250 50  0001 C CNN
	1    6150 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0109
U 1 1 5C5CC495
P 3600 4100
F 0 "#PWR0109" H 3600 3950 50  0001 C CNN
F 1 "+5V" H 3615 4273 50  0000 C CNN
F 2 "" H 3600 4100 50  0001 C CNN
F 3 "" H 3600 4100 50  0001 C CNN
	1    3600 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5C5CC613
P 6150 3400
F 0 "C3" H 6265 3446 50  0000 L CNN
F 1 "0.1uF" H 6265 3355 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 6188 3250 50  0001 C CNN
F 3 "~" H 6150 3400 50  0001 C CNN
	1    6150 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C2
U 1 1 5C5CFA81
P 3600 4250
F 0 "C2" H 3715 4296 50  0000 L CNN
F 1 "10uF" H 3715 4205 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 3600 4250 50  0001 C CNN
F 3 "~" H 3600 4250 50  0001 C CNN
	1    3600 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5C5CFBCB
P 3600 4400
F 0 "#PWR0110" H 3600 4150 50  0001 C CNN
F 1 "GND" H 3605 4227 50  0000 C CNN
F 2 "" H 3600 4400 50  0001 C CNN
F 3 "" H 3600 4400 50  0001 C CNN
	1    3600 4400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0111
U 1 1 5C5CFCDD
P 3100 4100
F 0 "#PWR0111" H 3100 3950 50  0001 C CNN
F 1 "+5V" H 3115 4273 50  0000 C CNN
F 2 "" H 3100 4100 50  0001 C CNN
F 3 "" H 3100 4100 50  0001 C CNN
	1    3100 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C1
U 1 1 5C5CFD38
P 3100 4250
F 0 "C1" H 3215 4296 50  0000 L CNN
F 1 "100uF" H 3215 4205 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 3100 4250 50  0001 C CNN
F 3 "~" H 3100 4250 50  0001 C CNN
	1    3100 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5C5CFDB4
P 3100 4400
F 0 "#PWR0112" H 3100 4150 50  0001 C CNN
F 1 "GND" H 3105 4227 50  0000 C CNN
F 2 "" H 3100 4400 50  0001 C CNN
F 3 "" H 3100 4400 50  0001 C CNN
	1    3100 4400
	1    0    0    -1  
$EndComp
Text Notes 6000 3950 1    50   ~ 0
Shift Register Bypass Cap
Text Notes 3000 3850 0    50   ~ 0
Board Bypassing Caps
NoConn ~ 4450 1400
$Comp
L power:+5V #PWR0113
U 1 1 5C5D4A16
P 4150 1400
F 0 "#PWR0113" H 4150 1250 50  0001 C CNN
F 1 "+5V" H 4165 1573 50  0000 C CNN
F 2 "" H 4150 1400 50  0001 C CNN
F 3 "" H 4150 1400 50  0001 C CNN
	1    4150 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  1850 1300 1850
$Comp
L Device:D D1
U 1 1 5C5E1220
P 1300 2000
F 0 "D1" V 1254 2079 50  0000 L CNN
F 1 "D" V 1345 2079 50  0000 L CNN
F 2 "Diodes_THT:D_DO-41_SOD81_P7.62mm_Horizontal" H 1300 2000 50  0001 C CNN
F 3 "~" H 1300 2000 50  0001 C CNN
	1    1300 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	950  1950 1050 1950
Wire Wire Line
	1050 1950 1050 2150
Wire Wire Line
	1050 2150 1300 2150
Connection ~ 1300 2150
Wire Wire Line
	2450 1950 2600 1950
Wire Wire Line
	2900 1950 2900 2700
$Comp
L power:+5V #PWR01
U 1 1 5C5EF92B
P 1300 1850
F 0 "#PWR01" H 1300 1700 50  0001 C CNN
F 1 "+5V" H 1315 2023 50  0000 C CNN
F 2 "" H 1300 1850 50  0001 C CNN
F 3 "" H 1300 1850 50  0001 C CNN
	1    1300 1850
	1    0    0    -1  
$EndComp
Connection ~ 1300 1850
Connection ~ 2600 1950
Wire Wire Line
	2600 1950 2900 1950
$Comp
L Device:R_US R2
U 1 1 5C5F3DAD
P 3200 1350
F 0 "R2" H 3268 1396 50  0000 L CNN
F 1 "3K3" H 3268 1305 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3240 1340 50  0001 C CNN
F 3 "~" H 3200 1350 50  0001 C CNN
	1    3200 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1500 3200 1500
$Comp
L power:+5V #PWR04
U 1 1 5C5F54E9
P 3200 1200
F 0 "#PWR04" H 3200 1050 50  0001 C CNN
F 1 "+5V" H 3215 1373 50  0000 C CNN
F 2 "" H 3200 1200 50  0001 C CNN
F 3 "" H 3200 1200 50  0001 C CNN
	1    3200 1200
	1    0    0    -1  
$EndComp
Text Notes 2450 1000 0    50   ~ 0
Power Output Test Button
Wire Wire Line
	1300 2150 2150 2150
$Comp
L Switch:SW_Push SWPWR_TEST1
U 1 1 5C60A109
P 2800 1500
F 0 "SWPWR_TEST1" H 2800 1785 50  0000 C CNN
F 1 "SW_Push" H 2800 1694 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 2800 1700 50  0001 C CNN
F 3 "" H 2800 1700 50  0001 C CNN
	1    2800 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1500 2600 1950
$Comp
L Device:R_US R3
U 1 1 5C62A71F
P 2600 2100
F 0 "R3" H 2668 2146 50  0000 L CNN
F 1 "1M" H 2668 2055 50  0000 L CNN
F 2 "" V 2640 2090 50  0001 C CNN
F 3 "~" H 2600 2100 50  0001 C CNN
	1    2600 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5C62A779
P 2600 2250
F 0 "#PWR05" H 2600 2000 50  0001 C CNN
F 1 "GND" H 2605 2077 50  0000 C CNN
F 2 "" H 2600 2250 50  0001 C CNN
F 3 "" H 2600 2250 50  0001 C CNN
	1    2600 2250
	1    0    0    -1  
$EndComp
$EndSCHEMATC
