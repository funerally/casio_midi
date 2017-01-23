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
LIBS:special
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
LIBS:STM32F407VGT6
LIBS:board-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "23 jan 2017"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STM32F407VGT6 IC1
U 1 1 5883E13E
P 5200 4650
F 0 "IC1" H 4800 9350 50  0000 L CNN
F 1 "STM32F407VGT6" H 4900 2000 50  0000 L BNN
F 2 "QFP50P1600X1600X160-100N" H 5200 4650 50  0001 L CNN
F 3 "LQFP-100 STMicroelectronics" H 5200 4650 50  0001 L CNN
F 4 "STM32F407VGT6" H 5200 4650 50  0001 L CNN "MP"
F 5 "STM32 Series 32-bit 1 MB Flash 192 kB RAM ARM Based Microcontroller - LQFP-100" H 5200 4650 50  0001 L CNN "Description"
F 6 "Good" H 5200 4650 50  0001 L CNN "Availability"
F 7 "10.34 USD" H 5200 4650 50  0001 L CNN "Price"
F 8 "STMicroelectronics" H 5200 4650 50  0001 L CNN "MF"
	1    5200 4650
	-1   0    0    -1  
$EndComp
$Comp
L USB-MINI-B CON1
U 1 1 5883EC2D
P 1250 1450
F 0 "CON1" H 1000 1900 60  0000 C CNN
F 1 "USB-MINI-B" H 1200 950 60  0000 C CNN
F 2 "~" H 1250 1450 60  0000 C CNN
F 3 "~" H 1250 1450 60  0000 C CNN
	1    1250 1450
	-1   0    0    -1  
$EndComp
$Comp
L CRYSTAL X1
U 1 1 5883EEF9
P 3800 2250
F 0 "X1" H 3800 2400 60  0000 C CNN
F 1 "CRYSTAL" H 3800 2100 60  0000 C CNN
F 2 "~" H 3800 2250 60  0000 C CNN
F 3 "~" H 3800 2250 60  0000 C CNN
	1    3800 2250
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 5883EF08
P 7150 2500
F 0 "D1" H 7150 2600 50  0000 C CNN
F 1 "LED" H 7150 2400 50  0000 C CNN
F 2 "~" H 7150 2500 60  0000 C CNN
F 3 "~" H 7150 2500 60  0000 C CNN
	1    7150 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 2800 3500 3300
$Comp
L C C5
U 1 1 58840603
P 3500 2600
F 0 "C5" H 3500 2700 40  0000 L CNN
F 1 "22p" H 3506 2515 40  0000 L CNN
F 2 "~" H 3538 2450 30  0000 C CNN
F 3 "~" H 3500 2600 60  0000 C CNN
	1    3500 2600
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 58840619
P 4100 2600
F 0 "C6" H 4100 2700 40  0000 L CNN
F 1 "22p" H 4106 2515 40  0000 L CNN
F 2 "~" H 4138 2450 30  0000 C CNN
F 3 "~" H 4100 2600 60  0000 C CNN
	1    4100 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3050 4100 2800
Wire Wire Line
	3500 1950 3500 2400
Wire Wire Line
	4100 2400 4100 2250
Wire Wire Line
	4100 2250 4600 2250
Connection ~ 3500 2250
Wire Wire Line
	4600 2150 4100 2150
Wire Wire Line
	4100 2150 4100 1950
Wire Wire Line
	4100 1950 3500 1950
Wire Wire Line
	4100 3050 3500 3050
Connection ~ 3500 3050
$Comp
L GND #PWR01
U 1 1 58840746
P 3500 3300
F 0 "#PWR01" H 3500 3300 30  0001 C CNN
F 1 "GND" H 3500 3230 30  0001 C CNN
F 2 "" H 3500 3300 60  0000 C CNN
F 3 "" H 3500 3300 60  0000 C CNN
	1    3500 3300
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5884084C
P 6350 1800
F 0 "C7" H 6350 1900 40  0000 L CNN
F 1 "100n" H 6356 1715 40  0000 L CNN
F 2 "~" H 6388 1650 30  0000 C CNN
F 3 "~" H 6350 1800 60  0000 C CNN
	1    6350 1800
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 58840852
P 7000 1600
F 0 "R1" V 7080 1600 40  0000 C CNN
F 1 "100k" V 7007 1601 40  0000 C CNN
F 2 "~" V 6930 1600 30  0000 C CNN
F 3 "~" H 7000 1600 30  0000 C CNN
	1    7000 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 1600 6750 1600
Connection ~ 6350 1600
$Comp
L GND #PWR02
U 1 1 58840871
P 6350 2000
F 0 "#PWR02" H 6350 2000 30  0001 C CNN
F 1 "GND" H 6350 1930 30  0001 C CNN
F 2 "" H 6350 2000 60  0000 C CNN
F 3 "" H 6350 2000 60  0000 C CNN
	1    6350 2000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR03
U 1 1 58840880
P 7250 1600
F 0 "#PWR03" H 7250 1560 30  0001 C CNN
F 1 "+3.3V" H 7250 1710 30  0000 C CNN
F 2 "" H 7250 1600 60  0000 C CNN
F 3 "" H 7250 1600 60  0000 C CNN
	1    7250 1600
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5884099F
P 6900 2150
F 0 "R2" V 6980 2150 40  0000 C CNN
F 1 "200" V 6907 2151 40  0000 C CNN
F 2 "~" V 6830 2150 30  0000 C CNN
F 3 "~" H 6900 2150 30  0000 C CNN
	1    6900 2150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 2150 6650 2150
Wire Wire Line
	7150 2150 7150 2300
$Comp
L GND #PWR04
U 1 1 58840BA4
P 7150 2800
F 0 "#PWR04" H 7150 2800 30  0001 C CNN
F 1 "GND" H 7150 2730 30  0001 C CNN
F 2 "" H 7150 2800 60  0000 C CNN
F 3 "" H 7150 2800 60  0000 C CNN
	1    7150 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3850 4600 3850
Wire Wire Line
	3500 3950 4600 3950
Wire Wire Line
	3500 4050 4600 4050
Wire Wire Line
	3500 4150 4600 4150
Wire Wire Line
	3500 4400 4600 4400
Wire Wire Line
	4600 4500 3500 4500
Wire Wire Line
	3500 4600 4600 4600
Wire Wire Line
	4600 4700 3500 4700
Wire Wire Line
	3500 4800 4600 4800
Wire Wire Line
	4600 4900 3500 4900
Wire Wire Line
	3500 5000 4600 5000
Wire Wire Line
	4600 5100 3500 5100
$Comp
L GND #PWR05
U 1 1 588411F3
P 2200 1950
F 0 "#PWR05" H 2200 1950 30  0001 C CNN
F 1 "GND" H 2200 1880 30  0001 C CNN
F 2 "" H 2200 1950 60  0000 C CNN
F 3 "" H 2200 1950 60  0000 C CNN
	1    2200 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 1750 2200 1750
Wire Wire Line
	2200 1750 2200 1950
$Comp
L CONN_1 PE0
U 1 1 5884FD9D
P 3350 3850
F 0 "PE0" H 3430 3850 40  0000 L CNN
F 1 "CONN_1" H 3350 3905 30  0001 C CNN
F 2 "~" H 3350 3850 60  0000 C CNN
F 3 "~" H 3350 3850 60  0000 C CNN
	1    3350 3850
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE1
U 1 1 5884FDAA
P 3350 3950
F 0 "PE1" H 3430 3950 40  0000 L CNN
F 1 "CONN_1" H 3350 4005 30  0001 C CNN
F 2 "~" H 3350 3950 60  0000 C CNN
F 3 "~" H 3350 3950 60  0000 C CNN
	1    3350 3950
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE2
U 1 1 5884FDB0
P 3350 4050
F 0 "PE2" H 3430 4050 40  0000 L CNN
F 1 "CONN_1" H 3350 4105 30  0001 C CNN
F 2 "~" H 3350 4050 60  0000 C CNN
F 3 "~" H 3350 4050 60  0000 C CNN
	1    3350 4050
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE3
U 1 1 5884FDB6
P 3350 4150
F 0 "PE3" H 3430 4150 40  0000 L CNN
F 1 "CONN_1" H 3350 4205 30  0001 C CNN
F 2 "~" H 3350 4150 60  0000 C CNN
F 3 "~" H 3350 4150 60  0000 C CNN
	1    3350 4150
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE4
U 1 1 5884FDBC
P 3350 4400
F 0 "PE4" H 3430 4400 40  0000 L CNN
F 1 "CONN_1" H 3350 4455 30  0001 C CNN
F 2 "~" H 3350 4400 60  0000 C CNN
F 3 "~" H 3350 4400 60  0000 C CNN
	1    3350 4400
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE5
U 1 1 5884FDC2
P 3350 4500
F 0 "PE5" H 3430 4500 40  0000 L CNN
F 1 "CONN_1" H 3350 4555 30  0001 C CNN
F 2 "~" H 3350 4500 60  0000 C CNN
F 3 "~" H 3350 4500 60  0000 C CNN
	1    3350 4500
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE6
U 1 1 5884FDC8
P 3350 4600
F 0 "PE6" H 3430 4600 40  0000 L CNN
F 1 "CONN_1" H 3350 4655 30  0001 C CNN
F 2 "~" H 3350 4600 60  0000 C CNN
F 3 "~" H 3350 4600 60  0000 C CNN
	1    3350 4600
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE7
U 1 1 5884FDCE
P 3350 4700
F 0 "PE7" H 3430 4700 40  0000 L CNN
F 1 "CONN_1" H 3350 4755 30  0001 C CNN
F 2 "~" H 3350 4700 60  0000 C CNN
F 3 "~" H 3350 4700 60  0000 C CNN
	1    3350 4700
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE8
U 1 1 5884FDD4
P 3350 4800
F 0 "PE8" H 3430 4800 40  0000 L CNN
F 1 "CONN_1" H 3350 4855 30  0001 C CNN
F 2 "~" H 3350 4800 60  0000 C CNN
F 3 "~" H 3350 4800 60  0000 C CNN
	1    3350 4800
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE9
U 1 1 5884FDDA
P 3350 4900
F 0 "PE9" H 3430 4900 40  0000 L CNN
F 1 "CONN_1" H 3350 4955 30  0001 C CNN
F 2 "~" H 3350 4900 60  0000 C CNN
F 3 "~" H 3350 4900 60  0000 C CNN
	1    3350 4900
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE10
U 1 1 5884FDE0
P 3350 5000
F 0 "PE10" H 3430 5000 40  0000 L CNN
F 1 "CONN_1" H 3350 5055 30  0001 C CNN
F 2 "~" H 3350 5000 60  0000 C CNN
F 3 "~" H 3350 5000 60  0000 C CNN
	1    3350 5000
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 PE11
U 1 1 5884FDE6
P 3350 5100
F 0 "PE11" H 3430 5100 40  0000 L CNN
F 1 "CONN_1" H 3350 5155 30  0001 C CNN
F 2 "~" H 3350 5100 60  0000 C CNN
F 3 "~" H 3350 5100 60  0000 C CNN
	1    3350 5100
	-1   0    0    -1  
$EndComp
$Comp
L 78L05 U1
U 1 1 588501DC
P 3350 1200
F 0 "U1" H 3500 1004 60  0000 C CNN
F 1 "78L33" H 3350 1400 60  0000 C CNN
F 2 "~" H 3350 1200 60  0000 C CNN
F 3 "~" H 3350 1200 60  0000 C CNN
	1    3350 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 58850AEA
P 3350 1700
F 0 "#PWR06" H 3350 1700 30  0001 C CNN
F 1 "GND" H 3350 1630 30  0001 C CNN
F 2 "" H 3350 1700 60  0000 C CNN
F 3 "" H 3350 1700 60  0000 C CNN
	1    3350 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1450 3350 1700
$Comp
L C C1
U 1 1 58850B8B
P 2550 1350
F 0 "C1" H 2550 1450 40  0000 L CNN
F 1 "100n" H 2556 1265 40  0000 L CNN
F 2 "~" H 2588 1200 30  0000 C CNN
F 3 "~" H 2550 1350 60  0000 C CNN
	1    2550 1350
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 58850B91
P 4150 1350
F 0 "C4" H 4150 1450 40  0000 L CNN
F 1 "100n" H 4156 1265 40  0000 L CNN
F 2 "~" H 4188 1200 30  0000 C CNN
F 3 "~" H 4150 1350 60  0000 C CNN
	1    4150 1350
	1    0    0    -1  
$EndComp
$Comp
L CP1 C2
U 1 1 58850BB3
P 2800 1350
F 0 "C2" H 2850 1450 50  0000 L CNN
F 1 "100u" H 2850 1250 50  0000 L CNN
F 2 "~" H 2800 1350 60  0000 C CNN
F 3 "~" H 2800 1350 60  0000 C CNN
	1    2800 1350
	1    0    0    -1  
$EndComp
$Comp
L CP1 C3
U 1 1 58850BCA
P 3900 1350
F 0 "C3" H 3950 1450 50  0000 L CNN
F 1 "100u" H 3950 1250 50  0000 L CNN
F 2 "~" H 3900 1350 60  0000 C CNN
F 3 "~" H 3900 1350 60  0000 C CNN
	1    3900 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 1550 4150 1550
Connection ~ 2800 1550
Connection ~ 3350 1550
Connection ~ 3900 1550
Wire Wire Line
	1800 1150 2950 1150
Connection ~ 2550 1150
Connection ~ 2800 1150
Wire Wire Line
	3750 1150 4600 1150
Connection ~ 3900 1150
Connection ~ 4150 1150
Wire Wire Line
	4600 1950 4350 1950
Wire Wire Line
	4350 1950 4350 1150
Connection ~ 4350 1150
$Comp
L R R?
U 1 1 588581F9
P 6450 1200
F 0 "R?" V 6530 1200 40  0000 C CNN
F 1 "100k" V 6457 1201 40  0000 C CNN
F 2 "~" V 6380 1200 30  0000 C CNN
F 3 "~" H 6450 1200 30  0000 C CNN
	1    6450 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 1200 6200 1200
$Comp
L GND #PWR?
U 1 1 5885822C
P 6800 1300
F 0 "#PWR?" H 6800 1300 30  0001 C CNN
F 1 "GND" H 6800 1230 30  0001 C CNN
F 2 "" H 6800 1300 60  0000 C CNN
F 3 "" H 6800 1300 60  0000 C CNN
	1    6800 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 1200 6800 1200
Wire Wire Line
	6800 1200 6800 1300
Wire Wire Line
	7150 2700 7150 2800
Text GLabel 1800 1300 2    60   Input ~ 0
USB_DM
Text GLabel 1800 1450 2    60   Input ~ 0
USB_DP
Text GLabel 5800 3250 2    60   Input ~ 0
USB_DM
Text GLabel 5800 3350 2    60   Input ~ 0
USB_DP
Text GLabel 5800 3450 2    60   Input ~ 0
SWDIO
Text GLabel 5800 3550 2    60   Input ~ 0
SWCLK
$EndSCHEMATC
