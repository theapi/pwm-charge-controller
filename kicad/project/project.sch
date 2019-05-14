EESchema Schematic File Version 4
LIBS:project-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "PWM Solar Charge Controller"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_ST_STM32L0:STM32L031F4Px U3
U 1 1 5C7AC20A
P 2300 4875
F 0 "U3" H 2425 4125 50  0000 C CNN
F 1 "STM32L031F4Px" H 2675 4050 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 1900 4175 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00140359.pdf" H 2300 4875 50  0001 C CNN
	1    2300 4875
	1    0    0    -1  
$EndComp
Text GLabel 2800 5475 2    50   Input ~ 0
SWCLK
Text GLabel 2800 5375 2    50   Input ~ 0
SWDI0
Text GLabel 2800 4575 2    50   Input ~ 0
UART_TX
Text GLabel 1800 4375 0    50   Input ~ 0
NRST
$Comp
L power:GND #PWR018
U 1 1 5C7AC377
P 2300 5675
F 0 "#PWR018" H 2300 5425 50  0001 C CNN
F 1 "GND" H 2305 5502 50  0000 C CNN
F 2 "" H 2300 5675 50  0001 C CNN
F 3 "" H 2300 5675 50  0001 C CNN
	1    2300 5675
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR017
U 1 1 5C7ACC5E
P 2300 3525
F 0 "#PWR017" H 2300 3375 50  0001 C CNN
F 1 "+3V3" H 2315 3698 50  0000 C CNN
F 2 "" H 2300 3525 50  0001 C CNN
F 3 "" H 2300 3525 50  0001 C CNN
	1    2300 3525
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3625 2300 4175
Wire Wire Line
	2400 4175 2400 3625
Wire Wire Line
	2400 3625 2300 3625
Connection ~ 2300 3625
$Comp
L Device:C_Small C6
U 1 1 5C7ACD13
P 1975 3725
F 0 "C6" H 2067 3771 50  0000 L CNN
F 1 "0.1uF" H 2067 3680 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1975 3725 50  0001 C CNN
F 3 "~" H 1975 3725 50  0001 C CNN
	1    1975 3725
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5C7ACD67
P 2625 3725
F 0 "C7" H 2717 3771 50  0000 L CNN
F 1 "0.1uF" H 2717 3680 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2625 3725 50  0001 C CNN
F 3 "~" H 2625 3725 50  0001 C CNN
	1    2625 3725
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3625 2300 3525
Wire Wire Line
	1975 3625 2300 3625
Wire Wire Line
	2625 3625 2400 3625
Connection ~ 2400 3625
$Comp
L power:GND #PWR015
U 1 1 5C7ACE2D
P 1975 3825
F 0 "#PWR015" H 1975 3575 50  0001 C CNN
F 1 "GND" H 1980 3652 50  0000 C CNN
F 2 "" H 1975 3825 50  0001 C CNN
F 3 "" H 1975 3825 50  0001 C CNN
	1    1975 3825
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5C7ACE47
P 2625 3825
F 0 "#PWR019" H 2625 3575 50  0001 C CNN
F 1 "GND" H 2630 3652 50  0000 C CNN
F 2 "" H 2625 3825 50  0001 C CNN
F 3 "" H 2625 3825 50  0001 C CNN
	1    2625 3825
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5C7ACF3C
P 3075 3725
F 0 "C8" H 3167 3771 50  0000 L CNN
F 1 "1uF" H 3167 3680 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3075 3725 50  0001 C CNN
F 3 "~" H 3075 3725 50  0001 C CNN
	1    3075 3725
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5C7ACF8A
P 1575 3725
F 0 "C4" H 1667 3771 50  0000 L CNN
F 1 "10uF" H 1667 3680 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1575 3725 50  0001 C CNN
F 3 "~" H 1575 3725 50  0001 C CNN
	1    1575 3725
	1    0    0    -1  
$EndComp
Wire Wire Line
	1575 3625 1975 3625
Connection ~ 1975 3625
Wire Wire Line
	3075 3625 2625 3625
Connection ~ 2625 3625
$Comp
L power:GND #PWR012
U 1 1 5C7AD02B
P 1575 3825
F 0 "#PWR012" H 1575 3575 50  0001 C CNN
F 1 "GND" H 1580 3652 50  0000 C CNN
F 2 "" H 1575 3825 50  0001 C CNN
F 3 "" H 1575 3825 50  0001 C CNN
	1    1575 3825
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5C7AD040
P 3075 3825
F 0 "#PWR020" H 3075 3575 50  0001 C CNN
F 1 "GND" H 3080 3652 50  0000 C CNN
F 2 "" H 3075 3825 50  0001 C CNN
F 3 "" H 3075 3825 50  0001 C CNN
	1    3075 3825
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5C7B0980
P 1650 1300
F 0 "C3" H 1742 1346 50  0000 L CNN
F 1 "0.1uF" H 1742 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1650 1300 50  0001 C CNN
F 3 "~" H 1650 1300 50  0001 C CNN
	1    1650 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5C7B0F3D
P 3475 1800
F 0 "#PWR013" H 3475 1550 50  0001 C CNN
F 1 "GND" H 3480 1627 50  0000 C CNN
F 2 "" H 3475 1800 50  0001 C CNN
F 3 "" H 3475 1800 50  0001 C CNN
	1    3475 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5C7B0F7E
P 1650 1400
F 0 "#PWR010" H 1650 1150 50  0001 C CNN
F 1 "GND" H 1655 1227 50  0000 C CNN
F 2 "" H 1650 1400 50  0001 C CNN
F 3 "" H 1650 1400 50  0001 C CNN
	1    1650 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR016
U 1 1 5C7B128E
P 3875 1600
F 0 "#PWR016" H 3875 1450 50  0001 C CNN
F 1 "+3V3" H 3890 1773 50  0000 C CNN
F 2 "" H 3875 1600 50  0001 C CNN
F 3 "" H 3875 1600 50  0001 C CNN
	1    3875 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5C7B13E3
P 2575 1900
F 0 "#PWR07" H 2575 1650 50  0001 C CNN
F 1 "GND" H 2580 1727 50  0000 C CNN
F 2 "" H 2575 1900 50  0001 C CNN
F 3 "" H 2575 1900 50  0001 C CNN
	1    2575 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4875 5250 4875
$Comp
L Device:R R17
U 1 1 5C7C0432
P 6425 6725
F 0 "R17" H 6495 6771 50  0000 L CNN
F 1 "10K" H 6495 6680 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6355 6725 50  0001 C CNN
F 3 "~" H 6425 6725 50  0001 C CNN
	1    6425 6725
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 5C7C051B
P 6425 7175
F 0 "D4" V 6463 7058 50  0000 R CNN
F 1 "LED" V 6372 7058 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6425 7175 50  0001 C CNN
F 3 "~" H 6425 7175 50  0001 C CNN
	1    6425 7175
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR032
U 1 1 5C7C05D8
P 6425 7325
F 0 "#PWR032" H 6425 7075 50  0001 C CNN
F 1 "GND" H 6430 7152 50  0000 C CNN
F 2 "" H 6425 7325 50  0001 C CNN
F 3 "" H 6425 7325 50  0001 C CNN
	1    6425 7325
	1    0    0    -1  
$EndComp
Wire Wire Line
	6425 7025 6425 6875
$Comp
L Device:LED D1
U 1 1 5C7C3D9B
P 900 5550
F 0 "D1" V 925 5725 50  0000 R CNN
F 1 "LED" V 825 5775 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 900 5550 50  0001 C CNN
F 3 "~" H 900 5550 50  0001 C CNN
	1    900  5550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5C7C3FBD
P 1575 5175
F 0 "R1" V 1675 5175 50  0000 C CNN
F 1 "1K" V 1575 5175 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 1505 5175 50  0001 C CNN
F 3 "~" H 1575 5175 50  0001 C CNN
	1    1575 5175
	0    -1   -1   0   
$EndComp
Text Label 1150 5175 0    50   ~ 0
LED_1
$Comp
L Device:R R15
U 1 1 5C7C5AEB
P 5600 3975
F 0 "R15" H 5670 4021 50  0000 L CNN
F 1 "100K" H 5670 3930 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5530 3975 50  0001 C CNN
F 3 "~" H 5600 3975 50  0001 C CNN
	1    5600 3975
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5C7C5BA1
P 5600 3475
F 0 "R14" H 5670 3521 50  0000 L CNN
F 1 "620K" H 5670 3430 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5530 3475 50  0001 C CNN
F 3 "~" H 5600 3475 50  0001 C CNN
	1    5600 3475
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3625 5600 3725
Connection ~ 5600 3725
Wire Wire Line
	5600 3725 5600 3825
$Comp
L power:GND #PWR027
U 1 1 5C7C73C4
P 5600 4125
F 0 "#PWR027" H 5600 3875 50  0001 C CNN
F 1 "GND" H 5605 3952 50  0000 C CNN
F 2 "" H 5600 4125 50  0001 C CNN
F 3 "" H 5600 4125 50  0001 C CNN
	1    5600 4125
	1    0    0    -1  
$EndComp
Text Label 4375 4475 0    50   ~ 0
BATT_SENSE
$Comp
L Device:C_Small C9
U 1 1 5C7CB72F
P 5125 4225
F 0 "C9" H 5217 4271 50  0000 L CNN
F 1 "1uF" H 5217 4180 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5125 4225 50  0001 C CNN
F 3 "~" H 5125 4225 50  0001 C CNN
	1    5125 4225
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5C7CB7D1
P 5125 4325
F 0 "#PWR026" H 5125 4075 50  0001 C CNN
F 1 "GND" H 5130 4152 50  0000 C CNN
F 2 "" H 5125 4325 50  0001 C CNN
F 3 "" H 5125 4325 50  0001 C CNN
	1    5125 4325
	1    0    0    -1  
$EndComp
Wire Wire Line
	5125 3725 5600 3725
$Comp
L Device:R R8
U 1 1 5C7CD93D
P -1075 3000
F 0 "R8" H -1005 3046 50  0000 L CNN
F 1 "620K" H -1005 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V -1145 3000 50  0001 C CNN
F 3 "~" H -1075 3000 50  0001 C CNN
	1    -1075 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5C7CD9C1
P -1075 3450
F 0 "R9" H -1005 3496 50  0000 L CNN
F 1 "100K" H -1005 3405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V -1145 3450 50  0001 C CNN
F 3 "~" H -1075 3450 50  0001 C CNN
	1    -1075 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5C7CDA7B
P -1075 3600
F 0 "#PWR023" H -1075 3350 50  0001 C CNN
F 1 "GND" H -1070 3427 50  0000 C CNN
F 2 "" H -1075 3600 50  0001 C CNN
F 3 "" H -1075 3600 50  0001 C CNN
	1    -1075 3600
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR022
U 1 1 5C7CDD58
P -1075 2850
F 0 "#PWR022" H -1075 2700 50  0001 C CNN
F 1 "+24V" H -1060 3023 50  0000 C CNN
F 2 "" H -1075 2850 50  0001 C CNN
F 3 "" H -1075 2850 50  0001 C CNN
	1    -1075 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3225 5600 3325
Text Label 3275 4875 0    50   ~ 0
PWM
$Comp
L Device:R R3
U 1 1 5C7E9055
P 1400 4725
F 0 "R3" H 1470 4771 50  0000 L CNN
F 1 "10K" H 1470 4680 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 1330 4725 50  0001 C CNN
F 3 "~" H 1400 4725 50  0001 C CNN
	1    1400 4725
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5C7E92E6
P 1400 4875
F 0 "#PWR08" H 1400 4625 50  0001 C CNN
F 1 "GND" H 1405 4702 50  0000 C CNN
F 2 "" H 1400 4875 50  0001 C CNN
F 3 "" H 1400 4875 50  0001 C CNN
	1    1400 4875
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 4575 1400 4575
$Comp
L power:+3V3 #PWR024
U 1 1 5C7C16CB
P 9525 1250
F 0 "#PWR024" H 9525 1100 50  0001 C CNN
F 1 "+3V3" H 9540 1423 50  0000 C CNN
F 2 "" H 9525 1250 50  0001 C CNN
F 3 "" H 9525 1250 50  0001 C CNN
	1    9525 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 1350 9525 1350
Wire Wire Line
	9525 1350 9525 1250
Text GLabel 9400 1450 2    50   Input ~ 0
SWCLK
Text GLabel 9400 1650 2    50   Input ~ 0
SWDI0
Text GLabel 9400 1750 2    50   Input ~ 0
NRST
$Comp
L power:GND #PWR025
U 1 1 5C7C33ED
P 9875 1550
F 0 "#PWR025" H 9875 1300 50  0001 C CNN
F 1 "GND" H 9880 1377 50  0000 C CNN
F 2 "" H 9875 1550 50  0001 C CNN
F 3 "" H 9875 1550 50  0001 C CNN
	1    9875 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 1550 9875 1550
$Comp
L Connector:Conn_01x06_Male J1
U 1 1 5C7C4E82
P 9200 1550
F 0 "J1" H 9100 1475 50  0000 C CNN
F 1 "SWD" H 9100 1550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 9200 1550 50  0001 C CNN
F 3 "~" H 9200 1550 50  0001 C CNN
	1    9200 1550
	1    0    0    -1  
$EndComp
Text GLabel 9400 1850 2    50   Input ~ 0
UART_TX
Wire Wire Line
	-1075 3150 -1075 3225
Wire Wire Line
	2800 4375 3975 4375
Wire Wire Line
	-1350 3900 -1350 3225
Wire Wire Line
	-1350 3225 -1075 3225
Connection ~ -1075 3225
Wire Wire Line
	-1075 3225 -1075 3300
Text Label 3400 4375 0    50   ~ 0
PANEL_SENSE
$Comp
L Device:R R7
U 1 1 5C7DB05A
P -700 5100
F 0 "R7" V -775 5100 50  0000 C CNN
F 1 "1k" V -700 5100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V -770 5100 50  0001 C CNN
F 3 "~" H -700 5100 50  0001 C CNN
	1    -700 5100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5C7DB1DC
P 1000 5800
F 0 "#PWR03" H 1000 5550 50  0001 C CNN
F 1 "GND" H 1005 5627 50  0000 C CNN
F 2 "" H 1000 5800 50  0001 C CNN
F 3 "" H 1000 5800 50  0001 C CNN
	1    1000 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1725 5275 1800 5275
Wire Wire Line
	1100 5800 1100 5700
Wire Wire Line
	900  5700 900  5800
Wire Wire Line
	1725 5175 1800 5175
$Comp
L Device:R R2
U 1 1 5C7F09D9
P 1575 5275
F 0 "R2" V 1475 5275 50  0000 C CNN
F 1 "1K" V 1575 5275 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 1505 5275 50  0001 C CNN
F 3 "~" H 1575 5275 50  0001 C CNN
	1    1575 5275
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5C7F0A89
P 1100 5550
F 0 "D2" V 1138 5433 50  0000 R CNN
F 1 "LED" V 1047 5433 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1100 5550 50  0001 C CNN
F 3 "~" H 1100 5550 50  0001 C CNN
	1    1100 5550
	0    -1   -1   0   
$EndComp
Wire Notes Line
	550  600  4625 600 
Wire Notes Line
	4625 600  4625 2350
Wire Notes Line
	4600 2350 550  2350
Wire Notes Line
	550  2350 550  600 
Text Notes 650  800  0    98   ~ 20
Power Supply
$Comp
L Device:Solar_Cells SC1
U 1 1 5C80516E
P 10450 5975
F 0 "SC1" H 10558 6021 50  0000 L CNN
F 1 "Solar_Panel" H 10558 5930 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 10450 6035 50  0001 C CNN
F 3 "~" V 10450 6035 50  0001 C CNN
	1    10450 5975
	1    0    0    -1  
$EndComp
Wire Wire Line
	1025 1450 1025 1200
$Comp
L power:GND #PWR01
U 1 1 5C80809F
P 1025 1850
F 0 "#PWR01" H 1025 1600 50  0001 C CNN
F 1 "GND" H 1030 1677 50  0000 C CNN
F 2 "" H 1025 1850 50  0001 C CNN
F 3 "" H 1025 1850 50  0001 C CNN
	1    1025 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5C8080FA
P 3475 1700
F 0 "C1" H 3567 1746 50  0000 L CNN
F 1 "10uF" H 3567 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3475 1700 50  0001 C CNN
F 3 "~" H 3475 1700 50  0001 C CNN
	1    3475 1700
	1    0    0    -1  
$EndComp
Wire Notes Line
	7950 600  11150 600 
Wire Notes Line
	11150 625  11150 2350
Wire Notes Line
	11125 2350 7950 2350
Wire Notes Line
	7950 2350 7950 625 
Text Notes 8025 800  0    98   ~ 20
SWD Header
Wire Wire Line
	1425 5275 1100 5275
Wire Wire Line
	1100 5275 1100 5400
Wire Wire Line
	1425 5175 900  5175
Wire Wire Line
	900  5175 900  5400
Wire Wire Line
	900  5800 1000 5800
Wire Wire Line
	1100 5800 1000 5800
Connection ~ 1000 5800
Wire Wire Line
	6425 6575 6425 6350
NoConn ~ 2800 4675
Text Label 1150 5275 0    50   ~ 0
LED_2
Text Label 6425 6975 0    50   ~ 0
LED_4
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5C8B15A7
P 2275 6625
F 0 "H1" H 2375 6676 50  0000 L CNN
F 1 "MountingHole_Pad" H 2375 6585 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 2275 6625 50  0001 C CNN
F 3 "~" H 2275 6625 50  0001 C CNN
	1    2275 6625
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5C8B1717
P 3300 6625
F 0 "H2" H 3400 6676 50  0000 L CNN
F 1 "MountingHole_Pad" H 3400 6585 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 3300 6625 50  0001 C CNN
F 3 "~" H 3300 6625 50  0001 C CNN
	1    3300 6625
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5C8B74C5
P 2275 7250
F 0 "H3" H 2375 7301 50  0000 L CNN
F 1 "MountingHole_Pad" H 2375 7210 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 2275 7250 50  0001 C CNN
F 3 "~" H 2275 7250 50  0001 C CNN
	1    2275 7250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5C8B74CC
P 3300 7250
F 0 "H4" H 3400 7301 50  0000 L CNN
F 1 "MountingHole_Pad" H 3400 7210 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 3300 7250 50  0001 C CNN
F 3 "~" H 3300 7250 50  0001 C CNN
	1    3300 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR034
U 1 1 5C8BA2A9
P 2275 6725
F 0 "#PWR034" H 2275 6475 50  0001 C CNN
F 1 "GND" H 2280 6552 50  0000 C CNN
F 2 "" H 2275 6725 50  0001 C CNN
F 3 "" H 2275 6725 50  0001 C CNN
	1    2275 6725
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR037
U 1 1 5C8BA36C
P 3300 6725
F 0 "#PWR037" H 3300 6475 50  0001 C CNN
F 1 "GND" H 3305 6552 50  0000 C CNN
F 2 "" H 3300 6725 50  0001 C CNN
F 3 "" H 3300 6725 50  0001 C CNN
	1    3300 6725
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR039
U 1 1 5C8BA3D3
P 3300 7350
F 0 "#PWR039" H 3300 7100 50  0001 C CNN
F 1 "GND" H 3305 7177 50  0000 C CNN
F 2 "" H 3300 7350 50  0001 C CNN
F 3 "" H 3300 7350 50  0001 C CNN
	1    3300 7350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR038
U 1 1 5C8BA43A
P 2275 7350
F 0 "#PWR038" H 2275 7100 50  0001 C CNN
F 1 "GND" H 2280 7177 50  0000 C CNN
F 2 "" H 2275 7350 50  0001 C CNN
F 3 "" H 2275 7350 50  0001 C CNN
	1    2275 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 4475 4900 4125
Wire Wire Line
	4900 4125 5125 4125
Wire Wire Line
	150  4525 150  4825
Wire Wire Line
	5125 3725 5125 4125
Connection ~ 5125 4125
$Comp
L Transistor_FET:Si2319CDS Q1
U 1 1 5CB330E9
P -250 4625
F 0 "Q1" V 93  4625 50  0000 C CNN
F 1 "Si2319CDS" V 2   4625 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H -50 4550 50  0001 L CIN
F 3 "http://www.vishay.com/docs/66709/si2319cd.pdf" H -250 4625 50  0001 L CNN
	1    -250 4625
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R10
U 1 1 5CB53F25
P 0 4825
F 0 "R10" V 75  4825 50  0000 C CNN
F 1 "10K" V 0   4825 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V -70 4825 50  0001 C CNN
F 3 "~" H 0   4825 50  0001 C CNN
	1    0    4825
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_FET:2N7002 Q2
U 1 1 5CB54DEC
P -350 5100
F 0 "Q2" H -145 5146 50  0000 L CNN
F 1 "2N7002" H -145 5055 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H -150 5025 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7002.pdf" H -350 5100 50  0001 L CNN
	1    -350 5100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5CB554B7
P -250 5300
F 0 "#PWR021" H -250 5050 50  0001 C CNN
F 1 "GND" H -245 5127 50  0000 C CNN
F 2 "" H -250 5300 50  0001 C CNN
F 3 "" H -250 5300 50  0001 C CNN
	1    -250 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	-250 4825 -250 4900
Wire Wire Line
	-50  4525 150  4525
Wire Wire Line
	-850 5100 -1400 5100
Connection ~ 150  4825
Wire Wire Line
	150  4825 150  6325
Wire Wire Line
	-2725 4525 -450 4525
Wire Wire Line
	-150 4825 -250 4825
Connection ~ -250 4825
Wire Wire Line
	2800 4475 4900 4475
$Comp
L Device:Battery BT?
U 1 1 5CDC3C72
P 1025 1650
F 0 "BT?" H 1133 1696 50  0000 L CNN
F 1 "Battery" H 1133 1605 50  0000 L CNN
F 2 "" V 1025 1710 50  0001 C CNN
F 3 "~" V 1025 1710 50  0001 C CNN
	1    1025 1650
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LT1129-3.3_SOT223 U?
U 1 1 5CDC40FD
P 2575 1600
F 0 "U?" H 2575 1842 50  0000 C CNN
F 1 "NCV4264-2CST33T3G" H 2575 1751 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 2575 1825 50  0001 C CIN
F 3 "http://cds.linear.com/docs/en/datasheet/112935ff.pdf" H 2575 1550 50  0001 C CNN
	1    2575 1600
	1    0    0    -1  
$EndComp
Connection ~ 3475 1600
Wire Wire Line
	3475 1600 3875 1600
Wire Wire Line
	1650 1200 2050 1200
Wire Wire Line
	2050 1600 2050 1200
Wire Wire Line
	2050 1600 2275 1600
Wire Wire Line
	2875 1600 3475 1600
NoConn ~ 1800 5475
NoConn ~ 2800 5175
NoConn ~ 2800 5275
NoConn ~ 2800 4775
Text GLabel 2800 4975 2    50   Input ~ 0
pump1
Text GLabel 2800 5075 2    50   Input ~ 0
pump2
Text GLabel 6350 2100 0    50   Input ~ 0
pump2
Text GLabel 5700 1875 0    50   Input ~ 0
pump1
Text Notes 4750 800  0    98   ~ 20
Charge Pump
Wire Notes Line
	4675 2350 4675 625 
Wire Notes Line
	7850 2350 4675 2350
Wire Notes Line
	7875 625  7875 2350
Wire Notes Line
	4675 600  7875 600 
$Comp
L Diode:BAV99 D?
U 1 1 5CDE048B
P 6050 1225
F 0 "D?" H 6050 1440 50  0000 C CNN
F 1 "BAV99" H 6050 1349 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6050 1075 50  0001 C CNN
F 3 "www.nxp.com/documents/data_sheet/BAV99_SER.pdf" H 6050 1325 50  0001 C CNN
	1    6050 1225
	-1   0    0    -1  
$EndComp
$Comp
L Diode:BAV99 D?
U 2 1 5CDE0586
P 6650 1225
F 0 "D?" H 6650 1440 50  0000 C CNN
F 1 "BAV99" H 6650 1349 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6650 1075 50  0001 C CNN
F 3 "www.nxp.com/documents/data_sheet/BAV99_SER.pdf" H 6650 1325 50  0001 C CNN
	2    6650 1225
	-1   0    0    -1  
$EndComp
$Comp
L theapi_Diode:BAW56 D?
U 1 1 5CDB608E
P 1350 1200
F 0 "D?" H 1350 975 50  0000 C CNN
F 1 "BAW56" H 1350 1066 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 1350 1025 50  0001 C CNN
F 3 "http://rohmfs.rohm.com/en/products/databook/datasheet/discrete/diode/switching/ump11n.pdf" H 1350 1300 50  0001 C CNN
	1    1350 1200
	-1   0    0    1   
$EndComp
$Comp
L theapi_Diode:BAW56 D?
U 2 1 5CDB618D
P 5450 1225
F 0 "D?" H 5450 1000 50  0000 C CNN
F 1 "BAW56" H 5450 1091 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 5450 1050 50  0001 C CNN
F 3 "http://rohmfs.rohm.com/en/products/databook/datasheet/discrete/diode/switching/ump11n.pdf" H 5450 1325 50  0001 C CNN
	2    5450 1225
	-1   0    0    1   
$EndComp
Wire Wire Line
	1025 1200 1200 1200
Wire Wire Line
	1500 1200 1650 1200
Connection ~ 1650 1200
Text Label 1025 1200 2    50   ~ 0
BATT
$Comp
L Device:C C?
U 1 1 5CDB96BA
P 7125 1800
F 0 "C?" H 7240 1846 50  0000 L CNN
F 1 "1uF" H 7240 1755 50  0000 L CNN
F 2 "" H 7163 1650 50  0001 C CNN
F 3 "~" H 7125 1800 50  0001 C CNN
	1    7125 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CDB9848
P 7125 1950
F 0 "#PWR?" H 7125 1700 50  0001 C CNN
F 1 "GND" H 7130 1777 50  0000 C CNN
F 2 "" H 7125 1950 50  0001 C CNN
F 3 "" H 7125 1950 50  0001 C CNN
	1    7125 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5CDB9DBD
P 5700 1600
F 0 "C?" H 5815 1646 50  0000 L CNN
F 1 "C" H 5815 1555 50  0000 L CNN
F 2 "" H 5738 1450 50  0001 C CNN
F 3 "~" H 5700 1600 50  0001 C CNN
	1    5700 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5CDB9EB6
P 6350 1800
F 0 "C?" H 6465 1846 50  0000 L CNN
F 1 "C" H 6465 1755 50  0000 L CNN
F 2 "" H 6388 1650 50  0001 C CNN
F 3 "~" H 6350 1800 50  0001 C CNN
	1    6350 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 1225 4950 1225
Wire Wire Line
	5600 1225 5700 1225
Wire Wire Line
	6200 1225 6350 1225
Wire Wire Line
	6800 1225 7125 1225
Wire Wire Line
	7125 1225 7125 1650
Wire Wire Line
	5700 1450 5700 1225
Connection ~ 5700 1225
Wire Wire Line
	5700 1225 5900 1225
Wire Wire Line
	6350 1650 6350 1225
Connection ~ 6350 1225
Wire Wire Line
	6350 1225 6500 1225
Wire Wire Line
	6350 1950 6350 2100
Wire Wire Line
	5700 1750 5700 1875
Text Label 4950 1225 0    50   ~ 0
BATT
Text Label 7675 1225 2    50   ~ 0
V_GATE
Connection ~ 7125 1225
Wire Wire Line
	7125 1225 7675 1225
$EndSCHEMATC
