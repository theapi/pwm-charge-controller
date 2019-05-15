EESchema Schematic File Version 4
LIBS:project-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "PWM Solar Charge Controller"
Date ""
Rev "2.x"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_ST_STM32L0:STM32L031F4Px U2
U 1 1 5C7AC20A
P 2300 4875
F 0 "U2" H 2425 4125 50  0000 C CNN
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
L power:GND #PWR020
U 1 1 5C7AC377
P 2300 5675
F 0 "#PWR020" H 2300 5425 50  0001 C CNN
F 1 "GND" H 2305 5502 50  0000 C CNN
F 2 "" H 2300 5675 50  0001 C CNN
F 3 "" H 2300 5675 50  0001 C CNN
	1    2300 5675
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR09
U 1 1 5C7ACC5E
P 2300 3525
F 0 "#PWR09" H 2300 3375 50  0001 C CNN
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
L Device:C_Small C7
U 1 1 5C7ACD13
P 1975 3725
F 0 "C7" H 2067 3771 50  0000 L CNN
F 1 "0.1uF" H 2067 3680 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1975 3725 50  0001 C CNN
F 3 "~" H 1975 3725 50  0001 C CNN
	1    1975 3725
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5C7ACD67
P 2625 3725
F 0 "C8" H 2717 3771 50  0000 L CNN
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
L power:GND #PWR011
U 1 1 5C7ACE2D
P 1975 3825
F 0 "#PWR011" H 1975 3575 50  0001 C CNN
F 1 "GND" H 1980 3652 50  0000 C CNN
F 2 "" H 1975 3825 50  0001 C CNN
F 3 "" H 1975 3825 50  0001 C CNN
	1    1975 3825
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5C7ACE47
P 2625 3825
F 0 "#PWR012" H 2625 3575 50  0001 C CNN
F 1 "GND" H 2630 3652 50  0000 C CNN
F 2 "" H 2625 3825 50  0001 C CNN
F 3 "" H 2625 3825 50  0001 C CNN
	1    2625 3825
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5C7ACF3C
P 3075 3725
F 0 "C9" H 3167 3771 50  0000 L CNN
F 1 "1uF" H 3167 3680 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3075 3725 50  0001 C CNN
F 3 "~" H 3075 3725 50  0001 C CNN
	1    3075 3725
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5C7ACF8A
P 1575 3725
F 0 "C6" H 1667 3771 50  0000 L CNN
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
L power:GND #PWR010
U 1 1 5C7AD02B
P 1575 3825
F 0 "#PWR010" H 1575 3575 50  0001 C CNN
F 1 "GND" H 1580 3652 50  0000 C CNN
F 2 "" H 1575 3825 50  0001 C CNN
F 3 "" H 1575 3825 50  0001 C CNN
	1    1575 3825
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5C7AD040
P 3075 3825
F 0 "#PWR013" H 3075 3575 50  0001 C CNN
F 1 "GND" H 3080 3652 50  0000 C CNN
F 2 "" H 3075 3825 50  0001 C CNN
F 3 "" H 3075 3825 50  0001 C CNN
	1    3075 3825
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5C7B0980
P 1650 1300
F 0 "C1" H 1742 1346 50  0000 L CNN
F 1 "0.1uF" H 1742 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1650 1300 50  0001 C CNN
F 3 "~" H 1650 1300 50  0001 C CNN
	1    1650 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5C7B0F3D
P 3475 1800
F 0 "#PWR05" H 3475 1550 50  0001 C CNN
F 1 "GND" H 3480 1627 50  0000 C CNN
F 2 "" H 3475 1800 50  0001 C CNN
F 3 "" H 3475 1800 50  0001 C CNN
	1    3475 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5C7B0F7E
P 1650 1400
F 0 "#PWR02" H 1650 1150 50  0001 C CNN
F 1 "GND" H 1655 1227 50  0000 C CNN
F 2 "" H 1650 1400 50  0001 C CNN
F 3 "" H 1650 1400 50  0001 C CNN
	1    1650 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR04
U 1 1 5C7B128E
P 3875 1600
F 0 "#PWR04" H 3875 1450 50  0001 C CNN
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
$Comp
L Device:R R10
U 1 1 5C7C0432
P 4275 5250
F 0 "R10" H 4345 5296 50  0000 L CNN
F 1 "1K" H 4345 5205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4205 5250 50  0001 C CNN
F 3 "~" H 4275 5250 50  0001 C CNN
	1    4275 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D6
U 1 1 5C7C051B
P 4275 5700
F 0 "D6" V 4313 5583 50  0000 R CNN
F 1 "LED" V 4222 5583 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4275 5700 50  0001 C CNN
F 3 "~" H 4275 5700 50  0001 C CNN
	1    4275 5700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5C7C05D8
P 4275 5850
F 0 "#PWR022" H 4275 5600 50  0001 C CNN
F 1 "GND" H 4280 5677 50  0000 C CNN
F 2 "" H 4275 5850 50  0001 C CNN
F 3 "" H 4275 5850 50  0001 C CNN
	1    4275 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4275 5550 4275 5400
$Comp
L Device:LED D4
U 1 1 5C7C3D9B
P 900 5550
F 0 "D4" V 925 5725 50  0000 R CNN
F 1 "LED" V 825 5775 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 900 5550 50  0001 C CNN
F 3 "~" H 900 5550 50  0001 C CNN
	1    900  5550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R9
U 1 1 5C7C3FBD
P 1575 5175
F 0 "R9" V 1675 5175 50  0000 C CNN
F 1 "1K" V 1575 5175 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 1505 5175 50  0001 C CNN
F 3 "~" H 1575 5175 50  0001 C CNN
	1    1575 5175
	0    -1   -1   0   
$EndComp
Text Label 1150 5175 0    50   ~ 0
LED_1
$Comp
L Device:R R6
U 1 1 5C7C5AEB
P 5600 3975
F 0 "R6" H 5670 4021 50  0000 L CNN
F 1 "100K" H 5670 3930 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5530 3975 50  0001 C CNN
F 3 "~" H 5600 3975 50  0001 C CNN
	1    5600 3975
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5C7C5BA1
P 5600 3475
F 0 "R2" H 5670 3521 50  0000 L CNN
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
L power:GND #PWR015
U 1 1 5C7C73C4
P 5600 4125
F 0 "#PWR015" H 5600 3875 50  0001 C CNN
F 1 "GND" H 5605 3952 50  0000 C CNN
F 2 "" H 5600 4125 50  0001 C CNN
F 3 "" H 5600 4125 50  0001 C CNN
	1    5600 4125
	1    0    0    -1  
$EndComp
Text Label 4375 4475 0    50   ~ 0
BATT_SENSE
$Comp
L Device:C_Small C10
U 1 1 5C7CB72F
P 5125 4225
F 0 "C10" H 5217 4271 50  0000 L CNN
F 1 "220pF" H 5217 4180 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5125 4225 50  0001 C CNN
F 3 "~" H 5125 4225 50  0001 C CNN
	1    5125 4225
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5C7CB7D1
P 5125 4325
F 0 "#PWR016" H 5125 4075 50  0001 C CNN
F 1 "GND" H 5130 4152 50  0000 C CNN
F 2 "" H 5125 4325 50  0001 C CNN
F 3 "" H 5125 4325 50  0001 C CNN
	1    5125 4325
	1    0    0    -1  
$EndComp
Wire Wire Line
	5125 3725 5600 3725
$Comp
L Device:R R1
U 1 1 5C7CD93D
P 4250 3475
F 0 "R1" H 4320 3521 50  0000 L CNN
F 1 "620K" H 4320 3430 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4180 3475 50  0001 C CNN
F 3 "~" H 4250 3475 50  0001 C CNN
	1    4250 3475
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5C7CD9C1
P 4250 3925
F 0 "R5" H 4320 3971 50  0000 L CNN
F 1 "100K" H 4320 3880 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4180 3925 50  0001 C CNN
F 3 "~" H 4250 3925 50  0001 C CNN
	1    4250 3925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5C7CDA7B
P 4250 4075
F 0 "#PWR014" H 4250 3825 50  0001 C CNN
F 1 "GND" H 4255 3902 50  0000 C CNN
F 2 "" H 4250 4075 50  0001 C CNN
F 3 "" H 4250 4075 50  0001 C CNN
	1    4250 4075
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3225 5600 3325
Text Label 3275 4875 0    50   ~ 0
PWM
$Comp
L Device:R R8
U 1 1 5C7E9055
P 1400 4725
F 0 "R8" H 1470 4771 50  0000 L CNN
F 1 "10K" H 1470 4680 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 1330 4725 50  0001 C CNN
F 3 "~" H 1400 4725 50  0001 C CNN
	1    1400 4725
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5C7E92E6
P 1400 4875
F 0 "#PWR017" H 1400 4625 50  0001 C CNN
F 1 "GND" H 1405 4702 50  0000 C CNN
F 2 "" H 1400 4875 50  0001 C CNN
F 3 "" H 1400 4875 50  0001 C CNN
	1    1400 4875
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 4575 1400 4575
$Comp
L power:+3V3 #PWR01
U 1 1 5C7C16CB
P 9525 1250
F 0 "#PWR01" H 9525 1100 50  0001 C CNN
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
L power:GND #PWR03
U 1 1 5C7C33ED
P 9875 1550
F 0 "#PWR03" H 9875 1300 50  0001 C CNN
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
	4250 3625 4250 3700
Wire Wire Line
	2800 4375 3975 4375
Wire Wire Line
	3975 4375 3975 3700
Wire Wire Line
	3975 3700 4250 3700
Connection ~ 4250 3700
Wire Wire Line
	4250 3700 4250 3775
Text Label 3400 4375 0    50   ~ 0
PANEL_SENSE
$Comp
L power:GND #PWR021
U 1 1 5C7DB1DC
P 1000 5800
F 0 "#PWR021" H 1000 5550 50  0001 C CNN
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
L Device:R R11
U 1 1 5C7F09D9
P 1575 5275
F 0 "R11" V 1475 5275 50  0000 C CNN
F 1 "1K" V 1575 5275 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 1505 5275 50  0001 C CNN
F 3 "~" H 1575 5275 50  0001 C CNN
	1    1575 5275
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 5C7F0A89
P 1100 5550
F 0 "D5" V 1138 5433 50  0000 R CNN
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
P 9175 5350
F 0 "SC1" H 9283 5396 50  0000 L CNN
F 1 "Solar_Panel" H 9283 5305 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 9175 5410 50  0001 C CNN
F 3 "~" V 9175 5410 50  0001 C CNN
	1    9175 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1025 1450 1025 1200
$Comp
L power:GND #PWR06
U 1 1 5C80809F
P 1025 1850
F 0 "#PWR06" H 1025 1600 50  0001 C CNN
F 1 "GND" H 1030 1677 50  0000 C CNN
F 2 "" H 1025 1850 50  0001 C CNN
F 3 "" H 1025 1850 50  0001 C CNN
	1    1025 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5C8080FA
P 3475 1700
F 0 "C3" H 3567 1746 50  0000 L CNN
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
	4275 5100 4275 4875
NoConn ~ 2800 4675
Text Label 1150 5275 0    50   ~ 0
LED_2
Text Label 4275 5500 0    50   ~ 0
LED_3
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
L power:GND #PWR023
U 1 1 5C8BA2A9
P 2275 6725
F 0 "#PWR023" H 2275 6475 50  0001 C CNN
F 1 "GND" H 2280 6552 50  0000 C CNN
F 2 "" H 2275 6725 50  0001 C CNN
F 3 "" H 2275 6725 50  0001 C CNN
	1    2275 6725
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 5C8BA36C
P 3300 6725
F 0 "#PWR024" H 3300 6475 50  0001 C CNN
F 1 "GND" H 3305 6552 50  0000 C CNN
F 2 "" H 3300 6725 50  0001 C CNN
F 3 "" H 3300 6725 50  0001 C CNN
	1    3300 6725
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5C8BA3D3
P 3300 7350
F 0 "#PWR026" H 3300 7100 50  0001 C CNN
F 1 "GND" H 3305 7177 50  0000 C CNN
F 2 "" H 3300 7350 50  0001 C CNN
F 3 "" H 3300 7350 50  0001 C CNN
	1    3300 7350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5C8BA43A
P 2275 7350
F 0 "#PWR025" H 2275 7100 50  0001 C CNN
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
	5125 3725 5125 4125
Connection ~ 5125 4125
Wire Wire Line
	2800 4475 4900 4475
$Comp
L Device:Battery BT1
U 1 1 5CDC3C72
P 1025 1650
F 0 "BT1" H 1133 1696 50  0000 L CNN
F 1 "Battery" H 1133 1605 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 1025 1710 50  0001 C CNN
F 3 "~" V 1025 1710 50  0001 C CNN
	1    1025 1650
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LT1129-3.3_SOT223 U1
U 1 1 5CDC40FD
P 2575 1600
F 0 "U1" H 2575 1842 50  0000 C CNN
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
L Diode:BAV99 D2
U 1 1 5CDE048B
P 6050 1225
F 0 "D2" H 6050 1440 50  0000 C CNN
F 1 "BAV99" H 6050 1349 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6050 1075 50  0001 C CNN
F 3 "www.nxp.com/documents/data_sheet/BAV99_SER.pdf" H 6050 1325 50  0001 C CNN
	1    6050 1225
	-1   0    0    -1  
$EndComp
$Comp
L Diode:BAV99 D2
U 2 1 5CDE0586
P 6650 1225
F 0 "D2" H 6650 1440 50  0000 C CNN
F 1 "BAV99" H 6650 1349 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6650 1075 50  0001 C CNN
F 3 "www.nxp.com/documents/data_sheet/BAV99_SER.pdf" H 6650 1325 50  0001 C CNN
	2    6650 1225
	-1   0    0    -1  
$EndComp
$Comp
L theapi_Diode:BAW56 D1
U 1 1 5CDB608E
P 1350 1200
F 0 "D1" H 1350 975 50  0000 C CNN
F 1 "BAW56" H 1350 1066 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1350 1025 50  0001 C CNN
F 3 "http://rohmfs.rohm.com/en/products/databook/datasheet/discrete/diode/switching/ump11n.pdf" H 1350 1300 50  0001 C CNN
	1    1350 1200
	-1   0    0    1   
$EndComp
$Comp
L theapi_Diode:BAW56 D1
U 2 1 5CDB618D
P 5450 1225
F 0 "D1" H 5450 1000 50  0000 C CNN
F 1 "BAW56" H 5450 1091 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5450 1050 50  0001 C CNN
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
L Device:C C5
U 1 1 5CDB96BA
P 7125 1800
F 0 "C5" H 7240 1846 50  0000 L CNN
F 1 "1uF" H 7240 1755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7163 1650 50  0001 C CNN
F 3 "~" H 7125 1800 50  0001 C CNN
	1    7125 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5CDB9848
P 7125 1950
F 0 "#PWR08" H 7125 1700 50  0001 C CNN
F 1 "GND" H 7130 1777 50  0000 C CNN
F 2 "" H 7125 1950 50  0001 C CNN
F 3 "" H 7125 1950 50  0001 C CNN
	1    7125 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5CDB9DBD
P 5700 1600
F 0 "C2" H 5815 1646 50  0000 L CNN
F 1 "47nF" H 5815 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5738 1450 50  0001 C CNN
F 3 "~" H 5700 1600 50  0001 C CNN
	1    5700 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5CDB9EB6
P 6350 1800
F 0 "C4" H 6465 1846 50  0000 L CNN
F 1 "47nF" H 6465 1755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6388 1650 50  0001 C CNN
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
Text Label 5600 3225 0    50   ~ 0
BATT
Wire Wire Line
	9175 5150 9175 4775
Text Label 9175 5000 0    50   ~ 0
SOLAR
Text Label 4250 3200 0    50   ~ 0
SOLAR
Wire Wire Line
	4250 3200 4250 3325
$Comp
L Device:R R3
U 1 1 5CDCBDC3
P 6225 3575
F 0 "R3" H 6295 3621 50  0000 L CNN
F 1 "220K" H 6295 3530 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6155 3575 50  0001 C CNN
F 3 "~" H 6225 3575 50  0001 C CNN
	1    6225 3575
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5CDCBF9D
P 6225 5350
F 0 "R12" H 6295 5396 50  0000 L CNN
F 1 "220K" H 6295 5305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6155 5350 50  0001 C CNN
F 3 "~" H 6225 5350 50  0001 C CNN
	1    6225 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5CDCC025
P 7375 3600
F 0 "R4" H 7445 3646 50  0000 L CNN
F 1 "220K" H 7445 3555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7305 3600 50  0001 C CNN
F 3 "~" H 7375 3600 50  0001 C CNN
	1    7375 3600
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:MMBT3904 Q4
U 1 1 5CDCC30D
P 6125 4875
F 0 "Q4" H 6316 4921 50  0000 L CNN
F 1 "MMBT3904" H 6316 4830 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6325 4800 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 6125 4875 50  0001 L CNN
	1    6125 4875
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4875 4275 4875
$Comp
L power:GND #PWR018
U 1 1 5CDCF0C3
P 6225 5500
F 0 "#PWR018" H 6225 5250 50  0001 C CNN
F 1 "GND" H 6230 5327 50  0000 C CNN
F 2 "" H 6225 5500 50  0001 C CNN
F 3 "" H 6225 5500 50  0001 C CNN
	1    6225 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6225 5075 6225 5200
Wire Wire Line
	6225 3425 6225 3350
Text Label 6225 3225 0    50   ~ 0
V_GATE
$Comp
L Transistor_BJT:MMBT3906 Q2
U 1 1 5CDD4674
P 6700 4175
F 0 "Q2" H 6891 4129 50  0000 L CNN
F 1 "MMBT3906" H 6891 4220 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6900 4100 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3906.pdf" H 6700 4175 50  0001 L CNN
	1    6700 4175
	1    0    0    1   
$EndComp
Wire Wire Line
	6225 3725 6225 4175
Wire Wire Line
	6500 4175 6225 4175
Connection ~ 6225 4175
Wire Wire Line
	6225 4175 6225 4675
Wire Wire Line
	6225 3350 6800 3350
Wire Wire Line
	6800 3350 6800 3975
Connection ~ 6225 3350
Wire Wire Line
	6225 3350 6225 3225
$Comp
L Transistor_BJT:MMBT3906 Q1
U 1 1 5CDDBCFD
P 7825 3900
F 0 "Q1" H 8016 3946 50  0000 L CNN
F 1 "MMBT3906" H 8016 3855 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8025 3825 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3906.pdf" H 7825 3900 50  0001 L CNN
	1    7825 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7375 3900 7625 3900
Wire Wire Line
	7375 3900 7375 4575
Wire Wire Line
	7375 4575 6800 4575
Wire Wire Line
	6800 4575 6800 4375
Text Label 7375 3225 0    50   ~ 0
BATT
Wire Wire Line
	7375 3225 7375 3350
Wire Wire Line
	7375 3750 7375 3900
Connection ~ 7375 3900
Wire Wire Line
	7925 3700 7925 3350
Wire Wire Line
	7925 3350 7375 3350
Connection ~ 7375 3350
Wire Wire Line
	7375 3350 7375 3450
$Comp
L Diode:1N4148 D3
U 1 1 5CDE7C5D
P 7625 4575
F 0 "D3" H 7625 4359 50  0000 C CNN
F 1 "1N4148" H 7625 4450 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 7625 4400 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/1N4148_1N4448.pdf" H 7625 4575 50  0001 C CNN
	1    7625 4575
	-1   0    0    1   
$EndComp
Wire Wire Line
	7925 4100 7925 4575
Wire Wire Line
	7925 4575 7775 4575
Wire Wire Line
	7475 4575 7375 4575
Connection ~ 7375 4575
$Comp
L Device:R R7
U 1 1 5CDEB9D9
P 8325 4575
F 0 "R7" V 8118 4575 50  0000 C CNN
F 1 "4K7" V 8209 4575 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8255 4575 50  0001 C CNN
F 3 "~" H 8325 4575 50  0001 C CNN
	1    8325 4575
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:IRF3205 Q3
U 1 1 5CDEBEE6
P 9075 4575
F 0 "Q3" H 9281 4529 50  0000 L CNN
F 1 "IRF3205" H 9281 4620 50  0000 L CNN
F 2 "theapi:TO-220F-3_Horizontal_TabUp" H 9325 4500 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf3205.pdf" H 9075 4575 50  0001 L CNN
	1    9075 4575
	1    0    0    1   
$EndComp
Wire Wire Line
	8875 4575 8475 4575
Wire Wire Line
	8175 4575 7925 4575
Connection ~ 7925 4575
Wire Wire Line
	9175 4375 9175 3350
Wire Wire Line
	9175 3350 7925 3350
Connection ~ 7925 3350
$Comp
L power:GND #PWR019
U 1 1 5CDF634C
P 9175 5550
F 0 "#PWR019" H 9175 5300 50  0001 C CNN
F 1 "GND" H 9180 5377 50  0000 C CNN
F 2 "" H 9175 5550 50  0001 C CNN
F 3 "" H 9175 5550 50  0001 C CNN
	1    9175 5550
	1    0    0    -1  
$EndComp
Connection ~ 4275 4875
Wire Wire Line
	4275 4875 5925 4875
Text Label 2050 1200 0    50   ~ 0
REG_IN
$EndSCHEMATC
