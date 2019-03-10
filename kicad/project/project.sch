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
P 4050 4325
F 0 "U3" H 4175 3575 50  0000 C CNN
F 1 "STM32L031F4Px" H 4425 3500 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 3650 3625 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00140359.pdf" H 4050 4325 50  0001 C CNN
	1    4050 4325
	1    0    0    -1  
$EndComp
Text GLabel 4550 4925 2    50   Input ~ 0
SWCLK
Text GLabel 4550 4825 2    50   Input ~ 0
SWDI0
Text GLabel 4550 4025 2    50   Input ~ 0
UART_TX
Text GLabel 3550 3825 0    50   Input ~ 0
NRST
$Comp
L power:GND #PWR018
U 1 1 5C7AC377
P 4050 5125
F 0 "#PWR018" H 4050 4875 50  0001 C CNN
F 1 "GND" H 4055 4952 50  0000 C CNN
F 2 "" H 4050 5125 50  0001 C CNN
F 3 "" H 4050 5125 50  0001 C CNN
	1    4050 5125
	1    0    0    -1  
$EndComp
Text GLabel 4550 4625 2    50   Input ~ 0
SCL
Text GLabel 4550 4725 2    50   Input ~ 0
SDA
Text GLabel 3550 4925 0    50   Input ~ 0
CURRENT_SENSE
$Comp
L power:+3V3 #PWR017
U 1 1 5C7ACC5E
P 4050 2975
F 0 "#PWR017" H 4050 2825 50  0001 C CNN
F 1 "+3V3" H 4065 3148 50  0000 C CNN
F 2 "" H 4050 2975 50  0001 C CNN
F 3 "" H 4050 2975 50  0001 C CNN
	1    4050 2975
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3075 4050 3625
Wire Wire Line
	4150 3625 4150 3075
Wire Wire Line
	4150 3075 4050 3075
Connection ~ 4050 3075
$Comp
L Device:C_Small C6
U 1 1 5C7ACD13
P 3725 3175
F 0 "C6" H 3817 3221 50  0000 L CNN
F 1 "0.1uF" H 3817 3130 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3725 3175 50  0001 C CNN
F 3 "~" H 3725 3175 50  0001 C CNN
	1    3725 3175
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5C7ACD67
P 4375 3175
F 0 "C7" H 4467 3221 50  0000 L CNN
F 1 "0.1uF" H 4467 3130 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4375 3175 50  0001 C CNN
F 3 "~" H 4375 3175 50  0001 C CNN
	1    4375 3175
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3075 4050 2975
Wire Wire Line
	3725 3075 4050 3075
Wire Wire Line
	4375 3075 4150 3075
Connection ~ 4150 3075
$Comp
L power:GND #PWR015
U 1 1 5C7ACE2D
P 3725 3275
F 0 "#PWR015" H 3725 3025 50  0001 C CNN
F 1 "GND" H 3730 3102 50  0000 C CNN
F 2 "" H 3725 3275 50  0001 C CNN
F 3 "" H 3725 3275 50  0001 C CNN
	1    3725 3275
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5C7ACE47
P 4375 3275
F 0 "#PWR019" H 4375 3025 50  0001 C CNN
F 1 "GND" H 4380 3102 50  0000 C CNN
F 2 "" H 4375 3275 50  0001 C CNN
F 3 "" H 4375 3275 50  0001 C CNN
	1    4375 3275
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5C7ACF3C
P 4825 3175
F 0 "C8" H 4917 3221 50  0000 L CNN
F 1 "1uF" H 4917 3130 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4825 3175 50  0001 C CNN
F 3 "~" H 4825 3175 50  0001 C CNN
	1    4825 3175
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5C7ACF8A
P 3325 3175
F 0 "C4" H 3417 3221 50  0000 L CNN
F 1 "10uF" H 3417 3130 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3325 3175 50  0001 C CNN
F 3 "~" H 3325 3175 50  0001 C CNN
	1    3325 3175
	1    0    0    -1  
$EndComp
Wire Wire Line
	3325 3075 3725 3075
Connection ~ 3725 3075
Wire Wire Line
	4825 3075 4375 3075
Connection ~ 4375 3075
$Comp
L power:GND #PWR012
U 1 1 5C7AD02B
P 3325 3275
F 0 "#PWR012" H 3325 3025 50  0001 C CNN
F 1 "GND" H 3330 3102 50  0000 C CNN
F 2 "" H 3325 3275 50  0001 C CNN
F 3 "" H 3325 3275 50  0001 C CNN
	1    3325 3275
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5C7AD040
P 4825 3275
F 0 "#PWR020" H 4825 3025 50  0001 C CNN
F 1 "GND" H 4830 3102 50  0000 C CNN
F 2 "" H 4825 3275 50  0001 C CNN
F 3 "" H 4825 3275 50  0001 C CNN
	1    4825 3275
	1    0    0    -1  
$EndComp
$Comp
L theapi_Regulator:AP7380 U2
U 1 1 5C7B07DE
P 2875 1725
F 0 "U2" H 2675 1475 50  0000 C CNN
F 1 "AP7380" H 3075 1475 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 2875 1725 50  0001 C CNN
F 3 "" H 2875 1725 50  0001 C CNN
	1    2875 1725
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5C7B0980
P 2050 1100
F 0 "C3" H 2142 1146 50  0000 L CNN
F 1 "1uF" H 2142 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2050 1100 50  0001 C CNN
F 3 "~" H 2050 1100 50  0001 C CNN
	1    2050 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5C7B0AAC
P 3525 1725
F 0 "C5" H 3617 1771 50  0000 L CNN
F 1 "1uF" H 3617 1680 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3525 1725 50  0001 C CNN
F 3 "~" H 3525 1725 50  0001 C CNN
	1    3525 1725
	1    0    0    -1  
$EndComp
Wire Wire Line
	2525 1675 2425 1675
Wire Wire Line
	2425 1675 2425 1225
Wire Wire Line
	2425 1225 2875 1225
Wire Wire Line
	2875 1225 2875 1325
Connection ~ 2875 1225
$Comp
L power:GND #PWR013
U 1 1 5C7B0F3D
P 3525 1825
F 0 "#PWR013" H 3525 1575 50  0001 C CNN
F 1 "GND" H 3530 1652 50  0000 C CNN
F 2 "" H 3525 1825 50  0001 C CNN
F 3 "" H 3525 1825 50  0001 C CNN
	1    3525 1825
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5C7B0F7E
P 2050 1200
F 0 "#PWR010" H 2050 950 50  0001 C CNN
F 1 "GND" H 2055 1027 50  0000 C CNN
F 2 "" H 2050 1200 50  0001 C CNN
F 3 "" H 2050 1200 50  0001 C CNN
	1    2050 1200
	1    0    0    -1  
$EndComp
Connection ~ 2875 1000
Wire Wire Line
	2875 1000 2875 1225
Wire Wire Line
	3525 1625 3225 1625
$Comp
L power:+3V3 #PWR016
U 1 1 5C7B128E
P 3925 1625
F 0 "#PWR016" H 3925 1475 50  0001 C CNN
F 1 "+3V3" H 3940 1798 50  0000 C CNN
F 2 "" H 3925 1625 50  0001 C CNN
F 3 "" H 3925 1625 50  0001 C CNN
	1    3925 1625
	1    0    0    -1  
$EndComp
Wire Wire Line
	3925 1625 3525 1625
Connection ~ 3525 1625
$Comp
L power:GND #PWR07
U 1 1 5C7B13E3
P 2875 2025
F 0 "#PWR07" H 2875 1775 50  0001 C CNN
F 1 "GND" H 2880 1852 50  0000 C CNN
F 2 "" H 2875 2025 50  0001 C CNN
F 3 "" H 2875 2025 50  0001 C CNN
	1    2875 2025
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR06
U 1 1 5C7B0CF4
P 2875 1000
F 0 "#PWR06" H 2875 850 50  0001 C CNN
F 1 "+24V" H 2890 1173 50  0000 C CNN
F 2 "" H 2875 1000 50  0001 C CNN
F 3 "" H 2875 1000 50  0001 C CNN
	1    2875 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5C7B3BD8
P 8625 3600
F 0 "C10" H 8717 3646 50  0000 L CNN
F 1 "0.1uF" H 8717 3555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8625 3600 50  0001 C CNN
F 3 "~" H 8625 3600 50  0001 C CNN
	1    8625 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5C7B45B8
P 8250 4875
F 0 "#PWR030" H 8250 4625 50  0001 C CNN
F 1 "GND" H 8255 4702 50  0000 C CNN
F 2 "" H 8250 4875 50  0001 C CNN
F 3 "" H 8250 4875 50  0001 C CNN
	1    8250 4875
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5C7B4673
P 8625 3700
F 0 "#PWR031" H 8625 3450 50  0001 C CNN
F 1 "GND" H 8630 3527 50  0000 C CNN
F 2 "" H 8625 3700 50  0001 C CNN
F 3 "" H 8625 3700 50  0001 C CNN
	1    8625 3700
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR029
U 1 1 5C7B474F
P 8200 3325
F 0 "#PWR029" H 8200 3175 50  0001 C CNN
F 1 "+24V" H 8215 3498 50  0000 C CNN
F 2 "" H 8200 3325 50  0001 C CNN
F 3 "" H 8200 3325 50  0001 C CNN
	1    8200 3325
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 4475 8900 4475
$Comp
L power:GND #PWR035
U 1 1 5C7B5C1D
P 10425 5150
F 0 "#PWR035" H 10425 4900 50  0001 C CNN
F 1 "GND" H 10430 4977 50  0000 C CNN
F 2 "" H 10425 5150 50  0001 C CNN
F 3 "" H 10425 5150 50  0001 C CNN
	1    10425 5150
	1    0    0    -1  
$EndComp
Text GLabel 6925 4575 0    50   Input ~ 0
CURRENT_SENSE
$Comp
L Device:R R11
U 1 1 5C7B657B
P 7150 4325
F 0 "R11" V 7100 4500 50  0000 C CNN
F 1 "10K" V 7150 4325 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7080 4325 50  0001 C CNN
F 3 "~" H 7150 4325 50  0001 C CNN
	1    7150 4325
	0    1    1    0   
$EndComp
$Comp
L Device:R R13
U 1 1 5C7B71BF
P 7150 4575
F 0 "R13" V 7100 4750 50  0000 C CNN
F 1 "10K" V 7150 4575 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7080 4575 50  0001 C CNN
F 3 "~" H 7150 4575 50  0001 C CNN
	1    7150 4575
	0    1    1    0   
$EndComp
Wire Wire Line
	6925 4575 7000 4575
Wire Wire Line
	7300 4575 7700 4575
Wire Wire Line
	8200 3325 8200 3500
Wire Wire Line
	8625 3500 8200 3500
Connection ~ 8200 3500
Wire Wire Line
	8200 3500 8200 3925
$Comp
L Device:R R16
U 1 1 5C7BA48C
P 7700 4975
F 0 "R16" H 7770 5021 50  0000 L CNN
F 1 "SENSE" H 7770 4930 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7630 4975 50  0001 C CNN
F 3 "~" H 7700 4975 50  0001 C CNN
	1    7700 4975
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 5C7BB477
P 7700 5125
F 0 "#PWR028" H 7700 4875 50  0001 C CNN
F 1 "GND" H 7705 4952 50  0000 C CNN
F 2 "" H 7700 5125 50  0001 C CNN
F 3 "" H 7700 5125 50  0001 C CNN
	1    7700 5125
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4825 7700 4575
Connection ~ 7700 4575
Wire Wire Line
	7700 4575 7950 4575
Wire Wire Line
	4550 4325 7000 4325
$Comp
L Device:R R12
U 1 1 5C7BE87C
P 7150 4425
F 0 "R12" V 7100 4600 50  0000 C CNN
F 1 "10K" V 7150 4425 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7080 4425 50  0001 C CNN
F 3 "~" H 7150 4425 50  0001 C CNN
	1    7150 4425
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C11
U 1 1 5C7BF9D7
P 9125 3600
F 0 "C11" H 9217 3646 50  0000 L CNN
F 1 "10uF" H 9217 3555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9125 3600 50  0001 C CNN
F 3 "~" H 9125 3600 50  0001 C CNN
	1    9125 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5C7BFB0D
P 9125 3700
F 0 "#PWR033" H 9125 3450 50  0001 C CNN
F 1 "GND" H 9130 3527 50  0000 C CNN
F 2 "" H 9125 3700 50  0001 C CNN
F 3 "" H 9125 3700 50  0001 C CNN
	1    9125 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8625 3500 9125 3500
Connection ~ 8625 3500
$Comp
L Device:R R17
U 1 1 5C7C0432
P 9150 4850
F 0 "R17" H 9220 4896 50  0000 L CNN
F 1 "10K" H 9220 4805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9080 4850 50  0001 C CNN
F 3 "~" H 9150 4850 50  0001 C CNN
	1    9150 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 5C7C051B
P 9150 5300
F 0 "D4" V 9188 5183 50  0000 R CNN
F 1 "LED" V 9097 5183 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9150 5300 50  0001 C CNN
F 3 "~" H 9150 5300 50  0001 C CNN
	1    9150 5300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR032
U 1 1 5C7C05D8
P 9150 5450
F 0 "#PWR032" H 9150 5200 50  0001 C CNN
F 1 "GND" H 9155 5277 50  0000 C CNN
F 2 "" H 9150 5450 50  0001 C CNN
F 3 "" H 9150 5450 50  0001 C CNN
	1    9150 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 5150 9150 5000
$Comp
L Device:LED D1
U 1 1 5C7C3D9B
P 1950 5000
F 0 "D1" V 1975 5175 50  0000 R CNN
F 1 "LED" V 1875 5225 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1950 5000 50  0001 C CNN
F 3 "~" H 1950 5000 50  0001 C CNN
	1    1950 5000
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5C7C3FBD
P 2625 4625
F 0 "R1" V 2725 4625 50  0000 C CNN
F 1 "220" V 2625 4625 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 2555 4625 50  0001 C CNN
F 3 "~" H 2625 4625 50  0001 C CNN
	1    2625 4625
	0    -1   -1   0   
$EndComp
Text Label 2825 4625 0    50   ~ 0
LED_1
$Comp
L Device:R R15
U 1 1 5C7C5AEB
P 7600 3425
F 0 "R15" H 7670 3471 50  0000 L CNN
F 1 "100K" H 7670 3380 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7530 3425 50  0001 C CNN
F 3 "~" H 7600 3425 50  0001 C CNN
	1    7600 3425
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5C7C5BA1
P 7600 2925
F 0 "R14" H 7670 2971 50  0000 L CNN
F 1 "620K" H 7670 2880 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7530 2925 50  0001 C CNN
F 3 "~" H 7600 2925 50  0001 C CNN
	1    7600 2925
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 3075 7600 3175
Connection ~ 7600 3175
Wire Wire Line
	7600 3175 7600 3275
$Comp
L power:GND #PWR027
U 1 1 5C7C73C4
P 7600 3575
F 0 "#PWR027" H 7600 3325 50  0001 C CNN
F 1 "GND" H 7605 3402 50  0000 C CNN
F 2 "" H 7600 3575 50  0001 C CNN
F 3 "" H 7600 3575 50  0001 C CNN
	1    7600 3575
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3175 7125 3175
Text Label 6375 3925 0    50   ~ 0
BATT_SENSE
$Comp
L Device:R R10
U 1 1 5C7CB5D6
P 7125 3425
F 0 "R10" H 7195 3471 50  0000 L CNN
F 1 "10K" H 7195 3380 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7055 3425 50  0001 C CNN
F 3 "~" H 7125 3425 50  0001 C CNN
	1    7125 3425
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5C7CB72F
P 7125 3675
F 0 "C9" H 7217 3721 50  0000 L CNN
F 1 "1uF" H 7217 3630 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7125 3675 50  0001 C CNN
F 3 "~" H 7125 3675 50  0001 C CNN
	1    7125 3675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5C7CB7D1
P 7125 3775
F 0 "#PWR026" H 7125 3525 50  0001 C CNN
F 1 "GND" H 7130 3602 50  0000 C CNN
F 2 "" H 7125 3775 50  0001 C CNN
F 3 "" H 7125 3775 50  0001 C CNN
	1    7125 3775
	1    0    0    -1  
$EndComp
Wire Wire Line
	7125 3275 7125 3175
Connection ~ 7125 3175
Wire Wire Line
	7125 3175 7600 3175
$Comp
L Device:R R8
U 1 1 5C7CD93D
P 6000 2925
F 0 "R8" H 6070 2971 50  0000 L CNN
F 1 "620K" H 6070 2880 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5930 2925 50  0001 C CNN
F 3 "~" H 6000 2925 50  0001 C CNN
	1    6000 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5C7CD9C1
P 6000 3375
F 0 "R9" H 6070 3421 50  0000 L CNN
F 1 "100K" H 6070 3330 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5930 3375 50  0001 C CNN
F 3 "~" H 6000 3375 50  0001 C CNN
	1    6000 3375
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5C7CDA7B
P 6000 3525
F 0 "#PWR023" H 6000 3275 50  0001 C CNN
F 1 "GND" H 6005 3352 50  0000 C CNN
F 2 "" H 6000 3525 50  0001 C CNN
F 3 "" H 6000 3525 50  0001 C CNN
	1    6000 3525
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR022
U 1 1 5C7CDD58
P 6000 2775
F 0 "#PWR022" H 6000 2625 50  0001 C CNN
F 1 "+24V" H 6015 2948 50  0000 C CNN
F 2 "" H 6000 2775 50  0001 C CNN
F 3 "" H 6000 2775 50  0001 C CNN
	1    6000 2775
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2675 7600 2775
Wire Wire Line
	7300 4425 7950 4425
Wire Wire Line
	4550 4425 7000 4425
Text Label 7550 4425 0    50   ~ 0
FR_DIAG
Text Label 5025 4325 0    50   ~ 0
PWM
$Comp
L Device:R R3
U 1 1 5C7E9055
P 3150 4175
F 0 "R3" H 3220 4221 50  0000 L CNN
F 1 "10K" H 3220 4130 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3080 4175 50  0001 C CNN
F 3 "~" H 3150 4175 50  0001 C CNN
	1    3150 4175
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5C7E92E6
P 3150 4325
F 0 "#PWR08" H 3150 4075 50  0001 C CNN
F 1 "GND" H 3155 4152 50  0000 C CNN
F 2 "" H 3150 4325 50  0001 C CNN
F 3 "" H 3150 4325 50  0001 C CNN
	1    3150 4325
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4025 3150 4025
$Comp
L power:+3V3 #PWR024
U 1 1 5C7C16CB
P 6275 1250
F 0 "#PWR024" H 6275 1100 50  0001 C CNN
F 1 "+3V3" H 6290 1423 50  0000 C CNN
F 2 "" H 6275 1250 50  0001 C CNN
F 3 "" H 6275 1250 50  0001 C CNN
	1    6275 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 1350 6275 1350
Wire Wire Line
	6275 1350 6275 1250
Text GLabel 6150 1450 2    50   Input ~ 0
SWCLK
Text GLabel 6150 1650 2    50   Input ~ 0
SWDI0
Text GLabel 6150 1750 2    50   Input ~ 0
NRST
$Comp
L power:GND #PWR025
U 1 1 5C7C33ED
P 6625 1550
F 0 "#PWR025" H 6625 1300 50  0001 C CNN
F 1 "GND" H 6630 1377 50  0000 C CNN
F 2 "" H 6625 1550 50  0001 C CNN
F 3 "" H 6625 1550 50  0001 C CNN
	1    6625 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 1550 6625 1550
$Comp
L Connector:Conn_01x06_Male J1
U 1 1 5C7C4E82
P 5950 1550
F 0 "J1" H 5850 1475 50  0000 C CNN
F 1 "SWD" H 5850 1550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 5950 1550 50  0001 C CNN
F 3 "~" H 5950 1550 50  0001 C CNN
	1    5950 1550
	1    0    0    -1  
$EndComp
Text GLabel 6150 1850 2    50   Input ~ 0
UART_TX
$Comp
L project-rescue:ADS1015-adc U1
U 1 1 592DD751
P 2350 6925
F 0 "U1" H 1750 7375 50  0000 L CNN
F 1 "ADS1015" H 2600 7375 50  0000 L CNN
F 2 "theapi:TSSOP-10_3x3mm_P0.5mm" H 2300 6875 50  0001 C CNN
F 3 "" H 1450 7325 50  0001 C CNN
	1    2350 6925
	1    0    0    -1  
$EndComp
Text Notes 600  5975 0    98   ~ 20
Optional External ADC
Text GLabel 3900 6725 2    60   Input ~ 0
SCL
Text GLabel 3900 6825 2    60   Input ~ 0
SDA
$Comp
L project-rescue:R-stm32l053c-rescue R5
U 1 1 59300E59
P 3650 6375
F 0 "R5" V 3730 6375 50  0000 C CNN
F 1 "10K" V 3650 6375 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3580 6375 50  0001 C CNN
F 3 "" H 3650 6375 50  0001 C CNN
	1    3650 6375
	1    0    0    -1  
$EndComp
$Comp
L project-rescue:R-stm32l053c-rescue R4
U 1 1 59300F14
P 3450 6375
F 0 "R4" V 3530 6375 50  0000 C CNN
F 1 "10K" V 3450 6375 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3380 6375 50  0001 C CNN
F 3 "" H 3450 6375 50  0001 C CNN
	1    3450 6375
	1    0    0    -1  
$EndComp
Text GLabel 3925 6925 2    60   Input ~ 0
ADC_RDY
$Comp
L project-rescue:C-stm32l053c-rescue C2
U 1 1 5930245B
P 3150 6275
F 0 "C2" H 3175 6375 50  0000 L CNN
F 1 "0.1UF" H 3175 6175 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3188 6125 50  0001 C CNN
F 3 "" H 3150 6275 50  0001 C CNN
	1    3150 6275
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR04
U 1 1 59303669
P 2350 6125
F 0 "#PWR04" H 2350 5975 50  0001 C CNN
F 1 "+3.3V" H 2350 6265 50  0000 C CNN
F 2 "" H 2350 6125 50  0001 C CNN
F 3 "" H 2350 6125 50  0001 C CNN
	1    2350 6125
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 59303771
P 3550 6125
F 0 "#PWR014" H 3550 5975 50  0001 C CNN
F 1 "+3.3V" H 3550 6265 50  0000 C CNN
F 2 "" H 3550 6125 50  0001 C CNN
F 3 "" H 3550 6125 50  0001 C CNN
	1    3550 6125
	1    0    0    -1  
$EndComp
Text Label 1400 7225 0    60   ~ 0
AIN3
Wire Wire Line
	3050 6725 3650 6725
Wire Wire Line
	3050 6825 3450 6825
Wire Wire Line
	3450 6525 3450 6825
Connection ~ 3450 6825
Wire Wire Line
	3650 6525 3650 6725
Connection ~ 3650 6725
Wire Wire Line
	3450 6225 3550 6225
Connection ~ 3550 6225
Wire Wire Line
	3250 7125 3250 6625
Wire Wire Line
	3250 6625 3050 6625
Wire Wire Line
	3050 6925 3850 6925
Wire Wire Line
	2350 6125 3150 6125
Wire Wire Line
	2350 6125 2350 6425
Wire Wire Line
	3550 6225 3550 6125
Wire Notes Line
	575  5800 575  7650
Wire Notes Line
	575  7650 4675 7650
Wire Notes Line
	4675 7650 4675 5800
Wire Notes Line
	4675 5800 575  5800
$Comp
L project-rescue:R-stm32l053c-rescue R6
U 1 1 59492AEA
P 3850 6375
F 0 "R6" V 3930 6375 50  0000 C CNN
F 1 "10K" V 3850 6375 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3780 6375 50  0001 C CNN
F 3 "" H 3850 6375 50  0001 C CNN
	1    3850 6375
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 6525 3850 6925
Connection ~ 3850 6925
Connection ~ 3650 6225
Wire Wire Line
	3450 6825 3900 6825
Wire Wire Line
	3650 6725 3900 6725
Wire Wire Line
	3550 6225 3650 6225
Wire Wire Line
	3850 6925 3925 6925
Wire Wire Line
	3650 6225 3850 6225
Connection ~ 2350 6125
Wire Wire Line
	1400 7125 1650 7125
Text GLabel 4550 4525 2    60   Input ~ 0
ADC_RDY
$Comp
L power:GND #PWR011
U 1 1 5C7F165E
P 3250 7125
F 0 "#PWR011" H 3250 6875 50  0001 C CNN
F 1 "GND" H 3255 6952 50  0000 C CNN
F 2 "" H 3250 7125 50  0001 C CNN
F 3 "" H 3250 7125 50  0001 C CNN
	1    3250 7125
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5C7F17E4
P 2350 7425
F 0 "#PWR05" H 2350 7175 50  0001 C CNN
F 1 "GND" H 2355 7252 50  0000 C CNN
F 2 "" H 2350 7425 50  0001 C CNN
F 3 "" H 2350 7425 50  0001 C CNN
	1    2350 7425
	1    0    0    -1  
$EndComp
Text Label 1150 6725 0    50   ~ 0
BATT_SENSE
Wire Wire Line
	1650 6725 1150 6725
Text GLabel 1650 7025 0    50   Input ~ 0
CURRENT_SENSE
$Comp
L Connector:TestPoint TP1
U 1 1 5C7F7EC9
P 1400 7275
F 0 "TP1" V 1400 7475 50  0000 L CNN
F 1 "TestPoint" V 1400 7475 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 1600 7275 50  0001 C CNN
F 3 "~" H 1600 7275 50  0001 C CNN
	1    1400 7275
	-1   0    0    1   
$EndComp
Wire Wire Line
	1400 7275 1400 7125
Wire Wire Line
	6900 3175 6900 3925
Wire Wire Line
	4550 3925 6900 3925
Wire Wire Line
	6000 3075 6000 3150
Wire Wire Line
	4550 3825 5725 3825
Wire Wire Line
	5725 3825 5725 3150
Wire Wire Line
	5725 3150 6000 3150
Connection ~ 6000 3150
Wire Wire Line
	6000 3150 6000 3225
Text Label 5150 3825 0    50   ~ 0
PANEL_SENSE
Text Label 1150 6825 0    50   ~ 0
PANEL_SENSE
Wire Wire Line
	1650 6825 1150 6825
$Comp
L Device:R R7
U 1 1 5C7DB05A
P 5675 4875
F 0 "R7" H 5575 4825 50  0000 C CNN
F 1 "220" H 5550 4925 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5605 4875 50  0001 C CNN
F 3 "~" H 5675 4875 50  0001 C CNN
	1    5675 4875
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D3
U 1 1 5C7DB146
P 5675 5175
F 0 "D3" V 5713 5058 50  0000 R CNN
F 1 "LED" V 5622 5058 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5675 5175 50  0001 C CNN
F 3 "~" H 5675 5175 50  0001 C CNN
	1    5675 5175
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5C7DB1DC
P 2050 5250
F 0 "#PWR03" H 2050 5000 50  0001 C CNN
F 1 "GND" H 2055 5077 50  0000 C CNN
F 2 "" H 2050 5250 50  0001 C CNN
F 3 "" H 2050 5250 50  0001 C CNN
	1    2050 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2775 4725 3550 4725
Wire Wire Line
	2150 5250 2150 5150
Wire Wire Line
	1950 5150 1950 5250
Wire Wire Line
	2775 4625 3550 4625
Wire Wire Line
	5675 4725 5675 4225
Wire Wire Line
	4550 4225 5675 4225
$Comp
L Device:R R2
U 1 1 5C7F09D9
P 2625 4725
F 0 "R2" V 2525 4725 50  0000 C CNN
F 1 "220" V 2625 4725 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 2555 4725 50  0001 C CNN
F 3 "~" H 2625 4725 50  0001 C CNN
	1    2625 4725
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5C7F0A89
P 2150 5000
F 0 "D2" V 2188 4883 50  0000 R CNN
F 1 "LED" V 2097 4883 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2150 5000 50  0001 C CNN
F 3 "~" H 2150 5000 50  0001 C CNN
	1    2150 5000
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5C7F0B29
P 5675 5325
F 0 "#PWR021" H 5675 5075 50  0001 C CNN
F 1 "GND" H 5680 5152 50  0000 C CNN
F 2 "" H 5675 5325 50  0001 C CNN
F 3 "" H 5675 5325 50  0001 C CNN
	1    5675 5325
	1    0    0    -1  
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
P 1025 1450
F 0 "SC1" H 1133 1496 50  0000 L CNN
F 1 "Solar_Panel" H 1133 1405 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" V 1025 1510 50  0001 C CNN
F 3 "~" V 1025 1510 50  0001 C CNN
	1    1025 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1025 1250 1025 1000
$Comp
L power:GND #PWR01
U 1 1 5C80809F
P 1025 1650
F 0 "#PWR01" H 1025 1400 50  0001 C CNN
F 1 "GND" H 1030 1477 50  0000 C CNN
F 2 "" H 1025 1650 50  0001 C CNN
F 3 "" H 1025 1650 50  0001 C CNN
	1    1025 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5C8080FA
P 1700 1100
F 0 "C1" H 1792 1146 50  0000 L CNN
F 1 "10uF" H 1792 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1700 1100 50  0001 C CNN
F 3 "~" H 1700 1100 50  0001 C CNN
	1    1700 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1000 2875 1000
$Comp
L power:GND #PWR02
U 1 1 5C808194
P 1700 1200
F 0 "#PWR02" H 1700 950 50  0001 C CNN
F 1 "GND" H 1705 1027 50  0000 C CNN
F 2 "" H 1700 1200 50  0001 C CNN
F 3 "" H 1700 1200 50  0001 C CNN
	1    1700 1200
	1    0    0    -1  
$EndComp
Wire Notes Line
	4700 600  7900 600 
Wire Notes Line
	7900 625  7900 2350
Wire Notes Line
	7875 2350 4700 2350
Wire Notes Line
	4700 2350 4700 625 
Text Notes 4775 800  0    98   ~ 20
SWD Header
Wire Wire Line
	2475 4725 2150 4725
Wire Wire Line
	2150 4725 2150 4850
Wire Wire Line
	2475 4625 1950 4625
Wire Wire Line
	1950 4625 1950 4850
Wire Wire Line
	1950 5250 2050 5250
Wire Wire Line
	2150 5250 2050 5250
Connection ~ 2050 5250
$Comp
L Device:D_Schottky D5
U 1 1 5C7C7D61
P 10000 4475
F 0 "D5" H 10000 4259 50  0000 C CNN
F 1 "D_Schottky" H 10000 4350 50  0000 C CNN
F 2 "Diode_THT:D_DO-201AD_P12.70mm_Horizontal" H 10000 4475 50  0001 C CNN
F 3 "~" H 10000 4475 50  0001 C CNN
	1    10000 4475
	-1   0    0    1   
$EndComp
$Comp
L theapi_driver:VN7003ALH U4
U 1 1 5C7F933B
P 8300 4225
F 0 "U4" H 8375 4475 50  0000 C CNN
F 1 "VN7003ALH" H 8550 3600 50  0000 C CNN
F 2 "theapi:Octapak" H 8300 4225 50  0001 C CNN
F 3 "" H 8300 4225 50  0001 C CNN
	1    8300 4225
	1    0    0    -1  
$EndComp
Wire Wire Line
	1025 1000 1700 1000
Connection ~ 2050 1000
Connection ~ 1700 1000
Wire Wire Line
	1700 1000 2050 1000
$Comp
L power:+24V #PWR036
U 1 1 5C83171F
P 10575 3575
F 0 "#PWR036" H 10575 3425 50  0001 C CNN
F 1 "+24V" H 10590 3748 50  0000 C CNN
F 2 "" H 10575 3575 50  0001 C CNN
F 3 "" H 10575 3575 50  0001 C CNN
	1    10575 3575
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 4475 10600 4675
Wire Wire Line
	8750 4575 8900 4575
Wire Wire Line
	8900 4575 8900 4475
Wire Wire Line
	8750 4675 8900 4675
Wire Wire Line
	8900 4675 8900 4575
Connection ~ 8900 4575
Wire Wire Line
	8900 4475 9150 4475
Connection ~ 8900 4475
Wire Wire Line
	9150 4700 9150 4475
Connection ~ 9150 4475
Wire Wire Line
	9150 4475 9650 4475
NoConn ~ 4550 4125
$Comp
L theapi_Battery:Battery_Meter BT1
U 1 1 5C87CDCA
P 10700 4875
F 0 "BT1" H 10475 4950 50  0000 L CNN
F 1 "Battery_Meter" H 10125 4875 50  0000 L CNN
F 2 "theapi:TerminalBlock_Meter" H 10700 4875 50  0001 C CNN
F 3 "" H 10700 4875 50  0001 C CNN
	1    10700 4875
	1    0    0    -1  
$EndComp
Wire Wire Line
	10425 5150 10425 5125
Wire Wire Line
	10425 5125 10600 5125
$Comp
L power:-BATT #PWR0101
U 1 1 5C892492
P 10800 5825
F 0 "#PWR0101" H 10800 5675 50  0001 C CNN
F 1 "-BATT" H 10815 5998 50  0000 C CNN
F 2 "" H 10800 5825 50  0001 C CNN
F 3 "" H 10800 5825 50  0001 C CNN
	1    10800 5825
	-1   0    0    1   
$EndComp
Wire Wire Line
	7600 2675 9650 2675
Text Label 9325 4475 0    50   ~ 0
OUT
Text Label 7550 4325 0    50   ~ 0
INPUT
Text Label 2825 4725 0    50   ~ 0
LED_2
Text Label 5025 4225 0    50   ~ 0
LED_3
Text Label 9150 5100 0    50   ~ 0
LED_4
$Comp
L power:GND #PWR0102
U 1 1 5C843977
P 3150 6425
F 0 "#PWR0102" H 3150 6175 50  0001 C CNN
F 1 "GND" H 3155 6252 50  0000 C CNN
F 2 "" H 3150 6425 50  0001 C CNN
F 3 "" H 3150 6425 50  0001 C CNN
	1    3150 6425
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 4325 7950 4325
Connection ~ 9650 4475
Wire Wire Line
	9650 4475 9850 4475
Connection ~ 10600 4475
Wire Wire Line
	9650 2675 9650 4475
Wire Wire Line
	10150 4475 10600 4475
Text Label 10225 4475 0    50   ~ 0
DIODE
$Comp
L Connector:Conn_01x03_Male J3
U 1 1 5C8760FB
P 10900 3925
F 0 "J3" H 11100 3600 50  0000 R CNN
F 1 "Voltmeter" H 11125 3700 50  0000 R CNN
F 2 "theapi:Voltmeter_Header_1x03" H 10900 3925 50  0001 C CNN
F 3 "~" H 10900 3925 50  0001 C CNN
	1    10900 3925
	-1   0    0    1   
$EndComp
Wire Wire Line
	10575 3575 10575 3825
Wire Wire Line
	10575 3825 10700 3825
Wire Wire Line
	10600 3925 10700 3925
Wire Wire Line
	10600 3925 10600 4475
$Comp
L power:GND #PWR09
U 1 1 5C881C03
P 10700 4025
F 0 "#PWR09" H 10700 3775 50  0001 C CNN
F 1 "GND" H 10705 3852 50  0000 C CNN
F 2 "" H 10700 4025 50  0001 C CNN
F 3 "" H 10700 4025 50  0001 C CNN
	1    10700 4025
	1    0    0    -1  
$EndComp
$EndSCHEMATC
