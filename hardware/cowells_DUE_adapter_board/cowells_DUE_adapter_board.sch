EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
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
L cowells_DUE_adapter_board-rescue:Conn_02x10_Odd_Even J4
U 1 1 5A8DE4A6
P 14000 5150
F 0 "J4" H 14050 5650 50  0000 C CNN
F 1 "DRV8305J2" H 14050 4550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x10_Pitch2.54mm" H 14000 5150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ug/slvuai8/slvuai8.pdf" H 14000 5150 50  0001 C CNN
	1    14000 5150
	-1   0    0    -1  
$EndComp
NoConn ~ 13450 5650
NoConn ~ 13450 5550
NoConn ~ 13450 5450
$Comp
L cowells_DUE_adapter_board-rescue:Conn_02x10_Odd_Even J1
U 1 1 5A8DF1BA
P 11450 5150
F 0 "J1" H 11500 5650 50  0000 C CNN
F 1 "DRV8305J1" H 11500 4550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x10_Pitch2.54mm" H 11450 5150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ug/slvuai8/slvuai8.pdf" H 11450 5150 50  0001 C CNN
	1    11450 5150
	-1   0    0    -1  
$EndComp
NoConn ~ 11650 4750
NoConn ~ 11150 5650
NoConn ~ 11150 5550
NoConn ~ 11150 5450
NoConn ~ 11150 4850
NoConn ~ 11150 5050
NoConn ~ 11150 5150
NoConn ~ 11150 5250
NoConn ~ 11650 5650
NoConn ~ 13450 5350
NoConn ~ 14450 4950
NoConn ~ 14450 5050
NoConn ~ 14450 5650
Text Label 10800 4950 0    60   ~ 0
nFAULT
Text Label 11150 5350 2    60   ~ 0
SCLK
Text Label 11650 4950 0    60   ~ 0
VsenA
Text Label 11650 5050 0    60   ~ 0
VsenB
Text Label 11650 5150 0    60   ~ 0
VsenC
Text Label 11650 5250 0    60   ~ 0
Vsen_PVDD
$Comp
L power:PWR_FLAG #VCCflag01
U 1 1 5A8DF3F6
P 2100 4850
F 0 "#VCCflag01" H 2100 4925 50  0001 C CNN
F 1 "PWR_FLAG" H 2100 5000 50  0000 C CNN
F 2 "" H 2100 4850 50  0001 C CNN
F 3 "" H 2100 4850 50  0001 C CNN
	1    2100 4850
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #GNDflag01
U 1 1 5A8DF413
P 1600 4850
F 0 "#GNDflag01" H 1600 4925 50  0001 C CNN
F 1 "PWR_FLAG" H 1600 5000 50  0000 C CNN
F 2 "" H 1600 4850 50  0001 C CNN
F 3 "" H 1600 4850 50  0001 C CNN
	1    1600 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5A8DF430
P 1600 5600
F 0 "#PWR01" H 1600 5350 50  0001 C CNN
F 1 "GND" H 1600 5450 50  0000 C CNN
F 2 "" H 1600 5600 50  0001 C CNN
F 3 "" H 1600 5600 50  0001 C CNN
	1    1600 5600
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR02
U 1 1 5A8DF44D
P 2100 5600
F 0 "#PWR02" H 2100 5450 50  0001 C CNN
F 1 "VCC" H 2100 5750 50  0000 C CNN
F 2 "" H 2100 5600 50  0001 C CNN
F 3 "" H 2100 5600 50  0001 C CNN
	1    2100 5600
	-1   0    0    1   
$EndComp
Text Label 4300 8750 2    60   ~ 0
gnd
Text Label 1600 5000 0    60   ~ 0
gnd
Text Label 4400 9250 2    60   ~ 0
3v3
Text Label 2100 5000 0    60   ~ 0
3v3
Text Label 11650 4850 0    60   ~ 0
gnd
Text Label 11650 5450 0    60   ~ 0
IsenB
Text Label 11650 5550 0    60   ~ 0
IsenC
Text Label 13450 4750 0    60   ~ 0
InH_A
Text Label 13450 4950 0    60   ~ 0
InH_B
Text Label 13450 5150 0    60   ~ 0
InH_C
Text Label 13450 4850 0    60   ~ 0
InL_A
Text Label 13450 5050 0    60   ~ 0
InL_B
Text Label 13450 5250 0    60   ~ 0
InL_C
Text Label 14450 4850 0    60   ~ 0
SCS
Text Label 14450 5150 0    60   ~ 0
PWRGD
Text Label 14200 5350 0    60   ~ 0
SDO
Text Label 14450 5450 0    60   ~ 0
ENGate
Text Label 14450 5550 0    60   ~ 0
Wake
NoConn ~ 14450 5550
Text Label 14450 4750 0    60   ~ 0
gnd
$Comp
L cowells_DUE_adapter_board-rescue:SW_DPST_x2 SW3
U 1 1 5A8DFC38
P 9000 6650
F 0 "SW3" H 9000 6775 50  0000 C CNN
F 1 "Run" H 9150 6550 50  0000 C CNN
F 2 "TerminalBlocks_Phoenix:TerminalBlock_Phoenix_MPT-2.54mm_2pol" H 9000 6650 50  0001 C CNN
F 3 "" H 9000 6650 50  0001 C CNN
	1    9000 6650
	1    0    0    -1  
$EndComp
$Comp
L cowells_DUE_adapter_board-rescue:SW_DPST_x2 SW4
U 1 1 5A8DFC7E
P 9000 7050
F 0 "SW4" H 9000 7175 50  0000 C CNN
F 1 "Dir" H 9100 6950 50  0000 C CNN
F 2 "TerminalBlocks_Phoenix:TerminalBlock_Phoenix_MPT-2.54mm_2pol" H 9000 7050 50  0001 C CNN
F 3 "" H 9000 7050 50  0001 C CNN
	1    9000 7050
	1    0    0    -1  
$EndComp
$Comp
L cowells_DUE_adapter_board-rescue:SW_DPST_x2 SW5
U 1 1 5A8DFD17
P 9000 7450
F 0 "SW5" H 9000 7575 50  0000 C CNN
F 1 "eStop" H 9150 7350 50  0000 C CNN
F 2 "TerminalBlocks_Phoenix:TerminalBlock_Phoenix_MPT-2.54mm_2pol" H 9000 7450 50  0001 C CNN
F 3 "" H 9000 7450 50  0001 C CNN
	1    9000 7450
	1    0    0    -1  
$EndComp
Text Label 4400 7550 2    60   ~ 0
IsenA
Text Label 4400 7750 2    60   ~ 0
IsenB
Text Label 4400 7450 2    60   ~ 0
IsenC
Text Label 4400 7350 2    60   ~ 0
Vsen_PVDD
$Comp
L power:GND #PWR03
U 1 1 5A8E0330
P 2650 9200
F 0 "#PWR03" H 2650 8950 50  0001 C CNN
F 1 "GND" H 2650 9050 50  0000 C CNN
F 2 "" H 2650 9200 50  0001 C CNN
F 3 "" H 2650 9200 50  0001 C CNN
	1    2650 9200
	1    0    0    -1  
$EndComp
$Comp
L cowells_DUE_adapter_board-rescue:POT spd1
U 1 1 5A9062B2
P 2650 8150
F 0 "spd1" V 2475 8150 50  0000 C CNN
F 1 "5k" V 2550 8150 50  0000 C CNN
F 2 "TerminalBlocks_Phoenix:TerminalBlock_Phoenix_MPT-2.54mm_3pol" H 2650 8150 50  0001 C CNN
F 3 "" H 2650 8150 50  0001 C CNN
	1    2650 8150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5A90A8DD
P 8100 7750
F 0 "#PWR06" H 8100 7500 50  0001 C CNN
F 1 "GND" H 8100 7600 50  0000 C CNN
F 2 "" H 8100 7750 50  0001 C CNN
F 3 "" H 8100 7750 50  0001 C CNN
	1    8100 7750
	1    0    0    -1  
$EndComp
Text Label 5650 4900 1    60   ~ 0
SCK
Text Label 7000 7450 0    60   ~ 0
SCS
Text Label 7000 7250 0    60   ~ 0
PWRGD
Text Label 7000 7350 0    60   ~ 0
ENGate
$Comp
L cowells_DUE_adapter_board-rescue:SW_Push SW2
U 1 1 5A90C30C
P 3500 8450
F 0 "SW2" H 3550 8550 50  0000 L CNN
F 1 "sw_rst" H 3500 8390 50  0000 C CNN
F 2 "TerminalBlocks_Phoenix:TerminalBlock_Phoenix_MPT-2.54mm_2pol" H 3500 8650 50  0001 C CNN
F 3 "" H 3500 8650 50  0001 C CNN
	1    3500 8450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5A90C36B
P 3200 8650
F 0 "R2" V 3280 8650 50  0000 C CNN
F 1 "10k" V 3200 8650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3130 8650 50  0001 C CNN
F 3 "" H 3200 8650 50  0001 C CNN
	1    3200 8650
	1    0    0    -1  
$EndComp
Text Label 7000 9250 0    60   ~ 0
nFAULT
Text Label 11650 5350 0    60   ~ 0
IsenA
NoConn ~ 5750 4900
$Comp
L power:GND #PWR05
U 1 1 5A90D9E4
P 5850 4700
F 0 "#PWR05" H 5850 4450 50  0001 C CNN
F 1 "GND" H 5850 4550 50  0000 C CNN
F 2 "" H 5850 4700 50  0001 C CNN
F 3 "" H 5850 4700 50  0001 C CNN
	1    5850 4700
	-1   0    0    1   
$EndComp
NoConn ~ 5950 4900
$Comp
L Device:Thermistor_NTC TH1
U 1 1 5A90E273
P 1800 8150
F 0 "TH1" V 1950 8150 50  0000 C CNN
F 1 "NTC3k" V 1600 8150 50  0000 C CNN
F 2 "TerminalBlocks_Phoenix:TerminalBlock_Phoenix_MPT-2.54mm_2pol" H 1800 8200 50  0001 C CNN
F 3 "" H 1800 8200 50  0001 C CNN
	1    1800 8150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5A90E3C9
P 1800 8650
F 0 "R1" V 1880 8650 50  0000 C CNN
F 1 "1k" V 1800 8650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1730 8650 50  0001 C CNN
F 3 "" H 1800 8650 50  0001 C CNN
	1    1800 8650
	1    0    0    -1  
$EndComp
Text Label 1800 7850 0    60   ~ 0
Th1Ex
Text Label 4400 8150 2    60   ~ 0
TH1
Text Label 2000 8400 0    60   ~ 0
TH1
$Comp
L cowells_DUE_adapter_board-rescue:Conn_01x08_Male J2
U 1 1 5A91E8D1
P 3400 1950
F 0 "J2" V 3400 2350 50  0000 C CNN
F 1 "AS5048A" V 3300 1950 50  0000 C CNN
F 2 "Connectors_Molex:Molex_PicoBlade_53047-0810_08x1.25mm_Straight" H 3400 1950 50  0001 C CNN
F 3 "http://ams.com/eng/Support/Demoboards/Magnetic-Position-Sensors/Angle-Position-On-Axis/AS5048A-Adapterboard" H 3400 1950 50  0001 C CNN
	1    3400 1950
	0    -1   1    0   
$EndComp
Text Label 3100 2150 3    60   ~ 0
5v
Text Label 3200 2150 3    60   ~ 0
3v3
Text Label 3300 2150 3    60   ~ 0
PWM
Text Label 3400 2500 3    60   ~ 0
CsnMot
Text Label 3500 2150 3    60   ~ 0
SCK
Text Label 3800 2150 3    60   ~ 0
gnd
Text Label 7500 2450 2    60   ~ 0
MISO
Text Label 7500 2550 2    60   ~ 0
MOSI
Text Label 7500 2350 2    60   ~ 0
SCK
$Comp
L cowells_DUE_adapter_board-rescue:Conn_01x08_Male J3
U 1 1 5A922496
P 9750 2350
F 0 "J3" V 9750 2750 50  0000 C CNN
F 1 "spindl enc." V 9650 2350 50  0000 C CNN
F 2 "Connectors_Molex:Molex_PicoBlade_53047-0810_08x1.25mm_Straight" H 9750 2350 50  0001 C CNN
F 3 "http://ams.com/eng/Support/Demoboards/Magnetic-Position-Sensors/Angle-Position-On-Axis/AS5048A-Adapterboard" H 9750 2350 50  0001 C CNN
	1    9750 2350
	-1   0    0    -1  
$EndComp
Text Label 8700 1950 0    60   ~ 0
3v3
Text Label 9550 2550 2    60   ~ 0
gnd
Text Label 7500 2150 2    60   ~ 0
gnd
Text Label 8700 2050 0    60   ~ 0
gnd
Text Notes 9350 2900 0    60   ~ 0
HEDS ??
Text Label 10450 2450 2    60   ~ 0
MISO
Text Label 10450 2550 2    60   ~ 0
MOSI
Text Label 10450 2350 2    60   ~ 0
SCK
$Comp
L cowells_DUE_adapter_board-rescue:Conn_01x08_Male J5
U 1 1 5A924C3B
P 12550 2350
F 0 "J5" V 12550 2750 50  0000 C CNN
F 1 "AS5311AB" V 12450 2350 50  0000 C CNN
F 2 "Connectors_Molex:Molex_PicoBlade_53047-0810_08x1.25mm_Straight" H 12550 2350 50  0001 C CNN
F 3 "http://ams.com/eng/Support/Demoboards/Magnetic-Position-Sensors/Angle-Position-On-Axis/AS5048A-Adapterboard" H 12550 2350 50  0001 C CNN
	1    12550 2350
	-1   0    0    -1  
$EndComp
Text Label 11650 1950 0    60   ~ 0
3v3
Text Label 12350 2550 2    60   ~ 0
ClkX
Text Label 10450 2150 2    60   ~ 0
gnd
Text Label 11650 2050 0    60   ~ 0
gnd
Text Label 13250 2350 2    60   ~ 0
SCK
$Comp
L cowells_DUE_adapter_board-rescue:Conn_01x08_Male J6
U 1 1 5A924CC3
P 15450 2350
F 0 "J6" V 15450 2750 50  0000 C CNN
F 1 "AS5311AB" V 15350 2350 50  0000 C CNN
F 2 "Connectors_Molex:Molex_PicoBlade_53047-0810_08x1.25mm_Straight" H 15450 2350 50  0001 C CNN
F 3 "http://ams.com/eng/Support/Demoboards/Magnetic-Position-Sensors/Angle-Position-On-Axis/AS5048A-Adapterboard" H 15450 2350 50  0001 C CNN
	1    15450 2350
	-1   0    0    -1  
$EndComp
Text Label 14450 1950 0    60   ~ 0
3v3
Text Label 13250 2150 2    60   ~ 0
gnd
Text Label 12350 2450 2    60   ~ 0
CsnX
$Comp
L cowells_DUE_adapter_board-rescue:NL17SZ14 U1
U 1 1 5A925548
P 7950 1150
F 0 "U1" H 7800 1300 60  0000 C CNN
F 1 "NL17SZ14" H 7950 800 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-353_SC-70-5" H 7950 1150 60  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/ON-Semicon-ON-NL17SZ14DFT2G_C113256.pdf" H 7950 1150 60  0001 C CNN
	1    7950 1150
	1    0    0    -1  
$EndComp
Text Label 7000 2850 3    60   ~ 0
SS_spin
Text Label 9950 3300 3    60   ~ 0
SS_x
Text Label 12800 3250 3    60   ~ 0
SS_z
$Comp
L Device:R R3
U 1 1 5A930DF7
P 6600 2150
F 0 "R3" V 6680 2150 50  0000 C CNN
F 1 "1M" V 6600 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6530 2150 50  0001 C CNN
F 3 "" H 6600 2150 50  0001 C CNN
	1    6600 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5A930E65
P 5800 2400
F 0 "C2" H 5825 2500 50  0000 L CNN
F 1 "18p" H 5825 2300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5838 2250 50  0001 C CNN
F 3 "" H 5800 2400 50  0001 C CNN
	1    5800 2400
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 5A930EDA
P 5800 1950
F 0 "C1" H 5825 2050 50  0000 L CNN
F 1 "18p" H 5825 1850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5838 1800 50  0001 C CNN
F 3 "" H 5800 1950 50  0001 C CNN
	1    5800 1950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5A93194C
P 5500 2600
F 0 "#PWR04" H 5500 2350 50  0001 C CNN
F 1 "GND" H 5500 2450 50  0000 C CNN
F 2 "" H 5500 2600 50  0001 C CNN
F 3 "" H 5500 2600 50  0001 C CNN
	1    5500 2600
	1    0    0    -1  
$EndComp
Text Notes 5350 1700 0    40   ~ 0
Cm = 2(CL - CP) - 10pF, where\nCL = manufacturer's specified crystal load capacitance.\nCP = PCB parasitic capacitance seen by the crystal. \n
Text Label 8400 1150 0    60   ~ 0
3v3
Text Label 7500 1350 2    60   ~ 0
gnd
NoConn ~ 7500 1150
NoConn ~ 3300 2150
NoConn ~ 10450 1950
NoConn ~ 13250 1950
Text Label 7000 7550 0    60   ~ 0
SS_spin
Text Label 7000 6750 0    60   ~ 0
SS_x
Text Label 7000 6950 0    60   ~ 0
SS_z
Text Label 7000 5550 0    60   ~ 0
CsnX
Text Label 15250 2450 2    60   ~ 0
CsnZ
Text Label 7000 5450 0    60   ~ 0
CsnZ
Text Label 7000 7150 0    60   ~ 0
CsnMot
NoConn ~ 3100 2150
$Comp
L LS7366R:LS7366R UX1
U 1 1 5A9380CD
P 11050 2250
F 0 "UX1" H 10649 2676 50  0000 L BNN
F 1 "LS7366R" H 10650 1750 50  0000 L BNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 10650 1500 50  0001 L BNN
F 3 "http://www.lsicsi.com/pdfs/Data_Sheets/LS7366R.pdf" H 10650 1650 50  0001 L BNN
F 4 "LSI" H 11050 2250 50  0001 L BNN "Manufacturer"
	1    11050 2250
	1    0    0    -1  
$EndComp
$Comp
L LS7366R:LS7366R UZ1
U 1 1 5A9382D5
P 13850 2250
F 0 "UZ1" H 13449 2676 50  0000 L BNN
F 1 "LS7366R" H 13450 1750 50  0000 L BNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 13450 1500 50  0001 L BNN
F 3 "http://www.lsicsi.com/pdfs/Data_Sheets/LS7366R.pdf" H 13450 1650 50  0001 L BNN
F 4 "LSI" H 13850 2250 50  0001 L BNN "Manufacturer"
	1    13850 2250
	1    0    0    -1  
$EndComp
$Comp
L LS7366R:LS7366R Uspin1
U 1 1 5A9387A9
P 8100 2250
F 0 "Uspin1" H 7699 2676 50  0000 L BNN
F 1 "LS7366R" H 7700 1750 50  0000 L BNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 7700 1500 50  0001 L BNN
F 3 "http://www.lsicsi.com/pdfs/Data_Sheets/LS7366R.pdf" H 7700 1650 50  0001 L BNN
F 4 "LSI" H 8100 2250 50  0001 L BNN "Manufacturer"
	1    8100 2250
	1    0    0    -1  
$EndComp
Text Label 7000 9350 0    60   ~ 0
SpnDFLAG
Text Label 7000 8750 0    60   ~ 0
SpnLFLAG
NoConn ~ 14450 2550
NoConn ~ 14450 2450
NoConn ~ 11650 2550
NoConn ~ 11650 2450
NoConn ~ 4400 5350
NoConn ~ 4400 5450
NoConn ~ 4400 5550
NoConn ~ 4400 5650
NoConn ~ 4400 5750
NoConn ~ 4400 5950
NoConn ~ 4400 6050
NoConn ~ 4400 6150
NoConn ~ 4400 6250
NoConn ~ 4400 6350
NoConn ~ 4400 6450
NoConn ~ 4400 6550
NoConn ~ 4400 6650
NoConn ~ 4400 6850
NoConn ~ 4400 6950
NoConn ~ 7000 5350
NoConn ~ 7000 6150
NoConn ~ 7000 6250
NoConn ~ 7000 6350
NoConn ~ 7000 6450
NoConn ~ 7000 8350
NoConn ~ 7000 8450
NoConn ~ 7000 8550
NoConn ~ 7000 8650
NoConn ~ 7000 8850
NoConn ~ 7000 9550
NoConn ~ 7000 9650
NoConn ~ 4400 9650
NoConn ~ 4400 9550
NoConn ~ 4400 9450
NoConn ~ 4400 9350
NoConn ~ 4400 8550
NoConn ~ 4400 8250
NoConn ~ 4400 7950
NoConn ~ 9550 2650
$Comp
L Device:C C6
U 1 1 5A945D92
P 9050 1700
F 0 "C6" H 9075 1800 50  0000 L CNN
F 1 "0.1uF" H 9075 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9088 1550 50  0001 C CNN
F 3 "" H 9050 1700 50  0001 C CNN
	1    9050 1700
	-1   0    0    1   
$EndComp
$Comp
L Device:C C7
U 1 1 5A945FCD
P 12000 1700
F 0 "C7" H 12025 1800 50  0000 L CNN
F 1 "0.1uF" H 12025 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 12038 1550 50  0001 C CNN
F 3 "" H 12000 1700 50  0001 C CNN
	1    12000 1700
	-1   0    0    1   
$EndComp
$Comp
L Device:C C8
U 1 1 5A946138
P 14800 1700
F 0 "C8" H 14825 1800 50  0000 L CNN
F 1 "0.1uF" H 14825 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 14838 1550 50  0001 C CNN
F 3 "" H 14800 1700 50  0001 C CNN
	1    14800 1700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5A9464B2
P 15150 950
F 0 "#PWR08" H 15150 700 50  0001 C CNN
F 1 "GND" H 15150 800 50  0000 C CNN
F 2 "" H 15150 950 50  0001 C CNN
F 3 "" H 15150 950 50  0001 C CNN
	1    15150 950 
	1    0    0    -1  
$EndComp
Text Label 8700 2450 0    60   ~ 0
SpnDFLAG
Text Label 8900 3300 3    60   ~ 0
SpnLFLAG
Text Label 12350 2650 2    60   ~ 0
DoX
Text Label 12350 2750 2    60   ~ 0
gnd
Text Label 15250 2750 2    60   ~ 0
gnd
Text Label 9550 2750 2    60   ~ 0
gnd
Wire Wire Line
	13450 4750 13700 4750
Wire Wire Line
	13700 4850 13450 4850
Wire Wire Line
	13700 4950 13450 4950
Wire Wire Line
	13700 5050 13450 5050
Wire Wire Line
	13700 5150 13450 5150
Wire Wire Line
	13700 5250 13450 5250
Wire Wire Line
	13700 5350 13450 5350
Wire Wire Line
	13700 5450 13450 5450
Wire Wire Line
	13700 5550 13450 5550
Wire Wire Line
	13700 5650 13450 5650
Wire Wire Line
	14200 5650 14450 5650
Wire Wire Line
	14200 5550 14450 5550
Wire Wire Line
	14200 5450 14450 5450
Wire Wire Line
	14200 5350 14450 5350
Wire Wire Line
	14200 5250 14450 5250
Wire Wire Line
	14200 5150 14450 5150
Wire Wire Line
	14200 5050 14450 5050
Wire Wire Line
	14200 4950 14450 4950
Wire Wire Line
	14200 4850 14450 4850
Wire Wire Line
	14200 4750 14450 4750
Wire Wire Line
	1600 4850 1600 5600
Wire Wire Line
	2100 4850 2100 5600
Wire Wire Line
	2650 8300 2650 9050
Wire Wire Line
	7000 6650 8350 6650
Wire Wire Line
	7000 6850 8100 6850
Wire Wire Line
	7000 7050 7850 7050
Wire Wire Line
	7850 7050 7850 7150
Wire Wire Line
	8100 6850 8100 7150
Wire Wire Line
	8350 6650 8350 7150
Wire Wire Line
	7850 7450 7850 7550
Wire Wire Line
	7850 7550 8100 7550
Wire Wire Line
	8350 7550 8350 7450
Wire Wire Line
	8100 7450 8100 7550
Connection ~ 8100 7550
Connection ~ 8350 6650
Connection ~ 8100 6850
Connection ~ 7850 7050
Wire Wire Line
	8550 7050 8550 7450
Wire Wire Line
	8650 6850 8650 7050
Wire Wire Line
	8650 7050 8800 7050
Wire Wire Line
	4400 8450 3700 8450
Wire Wire Line
	3300 8450 3200 8450
Wire Wire Line
	3200 8450 3200 8500
Wire Wire Line
	3200 9050 3200 8800
Wire Wire Line
	1800 9050 2650 9050
Connection ~ 2650 9050
Wire Wire Line
	5850 4700 5850 4900
Wire Wire Line
	1800 8300 1800 8400
Wire Wire Line
	1800 8800 1800 9050
Wire Wire Line
	1800 8000 1800 7850
Wire Wire Line
	1800 8400 2000 8400
Connection ~ 1800 8400
Wire Wire Line
	3400 2150 3400 2400
Wire Wire Line
	8700 2150 9550 2150
Wire Wire Line
	8700 2250 9550 2250
Wire Wire Line
	8700 2350 9550 2350
Wire Wire Line
	8700 1950 9050 1950
Wire Wire Line
	9050 1950 9050 2050
Wire Wire Line
	9050 2050 9550 2050
Wire Wire Line
	11650 2150 12350 2150
Wire Wire Line
	11650 2250 12350 2250
Wire Wire Line
	11650 2350 12350 2350
Wire Wire Line
	11650 1950 12000 1950
Wire Wire Line
	12000 1950 12000 2050
Wire Wire Line
	12000 2050 12350 2050
Wire Wire Line
	14450 2150 15250 2150
Wire Wire Line
	14450 2250 15250 2250
Wire Wire Line
	14450 2350 15250 2350
Wire Wire Line
	14450 1950 14800 1950
Wire Wire Line
	14800 1950 14800 2050
Wire Wire Line
	14800 2050 15250 2050
Wire Wire Line
	7200 1950 7200 1250
Wire Wire Line
	7200 1250 7500 1250
Wire Wire Line
	8400 1350 10150 1350
Wire Wire Line
	10150 1350 10150 2050
Wire Wire Line
	10150 2050 10450 2050
Wire Wire Line
	12950 1350 12950 2050
Wire Wire Line
	12950 2050 13250 2050
Connection ~ 10150 1350
Connection ~ 7200 1950
Wire Wire Line
	5950 2400 6200 2400
Wire Wire Line
	6800 2400 6800 2050
Wire Wire Line
	6800 2050 7500 2050
Connection ~ 6200 2400
Wire Wire Line
	5650 1950 5500 1950
Wire Wire Line
	5500 1950 5500 2150
Wire Wire Line
	5650 2400 5500 2400
Connection ~ 5500 2400
Connection ~ 14800 950 
Wire Wire Line
	4300 9150 4400 9150
Wire Wire Line
	4300 8750 4300 8850
Wire Wire Line
	4300 8750 4400 8750
Wire Wire Line
	4400 9050 4300 9050
Connection ~ 4300 9050
Wire Wire Line
	4400 8950 4300 8950
Connection ~ 4300 8950
Wire Wire Line
	4400 8850 4300 8850
Connection ~ 4300 8850
Text Notes 9550 7600 1    60   ~ 0
Use with Input_Pullup
Wire Wire Line
	8550 7450 8800 7450
$Comp
L Device:C C3
U 1 1 5AAAC245
P 7850 7300
F 0 "C3" H 7875 7400 50  0000 L CNN
F 1 "15n" H 7875 7200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7888 7150 50  0001 C CNN
F 3 "" H 7850 7300 50  0001 C CNN
	1    7850 7300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5AAAC896
P 9350 7600
F 0 "#PWR07" H 9350 7350 50  0001 C CNN
F 1 "GND" H 9350 7450 50  0000 C CNN
F 2 "" H 9350 7600 50  0001 C CNN
F 3 "" H 9350 7600 50  0001 C CNN
	1    9350 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 7450 9200 7450
Wire Wire Line
	9350 6650 9350 7050
Wire Wire Line
	9200 7050 9350 7050
Connection ~ 9350 7450
Wire Wire Line
	9200 6650 9350 6650
Connection ~ 9350 7050
$Comp
L Device:C C4
U 1 1 5AAACBD5
P 8100 7300
F 0 "C4" H 8125 7400 50  0000 L CNN
F 1 "15n" H 8125 7200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8138 7150 50  0001 C CNN
F 3 "" H 8100 7300 50  0001 C CNN
	1    8100 7300
	-1   0    0    1   
$EndComp
$Comp
L Device:C C5
U 1 1 5AAACC3C
P 8350 7300
F 0 "C5" H 8375 7400 50  0000 L CNN
F 1 "15n" H 8375 7200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8388 7150 50  0001 C CNN
F 3 "" H 8350 7300 50  0001 C CNN
	1    8350 7300
	-1   0    0    1   
$EndComp
Wire Wire Line
	8100 7550 8350 7550
Wire Wire Line
	8100 7550 8100 7750
Wire Wire Line
	8350 6650 8800 6650
Wire Wire Line
	8100 6850 8650 6850
Wire Wire Line
	7850 7050 8550 7050
Wire Wire Line
	2650 9050 2650 9200
Wire Wire Line
	2650 9050 3200 9050
Wire Wire Line
	1800 8400 1800 8500
Wire Wire Line
	10150 1350 12950 1350
Wire Wire Line
	7200 1950 7500 1950
Wire Wire Line
	5500 2400 5500 2600
Wire Wire Line
	14800 950  15150 950 
Wire Wire Line
	4300 9050 4300 9150
Wire Wire Line
	4300 8950 4300 9050
Wire Wire Line
	4300 8850 4300 8950
Wire Wire Line
	9350 7450 9350 7600
Wire Wire Line
	9350 7050 9350 7450
NoConn ~ 11650 5150
NoConn ~ 11650 5050
NoConn ~ 11650 4950
NoConn ~ 4400 7850
NoConn ~ 4400 7250
NoConn ~ 4400 7650
NoConn ~ 4400 8050
Text Label 14450 2050 0    60   ~ 0
gnd
Text Label 7000 7750 0    60   ~ 0
InH_A
Text Label 7000 7850 0    60   ~ 0
InL_A
Text Label 7000 8050 0    60   ~ 0
InL_B
Text Label 7000 8150 0    60   ~ 0
InH_C
Text Label 7000 8250 0    60   ~ 0
InL_C
Text Label 7000 7950 0    60   ~ 0
InH_B
NoConn ~ 4400 5850
Text Notes 7300 5900 0    50   ~ 0
update in code
Wire Wire Line
	6200 2400 6600 2400
Wire Wire Line
	6600 2300 6600 2400
Connection ~ 6600 2400
Wire Wire Line
	6600 2400 6800 2400
Connection ~ 5500 2150
Wire Wire Line
	5500 2150 5500 2400
Wire Wire Line
	6400 2150 6400 2350
Wire Wire Line
	6400 2350 5950 2350
Wire Wire Line
	5950 2350 5950 2150
Wire Wire Line
	5950 2150 5500 2150
Wire Wire Line
	6600 2000 6600 1950
Connection ~ 6600 1950
Wire Wire Line
	6600 1950 7200 1950
NoConn ~ 4400 7150
Text Label 7000 5650 0    50   ~ 0
Th1Ex
Wire Wire Line
	2650 8000 2650 7850
Wire Wire Line
	2800 8150 3250 8150
Wire Wire Line
	3250 8150 3250 7050
Wire Wire Line
	3250 7050 4400 7050
Text Notes 1800 7650 0    50   ~ 0
wired to screw terminals
NoConn ~ 11150 4750
$Comp
L Device:Crystal_GND24_Small Y1
U 1 1 5A930AF4
P 6200 2150
F 0 "Y1" V 6350 2250 50  0000 C CNN
F 1 "10MHz" V 6050 2300 50  0000 C CNN
F 2 "Crystals:Crystal_SMD_3225-4pin_3.2x2.5mm" H 6200 2150 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/Yangxing-Tech-X322516MLB4SI_C13738.pdf" H 6200 2150 50  0001 C CNN
	1    6200 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 2150 6400 2150
Connection ~ 5950 2150
Wire Wire Line
	5950 2150 6100 2150
Wire Wire Line
	6200 1950 6600 1950
Wire Wire Line
	5950 1950 6200 1950
Connection ~ 6200 1950
Wire Wire Line
	6200 1950 6200 2050
Wire Wire Line
	6200 2250 6200 2400
Text Notes 12100 5000 0    50   ~ 0
reconnect Vsen ABC
Text Label 2800 8150 0    50   ~ 0
speed
Text Label 2650 7850 2    60   ~ 0
3v3
Text Label 2450 2400 2    60   ~ 0
3v3
$Comp
L Device:R_Small R4
U 1 1 6076E8F6
P 2700 2400
F 0 "R4" V 2504 2400 50  0000 C CNN
F 1 "10k" V 2595 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 2700 2400 50  0001 C CNN
F 3 "~" H 2700 2400 50  0001 C CNN
	1    2700 2400
	0    1    1    0   
$EndComp
Connection ~ 3400 2400
Wire Wire Line
	3400 2400 3400 2500
Wire Wire Line
	2800 2400 3400 2400
Wire Wire Line
	2600 2400 2450 2400
$Comp
L Device:R_Small R5
U 1 1 6078BBB0
P 6800 2750
F 0 "R5" V 6604 2750 50  0000 C CNN
F 1 "10k" V 6695 2750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6800 2750 50  0001 C CNN
F 3 "~" H 6800 2750 50  0001 C CNN
	1    6800 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	7500 2250 7000 2250
Wire Wire Line
	7000 2250 7000 2750
Wire Wire Line
	6900 2750 7000 2750
Connection ~ 7000 2750
Wire Wire Line
	7000 2750 7000 2850
Text Label 6500 2750 2    60   ~ 0
3v3
Wire Wire Line
	6500 2750 6700 2750
$Comp
L Device:R_Small R6
U 1 1 607B46D4
P 9750 3200
F 0 "R6" V 9554 3200 50  0000 C CNN
F 1 "10k" V 9645 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 9750 3200 50  0001 C CNN
F 3 "~" H 9750 3200 50  0001 C CNN
	1    9750 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	9850 3200 9950 3200
Connection ~ 9950 3200
Wire Wire Line
	9950 3200 9950 3300
Text Label 9450 3200 2    60   ~ 0
3v3
Wire Wire Line
	9450 3200 9650 3200
Wire Wire Line
	9950 2250 10450 2250
Wire Wire Line
	9950 2250 9950 3200
$Comp
L Device:R_Small R7
U 1 1 607D16A2
P 12600 3150
F 0 "R7" V 12404 3150 50  0000 C CNN
F 1 "10k" V 12495 3150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 12600 3150 50  0001 C CNN
F 3 "~" H 12600 3150 50  0001 C CNN
	1    12600 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	12700 3150 12800 3150
Connection ~ 12800 3150
Wire Wire Line
	12800 3150 12800 3250
Text Label 12300 3150 2    60   ~ 0
3v3
Wire Wire Line
	12300 3150 12500 3150
Wire Wire Line
	12800 2250 13250 2250
Wire Wire Line
	12800 2250 12800 3150
Text Label 3700 3400 3    50   ~ 0
MISO
Text Label 3600 2150 3    50   ~ 0
MOSI
Text Label 5550 4900 1    50   ~ 0
MOSI
Text Label 5450 4900 1    50   ~ 0
MISO
Text Label 14200 5250 0    60   ~ 0
SDI
Text Label 14450 5250 0    60   ~ 0
MOSI
Text Label 14450 5350 0    60   ~ 0
MISO
Wire Wire Line
	10800 5350 11150 5350
Text Label 10800 5350 2    60   ~ 0
SCK
Wire Wire Line
	10800 4950 11150 4950
$Comp
L 74xGxx:74AHC1G125 U2
U 1 1 6093F928
P 3700 3150
F 0 "U2" V 3629 3021 50  0000 R CNN
F 1 "74AHC1G125" V 3720 3021 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:TSOT-23-5" H 3700 3150 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/Texas-Instruments-TI-SN74AHC1G125DBVR_C7468.pdf" H 3700 3150 50  0001 C CNN
	1    3700 3150
	0    1    1    0   
$EndComp
Text Label 3900 3150 0    60   ~ 0
3v3
Text Label 13250 2450 2    60   ~ 0
MISO
Text Label 13250 2550 2    60   ~ 0
MOSI
Wire Wire Line
	3700 2150 3700 2850
$Comp
L arduino:Arduino_Due_Shield XA1
U 1 1 5A8DDD59
P 5700 7500
F 0 "XA1" V 5800 7500 60  0000 C CNN
F 1 "Arduino_Due_Shield" V 5600 7500 60  0000 C CNN
F 2 "customFootPrints:Arduino_Due_Shield_drv8305" H 6400 10250 60  0001 C CNN
F 3 "https://store.arduino.cc/arduino-due" H 6400 10250 60  0001 C CNN
	1    5700 7500
	1    0    0    -1  
$EndComp
Text Label 15250 2650 2    60   ~ 0
DoZ
Text Label 15250 2550 2    60   ~ 0
ClkZ
Text Label 7000 5950 0    60   ~ 0
ClkX
Text Label 7000 6050 0    60   ~ 0
DoX
Text Label 7000 5750 0    60   ~ 0
ClkZ
Text Label 7000 5850 0    60   ~ 0
DoZ
Wire Notes Line
	3800 7700 3800 9550
Wire Notes Line
	3800 9550 1350 9550
Wire Notes Line
	1350 9550 1350 7700
Wire Notes Line
	1350 7700 3800 7700
Text Notes 11750 4350 0    50   ~ 0
BOOSTXL-DRV8305EVM
NoConn ~ 7000 8950
NoConn ~ 7000 9150
NoConn ~ 7000 9450
NoConn ~ 9550 2450
NoConn ~ 7000 6550
NoConn ~ 7000 9050
NoConn ~ 7000 7650
Text Notes 7450 7500 1    50   ~ 0
Update!!
$Comp
L Device:R_Small R8
U 1 1 60AB6ED9
P 8650 3100
F 0 "R8" V 8454 3100 50  0000 C CNN
F 1 "10k" V 8545 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 8650 3100 50  0001 C CNN
F 3 "~" H 8650 3100 50  0001 C CNN
	1    8650 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	8700 2550 8900 2550
Wire Wire Line
	8900 2550 8900 3100
Wire Wire Line
	8750 3100 8900 3100
Connection ~ 8900 3100
Wire Wire Line
	8550 3100 8450 3100
Text Label 8450 3100 2    60   ~ 0
3v3
Wire Wire Line
	8900 3100 8900 3300
Wire Wire Line
	14800 1550 14800 950 
Wire Wire Line
	14800 1950 14800 1850
Connection ~ 14800 1950
Wire Wire Line
	12000 1950 12000 1850
Connection ~ 12000 1950
Wire Wire Line
	9050 950  12000 950 
Wire Wire Line
	12000 1550 12000 950 
Connection ~ 12000 950 
Wire Wire Line
	12000 950  14800 950 
Wire Wire Line
	9050 1950 9050 1850
Connection ~ 9050 1950
Wire Wire Line
	9050 1550 9050 950 
$EndSCHEMATC
