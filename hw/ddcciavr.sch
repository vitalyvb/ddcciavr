v 20110115 2
C 40000 40000 0 0 0 title-B.sym
C 49200 42700 1 0 0 mega48-tqfp32.sym
{
T 53700 49200 5 10 1 1 0 6 1
refdes=U?
T 49500 49500 5 10 0 0 0 0 1
device=ATMega48-TQFP32
T 49500 49700 5 10 0 0 0 0 1
footprint=TQFP32_7
}
C 47100 43400 1 0 0 resistor-1.sym
{
T 47400 43800 5 10 0 0 0 0 1
device=RESISTOR
T 47300 43700 5 10 1 1 0 0 1
refdes=R2
T 47600 43700 5 10 1 1 0 0 1
value=4.7K
}
C 46700 43400 1 270 0 capacitor-2.sym
{
T 47400 43200 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 47200 43200 5 10 1 1 270 0 1
refdes=C3
T 47600 43200 5 10 0 0 270 0 1
symversion=0.1
T 46400 43100 5 10 1 1 0 0 1
value=22uF
}
C 44500 49500 1 270 0 capacitor-2.sym
{
T 45200 49300 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 45000 49300 5 10 1 1 270 0 1
refdes=C2
T 45400 49300 5 10 0 0 270 0 1
symversion=0.1
T 44200 48700 5 10 1 1 0 0 1
value=10uF
}
C 43800 49500 1 270 0 capacitor-1.sym
{
T 44500 49300 5 10 0 0 270 0 1
device=CAPACITOR
T 44300 49300 5 10 1 1 270 0 1
refdes=C1
T 44700 49300 5 10 0 0 270 0 1
symversion=0.1
T 43500 48700 5 10 1 1 0 0 1
value=0.1uF
}
C 49000 48400 1 0 1 led-1.sym
{
T 48200 49000 5 10 0 0 0 6 1
device=LED
T 48800 49000 5 10 1 1 0 6 1
refdes=LED1
T 48200 49200 5 10 0 0 0 6 1
symversion=0.1
}
C 41000 45200 1 0 0 switch-spst-1.sym
{
T 41400 45900 5 10 0 0 0 0 1
device=SPST
T 41300 45500 5 10 1 1 0 0 1
refdes=S1
}
C 41000 44600 1 0 0 switch-spst-1.sym
{
T 41400 45300 5 10 0 0 0 0 1
device=SPST
T 41300 44900 5 10 1 1 0 0 1
refdes=S2
}
C 45200 49600 1 0 0 3.3V-plus-1.sym
C 46600 48100 1 0 0 gnd-1.sym
C 45300 48100 1 0 0 gnd-1.sym
C 51700 42100 1 0 0 gnd-1.sym
C 46800 42100 1 0 0 gnd-1.sym
N 51400 49500 51400 49400 4
N 51800 49400 51800 49600 4
C 46900 48500 1 0 0 resistor-1.sym
{
T 47200 48900 5 10 0 0 0 0 1
device=RESISTOR
T 47100 48800 5 10 1 1 0 0 1
refdes=R1
T 47100 48300 5 10 1 1 0 0 1
value=4.7K
}
N 51400 42700 51800 42700 4
N 46200 43500 47100 43500 4
N 51800 42400 51800 42700 4
N 49000 48600 49200 48600 4
N 48100 48600 47800 48600 4
N 46900 48600 46700 48600 4
N 46700 48600 46700 48400 4
N 46900 42400 46900 42500 4
C 51600 49600 1 0 0 3.3V-plus-1.sym
N 51400 49500 51800 49500 4
C 54200 48100 1 0 0 3.3V-plus-1.sym
C 54300 47300 1 0 0 gnd-1.sym
N 54000 47700 54400 47700 4
N 54400 47700 54400 47600 4
N 54000 48000 54400 48000 4
N 54400 48000 54400 48100 4
N 44000 49500 45400 49500 4
N 44000 48600 45400 48600 4
N 46900 43400 46900 43500 4
N 46200 42700 46200 43500 4
N 46200 42700 45400 42700 4
C 42000 43900 1 0 0 connector3-2.sym
{
T 42700 45600 5 10 1 1 0 6 1
refdes=CONN1B
T 42300 45550 5 10 0 0 0 0 1
device=CONNECTOR_3
T 42300 45750 5 10 0 0 0 0 1
footprint=SIP3N
}
C 43700 44200 1 0 0 connector3-1.sym
{
T 45500 45100 5 10 0 0 0 0 1
device=CONNECTOR_3
T 43700 45300 5 10 1 1 0 0 1
refdes=CONN1A
T 44300 44500 5 10 1 1 0 0 1
description=GND
T 44300 45100 5 10 1 1 0 0 1
description=BR_UP
T 44300 44800 5 10 1 1 0 0 1
description=BR_DN
}
N 49200 45000 45400 45000 4
N 49200 44400 48400 44400 4
N 48400 44400 48400 44700 4
N 48400 44700 45400 44700 4
C 45300 44100 1 0 0 gnd-1.sym
C 45300 42100 1 0 0 gnd-1.sym
N 41000 44300 41000 45200 4
N 41000 44300 42000 44300 4
N 42000 44700 41800 44700 4
N 41800 44700 41800 44600 4
N 42000 45100 41800 45100 4
N 41800 45100 41800 45200 4
N 48000 43500 48400 43500 4
N 48400 43500 48400 43800 4
N 49200 43800 48400 43800 4
C 54900 42900 1 0 0 gnd-1.sym
C 54800 44400 1 0 0 3.3V-plus-1.sym
N 54000 43800 55000 43800 4
N 54000 44100 55000 44100 4
C 56700 43000 1 0 1 connector5-1.sym
{
T 54900 44500 5 10 0 0 0 6 1
device=CONNECTOR_5
T 56600 44700 5 10 1 1 0 6 1
refdes=CONN3
T 55400 44500 5 10 1 1 0 0 1
description=+3.3V
T 55400 43300 5 10 1 1 0 0 1
description=GND
T 55400 43900 5 10 1 1 0 0 1
description=SCL
T 55400 44200 5 10 1 1 0 0 1
description=SDA
T 55400 43600 5 10 1 1 0 0 1
description=EN_IN
}
C 43700 42200 1 0 0 connector3-1.sym
{
T 45500 43100 5 10 0 0 0 0 1
device=CONNECTOR_3
T 43700 43300 5 10 1 1 0 0 1
refdes=CONN2
T 44300 43100 5 10 1 1 0 0 1
description=EN_OUT
T 44300 42800 5 10 1 1 0 0 1
description=DIM
T 44300 42500 5 10 1 1 0 0 1
description=GND
}
N 55000 43500 54500 43500 4
N 54500 43500 54500 41700 4
N 54500 41700 45700 41700 4
N 45700 41700 45700 43000 4
N 45700 43000 45400 43000 4
N 45400 48600 45400 48400 4
N 45400 49500 45400 49600 4
