1 CRCP$="RECIPE01"       ' RECIPE NAME
2 MTimeout#=50.0#        ' Timeout for COM connnect
3 ' --------------------------------------------------
4 ' MAIN PROGRAM
5 ' --------------------------------------------------
6 *PROC_START:
7 Base P_NBase           ' Reset base coordinate.
8 M_Timer(1)=0.0#        ' Reset COM timer.
9 Servo On
10 *SRVON:If M_Svo<>1 GoTo *SRVON
11 Mov PORG              ' Move to start position.
12 Dly 1.0#
13 *LOOP
14 Base P_NBase          ' Reset base coordinate.
15 Mov PORG              ' Move to start position.
16 GoSub *X3             ' Send PC X3
17 If COK$<>"OK" Then GoTo *LOOP
18 GoSub *X0             ' Send PC X0
19 If COK$<>"OK" Then GoTo *LOOP
20 Mov PCAPT             ' Move to capture position.
21 Dly 0.2#
22 nretry%=0
23 *RECAPT
24 If nretry%>2 GoTo *LOOP
25 GoSub *X1             ' Send PC X1
26 If COK$<>"OK" Then
27  nretry%=nretry%+1
28  GoTo *RECAPT
29 EndIf
30 GoSub *X2             ' Send PC X2
31 If COK$<>"OK" Then GoTo *LOOP
32 Base PX2              ' Transform base coordinate.
33 Mov PPCK,-50
34 Mvs PPCK              ' Approach to the work.
35 Dly 2.0#
36 Mvs PPCK,-50
37 GoTo *LOOP
38 *PROC_END:
39 Mov PORG              ' Move to start position.
40 Dly 0.5#
41 Servo Off
42 End
43 ' --------------------------------------------------
44 ' [COM] CONNECT SERVER
45 ' --------------------------------------------------
46 *COM_OPEN:
47 M_Timer(1)=0.0#     ' Reset COM timer.
48 Open "COM5:" As #1
49 *WAIT_OPEN:If (M_Open(1)<>1 And M_Timer(1)<MTimeout#) Then GoTo *WAIT_OPEN
50 If M_Open(1)<>1 Then
51  GoTo *COM_OPEN
52 EndIf
53 Return
54 ' --------------------------------------------------
55 ' [COM] DISCONNECT SERVER
56 ' --------------------------------------------------
57 *COM_CLOSE:
58 Close #1
59 Return
60 ' --------------------------------------------------
61 ' [X0] RESET
62 ' --------------------------------------------------
63 *X0
64 PX2=SetPos(0,0,0,0,0,0,0)
65 GoSub *COM_OPEN
66 Print #1, "X0()"
67 Input #1, COK$    ' Reset OK or not OK.
68 GoSub *COM_CLOSE
69 Return
70 ' --------------------------------------------------
71 ' [X1] CAPTURE
72 ' --------------------------------------------------
73 *X1
74 GoSub *COM_OPEN
75 Print #1, "X1";P_Curr
76 Input #1, COK$    ' Capture OK or not OK.
77 GoSub *COM_CLOSE
78 Return
79 ' --------------------------------------------------
80 ' [X2] SOLVE
81 ' --------------------------------------------------
82 *X2
83 GoSub *COM_OPEN
84 Print #1, "X2()"
85 Input #1, COK$    ' Solve OK or not OK.
86 If COK$="OK" Then Input #1, PX2
87 GoSub *COM_CLOSE
88 Return
89 ' --------------------------------------------------
90 ' [X3] RECIPE
91 ' --------------------------------------------------
92 *X3
93 GoSub *COM_OPEN
94 Print #1, "X3("+CRCP$+")"
95 Input #1, COK$    ' Recipe OK or not OK.
96 GoSub *COM_CLOSE
97 Return
PCAPT=(355.000,-300.000,640.000,-180.000,0.000,-180.000)(7,0)
PORG=(355.000,0.000,640.000,180.000,0.000,180.000)(7,0)
PPCK=(444.960,4.920,309.000,180.000,0.000,180.000,0.000,0.000)(7,0)
