clear;
clc;

%%% Number 1 Matrix Multiplication
syms c01 s01 c02 s02 c03 s03 c04 s04 c05 s05 c06 s06 d1 a2 d2 d4 d5 d6

BaseOffset = 5 + 17; %mm
d1 = 106.75; %mm - Gear to Cycloidal
a2 = 200; %mm - Link 1
d2 = 25;
d4 = 106.5 + 36.96 + 17.1; %mm - Link 2 to Differential Center
d5 = 29.59; %mm - Differential Center to Output Face
d6 = 0; %mm - Pinion Face to EE Point

    
A1 = [c01,0,-s01,0;
    s01,0,c01,0;
    0,-1,0,0;
    0,0,0,1]

A2 = [s02,c02,0,a2*s02;
    -c02,s02,0,-a2*c02;
    0,0,1,d2;
    0,0,0,1]

A3 = [c03,0,-s03,0;
    s03,0,c03,0;
    0,-1,0,0;
    0,0,0,1]

A4 = [c04,0,s04,0;
    s04,0,-c04,0;
    0,1,0,d4;
    0,0,0,1]

A5 = [c05, 0,-s05,0,;
    s05,0,c05,0;
    0,-1,0,d5;
    0,0,0,1]

A6 = [c06,-s06,0,0;
    s06,c06,0,0;
    0,0,1,d6;
    0,0,0,1]


%A12 = A1*A2
%A34 = A3*A4
%A56 = A5*A6
%A14 = A1*A2*A3*A4

T = A1*A2*A3*A4*A5*A6
