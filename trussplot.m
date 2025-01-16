%% Support reaction analysis for whole structure for loading condition 50g
% List of positions of joints
rA = [0 0 0];
rB = [4 3 0];
rC = [8 3 0];
rD = [12 3 0];
rE = [16 0 0];
rF = [12 0 0];
rG = [8 0 0];
rH = [4 0 0];

% Load definitions 
Dy1 = -50e-3*9.81; 

% List of forces acting on joints 
syms Ax Ay Ey

fA = [Ax Ay 0];
fB = [0 0 0];
fC = [0 0 0];
fD1 = [0 Dy1 0];
fE = [0 Ey 0];
fF = [0 0 0];
fG = [0 0 0];
fH = [0 0 0];

ffs = [ fA; fB; fC; fD1; fE; fF; fD1*0; fC*0; fG; fD1*0; fF*0; fG*0; fB*0; fH; fG*0; fH*0; fA*0 ];
    
rrs = [ rA; rB; rC; rD; rE; rF; rD; rC; rG; rD; rF; rG; rB; rH; rG; rH; rA ];
figure(1);
hold on; 
[mB, fB, VarSoln, VarList, ffnew_ii] = GenRigidBodyEquilibrium(ffs, rrs);

%% Internal load equillibrium analysis for loading condition 50g - cut at E to find ED and EF

figure(2)
hold on;

% Variables with joint E removed

Ax = 0;
Ay = .122;
Dy1 = -50e-3*9.81; 


rA = [0 0 0];
rB = [4 3 0];
rC = [8 3 0];
rD = [12 3 0];
rF = [12 0 0];
rE = [16 0 0];
rG = [8 0 0];
rH = [4 0 0];

fA = [Ax Ay 0];
fB = [0 0 0];
fC = [0 0 0];
fD1 = [0 Dy1 0];
syms fEF fDE

qD = (360-36.87)*pi/180; % Angle calculations (radians) differ from video because using 3,4,5, triangle
fDEx = fDE*cos(qD); % Defining components of force DE
fDEy = fDE*sin(qD); 
fD11 = [fDEx fDEy 0]; 

fF = [fEF 0 0]; % Defining the force at F by the internal force fEF

fG = [0 0 0];
fH = [0 0 0];


ffs = [ fA; fB; fC; fD1; fF; fD1*0; fC*0; fG; fD1*0; fF*0; fG*0; fB*0; fH; fG*0; fH*0; fA*0; fD11 ]; 
rrs = [ rA; rB; rC; rD; rF; rD; rC; rG; rD; rF; rG; rB; rH; rG; rH; rA; rD ];
[mB2, fB2, VarSoln2, VarList2, ffnew_ii2] = GenRigidBodyEquilibrium(ffs, rrs);

EF = ffnew_ii2(5,:);
DE = ffnew_ii2(17,:);
flist = [EF; DE];
rlist1 = [ rF; rD ];
rlist2 = [ rE; rE ];
VisualizeTensionCompression(flist, rlist1, rlist2);

%% Internal load equillibrium analysis for loading condition 50g - cut at A to find AB and AH

figure(2);
hold on; 

% Load definitions 
Dy1 = -50e-3*9.81; 
Ax = 0;
Ay = .122;
Ey = .365;

% variables with joint A removed
rA = [0 0 0];
rB = [4 3 0];
rC = [8 3 0];
rD = [12 3 0];
rE = [16 0 0];
rF = [12 0 0];
rG = [8 0 0];
rH = [4 0 0];

syms fAB fAH

qB = (360-36.87)*pi/180; % Angle in degrees calculated from knowlege of 3, 4, 5 triangle 
fABx = -fAB*cos(qB);
fABy = fAB*sin(qB);
fB = [fABx fABy 0];
fC = [0 0 0];
fD1 = [0 Dy1 0];
fE = [0 Ey 0];
fF = [0 0 0];
fG = [0 0 0];
fH = [fAH 0 0];

ffs = [ fB; fC; fD1; fE; fF; fD1*0; fC*0; fG; fD1*0; fF*0; fG*0; fB*0; fH; fG*0; fH*0 ];
    
rrs = [ rB; rC; rD; rE; rF; rD; rC; rG; rD; rF; rG; rB; rH; rG; rH; ];

[mB3, fB3, VarSoln3, VarList3, ffnew_ii3] = GenRigidBodyEquilibrium(ffs, rrs);

AB = ffnew_ii3(1,:);
AH = ffnew_ii3(13,:);
flist = [AB; AH];
rlist1 = [ rB; rH ];
rlist2 = [ rA; rA ];
VisualizeTensionCompression(flist, rlist1, rlist2);

%% Internal load equillibrium analysis for loading condition 50g - cut at CD/DG/CF 

figure(3);
hold on; 

% Load definitions 
Dy1 = -50e-3*9.81; 
Ax = 0;
Ay = .122;
Ey = .365;

% variables with joint A removed
rC = [8 3 0];
rD = [12 3 0];
rE = [16 0 0];
rF = [12 0 0];
rG = [8 0 0];

syms CD GD FG

fC = [0 0 0];

fD1 = [0 Dy1 0];
fD11 = [CD 0 0];
qG = (360-36.87)*pi/180;
GDx = -GD*cos(qG);
GDy = GD*sin(qG);
fD12 = [ GDx GDy 0];
fE = [0 Ey 0];
fF = [FG 0 0];
fG = [0 0 0];

ffs = [ fD1; fD11; fD12; fE; fF; fD1*0; fD11*0; fD12*0; fF*0 ];
    
rrs = [ rD; rD; rD; rE; rF; rD; rD; rD; rF ];
[mB, fB1, VarSoln, VarList, ffnew_ii] = GenRigidBodyEquilibrium(ffs, rrs);

CD = ffnew_ii(2,:);
GD = ffnew_ii(3,:);
FG = ffnew_ii(5,:);

flist = [CD; GD; FG];
rlist1 = [ rD; rD; rF];
rlist2 = [ rC; rG; rG ];
VisualizeTensionCompression(flist, rlist1, rlist2);

%% Internal load equillibrium analysis for loading condition 50g - cut at BC/BG/HG 

figure(4);
hold on; 

% Load definitions 
Ax = 0;
Ay = .122;

rA = [0 0 0];
rB = [4 3 0];
rC = [8 3 0];
rG = [8 0 0];
rH = [4 0 0];


syms BC BG HG

fA = [Ax Ay 0];
fB = [BC 0 0];
qB = (360-36.87)*pi/180;
BGx = BG*cos(qB);
BGy = BG*sin(qB);
fB1 = [BGx BGy 0];
fH = [HG 0 0];


ffs = [ fA; fB; fB1; fH; fA*0 ];
    
rrs = [ rA; rB; rB; rH; rA ];

[mB4, fB4, VarSoln4, VarList4, ffnew_ii4] = GenRigidBodyEquilibrium(ffs, rrs);


BC = ffnew_ii4(2,:);
BG = ffnew_ii4(3,:);
HG = ffnew_ii4(4,:);

flist = [BC; BG; HG];
rlist1 = [ rB; rB; rH];
rlist2 = [ rC; rG; rG ];
VisualizeTensionCompression(flist, rlist1, rlist2);


%% Support reaction analysis for whole structure for loading condition 200g

% List of positions of joints
rA = [0 0 0];
rB = [4 3 0];
rC = [8 3 0];
rD = [12 3 0];
rE = [16 0 0];
rF = [12 0 0];
rG = [8 0 0];
rH = [4 0 0];

% Load definitions 
Dy2 = -200e-3*9.81;

% List of forces acting on joints 
syms Ax Ay Ex 

fA = [Ax Ay 0];
fB = [0 0 0];
fC = [0 0 0];
fD2 = [0 Dy2 0];
fE = [0 Ex 0];
fF = [0 0 0];
fG = [0 0 0];
fH = [0 0 0];

ffs = [ fA; fB; fC; fD2; fE; fF; fD2*0; fC*0; fG; fD2*0; fF*0; fG*0; fB*0; fH; fG*0; fH*0; fA*0 ];
    
rrs = [ rA; rB; rC; rD; rE; rF; rD; rC; rG; rD; rF; rG; rB; rH; rG; rH; rA ];
figure(2);
hold on; 
[mB2, fB2, VarSoln2, VarList2, ffnew_ii2] = GenRigidBodyEquilibrium(ffs, rrs);

%% Internal load equillibrium analysis for loading condition 200g - cut at E to find ED and DE
figure(2)
hold on;

% Variables with joint E removed

Ax = 0;
Ay = .49;
Dy2 = -200e-3*9.81; 


rA = [0 0 0];
rB = [4 3 0];
rC = [8 3 0];
rD = [12 3 0];
rF = [12 0 0];
rE = [16 0 0];
rG = [8 0 0];
rH = [4 0 0];

fA = [Ax Ay 0];
fB = [0 0 0];
fC = [0 0 0];
fD1 = [0 Dy2 0];
syms fEF fDE

qD = (360-36.87)*pi/180; % Angle calculations (radians) differ from video because using 3,4,5, triangle
fDEx = fDE*cos(qD); % Defining components of force DE
fDEy = fDE*sin(qD); 
fD11 = [fDEx fDEy 0]; 

fF = [fEF 0 0]; % Defining the force at F by the internal force fEF

fG = [0 0 0];
fH = [0 0 0];


ffs = [ fA; fB; fC; fD1; fF; fD1*0; fC*0; fG; fD1*0; fF*0; fG*0; fB*0; fH; fG*0; fH*0; fA*0; fD11 ]; 
rrs = [ rA; rB; rC; rD; rF; rD; rC; rG; rD; rF; rG; rB; rH; rG; rH; rA; rD ];
[mB2, fB2, VarSoln2, VarList2, ffnew_ii2] = GenRigidBodyEquilibrium(ffs, rrs);

EF = ffnew_ii2(5,:);
DE = ffnew_ii2(17,:);
flist = [EF; DE];
rlist1 = [ rF; rD ];
rlist2 = [ rE; rE ];
VisualizeTensionCompression(flist, rlist1, rlist2);

%% Internal load equillibrium analysis for loading condition 200g - cut at A to find AB and AH

figure(2);
hold on; 

% Load definitions 
Dy2 = -200e-3*9.81; 
Ax = 0;
Ay = .49;
Ey = 1.472;

% variables with joint A removed
rA = [0 0 0];
rB = [4 3 0];
rC = [8 3 0];
rD = [12 3 0];
rE = [16 0 0];
rF = [12 0 0];
rG = [8 0 0];
rH = [4 0 0];

syms fAB fAH

qB = (360-36.87)*pi/180; % Angle in degrees calculated from knowlege of 3, 4, 5 triangle 
fABx = -fAB*cos(qB);
fABy = fAB*sin(qB);
fB = [fABx fABy 0];
fC = [0 0 0];
fD1 = [0 Dy2 0];
fE = [0 Ey 0];
fF = [0 0 0];
fG = [0 0 0];
fH = [fAH 0 0];

ffs = [ fB; fC; fD1; fE; fF; fD1*0; fC*0; fG; fD1*0; fF*0; fG*0; fB*0; fH; fG*0; fH*0 ];
    
rrs = [ rB; rC; rD; rE; rF; rD; rC; rG; rD; rF; rG; rB; rH; rG; rH; ];

[mB3, fB3, VarSoln3, VarList3, ffnew_ii3] = GenRigidBodyEquilibrium(ffs, rrs);

AB = ffnew_ii3(1,:);
AH = ffnew_ii3(13,:);
flist = [AB; AH];
rlist1 = [ rB; rH ];
rlist2 = [ rA; rA ];
VisualizeTensionCompression(flist, rlist1, rlist2);

%% Internal load equillibrium analysis for loading condition 200g - cut at CD/DG/CF 

figure(3);
hold on; 

% Load definitions 
Dy2 = -200e-3*9.81; 
Ax = 0;
Ay = .49;
Ey = 1.472;

% variables with joint A removed
rC = [8 3 0];
rD = [12 3 0];
rE = [16 0 0];
rF = [12 0 0];
rG = [8 0 0];

syms CD GD FG

fC = [0 0 0];

fD1 = [0 Dy2 0];
fD11 = [CD 0 0];
qG = (360-36.87)*pi/180;
GDx = -GD*cos(qG);
GDy = GD*sin(qG);
fD12 = [ GDx GDy 0];
fE = [0 Ey 0];
fF = [FG 0 0];
fG = [0 0 0];

ffs = [ fD1; fD11; fD12; fE; fF; fD1*0; fD11*0; fD12*0; fF*0 ];
    
rrs = [ rD; rD; rD; rE; rF; rD; rD; rD; rF ];
[mB, fB1, VarSoln, VarList, ffnew_ii] = GenRigidBodyEquilibrium(ffs, rrs);

CD = ffnew_ii(2,:);
GD = ffnew_ii(3,:);
FG = ffnew_ii(5,:);

flist = [CD; GD; FG];
rlist1 = [ rD; rD; rF];
rlist2 = [ rC; rG; rG ];
VisualizeTensionCompression(flist, rlist1, rlist2);

%% Internal load equillibrium analysis for loading condition 200g - cut at BC/BG/HG 

figure(4);
hold on; 

% Load definitions 
Ax = 0;
Ay = .49;
Ey = 1.472;

rA = [0 0 0];
rB = [4 3 0];
rC = [8 3 0];
rG = [8 0 0];
rH = [4 0 0];


syms BC BG HG

fA = [Ax Ay 0];
fB = [BC 0 0];
qB = (360-36.87)*pi/180;
BGx = BG*cos(qB);
BGy = BG*sin(qB);
fB1 = [BGx BGy 0];
fH = [HG 0 0];


ffs = [ fA; fB; fB1; fH; fA*0 ];
    
rrs = [ rA; rB; rB; rH; rA ];

[mB4, fB4, VarSoln4, VarList4, ffnew_ii4] = GenRigidBodyEquilibrium(ffs, rrs);


BC = ffnew_ii4(2,:);
BG = ffnew_ii4(3,:);
HG = ffnew_ii4(4,:);

flist = [BC; BG; HG];
rlist1 = [ rB; rB; rH];
rlist2 = [ rC; rG; rG ];
VisualizeTensionCompression(flist, rlist1, rlist2);

