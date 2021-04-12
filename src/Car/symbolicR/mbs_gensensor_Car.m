%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Thu Apr  1 15:04:07 2021
%
%	==> Project name : Car
%	==> using XML input file 
%
%	==> Number of joints : 50
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 10919
%
%	==> Generation Time :  0.170 seconds
%	==> Post-Processing :  0.230 seconds
%
%-------------------------------------------------------------
%
function [sens] = gensensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,50);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));
  C5 = cos(q(5));
  S5 = sin(q(5));
  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C8 = cos(q(8));
  S8 = sin(q(8));
  C9 = cos(q(9));
  S9 = sin(q(9));
  C10 = cos(q(10));
  S10 = sin(q(10));
  C11 = cos(q(11));
  S11 = sin(q(11));
  C12 = cos(q(12));
  S12 = sin(q(12));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C13 = cos(q(13));
  S13 = sin(q(13));
  C14 = cos(q(14));
  S14 = sin(q(14));
  C15 = cos(q(15));
  S15 = sin(q(15));
  C16 = cos(q(16));
  S16 = sin(q(16));
  C17 = cos(q(17));
  S17 = sin(q(17));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C18 = cos(q(18));
  S18 = sin(q(18));

% = = Block_0_0_0_0_0_6 = = 
 
% Trigonometric Variables  

  C19 = cos(q(19));
  S19 = sin(q(19));

% = = Block_0_0_0_0_0_7 = = 
 
% Trigonometric Variables  

  C20 = cos(q(20));
  S20 = sin(q(20));

% = = Block_0_0_0_0_0_8 = = 
 
% Trigonometric Variables  

  C21 = cos(q(21));
  S21 = sin(q(21));

% = = Block_0_0_0_0_0_9 = = 
 
% Trigonometric Variables  

  C22 = cos(q(22));
  S22 = sin(q(22));

% = = Block_0_0_0_0_0_10 = = 
 
% Trigonometric Variables  

  C23 = cos(q(23));
  S23 = sin(q(23));
  C24 = cos(q(24));
  S24 = sin(q(24));

% = = Block_0_0_0_0_0_11 = = 
 
% Trigonometric Variables  

  C25 = cos(q(25));
  S25 = sin(q(25));
  C26 = cos(q(26));
  S26 = sin(q(26));
  C27 = cos(q(27));
  S27 = sin(q(27));

% = = Block_0_0_0_0_0_13 = = 
 
% Trigonometric Variables  

  C29 = cos(q(29));
  S29 = sin(q(29));
  C30 = cos(q(30));
  S30 = sin(q(30));

% = = Block_0_0_0_0_0_14 = = 
 
% Trigonometric Variables  

  C31 = cos(q(31));
  S31 = sin(q(31));
  C32 = cos(q(32));
  S32 = sin(q(32));

% = = Block_0_0_0_0_0_15 = = 
 
% Augmented Joint Position Vectors   

  Dz332 = q(33)+s.dpt(2,46);
 
% Trigonometric Variables  

  C34 = cos(q(34));
  S34 = sin(q(34));

% = = Block_0_0_0_0_0_16 = = 
 
% Trigonometric Variables  

  C35 = cos(q(35));
  S35 = sin(q(35));
  C36 = cos(q(36));
  S36 = sin(q(36));
  C37 = cos(q(37));
  S37 = sin(q(37));

% = = Block_0_0_0_0_0_17 = = 
 
% Trigonometric Variables  

  C38 = cos(q(38));
  S38 = sin(q(38));
  C39 = cos(q(39));
  S39 = sin(q(39));
  C40 = cos(q(40));
  S40 = sin(q(40));

% = = Block_0_0_0_0_0_18 = = 
 
% Trigonometric Variables  

  C41 = cos(q(41));
  S41 = sin(q(41));

% = = Block_0_0_0_0_0_19 = = 
 
% Trigonometric Variables  

  C42 = cos(q(42));
  S42 = sin(q(42));
  C43 = cos(q(43));
  S43 = sin(q(43));
  C44 = cos(q(44));
  S44 = sin(q(44));

% = = Block_0_0_0_0_0_20 = = 
 
% Trigonometric Variables  

  C45 = cos(q(45));
  S45 = sin(q(45));
  C46 = cos(q(46));
  S46 = sin(q(46));

% = = Block_0_0_0_0_0_21 = = 
 
% Trigonometric Variables  

  C47 = cos(q(47));
  S47 = sin(q(47));
  C48 = cos(q(48));
  S48 = sin(q(48));
  C49 = cos(q(49));
  S49 = sin(q(49));
  C50 = cos(q(50));
  S50 = sin(q(50));

% = = Block_0_0_0_22_0_9 = = 
 
% Trigonometric Variables  

  S22p6 = C22*S6+S22*C6;
  C22p6 = C22*C6-S22*S6;

% = = Block_0_0_0_25_0_11 = = 
 
% Trigonometric Variables  

  S25p22p6 = C25*S22p6+S25*C22p6;
  C25p22p6 = C25*C22p6-S25*S22p6;

% = = Block_0_0_0_41_0_18 = = 
 
% Trigonometric Variables  

  S41p6 = C41*S6+S41*C6;
  C41p6 = C41*C6-S41*S6;

% = = Block_0_0_0_42_0_19 = = 
 
% Trigonometric Variables  

  S42p41p6 = C42*S41p6+S42*C41p6;
  C42p41p6 = C42*C41p6-S42*S41p6;

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.A(1) = qdd(1);
 
% 
case 2, 


% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
 
% 
case 3, 


% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
 
% 
case 4, 


% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = C4;
    sens.R(1,2) = S4;
    sens.R(2,1) = -S4;
    sens.R(2,2) = C4;
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(3) = qd(4);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(3) = qdd(4);
 
% 
case 5, 


% = = Block_1_0_0_5_0_1 = = 
 
% Sensor Kinematics 


    ROcp4_45 = -S4*C5;
    ROcp4_55 = C4*C5;
    ROcp4_75 = S4*S5;
    ROcp4_85 = -C4*S5;
    OMcp4_15 = qd(5)*C4;
    OMcp4_25 = qd(5)*S4;
    OPcp4_15 = qdd(5)*C4-qd(4)*qd(5)*S4;
    OPcp4_25 = qdd(5)*S4+qd(4)*qd(5)*C4;

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = C4;
    sens.R(1,2) = S4;
    sens.R(2,1) = ROcp4_45;
    sens.R(2,2) = ROcp4_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp4_75;
    sens.R(3,2) = ROcp4_85;
    sens.R(3,3) = C5;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp4_15;
    sens.OM(2) = OMcp4_25;
    sens.OM(3) = qd(4);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp4_15;
    sens.OMP(2) = OPcp4_25;
    sens.OMP(3) = qdd(4);
 
% 
case 6, 


% = = Block_1_0_0_6_0_1 = = 
 
% Sensor Kinematics 


    ROcp5_45 = -S4*C5;
    ROcp5_55 = C4*C5;
    ROcp5_75 = S4*S5;
    ROcp5_85 = -C4*S5;
    ROcp5_16 = -(ROcp5_75*S6-C4*C6);
    ROcp5_26 = -(ROcp5_85*S6-S4*C6);
    ROcp5_36 = -C5*S6;
    ROcp5_76 = ROcp5_75*C6+C4*S6;
    ROcp5_86 = ROcp5_85*C6+S4*S6;
    ROcp5_96 = C5*C6;
    OMcp5_15 = qd(5)*C4;
    OMcp5_25 = qd(5)*S4;
    OMcp5_16 = OMcp5_15+ROcp5_45*qd(6);
    OMcp5_26 = OMcp5_25+ROcp5_55*qd(6);
    OMcp5_36 = qd(4)+qd(6)*S5;
    OPcp5_16 = ROcp5_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp5_25*S5-ROcp5_55*qd(4));
    OPcp5_26 = ROcp5_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp5_15*S5-ROcp5_45*qd(4));
    OPcp5_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = ROcp5_16;
    sens.R(1,2) = ROcp5_26;
    sens.R(1,3) = ROcp5_36;
    sens.R(2,1) = ROcp5_45;
    sens.R(2,2) = ROcp5_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp5_76;
    sens.R(3,2) = ROcp5_86;
    sens.R(3,3) = ROcp5_96;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp5_16;
    sens.OM(2) = OMcp5_26;
    sens.OM(3) = OMcp5_36;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp5_16;
    sens.OMP(2) = OPcp5_26;
    sens.OMP(3) = OPcp5_36;
 
% 
case 7, 


% = = Block_1_0_0_7_0_1 = = 
 
% Sensor Kinematics 


    ROcp6_45 = -S4*C5;
    ROcp6_55 = C4*C5;
    ROcp6_75 = S4*S5;
    ROcp6_85 = -C4*S5;
    ROcp6_16 = -(ROcp6_75*S6-C4*C6);
    ROcp6_26 = -(ROcp6_85*S6-S4*C6);
    ROcp6_36 = -C5*S6;
    ROcp6_76 = ROcp6_75*C6+C4*S6;
    ROcp6_86 = ROcp6_85*C6+S4*S6;
    ROcp6_96 = C5*C6;
    OMcp6_15 = qd(5)*C4;
    OMcp6_25 = qd(5)*S4;
    OMcp6_16 = OMcp6_15+ROcp6_45*qd(6);
    OMcp6_26 = OMcp6_25+ROcp6_55*qd(6);
    OMcp6_36 = qd(4)+qd(6)*S5;
    OPcp6_16 = ROcp6_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp6_25*S5-ROcp6_55*qd(4));
    OPcp6_26 = ROcp6_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp6_15*S5-ROcp6_45*qd(4));
    OPcp6_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_7_0_2 = = 
 
% Sensor Kinematics 


    RLcp6_17 = ROcp6_16*q(7);
    RLcp6_27 = ROcp6_26*q(7);
    RLcp6_37 = ROcp6_36*q(7);
    POcp6_17 = RLcp6_17+q(1);
    POcp6_27 = RLcp6_27+q(2);
    POcp6_37 = RLcp6_37+q(3);
    ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
    ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
    ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
    VIcp6_17 = ORcp6_17+qd(1)+ROcp6_16*qd(7);
    VIcp6_27 = ORcp6_27+qd(2)+ROcp6_26*qd(7);
    VIcp6_37 = ORcp6_37+qd(3)+ROcp6_36*qd(7);
    ACcp6_17 = qdd(1)+OMcp6_26*ORcp6_37-OMcp6_36*ORcp6_27+OPcp6_26*RLcp6_37-OPcp6_36*RLcp6_27+ROcp6_16*qdd(7)+(2.0)*qd(7)*(OMcp6_26*ROcp6_36-OMcp6_36*...
 ROcp6_26);
    ACcp6_27 = qdd(2)-OMcp6_16*ORcp6_37+OMcp6_36*ORcp6_17-OPcp6_16*RLcp6_37+OPcp6_36*RLcp6_17+ROcp6_26*qdd(7)-(2.0)*qd(7)*(OMcp6_16*ROcp6_36-OMcp6_36*...
 ROcp6_16);
    ACcp6_37 = qdd(3)+OMcp6_16*ORcp6_27-OMcp6_26*ORcp6_17+OPcp6_16*RLcp6_27-OPcp6_26*RLcp6_17+ROcp6_36*qdd(7)+(2.0)*qd(7)*(OMcp6_16*ROcp6_26-OMcp6_26*...
 ROcp6_16);

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp6_17;
    sens.P(2) = POcp6_27;
    sens.P(3) = POcp6_37;
    sens.R(1,1) = ROcp6_16;
    sens.R(1,2) = ROcp6_26;
    sens.R(1,3) = ROcp6_36;
    sens.R(2,1) = ROcp6_45;
    sens.R(2,2) = ROcp6_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp6_76;
    sens.R(3,2) = ROcp6_86;
    sens.R(3,3) = ROcp6_96;
    sens.V(1) = VIcp6_17;
    sens.V(2) = VIcp6_27;
    sens.V(3) = VIcp6_37;
    sens.OM(1) = OMcp6_16;
    sens.OM(2) = OMcp6_26;
    sens.OM(3) = OMcp6_36;
    sens.A(1) = ACcp6_17;
    sens.A(2) = ACcp6_27;
    sens.A(3) = ACcp6_37;
    sens.OMP(1) = OPcp6_16;
    sens.OMP(2) = OPcp6_26;
    sens.OMP(3) = OPcp6_36;
 
% 
case 8, 


% = = Block_1_0_0_8_0_1 = = 
 
% Sensor Kinematics 


    ROcp7_45 = -S4*C5;
    ROcp7_55 = C4*C5;
    ROcp7_75 = S4*S5;
    ROcp7_85 = -C4*S5;
    ROcp7_16 = -(ROcp7_75*S6-C4*C6);
    ROcp7_26 = -(ROcp7_85*S6-S4*C6);
    ROcp7_36 = -C5*S6;
    ROcp7_76 = ROcp7_75*C6+C4*S6;
    ROcp7_86 = ROcp7_85*C6+S4*S6;
    ROcp7_96 = C5*C6;
    OMcp7_15 = qd(5)*C4;
    OMcp7_25 = qd(5)*S4;
    OMcp7_16 = OMcp7_15+ROcp7_45*qd(6);
    OMcp7_26 = OMcp7_25+ROcp7_55*qd(6);
    OMcp7_36 = qd(4)+qd(6)*S5;
    OPcp7_16 = ROcp7_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp7_25*S5-ROcp7_55*qd(4));
    OPcp7_26 = ROcp7_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp7_15*S5-ROcp7_45*qd(4));
    OPcp7_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_8_0_3 = = 
 
% Sensor Kinematics 


    ROcp7_48 = ROcp7_45*C8+ROcp7_76*S8;
    ROcp7_58 = ROcp7_55*C8+ROcp7_86*S8;
    ROcp7_68 = ROcp7_96*S8+S5*C8;
    ROcp7_78 = -(ROcp7_45*S8-ROcp7_76*C8);
    ROcp7_88 = -(ROcp7_55*S8-ROcp7_86*C8);
    ROcp7_98 = ROcp7_96*C8-S5*S8;
    RLcp7_18 = ROcp7_16*s.dpt(1,1)+ROcp7_45*s.dpt(2,1)+ROcp7_76*s.dpt(3,1);
    RLcp7_28 = ROcp7_26*s.dpt(1,1)+ROcp7_55*s.dpt(2,1)+ROcp7_86*s.dpt(3,1);
    RLcp7_38 = ROcp7_36*s.dpt(1,1)+ROcp7_96*s.dpt(3,1)+s.dpt(2,1)*S5;
    POcp7_18 = RLcp7_18+q(1);
    POcp7_28 = RLcp7_28+q(2);
    POcp7_38 = RLcp7_38+q(3);
    OMcp7_18 = OMcp7_16+ROcp7_16*qd(8);
    OMcp7_28 = OMcp7_26+ROcp7_26*qd(8);
    OMcp7_38 = OMcp7_36+ROcp7_36*qd(8);
    ORcp7_18 = OMcp7_26*RLcp7_38-OMcp7_36*RLcp7_28;
    ORcp7_28 = -(OMcp7_16*RLcp7_38-OMcp7_36*RLcp7_18);
    ORcp7_38 = OMcp7_16*RLcp7_28-OMcp7_26*RLcp7_18;
    VIcp7_18 = ORcp7_18+qd(1);
    VIcp7_28 = ORcp7_28+qd(2);
    VIcp7_38 = ORcp7_38+qd(3);
    OPcp7_18 = OPcp7_16+ROcp7_16*qdd(8)+qd(8)*(OMcp7_26*ROcp7_36-OMcp7_36*ROcp7_26);
    OPcp7_28 = OPcp7_26+ROcp7_26*qdd(8)-qd(8)*(OMcp7_16*ROcp7_36-OMcp7_36*ROcp7_16);
    OPcp7_38 = OPcp7_36+ROcp7_36*qdd(8)+qd(8)*(OMcp7_16*ROcp7_26-OMcp7_26*ROcp7_16);
    ACcp7_18 = qdd(1)+OMcp7_26*ORcp7_38-OMcp7_36*ORcp7_28+OPcp7_26*RLcp7_38-OPcp7_36*RLcp7_28;
    ACcp7_28 = qdd(2)-OMcp7_16*ORcp7_38+OMcp7_36*ORcp7_18-OPcp7_16*RLcp7_38+OPcp7_36*RLcp7_18;
    ACcp7_38 = qdd(3)+OMcp7_16*ORcp7_28-OMcp7_26*ORcp7_18+OPcp7_16*RLcp7_28-OPcp7_26*RLcp7_18;

% = = Block_1_0_0_8_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp7_18;
    sens.P(2) = POcp7_28;
    sens.P(3) = POcp7_38;
    sens.R(1,1) = ROcp7_16;
    sens.R(1,2) = ROcp7_26;
    sens.R(1,3) = ROcp7_36;
    sens.R(2,1) = ROcp7_48;
    sens.R(2,2) = ROcp7_58;
    sens.R(2,3) = ROcp7_68;
    sens.R(3,1) = ROcp7_78;
    sens.R(3,2) = ROcp7_88;
    sens.R(3,3) = ROcp7_98;
    sens.V(1) = VIcp7_18;
    sens.V(2) = VIcp7_28;
    sens.V(3) = VIcp7_38;
    sens.OM(1) = OMcp7_18;
    sens.OM(2) = OMcp7_28;
    sens.OM(3) = OMcp7_38;
    sens.A(1) = ACcp7_18;
    sens.A(2) = ACcp7_28;
    sens.A(3) = ACcp7_38;
    sens.OMP(1) = OPcp7_18;
    sens.OMP(2) = OPcp7_28;
    sens.OMP(3) = OPcp7_38;
 
% 
case 9, 


% = = Block_1_0_0_9_0_1 = = 
 
% Sensor Kinematics 


    ROcp8_45 = -S4*C5;
    ROcp8_55 = C4*C5;
    ROcp8_75 = S4*S5;
    ROcp8_85 = -C4*S5;
    ROcp8_16 = -(ROcp8_75*S6-C4*C6);
    ROcp8_26 = -(ROcp8_85*S6-S4*C6);
    ROcp8_36 = -C5*S6;
    ROcp8_76 = ROcp8_75*C6+C4*S6;
    ROcp8_86 = ROcp8_85*C6+S4*S6;
    ROcp8_96 = C5*C6;
    OMcp8_15 = qd(5)*C4;
    OMcp8_25 = qd(5)*S4;
    OMcp8_16 = OMcp8_15+ROcp8_45*qd(6);
    OMcp8_26 = OMcp8_25+ROcp8_55*qd(6);
    OMcp8_36 = qd(4)+qd(6)*S5;
    OPcp8_16 = ROcp8_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp8_25*S5-ROcp8_55*qd(4));
    OPcp8_26 = ROcp8_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp8_15*S5-ROcp8_45*qd(4));
    OPcp8_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_9_0_3 = = 
 
% Sensor Kinematics 


    ROcp8_48 = ROcp8_45*C8+ROcp8_76*S8;
    ROcp8_58 = ROcp8_55*C8+ROcp8_86*S8;
    ROcp8_68 = ROcp8_96*S8+S5*C8;
    ROcp8_78 = -(ROcp8_45*S8-ROcp8_76*C8);
    ROcp8_88 = -(ROcp8_55*S8-ROcp8_86*C8);
    ROcp8_98 = ROcp8_96*C8-S5*S8;
    ROcp8_19 = ROcp8_16*C9-ROcp8_78*S9;
    ROcp8_29 = ROcp8_26*C9-ROcp8_88*S9;
    ROcp8_39 = ROcp8_36*C9-ROcp8_98*S9;
    ROcp8_79 = ROcp8_16*S9+ROcp8_78*C9;
    ROcp8_89 = ROcp8_26*S9+ROcp8_88*C9;
    ROcp8_99 = ROcp8_36*S9+ROcp8_98*C9;
    RLcp8_18 = ROcp8_16*s.dpt(1,1)+ROcp8_45*s.dpt(2,1)+ROcp8_76*s.dpt(3,1);
    RLcp8_28 = ROcp8_26*s.dpt(1,1)+ROcp8_55*s.dpt(2,1)+ROcp8_86*s.dpt(3,1);
    RLcp8_38 = ROcp8_36*s.dpt(1,1)+ROcp8_96*s.dpt(3,1)+s.dpt(2,1)*S5;
    OMcp8_18 = OMcp8_16+ROcp8_16*qd(8);
    OMcp8_28 = OMcp8_26+ROcp8_26*qd(8);
    OMcp8_38 = OMcp8_36+ROcp8_36*qd(8);
    ORcp8_18 = OMcp8_26*RLcp8_38-OMcp8_36*RLcp8_28;
    ORcp8_28 = -(OMcp8_16*RLcp8_38-OMcp8_36*RLcp8_18);
    ORcp8_38 = OMcp8_16*RLcp8_28-OMcp8_26*RLcp8_18;
    OPcp8_18 = OPcp8_16+ROcp8_16*qdd(8)+qd(8)*(OMcp8_26*ROcp8_36-OMcp8_36*ROcp8_26);
    OPcp8_28 = OPcp8_26+ROcp8_26*qdd(8)-qd(8)*(OMcp8_16*ROcp8_36-OMcp8_36*ROcp8_16);
    OPcp8_38 = OPcp8_36+ROcp8_36*qdd(8)+qd(8)*(OMcp8_16*ROcp8_26-OMcp8_26*ROcp8_16);
    RLcp8_19 = ROcp8_48*s.dpt(2,18);
    RLcp8_29 = ROcp8_58*s.dpt(2,18);
    RLcp8_39 = ROcp8_68*s.dpt(2,18);
    POcp8_19 = RLcp8_18+RLcp8_19+q(1);
    POcp8_29 = RLcp8_28+RLcp8_29+q(2);
    POcp8_39 = RLcp8_38+RLcp8_39+q(3);
    OMcp8_19 = OMcp8_18+ROcp8_48*qd(9);
    OMcp8_29 = OMcp8_28+ROcp8_58*qd(9);
    OMcp8_39 = OMcp8_38+ROcp8_68*qd(9);
    ORcp8_19 = OMcp8_28*RLcp8_39-OMcp8_38*RLcp8_29;
    ORcp8_29 = -(OMcp8_18*RLcp8_39-OMcp8_38*RLcp8_19);
    ORcp8_39 = OMcp8_18*RLcp8_29-OMcp8_28*RLcp8_19;
    VIcp8_19 = ORcp8_18+ORcp8_19+qd(1);
    VIcp8_29 = ORcp8_28+ORcp8_29+qd(2);
    VIcp8_39 = ORcp8_38+ORcp8_39+qd(3);
    OPcp8_19 = OPcp8_18+ROcp8_48*qdd(9)+qd(9)*(OMcp8_28*ROcp8_68-OMcp8_38*ROcp8_58);
    OPcp8_29 = OPcp8_28+ROcp8_58*qdd(9)-qd(9)*(OMcp8_18*ROcp8_68-OMcp8_38*ROcp8_48);
    OPcp8_39 = OPcp8_38+ROcp8_68*qdd(9)+qd(9)*(OMcp8_18*ROcp8_58-OMcp8_28*ROcp8_48);
    ACcp8_19 = qdd(1)+OMcp8_26*ORcp8_38+OMcp8_28*ORcp8_39-OMcp8_36*ORcp8_28-OMcp8_38*ORcp8_29+OPcp8_26*RLcp8_38+OPcp8_28*RLcp8_39-OPcp8_36*RLcp8_28...
 -OPcp8_38*RLcp8_29;
    ACcp8_29 = qdd(2)-OMcp8_16*ORcp8_38-OMcp8_18*ORcp8_39+OMcp8_36*ORcp8_18+OMcp8_38*ORcp8_19-OPcp8_16*RLcp8_38-OPcp8_18*RLcp8_39+OPcp8_36*RLcp8_18...
 +OPcp8_38*RLcp8_19;
    ACcp8_39 = qdd(3)+OMcp8_16*ORcp8_28+OMcp8_18*ORcp8_29-OMcp8_26*ORcp8_18-OMcp8_28*ORcp8_19+OPcp8_16*RLcp8_28+OPcp8_18*RLcp8_29-OPcp8_26*RLcp8_18...
 -OPcp8_28*RLcp8_19;

% = = Block_1_0_0_9_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp8_19;
    sens.P(2) = POcp8_29;
    sens.P(3) = POcp8_39;
    sens.R(1,1) = ROcp8_19;
    sens.R(1,2) = ROcp8_29;
    sens.R(1,3) = ROcp8_39;
    sens.R(2,1) = ROcp8_48;
    sens.R(2,2) = ROcp8_58;
    sens.R(2,3) = ROcp8_68;
    sens.R(3,1) = ROcp8_79;
    sens.R(3,2) = ROcp8_89;
    sens.R(3,3) = ROcp8_99;
    sens.V(1) = VIcp8_19;
    sens.V(2) = VIcp8_29;
    sens.V(3) = VIcp8_39;
    sens.OM(1) = OMcp8_19;
    sens.OM(2) = OMcp8_29;
    sens.OM(3) = OMcp8_39;
    sens.A(1) = ACcp8_19;
    sens.A(2) = ACcp8_29;
    sens.A(3) = ACcp8_39;
    sens.OMP(1) = OPcp8_19;
    sens.OMP(2) = OPcp8_29;
    sens.OMP(3) = OPcp8_39;
 
% 
case 10, 


% = = Block_1_0_0_10_0_1 = = 
 
% Sensor Kinematics 


    ROcp9_45 = -S4*C5;
    ROcp9_55 = C4*C5;
    ROcp9_75 = S4*S5;
    ROcp9_85 = -C4*S5;
    ROcp9_16 = -(ROcp9_75*S6-C4*C6);
    ROcp9_26 = -(ROcp9_85*S6-S4*C6);
    ROcp9_36 = -C5*S6;
    ROcp9_76 = ROcp9_75*C6+C4*S6;
    ROcp9_86 = ROcp9_85*C6+S4*S6;
    ROcp9_96 = C5*C6;
    OMcp9_15 = qd(5)*C4;
    OMcp9_25 = qd(5)*S4;
    OMcp9_16 = OMcp9_15+ROcp9_45*qd(6);
    OMcp9_26 = OMcp9_25+ROcp9_55*qd(6);
    OMcp9_36 = qd(4)+qd(6)*S5;
    OPcp9_16 = ROcp9_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp9_25*S5-ROcp9_55*qd(4));
    OPcp9_26 = ROcp9_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp9_15*S5-ROcp9_45*qd(4));
    OPcp9_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_10_0_3 = = 
 
% Sensor Kinematics 


    ROcp9_48 = ROcp9_45*C8+ROcp9_76*S8;
    ROcp9_58 = ROcp9_55*C8+ROcp9_86*S8;
    ROcp9_68 = ROcp9_96*S8+S5*C8;
    ROcp9_78 = -(ROcp9_45*S8-ROcp9_76*C8);
    ROcp9_88 = -(ROcp9_55*S8-ROcp9_86*C8);
    ROcp9_98 = ROcp9_96*C8-S5*S8;
    ROcp9_19 = ROcp9_16*C9-ROcp9_78*S9;
    ROcp9_29 = ROcp9_26*C9-ROcp9_88*S9;
    ROcp9_39 = ROcp9_36*C9-ROcp9_98*S9;
    ROcp9_79 = ROcp9_16*S9+ROcp9_78*C9;
    ROcp9_89 = ROcp9_26*S9+ROcp9_88*C9;
    ROcp9_99 = ROcp9_36*S9+ROcp9_98*C9;
    ROcp9_410 = ROcp9_48*C10+ROcp9_79*S10;
    ROcp9_510 = ROcp9_58*C10+ROcp9_89*S10;
    ROcp9_610 = ROcp9_68*C10+ROcp9_99*S10;
    ROcp9_710 = -(ROcp9_48*S10-ROcp9_79*C10);
    ROcp9_810 = -(ROcp9_58*S10-ROcp9_89*C10);
    ROcp9_910 = -(ROcp9_68*S10-ROcp9_99*C10);
    RLcp9_18 = ROcp9_16*s.dpt(1,1)+ROcp9_45*s.dpt(2,1)+ROcp9_76*s.dpt(3,1);
    RLcp9_28 = ROcp9_26*s.dpt(1,1)+ROcp9_55*s.dpt(2,1)+ROcp9_86*s.dpt(3,1);
    RLcp9_38 = ROcp9_36*s.dpt(1,1)+ROcp9_96*s.dpt(3,1)+s.dpt(2,1)*S5;
    OMcp9_18 = OMcp9_16+ROcp9_16*qd(8);
    OMcp9_28 = OMcp9_26+ROcp9_26*qd(8);
    OMcp9_38 = OMcp9_36+ROcp9_36*qd(8);
    ORcp9_18 = OMcp9_26*RLcp9_38-OMcp9_36*RLcp9_28;
    ORcp9_28 = -(OMcp9_16*RLcp9_38-OMcp9_36*RLcp9_18);
    ORcp9_38 = OMcp9_16*RLcp9_28-OMcp9_26*RLcp9_18;
    OPcp9_18 = OPcp9_16+ROcp9_16*qdd(8)+qd(8)*(OMcp9_26*ROcp9_36-OMcp9_36*ROcp9_26);
    OPcp9_28 = OPcp9_26+ROcp9_26*qdd(8)-qd(8)*(OMcp9_16*ROcp9_36-OMcp9_36*ROcp9_16);
    OPcp9_38 = OPcp9_36+ROcp9_36*qdd(8)+qd(8)*(OMcp9_16*ROcp9_26-OMcp9_26*ROcp9_16);
    RLcp9_19 = ROcp9_48*s.dpt(2,18);
    RLcp9_29 = ROcp9_58*s.dpt(2,18);
    RLcp9_39 = ROcp9_68*s.dpt(2,18);
    POcp9_19 = RLcp9_18+RLcp9_19+q(1);
    POcp9_29 = RLcp9_28+RLcp9_29+q(2);
    POcp9_39 = RLcp9_38+RLcp9_39+q(3);
    OMcp9_19 = OMcp9_18+ROcp9_48*qd(9);
    OMcp9_29 = OMcp9_28+ROcp9_58*qd(9);
    OMcp9_39 = OMcp9_38+ROcp9_68*qd(9);
    ORcp9_19 = OMcp9_28*RLcp9_39-OMcp9_38*RLcp9_29;
    ORcp9_29 = -(OMcp9_18*RLcp9_39-OMcp9_38*RLcp9_19);
    ORcp9_39 = OMcp9_18*RLcp9_29-OMcp9_28*RLcp9_19;
    VIcp9_19 = ORcp9_18+ORcp9_19+qd(1);
    VIcp9_29 = ORcp9_28+ORcp9_29+qd(2);
    VIcp9_39 = ORcp9_38+ORcp9_39+qd(3);
    ACcp9_19 = qdd(1)+OMcp9_26*ORcp9_38+OMcp9_28*ORcp9_39-OMcp9_36*ORcp9_28-OMcp9_38*ORcp9_29+OPcp9_26*RLcp9_38+OPcp9_28*RLcp9_39-OPcp9_36*RLcp9_28...
 -OPcp9_38*RLcp9_29;
    ACcp9_29 = qdd(2)-OMcp9_16*ORcp9_38-OMcp9_18*ORcp9_39+OMcp9_36*ORcp9_18+OMcp9_38*ORcp9_19-OPcp9_16*RLcp9_38-OPcp9_18*RLcp9_39+OPcp9_36*RLcp9_18...
 +OPcp9_38*RLcp9_19;
    ACcp9_39 = qdd(3)+OMcp9_16*ORcp9_28+OMcp9_18*ORcp9_29-OMcp9_26*ORcp9_18-OMcp9_28*ORcp9_19+OPcp9_16*RLcp9_28+OPcp9_18*RLcp9_29-OPcp9_26*RLcp9_18...
 -OPcp9_28*RLcp9_19;
    OMcp9_110 = OMcp9_19+ROcp9_19*qd(10);
    OMcp9_210 = OMcp9_29+ROcp9_29*qd(10);
    OMcp9_310 = OMcp9_39+ROcp9_39*qd(10);
    OPcp9_110 = OPcp9_18+ROcp9_19*qdd(10)+ROcp9_48*qdd(9)+qd(10)*(OMcp9_29*ROcp9_39-OMcp9_39*ROcp9_29)+qd(9)*(OMcp9_28*ROcp9_68-OMcp9_38*ROcp9_58);
    OPcp9_210 = OPcp9_28+ROcp9_29*qdd(10)+ROcp9_58*qdd(9)-qd(10)*(OMcp9_19*ROcp9_39-OMcp9_39*ROcp9_19)-qd(9)*(OMcp9_18*ROcp9_68-OMcp9_38*ROcp9_48);
    OPcp9_310 = OPcp9_38+ROcp9_39*qdd(10)+ROcp9_68*qdd(9)+qd(10)*(OMcp9_19*ROcp9_29-OMcp9_29*ROcp9_19)+qd(9)*(OMcp9_18*ROcp9_58-OMcp9_28*ROcp9_48);

% = = Block_1_0_0_10_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp9_19;
    sens.P(2) = POcp9_29;
    sens.P(3) = POcp9_39;
    sens.R(1,1) = ROcp9_19;
    sens.R(1,2) = ROcp9_29;
    sens.R(1,3) = ROcp9_39;
    sens.R(2,1) = ROcp9_410;
    sens.R(2,2) = ROcp9_510;
    sens.R(2,3) = ROcp9_610;
    sens.R(3,1) = ROcp9_710;
    sens.R(3,2) = ROcp9_810;
    sens.R(3,3) = ROcp9_910;
    sens.V(1) = VIcp9_19;
    sens.V(2) = VIcp9_29;
    sens.V(3) = VIcp9_39;
    sens.OM(1) = OMcp9_110;
    sens.OM(2) = OMcp9_210;
    sens.OM(3) = OMcp9_310;
    sens.A(1) = ACcp9_19;
    sens.A(2) = ACcp9_29;
    sens.A(3) = ACcp9_39;
    sens.OMP(1) = OPcp9_110;
    sens.OMP(2) = OPcp9_210;
    sens.OMP(3) = OPcp9_310;
 
% 
case 11, 


% = = Block_1_0_0_11_0_1 = = 
 
% Sensor Kinematics 


    ROcp10_45 = -S4*C5;
    ROcp10_55 = C4*C5;
    ROcp10_75 = S4*S5;
    ROcp10_85 = -C4*S5;
    ROcp10_16 = -(ROcp10_75*S6-C4*C6);
    ROcp10_26 = -(ROcp10_85*S6-S4*C6);
    ROcp10_36 = -C5*S6;
    ROcp10_76 = ROcp10_75*C6+C4*S6;
    ROcp10_86 = ROcp10_85*C6+S4*S6;
    ROcp10_96 = C5*C6;
    OMcp10_15 = qd(5)*C4;
    OMcp10_25 = qd(5)*S4;
    OMcp10_16 = OMcp10_15+ROcp10_45*qd(6);
    OMcp10_26 = OMcp10_25+ROcp10_55*qd(6);
    OMcp10_36 = qd(4)+qd(6)*S5;
    OPcp10_16 = ROcp10_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp10_25*S5-ROcp10_55*qd(4));
    OPcp10_26 = ROcp10_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp10_15*S5-ROcp10_45*qd(4));
    OPcp10_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_11_0_3 = = 
 
% Sensor Kinematics 


    ROcp10_48 = ROcp10_45*C8+ROcp10_76*S8;
    ROcp10_58 = ROcp10_55*C8+ROcp10_86*S8;
    ROcp10_68 = ROcp10_96*S8+S5*C8;
    ROcp10_78 = -(ROcp10_45*S8-ROcp10_76*C8);
    ROcp10_88 = -(ROcp10_55*S8-ROcp10_86*C8);
    ROcp10_98 = ROcp10_96*C8-S5*S8;
    ROcp10_19 = ROcp10_16*C9-ROcp10_78*S9;
    ROcp10_29 = ROcp10_26*C9-ROcp10_88*S9;
    ROcp10_39 = ROcp10_36*C9-ROcp10_98*S9;
    ROcp10_79 = ROcp10_16*S9+ROcp10_78*C9;
    ROcp10_89 = ROcp10_26*S9+ROcp10_88*C9;
    ROcp10_99 = ROcp10_36*S9+ROcp10_98*C9;
    ROcp10_410 = ROcp10_48*C10+ROcp10_79*S10;
    ROcp10_510 = ROcp10_58*C10+ROcp10_89*S10;
    ROcp10_610 = ROcp10_68*C10+ROcp10_99*S10;
    ROcp10_710 = -(ROcp10_48*S10-ROcp10_79*C10);
    ROcp10_810 = -(ROcp10_58*S10-ROcp10_89*C10);
    ROcp10_910 = -(ROcp10_68*S10-ROcp10_99*C10);
    ROcp10_111 = ROcp10_19*C11+ROcp10_410*S11;
    ROcp10_211 = ROcp10_29*C11+ROcp10_510*S11;
    ROcp10_311 = ROcp10_39*C11+ROcp10_610*S11;
    ROcp10_411 = -(ROcp10_19*S11-ROcp10_410*C11);
    ROcp10_511 = -(ROcp10_29*S11-ROcp10_510*C11);
    ROcp10_611 = -(ROcp10_39*S11-ROcp10_610*C11);
    RLcp10_18 = ROcp10_16*s.dpt(1,1)+ROcp10_45*s.dpt(2,1)+ROcp10_76*s.dpt(3,1);
    RLcp10_28 = ROcp10_26*s.dpt(1,1)+ROcp10_55*s.dpt(2,1)+ROcp10_86*s.dpt(3,1);
    RLcp10_38 = ROcp10_36*s.dpt(1,1)+ROcp10_96*s.dpt(3,1)+s.dpt(2,1)*S5;
    OMcp10_18 = OMcp10_16+ROcp10_16*qd(8);
    OMcp10_28 = OMcp10_26+ROcp10_26*qd(8);
    OMcp10_38 = OMcp10_36+ROcp10_36*qd(8);
    ORcp10_18 = OMcp10_26*RLcp10_38-OMcp10_36*RLcp10_28;
    ORcp10_28 = -(OMcp10_16*RLcp10_38-OMcp10_36*RLcp10_18);
    ORcp10_38 = OMcp10_16*RLcp10_28-OMcp10_26*RLcp10_18;
    OPcp10_18 = OPcp10_16+ROcp10_16*qdd(8)+qd(8)*(OMcp10_26*ROcp10_36-OMcp10_36*ROcp10_26);
    OPcp10_28 = OPcp10_26+ROcp10_26*qdd(8)-qd(8)*(OMcp10_16*ROcp10_36-OMcp10_36*ROcp10_16);
    OPcp10_38 = OPcp10_36+ROcp10_36*qdd(8)+qd(8)*(OMcp10_16*ROcp10_26-OMcp10_26*ROcp10_16);
    RLcp10_19 = ROcp10_48*s.dpt(2,18);
    RLcp10_29 = ROcp10_58*s.dpt(2,18);
    RLcp10_39 = ROcp10_68*s.dpt(2,18);
    POcp10_19 = RLcp10_18+RLcp10_19+q(1);
    POcp10_29 = RLcp10_28+RLcp10_29+q(2);
    POcp10_39 = RLcp10_38+RLcp10_39+q(3);
    OMcp10_19 = OMcp10_18+ROcp10_48*qd(9);
    OMcp10_29 = OMcp10_28+ROcp10_58*qd(9);
    OMcp10_39 = OMcp10_38+ROcp10_68*qd(9);
    ORcp10_19 = OMcp10_28*RLcp10_39-OMcp10_38*RLcp10_29;
    ORcp10_29 = -(OMcp10_18*RLcp10_39-OMcp10_38*RLcp10_19);
    ORcp10_39 = OMcp10_18*RLcp10_29-OMcp10_28*RLcp10_19;
    VIcp10_19 = ORcp10_18+ORcp10_19+qd(1);
    VIcp10_29 = ORcp10_28+ORcp10_29+qd(2);
    VIcp10_39 = ORcp10_38+ORcp10_39+qd(3);
    ACcp10_19 = qdd(1)+OMcp10_26*ORcp10_38+OMcp10_28*ORcp10_39-OMcp10_36*ORcp10_28-OMcp10_38*ORcp10_29+OPcp10_26*RLcp10_38+OPcp10_28*RLcp10_39-...
 OPcp10_36*RLcp10_28-OPcp10_38*RLcp10_29;
    ACcp10_29 = qdd(2)-OMcp10_16*ORcp10_38-OMcp10_18*ORcp10_39+OMcp10_36*ORcp10_18+OMcp10_38*ORcp10_19-OPcp10_16*RLcp10_38-OPcp10_18*RLcp10_39+...
 OPcp10_36*RLcp10_18+OPcp10_38*RLcp10_19;
    ACcp10_39 = qdd(3)+OMcp10_16*ORcp10_28+OMcp10_18*ORcp10_29-OMcp10_26*ORcp10_18-OMcp10_28*ORcp10_19+OPcp10_16*RLcp10_28+OPcp10_18*RLcp10_29-...
 OPcp10_26*RLcp10_18-OPcp10_28*RLcp10_19;
    OMcp10_110 = OMcp10_19+ROcp10_19*qd(10);
    OMcp10_210 = OMcp10_29+ROcp10_29*qd(10);
    OMcp10_310 = OMcp10_39+ROcp10_39*qd(10);
    OMcp10_111 = OMcp10_110+ROcp10_710*qd(11);
    OMcp10_211 = OMcp10_210+ROcp10_810*qd(11);
    OMcp10_311 = OMcp10_310+ROcp10_910*qd(11);
    OPcp10_111 = OPcp10_18+ROcp10_19*qdd(10)+ROcp10_48*qdd(9)+ROcp10_710*qdd(11)+qd(10)*(OMcp10_29*ROcp10_39-OMcp10_39*ROcp10_29)+qd(11)*(...
 OMcp10_210*ROcp10_910-OMcp10_310*ROcp10_810)+qd(9)*(OMcp10_28*ROcp10_68-OMcp10_38*ROcp10_58);
    OPcp10_211 = OPcp10_28+ROcp10_29*qdd(10)+ROcp10_58*qdd(9)+ROcp10_810*qdd(11)-qd(10)*(OMcp10_19*ROcp10_39-OMcp10_39*ROcp10_19)-qd(11)*(...
 OMcp10_110*ROcp10_910-OMcp10_310*ROcp10_710)-qd(9)*(OMcp10_18*ROcp10_68-OMcp10_38*ROcp10_48);
    OPcp10_311 = OPcp10_38+ROcp10_39*qdd(10)+ROcp10_68*qdd(9)+ROcp10_910*qdd(11)+qd(10)*(OMcp10_19*ROcp10_29-OMcp10_29*ROcp10_19)+qd(11)*(...
 OMcp10_110*ROcp10_810-OMcp10_210*ROcp10_710)+qd(9)*(OMcp10_18*ROcp10_58-OMcp10_28*ROcp10_48);

% = = Block_1_0_0_11_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp10_19;
    sens.P(2) = POcp10_29;
    sens.P(3) = POcp10_39;
    sens.R(1,1) = ROcp10_111;
    sens.R(1,2) = ROcp10_211;
    sens.R(1,3) = ROcp10_311;
    sens.R(2,1) = ROcp10_411;
    sens.R(2,2) = ROcp10_511;
    sens.R(2,3) = ROcp10_611;
    sens.R(3,1) = ROcp10_710;
    sens.R(3,2) = ROcp10_810;
    sens.R(3,3) = ROcp10_910;
    sens.V(1) = VIcp10_19;
    sens.V(2) = VIcp10_29;
    sens.V(3) = VIcp10_39;
    sens.OM(1) = OMcp10_111;
    sens.OM(2) = OMcp10_211;
    sens.OM(3) = OMcp10_311;
    sens.A(1) = ACcp10_19;
    sens.A(2) = ACcp10_29;
    sens.A(3) = ACcp10_39;
    sens.OMP(1) = OPcp10_111;
    sens.OMP(2) = OPcp10_211;
    sens.OMP(3) = OPcp10_311;
 
% 
case 12, 


% = = Block_1_0_0_12_0_1 = = 
 
% Sensor Kinematics 


    ROcp11_45 = -S4*C5;
    ROcp11_55 = C4*C5;
    ROcp11_75 = S4*S5;
    ROcp11_85 = -C4*S5;
    ROcp11_16 = -(ROcp11_75*S6-C4*C6);
    ROcp11_26 = -(ROcp11_85*S6-S4*C6);
    ROcp11_36 = -C5*S6;
    ROcp11_76 = ROcp11_75*C6+C4*S6;
    ROcp11_86 = ROcp11_85*C6+S4*S6;
    ROcp11_96 = C5*C6;
    OMcp11_15 = qd(5)*C4;
    OMcp11_25 = qd(5)*S4;
    OMcp11_16 = OMcp11_15+ROcp11_45*qd(6);
    OMcp11_26 = OMcp11_25+ROcp11_55*qd(6);
    OMcp11_36 = qd(4)+qd(6)*S5;
    OPcp11_16 = ROcp11_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp11_25*S5-ROcp11_55*qd(4));
    OPcp11_26 = ROcp11_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp11_15*S5-ROcp11_45*qd(4));
    OPcp11_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_12_0_3 = = 
 
% Sensor Kinematics 


    ROcp11_48 = ROcp11_45*C8+ROcp11_76*S8;
    ROcp11_58 = ROcp11_55*C8+ROcp11_86*S8;
    ROcp11_68 = ROcp11_96*S8+S5*C8;
    ROcp11_78 = -(ROcp11_45*S8-ROcp11_76*C8);
    ROcp11_88 = -(ROcp11_55*S8-ROcp11_86*C8);
    ROcp11_98 = ROcp11_96*C8-S5*S8;
    ROcp11_19 = ROcp11_16*C9-ROcp11_78*S9;
    ROcp11_29 = ROcp11_26*C9-ROcp11_88*S9;
    ROcp11_39 = ROcp11_36*C9-ROcp11_98*S9;
    ROcp11_79 = ROcp11_16*S9+ROcp11_78*C9;
    ROcp11_89 = ROcp11_26*S9+ROcp11_88*C9;
    ROcp11_99 = ROcp11_36*S9+ROcp11_98*C9;
    ROcp11_410 = ROcp11_48*C10+ROcp11_79*S10;
    ROcp11_510 = ROcp11_58*C10+ROcp11_89*S10;
    ROcp11_610 = ROcp11_68*C10+ROcp11_99*S10;
    ROcp11_710 = -(ROcp11_48*S10-ROcp11_79*C10);
    ROcp11_810 = -(ROcp11_58*S10-ROcp11_89*C10);
    ROcp11_910 = -(ROcp11_68*S10-ROcp11_99*C10);
    ROcp11_111 = ROcp11_19*C11+ROcp11_410*S11;
    ROcp11_211 = ROcp11_29*C11+ROcp11_510*S11;
    ROcp11_311 = ROcp11_39*C11+ROcp11_610*S11;
    ROcp11_411 = -(ROcp11_19*S11-ROcp11_410*C11);
    ROcp11_511 = -(ROcp11_29*S11-ROcp11_510*C11);
    ROcp11_611 = -(ROcp11_39*S11-ROcp11_610*C11);
    ROcp11_112 = ROcp11_111*C12-ROcp11_710*S12;
    ROcp11_212 = ROcp11_211*C12-ROcp11_810*S12;
    ROcp11_312 = ROcp11_311*C12-ROcp11_910*S12;
    ROcp11_712 = ROcp11_111*S12+ROcp11_710*C12;
    ROcp11_812 = ROcp11_211*S12+ROcp11_810*C12;
    ROcp11_912 = ROcp11_311*S12+ROcp11_910*C12;
    RLcp11_18 = ROcp11_16*s.dpt(1,1)+ROcp11_45*s.dpt(2,1)+ROcp11_76*s.dpt(3,1);
    RLcp11_28 = ROcp11_26*s.dpt(1,1)+ROcp11_55*s.dpt(2,1)+ROcp11_86*s.dpt(3,1);
    RLcp11_38 = ROcp11_36*s.dpt(1,1)+ROcp11_96*s.dpt(3,1)+s.dpt(2,1)*S5;
    OMcp11_18 = OMcp11_16+ROcp11_16*qd(8);
    OMcp11_28 = OMcp11_26+ROcp11_26*qd(8);
    OMcp11_38 = OMcp11_36+ROcp11_36*qd(8);
    ORcp11_18 = OMcp11_26*RLcp11_38-OMcp11_36*RLcp11_28;
    ORcp11_28 = -(OMcp11_16*RLcp11_38-OMcp11_36*RLcp11_18);
    ORcp11_38 = OMcp11_16*RLcp11_28-OMcp11_26*RLcp11_18;
    OPcp11_18 = OPcp11_16+ROcp11_16*qdd(8)+qd(8)*(OMcp11_26*ROcp11_36-OMcp11_36*ROcp11_26);
    OPcp11_28 = OPcp11_26+ROcp11_26*qdd(8)-qd(8)*(OMcp11_16*ROcp11_36-OMcp11_36*ROcp11_16);
    OPcp11_38 = OPcp11_36+ROcp11_36*qdd(8)+qd(8)*(OMcp11_16*ROcp11_26-OMcp11_26*ROcp11_16);
    RLcp11_19 = ROcp11_48*s.dpt(2,18);
    RLcp11_29 = ROcp11_58*s.dpt(2,18);
    RLcp11_39 = ROcp11_68*s.dpt(2,18);
    OMcp11_19 = OMcp11_18+ROcp11_48*qd(9);
    OMcp11_29 = OMcp11_28+ROcp11_58*qd(9);
    OMcp11_39 = OMcp11_38+ROcp11_68*qd(9);
    ORcp11_19 = OMcp11_28*RLcp11_39-OMcp11_38*RLcp11_29;
    ORcp11_29 = -(OMcp11_18*RLcp11_39-OMcp11_38*RLcp11_19);
    ORcp11_39 = OMcp11_18*RLcp11_29-OMcp11_28*RLcp11_19;
    OMcp11_110 = OMcp11_19+ROcp11_19*qd(10);
    OMcp11_210 = OMcp11_29+ROcp11_29*qd(10);
    OMcp11_310 = OMcp11_39+ROcp11_39*qd(10);
    OMcp11_111 = OMcp11_110+ROcp11_710*qd(11);
    OMcp11_211 = OMcp11_210+ROcp11_810*qd(11);
    OMcp11_311 = OMcp11_310+ROcp11_910*qd(11);
    OPcp11_111 = OPcp11_18+ROcp11_19*qdd(10)+ROcp11_48*qdd(9)+ROcp11_710*qdd(11)+qd(10)*(OMcp11_29*ROcp11_39-OMcp11_39*ROcp11_29)+qd(11)*(...
 OMcp11_210*ROcp11_910-OMcp11_310*ROcp11_810)+qd(9)*(OMcp11_28*ROcp11_68-OMcp11_38*ROcp11_58);
    OPcp11_211 = OPcp11_28+ROcp11_29*qdd(10)+ROcp11_58*qdd(9)+ROcp11_810*qdd(11)-qd(10)*(OMcp11_19*ROcp11_39-OMcp11_39*ROcp11_19)-qd(11)*(...
 OMcp11_110*ROcp11_910-OMcp11_310*ROcp11_710)-qd(9)*(OMcp11_18*ROcp11_68-OMcp11_38*ROcp11_48);
    OPcp11_311 = OPcp11_38+ROcp11_39*qdd(10)+ROcp11_68*qdd(9)+ROcp11_910*qdd(11)+qd(10)*(OMcp11_19*ROcp11_29-OMcp11_29*ROcp11_19)+qd(11)*(...
 OMcp11_110*ROcp11_810-OMcp11_210*ROcp11_710)+qd(9)*(OMcp11_18*ROcp11_58-OMcp11_28*ROcp11_48);
    RLcp11_112 = ROcp11_710*s.dpt(3,22);
    RLcp11_212 = ROcp11_810*s.dpt(3,22);
    RLcp11_312 = ROcp11_910*s.dpt(3,22);
    POcp11_112 = RLcp11_112+RLcp11_18+RLcp11_19+q(1);
    POcp11_212 = RLcp11_212+RLcp11_28+RLcp11_29+q(2);
    POcp11_312 = RLcp11_312+RLcp11_38+RLcp11_39+q(3);
    OMcp11_112 = OMcp11_111+ROcp11_411*qd(12);
    OMcp11_212 = OMcp11_211+ROcp11_511*qd(12);
    OMcp11_312 = OMcp11_311+ROcp11_611*qd(12);
    ORcp11_112 = OMcp11_211*RLcp11_312-OMcp11_311*RLcp11_212;
    ORcp11_212 = -(OMcp11_111*RLcp11_312-OMcp11_311*RLcp11_112);
    ORcp11_312 = OMcp11_111*RLcp11_212-OMcp11_211*RLcp11_112;
    VIcp11_112 = ORcp11_112+ORcp11_18+ORcp11_19+qd(1);
    VIcp11_212 = ORcp11_212+ORcp11_28+ORcp11_29+qd(2);
    VIcp11_312 = ORcp11_312+ORcp11_38+ORcp11_39+qd(3);
    OPcp11_112 = OPcp11_111+ROcp11_411*qdd(12)+qd(12)*(OMcp11_211*ROcp11_611-OMcp11_311*ROcp11_511);
    OPcp11_212 = OPcp11_211+ROcp11_511*qdd(12)-qd(12)*(OMcp11_111*ROcp11_611-OMcp11_311*ROcp11_411);
    OPcp11_312 = OPcp11_311+ROcp11_611*qdd(12)+qd(12)*(OMcp11_111*ROcp11_511-OMcp11_211*ROcp11_411);
    ACcp11_112 = qdd(1)+OMcp11_211*ORcp11_312+OMcp11_26*ORcp11_38+OMcp11_28*ORcp11_39-OMcp11_311*ORcp11_212-OMcp11_36*ORcp11_28-OMcp11_38*ORcp11_29...
 +OPcp11_211*RLcp11_312+OPcp11_26*RLcp11_38+OPcp11_28*RLcp11_39-OPcp11_311*RLcp11_212-OPcp11_36*RLcp11_28-OPcp11_38*RLcp11_29;
    ACcp11_212 = qdd(2)-OMcp11_111*ORcp11_312-OMcp11_16*ORcp11_38-OMcp11_18*ORcp11_39+OMcp11_311*ORcp11_112+OMcp11_36*ORcp11_18+OMcp11_38*ORcp11_19...
 -OPcp11_111*RLcp11_312-OPcp11_16*RLcp11_38-OPcp11_18*RLcp11_39+OPcp11_311*RLcp11_112+OPcp11_36*RLcp11_18+OPcp11_38*RLcp11_19;
    ACcp11_312 = qdd(3)+OMcp11_111*ORcp11_212+OMcp11_16*ORcp11_28+OMcp11_18*ORcp11_29-OMcp11_211*ORcp11_112-OMcp11_26*ORcp11_18-OMcp11_28*ORcp11_19...
 +OPcp11_111*RLcp11_212+OPcp11_16*RLcp11_28+OPcp11_18*RLcp11_29-OPcp11_211*RLcp11_112-OPcp11_26*RLcp11_18-OPcp11_28*RLcp11_19;

% = = Block_1_0_0_12_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp11_112;
    sens.P(2) = POcp11_212;
    sens.P(3) = POcp11_312;
    sens.R(1,1) = ROcp11_112;
    sens.R(1,2) = ROcp11_212;
    sens.R(1,3) = ROcp11_312;
    sens.R(2,1) = ROcp11_411;
    sens.R(2,2) = ROcp11_511;
    sens.R(2,3) = ROcp11_611;
    sens.R(3,1) = ROcp11_712;
    sens.R(3,2) = ROcp11_812;
    sens.R(3,3) = ROcp11_912;
    sens.V(1) = VIcp11_112;
    sens.V(2) = VIcp11_212;
    sens.V(3) = VIcp11_312;
    sens.OM(1) = OMcp11_112;
    sens.OM(2) = OMcp11_212;
    sens.OM(3) = OMcp11_312;
    sens.A(1) = ACcp11_112;
    sens.A(2) = ACcp11_212;
    sens.A(3) = ACcp11_312;
    sens.OMP(1) = OPcp11_112;
    sens.OMP(2) = OPcp11_212;
    sens.OMP(3) = OPcp11_312;
 
% 
case 13, 


% = = Block_1_0_0_13_0_1 = = 
 
% Sensor Kinematics 


    ROcp12_45 = -S4*C5;
    ROcp12_55 = C4*C5;
    ROcp12_75 = S4*S5;
    ROcp12_85 = -C4*S5;
    ROcp12_16 = -(ROcp12_75*S6-C4*C6);
    ROcp12_26 = -(ROcp12_85*S6-S4*C6);
    ROcp12_36 = -C5*S6;
    ROcp12_76 = ROcp12_75*C6+C4*S6;
    ROcp12_86 = ROcp12_85*C6+S4*S6;
    ROcp12_96 = C5*C6;
    OMcp12_15 = qd(5)*C4;
    OMcp12_25 = qd(5)*S4;
    OMcp12_16 = OMcp12_15+ROcp12_45*qd(6);
    OMcp12_26 = OMcp12_25+ROcp12_55*qd(6);
    OMcp12_36 = qd(4)+qd(6)*S5;
    OPcp12_16 = ROcp12_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp12_25*S5-ROcp12_55*qd(4));
    OPcp12_26 = ROcp12_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp12_15*S5-ROcp12_45*qd(4));
    OPcp12_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_13_0_4 = = 
 
% Sensor Kinematics 


    ROcp12_413 = ROcp12_45*C13+ROcp12_76*S13;
    ROcp12_513 = ROcp12_55*C13+ROcp12_86*S13;
    ROcp12_613 = ROcp12_96*S13+C13*S5;
    ROcp12_713 = -(ROcp12_45*S13-ROcp12_76*C13);
    ROcp12_813 = -(ROcp12_55*S13-ROcp12_86*C13);
    ROcp12_913 = ROcp12_96*C13-S13*S5;
    RLcp12_113 = ROcp12_16*s.dpt(1,2)+ROcp12_45*s.dpt(2,2)+ROcp12_76*s.dpt(3,2);
    RLcp12_213 = ROcp12_26*s.dpt(1,2)+ROcp12_55*s.dpt(2,2)+ROcp12_86*s.dpt(3,2);
    RLcp12_313 = ROcp12_36*s.dpt(1,2)+ROcp12_96*s.dpt(3,2)+s.dpt(2,2)*S5;
    POcp12_113 = RLcp12_113+q(1);
    POcp12_213 = RLcp12_213+q(2);
    POcp12_313 = RLcp12_313+q(3);
    OMcp12_113 = OMcp12_16+ROcp12_16*qd(13);
    OMcp12_213 = OMcp12_26+ROcp12_26*qd(13);
    OMcp12_313 = OMcp12_36+ROcp12_36*qd(13);
    ORcp12_113 = OMcp12_26*RLcp12_313-OMcp12_36*RLcp12_213;
    ORcp12_213 = -(OMcp12_16*RLcp12_313-OMcp12_36*RLcp12_113);
    ORcp12_313 = OMcp12_16*RLcp12_213-OMcp12_26*RLcp12_113;
    VIcp12_113 = ORcp12_113+qd(1);
    VIcp12_213 = ORcp12_213+qd(2);
    VIcp12_313 = ORcp12_313+qd(3);
    OPcp12_113 = OPcp12_16+ROcp12_16*qdd(13)+qd(13)*(OMcp12_26*ROcp12_36-OMcp12_36*ROcp12_26);
    OPcp12_213 = OPcp12_26+ROcp12_26*qdd(13)-qd(13)*(OMcp12_16*ROcp12_36-OMcp12_36*ROcp12_16);
    OPcp12_313 = OPcp12_36+ROcp12_36*qdd(13)+qd(13)*(OMcp12_16*ROcp12_26-OMcp12_26*ROcp12_16);
    ACcp12_113 = qdd(1)+OMcp12_26*ORcp12_313-OMcp12_36*ORcp12_213+OPcp12_26*RLcp12_313-OPcp12_36*RLcp12_213;
    ACcp12_213 = qdd(2)-OMcp12_16*ORcp12_313+OMcp12_36*ORcp12_113-OPcp12_16*RLcp12_313+OPcp12_36*RLcp12_113;
    ACcp12_313 = qdd(3)+OMcp12_16*ORcp12_213-OMcp12_26*ORcp12_113+OPcp12_16*RLcp12_213-OPcp12_26*RLcp12_113;

% = = Block_1_0_0_13_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp12_113;
    sens.P(2) = POcp12_213;
    sens.P(3) = POcp12_313;
    sens.R(1,1) = ROcp12_16;
    sens.R(1,2) = ROcp12_26;
    sens.R(1,3) = ROcp12_36;
    sens.R(2,1) = ROcp12_413;
    sens.R(2,2) = ROcp12_513;
    sens.R(2,3) = ROcp12_613;
    sens.R(3,1) = ROcp12_713;
    sens.R(3,2) = ROcp12_813;
    sens.R(3,3) = ROcp12_913;
    sens.V(1) = VIcp12_113;
    sens.V(2) = VIcp12_213;
    sens.V(3) = VIcp12_313;
    sens.OM(1) = OMcp12_113;
    sens.OM(2) = OMcp12_213;
    sens.OM(3) = OMcp12_313;
    sens.A(1) = ACcp12_113;
    sens.A(2) = ACcp12_213;
    sens.A(3) = ACcp12_313;
    sens.OMP(1) = OPcp12_113;
    sens.OMP(2) = OPcp12_213;
    sens.OMP(3) = OPcp12_313;
 
% 
case 14, 


% = = Block_1_0_0_14_0_1 = = 
 
% Sensor Kinematics 


    ROcp13_45 = -S4*C5;
    ROcp13_55 = C4*C5;
    ROcp13_75 = S4*S5;
    ROcp13_85 = -C4*S5;
    ROcp13_16 = -(ROcp13_75*S6-C4*C6);
    ROcp13_26 = -(ROcp13_85*S6-S4*C6);
    ROcp13_36 = -C5*S6;
    ROcp13_76 = ROcp13_75*C6+C4*S6;
    ROcp13_86 = ROcp13_85*C6+S4*S6;
    ROcp13_96 = C5*C6;
    OMcp13_15 = qd(5)*C4;
    OMcp13_25 = qd(5)*S4;
    OMcp13_16 = OMcp13_15+ROcp13_45*qd(6);
    OMcp13_26 = OMcp13_25+ROcp13_55*qd(6);
    OMcp13_36 = qd(4)+qd(6)*S5;
    OPcp13_16 = ROcp13_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp13_25*S5-ROcp13_55*qd(4));
    OPcp13_26 = ROcp13_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp13_15*S5-ROcp13_45*qd(4));
    OPcp13_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_14_0_4 = = 
 
% Sensor Kinematics 


    ROcp13_413 = ROcp13_45*C13+ROcp13_76*S13;
    ROcp13_513 = ROcp13_55*C13+ROcp13_86*S13;
    ROcp13_613 = ROcp13_96*S13+C13*S5;
    ROcp13_713 = -(ROcp13_45*S13-ROcp13_76*C13);
    ROcp13_813 = -(ROcp13_55*S13-ROcp13_86*C13);
    ROcp13_913 = ROcp13_96*C13-S13*S5;
    ROcp13_114 = ROcp13_16*C14-ROcp13_713*S14;
    ROcp13_214 = ROcp13_26*C14-ROcp13_813*S14;
    ROcp13_314 = ROcp13_36*C14-ROcp13_913*S14;
    ROcp13_714 = ROcp13_16*S14+ROcp13_713*C14;
    ROcp13_814 = ROcp13_26*S14+ROcp13_813*C14;
    ROcp13_914 = ROcp13_36*S14+ROcp13_913*C14;
    RLcp13_113 = ROcp13_16*s.dpt(1,2)+ROcp13_45*s.dpt(2,2)+ROcp13_76*s.dpt(3,2);
    RLcp13_213 = ROcp13_26*s.dpt(1,2)+ROcp13_55*s.dpt(2,2)+ROcp13_86*s.dpt(3,2);
    RLcp13_313 = ROcp13_36*s.dpt(1,2)+ROcp13_96*s.dpt(3,2)+s.dpt(2,2)*S5;
    OMcp13_113 = OMcp13_16+ROcp13_16*qd(13);
    OMcp13_213 = OMcp13_26+ROcp13_26*qd(13);
    OMcp13_313 = OMcp13_36+ROcp13_36*qd(13);
    ORcp13_113 = OMcp13_26*RLcp13_313-OMcp13_36*RLcp13_213;
    ORcp13_213 = -(OMcp13_16*RLcp13_313-OMcp13_36*RLcp13_113);
    ORcp13_313 = OMcp13_16*RLcp13_213-OMcp13_26*RLcp13_113;
    OPcp13_113 = OPcp13_16+ROcp13_16*qdd(13)+qd(13)*(OMcp13_26*ROcp13_36-OMcp13_36*ROcp13_26);
    OPcp13_213 = OPcp13_26+ROcp13_26*qdd(13)-qd(13)*(OMcp13_16*ROcp13_36-OMcp13_36*ROcp13_16);
    OPcp13_313 = OPcp13_36+ROcp13_36*qdd(13)+qd(13)*(OMcp13_16*ROcp13_26-OMcp13_26*ROcp13_16);
    RLcp13_114 = ROcp13_413*s.dpt(2,25);
    RLcp13_214 = ROcp13_513*s.dpt(2,25);
    RLcp13_314 = ROcp13_613*s.dpt(2,25);
    POcp13_114 = RLcp13_113+RLcp13_114+q(1);
    POcp13_214 = RLcp13_213+RLcp13_214+q(2);
    POcp13_314 = RLcp13_313+RLcp13_314+q(3);
    OMcp13_114 = OMcp13_113+ROcp13_413*qd(14);
    OMcp13_214 = OMcp13_213+ROcp13_513*qd(14);
    OMcp13_314 = OMcp13_313+ROcp13_613*qd(14);
    ORcp13_114 = OMcp13_213*RLcp13_314-OMcp13_313*RLcp13_214;
    ORcp13_214 = -(OMcp13_113*RLcp13_314-OMcp13_313*RLcp13_114);
    ORcp13_314 = OMcp13_113*RLcp13_214-OMcp13_213*RLcp13_114;
    VIcp13_114 = ORcp13_113+ORcp13_114+qd(1);
    VIcp13_214 = ORcp13_213+ORcp13_214+qd(2);
    VIcp13_314 = ORcp13_313+ORcp13_314+qd(3);
    OPcp13_114 = OPcp13_113+ROcp13_413*qdd(14)+qd(14)*(OMcp13_213*ROcp13_613-OMcp13_313*ROcp13_513);
    OPcp13_214 = OPcp13_213+ROcp13_513*qdd(14)-qd(14)*(OMcp13_113*ROcp13_613-OMcp13_313*ROcp13_413);
    OPcp13_314 = OPcp13_313+ROcp13_613*qdd(14)+qd(14)*(OMcp13_113*ROcp13_513-OMcp13_213*ROcp13_413);
    ACcp13_114 = qdd(1)+OMcp13_213*ORcp13_314+OMcp13_26*ORcp13_313-OMcp13_313*ORcp13_214-OMcp13_36*ORcp13_213+OPcp13_213*RLcp13_314+OPcp13_26*...
 RLcp13_313-OPcp13_313*RLcp13_214-OPcp13_36*RLcp13_213;
    ACcp13_214 = qdd(2)-OMcp13_113*ORcp13_314-OMcp13_16*ORcp13_313+OMcp13_313*ORcp13_114+OMcp13_36*ORcp13_113-OPcp13_113*RLcp13_314-OPcp13_16*...
 RLcp13_313+OPcp13_313*RLcp13_114+OPcp13_36*RLcp13_113;
    ACcp13_314 = qdd(3)+OMcp13_113*ORcp13_214+OMcp13_16*ORcp13_213-OMcp13_213*ORcp13_114-OMcp13_26*ORcp13_113+OPcp13_113*RLcp13_214+OPcp13_16*...
 RLcp13_213-OPcp13_213*RLcp13_114-OPcp13_26*RLcp13_113;

% = = Block_1_0_0_14_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp13_114;
    sens.P(2) = POcp13_214;
    sens.P(3) = POcp13_314;
    sens.R(1,1) = ROcp13_114;
    sens.R(1,2) = ROcp13_214;
    sens.R(1,3) = ROcp13_314;
    sens.R(2,1) = ROcp13_413;
    sens.R(2,2) = ROcp13_513;
    sens.R(2,3) = ROcp13_613;
    sens.R(3,1) = ROcp13_714;
    sens.R(3,2) = ROcp13_814;
    sens.R(3,3) = ROcp13_914;
    sens.V(1) = VIcp13_114;
    sens.V(2) = VIcp13_214;
    sens.V(3) = VIcp13_314;
    sens.OM(1) = OMcp13_114;
    sens.OM(2) = OMcp13_214;
    sens.OM(3) = OMcp13_314;
    sens.A(1) = ACcp13_114;
    sens.A(2) = ACcp13_214;
    sens.A(3) = ACcp13_314;
    sens.OMP(1) = OPcp13_114;
    sens.OMP(2) = OPcp13_214;
    sens.OMP(3) = OPcp13_314;
 
% 
case 15, 


% = = Block_1_0_0_15_0_1 = = 
 
% Sensor Kinematics 


    ROcp14_45 = -S4*C5;
    ROcp14_55 = C4*C5;
    ROcp14_75 = S4*S5;
    ROcp14_85 = -C4*S5;
    ROcp14_16 = -(ROcp14_75*S6-C4*C6);
    ROcp14_26 = -(ROcp14_85*S6-S4*C6);
    ROcp14_36 = -C5*S6;
    ROcp14_76 = ROcp14_75*C6+C4*S6;
    ROcp14_86 = ROcp14_85*C6+S4*S6;
    ROcp14_96 = C5*C6;
    OMcp14_15 = qd(5)*C4;
    OMcp14_25 = qd(5)*S4;
    OMcp14_16 = OMcp14_15+ROcp14_45*qd(6);
    OMcp14_26 = OMcp14_25+ROcp14_55*qd(6);
    OMcp14_36 = qd(4)+qd(6)*S5;
    OPcp14_16 = ROcp14_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp14_25*S5-ROcp14_55*qd(4));
    OPcp14_26 = ROcp14_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp14_15*S5-ROcp14_45*qd(4));
    OPcp14_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_15_0_4 = = 
 
% Sensor Kinematics 


    ROcp14_413 = ROcp14_45*C13+ROcp14_76*S13;
    ROcp14_513 = ROcp14_55*C13+ROcp14_86*S13;
    ROcp14_613 = ROcp14_96*S13+C13*S5;
    ROcp14_713 = -(ROcp14_45*S13-ROcp14_76*C13);
    ROcp14_813 = -(ROcp14_55*S13-ROcp14_86*C13);
    ROcp14_913 = ROcp14_96*C13-S13*S5;
    ROcp14_114 = ROcp14_16*C14-ROcp14_713*S14;
    ROcp14_214 = ROcp14_26*C14-ROcp14_813*S14;
    ROcp14_314 = ROcp14_36*C14-ROcp14_913*S14;
    ROcp14_714 = ROcp14_16*S14+ROcp14_713*C14;
    ROcp14_814 = ROcp14_26*S14+ROcp14_813*C14;
    ROcp14_914 = ROcp14_36*S14+ROcp14_913*C14;
    ROcp14_415 = ROcp14_413*C15+ROcp14_714*S15;
    ROcp14_515 = ROcp14_513*C15+ROcp14_814*S15;
    ROcp14_615 = ROcp14_613*C15+ROcp14_914*S15;
    ROcp14_715 = -(ROcp14_413*S15-ROcp14_714*C15);
    ROcp14_815 = -(ROcp14_513*S15-ROcp14_814*C15);
    ROcp14_915 = -(ROcp14_613*S15-ROcp14_914*C15);
    RLcp14_113 = ROcp14_16*s.dpt(1,2)+ROcp14_45*s.dpt(2,2)+ROcp14_76*s.dpt(3,2);
    RLcp14_213 = ROcp14_26*s.dpt(1,2)+ROcp14_55*s.dpt(2,2)+ROcp14_86*s.dpt(3,2);
    RLcp14_313 = ROcp14_36*s.dpt(1,2)+ROcp14_96*s.dpt(3,2)+s.dpt(2,2)*S5;
    OMcp14_113 = OMcp14_16+ROcp14_16*qd(13);
    OMcp14_213 = OMcp14_26+ROcp14_26*qd(13);
    OMcp14_313 = OMcp14_36+ROcp14_36*qd(13);
    ORcp14_113 = OMcp14_26*RLcp14_313-OMcp14_36*RLcp14_213;
    ORcp14_213 = -(OMcp14_16*RLcp14_313-OMcp14_36*RLcp14_113);
    ORcp14_313 = OMcp14_16*RLcp14_213-OMcp14_26*RLcp14_113;
    OPcp14_113 = OPcp14_16+ROcp14_16*qdd(13)+qd(13)*(OMcp14_26*ROcp14_36-OMcp14_36*ROcp14_26);
    OPcp14_213 = OPcp14_26+ROcp14_26*qdd(13)-qd(13)*(OMcp14_16*ROcp14_36-OMcp14_36*ROcp14_16);
    OPcp14_313 = OPcp14_36+ROcp14_36*qdd(13)+qd(13)*(OMcp14_16*ROcp14_26-OMcp14_26*ROcp14_16);
    RLcp14_114 = ROcp14_413*s.dpt(2,25);
    RLcp14_214 = ROcp14_513*s.dpt(2,25);
    RLcp14_314 = ROcp14_613*s.dpt(2,25);
    POcp14_114 = RLcp14_113+RLcp14_114+q(1);
    POcp14_214 = RLcp14_213+RLcp14_214+q(2);
    POcp14_314 = RLcp14_313+RLcp14_314+q(3);
    OMcp14_114 = OMcp14_113+ROcp14_413*qd(14);
    OMcp14_214 = OMcp14_213+ROcp14_513*qd(14);
    OMcp14_314 = OMcp14_313+ROcp14_613*qd(14);
    ORcp14_114 = OMcp14_213*RLcp14_314-OMcp14_313*RLcp14_214;
    ORcp14_214 = -(OMcp14_113*RLcp14_314-OMcp14_313*RLcp14_114);
    ORcp14_314 = OMcp14_113*RLcp14_214-OMcp14_213*RLcp14_114;
    VIcp14_114 = ORcp14_113+ORcp14_114+qd(1);
    VIcp14_214 = ORcp14_213+ORcp14_214+qd(2);
    VIcp14_314 = ORcp14_313+ORcp14_314+qd(3);
    ACcp14_114 = qdd(1)+OMcp14_213*ORcp14_314+OMcp14_26*ORcp14_313-OMcp14_313*ORcp14_214-OMcp14_36*ORcp14_213+OPcp14_213*RLcp14_314+OPcp14_26*...
 RLcp14_313-OPcp14_313*RLcp14_214-OPcp14_36*RLcp14_213;
    ACcp14_214 = qdd(2)-OMcp14_113*ORcp14_314-OMcp14_16*ORcp14_313+OMcp14_313*ORcp14_114+OMcp14_36*ORcp14_113-OPcp14_113*RLcp14_314-OPcp14_16*...
 RLcp14_313+OPcp14_313*RLcp14_114+OPcp14_36*RLcp14_113;
    ACcp14_314 = qdd(3)+OMcp14_113*ORcp14_214+OMcp14_16*ORcp14_213-OMcp14_213*ORcp14_114-OMcp14_26*ORcp14_113+OPcp14_113*RLcp14_214+OPcp14_16*...
 RLcp14_213-OPcp14_213*RLcp14_114-OPcp14_26*RLcp14_113;
    OMcp14_115 = OMcp14_114+ROcp14_114*qd(15);
    OMcp14_215 = OMcp14_214+ROcp14_214*qd(15);
    OMcp14_315 = OMcp14_314+ROcp14_314*qd(15);
    OPcp14_115 = OPcp14_113+ROcp14_114*qdd(15)+ROcp14_413*qdd(14)+qd(14)*(OMcp14_213*ROcp14_613-OMcp14_313*ROcp14_513)+qd(15)*(OMcp14_214*...
 ROcp14_314-OMcp14_314*ROcp14_214);
    OPcp14_215 = OPcp14_213+ROcp14_214*qdd(15)+ROcp14_513*qdd(14)-qd(14)*(OMcp14_113*ROcp14_613-OMcp14_313*ROcp14_413)-qd(15)*(OMcp14_114*...
 ROcp14_314-OMcp14_314*ROcp14_114);
    OPcp14_315 = OPcp14_313+ROcp14_314*qdd(15)+ROcp14_613*qdd(14)+qd(14)*(OMcp14_113*ROcp14_513-OMcp14_213*ROcp14_413)+qd(15)*(OMcp14_114*...
 ROcp14_214-OMcp14_214*ROcp14_114);

% = = Block_1_0_0_15_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp14_114;
    sens.P(2) = POcp14_214;
    sens.P(3) = POcp14_314;
    sens.R(1,1) = ROcp14_114;
    sens.R(1,2) = ROcp14_214;
    sens.R(1,3) = ROcp14_314;
    sens.R(2,1) = ROcp14_415;
    sens.R(2,2) = ROcp14_515;
    sens.R(2,3) = ROcp14_615;
    sens.R(3,1) = ROcp14_715;
    sens.R(3,2) = ROcp14_815;
    sens.R(3,3) = ROcp14_915;
    sens.V(1) = VIcp14_114;
    sens.V(2) = VIcp14_214;
    sens.V(3) = VIcp14_314;
    sens.OM(1) = OMcp14_115;
    sens.OM(2) = OMcp14_215;
    sens.OM(3) = OMcp14_315;
    sens.A(1) = ACcp14_114;
    sens.A(2) = ACcp14_214;
    sens.A(3) = ACcp14_314;
    sens.OMP(1) = OPcp14_115;
    sens.OMP(2) = OPcp14_215;
    sens.OMP(3) = OPcp14_315;
 
% 
case 16, 


% = = Block_1_0_0_16_0_1 = = 
 
% Sensor Kinematics 


    ROcp15_45 = -S4*C5;
    ROcp15_55 = C4*C5;
    ROcp15_75 = S4*S5;
    ROcp15_85 = -C4*S5;
    ROcp15_16 = -(ROcp15_75*S6-C4*C6);
    ROcp15_26 = -(ROcp15_85*S6-S4*C6);
    ROcp15_36 = -C5*S6;
    ROcp15_76 = ROcp15_75*C6+C4*S6;
    ROcp15_86 = ROcp15_85*C6+S4*S6;
    ROcp15_96 = C5*C6;
    OMcp15_15 = qd(5)*C4;
    OMcp15_25 = qd(5)*S4;
    OMcp15_16 = OMcp15_15+ROcp15_45*qd(6);
    OMcp15_26 = OMcp15_25+ROcp15_55*qd(6);
    OMcp15_36 = qd(4)+qd(6)*S5;
    OPcp15_16 = ROcp15_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp15_25*S5-ROcp15_55*qd(4));
    OPcp15_26 = ROcp15_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp15_15*S5-ROcp15_45*qd(4));
    OPcp15_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_16_0_4 = = 
 
% Sensor Kinematics 


    ROcp15_413 = ROcp15_45*C13+ROcp15_76*S13;
    ROcp15_513 = ROcp15_55*C13+ROcp15_86*S13;
    ROcp15_613 = ROcp15_96*S13+C13*S5;
    ROcp15_713 = -(ROcp15_45*S13-ROcp15_76*C13);
    ROcp15_813 = -(ROcp15_55*S13-ROcp15_86*C13);
    ROcp15_913 = ROcp15_96*C13-S13*S5;
    ROcp15_114 = ROcp15_16*C14-ROcp15_713*S14;
    ROcp15_214 = ROcp15_26*C14-ROcp15_813*S14;
    ROcp15_314 = ROcp15_36*C14-ROcp15_913*S14;
    ROcp15_714 = ROcp15_16*S14+ROcp15_713*C14;
    ROcp15_814 = ROcp15_26*S14+ROcp15_813*C14;
    ROcp15_914 = ROcp15_36*S14+ROcp15_913*C14;
    ROcp15_415 = ROcp15_413*C15+ROcp15_714*S15;
    ROcp15_515 = ROcp15_513*C15+ROcp15_814*S15;
    ROcp15_615 = ROcp15_613*C15+ROcp15_914*S15;
    ROcp15_715 = -(ROcp15_413*S15-ROcp15_714*C15);
    ROcp15_815 = -(ROcp15_513*S15-ROcp15_814*C15);
    ROcp15_915 = -(ROcp15_613*S15-ROcp15_914*C15);
    ROcp15_116 = ROcp15_114*C16+ROcp15_415*S16;
    ROcp15_216 = ROcp15_214*C16+ROcp15_515*S16;
    ROcp15_316 = ROcp15_314*C16+ROcp15_615*S16;
    ROcp15_416 = -(ROcp15_114*S16-ROcp15_415*C16);
    ROcp15_516 = -(ROcp15_214*S16-ROcp15_515*C16);
    ROcp15_616 = -(ROcp15_314*S16-ROcp15_615*C16);
    RLcp15_113 = ROcp15_16*s.dpt(1,2)+ROcp15_45*s.dpt(2,2)+ROcp15_76*s.dpt(3,2);
    RLcp15_213 = ROcp15_26*s.dpt(1,2)+ROcp15_55*s.dpt(2,2)+ROcp15_86*s.dpt(3,2);
    RLcp15_313 = ROcp15_36*s.dpt(1,2)+ROcp15_96*s.dpt(3,2)+s.dpt(2,2)*S5;
    OMcp15_113 = OMcp15_16+ROcp15_16*qd(13);
    OMcp15_213 = OMcp15_26+ROcp15_26*qd(13);
    OMcp15_313 = OMcp15_36+ROcp15_36*qd(13);
    ORcp15_113 = OMcp15_26*RLcp15_313-OMcp15_36*RLcp15_213;
    ORcp15_213 = -(OMcp15_16*RLcp15_313-OMcp15_36*RLcp15_113);
    ORcp15_313 = OMcp15_16*RLcp15_213-OMcp15_26*RLcp15_113;
    OPcp15_113 = OPcp15_16+ROcp15_16*qdd(13)+qd(13)*(OMcp15_26*ROcp15_36-OMcp15_36*ROcp15_26);
    OPcp15_213 = OPcp15_26+ROcp15_26*qdd(13)-qd(13)*(OMcp15_16*ROcp15_36-OMcp15_36*ROcp15_16);
    OPcp15_313 = OPcp15_36+ROcp15_36*qdd(13)+qd(13)*(OMcp15_16*ROcp15_26-OMcp15_26*ROcp15_16);
    RLcp15_114 = ROcp15_413*s.dpt(2,25);
    RLcp15_214 = ROcp15_513*s.dpt(2,25);
    RLcp15_314 = ROcp15_613*s.dpt(2,25);
    POcp15_114 = RLcp15_113+RLcp15_114+q(1);
    POcp15_214 = RLcp15_213+RLcp15_214+q(2);
    POcp15_314 = RLcp15_313+RLcp15_314+q(3);
    OMcp15_114 = OMcp15_113+ROcp15_413*qd(14);
    OMcp15_214 = OMcp15_213+ROcp15_513*qd(14);
    OMcp15_314 = OMcp15_313+ROcp15_613*qd(14);
    ORcp15_114 = OMcp15_213*RLcp15_314-OMcp15_313*RLcp15_214;
    ORcp15_214 = -(OMcp15_113*RLcp15_314-OMcp15_313*RLcp15_114);
    ORcp15_314 = OMcp15_113*RLcp15_214-OMcp15_213*RLcp15_114;
    VIcp15_114 = ORcp15_113+ORcp15_114+qd(1);
    VIcp15_214 = ORcp15_213+ORcp15_214+qd(2);
    VIcp15_314 = ORcp15_313+ORcp15_314+qd(3);
    ACcp15_114 = qdd(1)+OMcp15_213*ORcp15_314+OMcp15_26*ORcp15_313-OMcp15_313*ORcp15_214-OMcp15_36*ORcp15_213+OPcp15_213*RLcp15_314+OPcp15_26*...
 RLcp15_313-OPcp15_313*RLcp15_214-OPcp15_36*RLcp15_213;
    ACcp15_214 = qdd(2)-OMcp15_113*ORcp15_314-OMcp15_16*ORcp15_313+OMcp15_313*ORcp15_114+OMcp15_36*ORcp15_113-OPcp15_113*RLcp15_314-OPcp15_16*...
 RLcp15_313+OPcp15_313*RLcp15_114+OPcp15_36*RLcp15_113;
    ACcp15_314 = qdd(3)+OMcp15_113*ORcp15_214+OMcp15_16*ORcp15_213-OMcp15_213*ORcp15_114-OMcp15_26*ORcp15_113+OPcp15_113*RLcp15_214+OPcp15_16*...
 RLcp15_213-OPcp15_213*RLcp15_114-OPcp15_26*RLcp15_113;
    OMcp15_115 = OMcp15_114+ROcp15_114*qd(15);
    OMcp15_215 = OMcp15_214+ROcp15_214*qd(15);
    OMcp15_315 = OMcp15_314+ROcp15_314*qd(15);
    OMcp15_116 = OMcp15_115+ROcp15_715*qd(16);
    OMcp15_216 = OMcp15_215+ROcp15_815*qd(16);
    OMcp15_316 = OMcp15_315+ROcp15_915*qd(16);
    OPcp15_116 = OPcp15_113+ROcp15_114*qdd(15)+ROcp15_413*qdd(14)+ROcp15_715*qdd(16)+qd(14)*(OMcp15_213*ROcp15_613-OMcp15_313*ROcp15_513)+qd(15)*(...
 OMcp15_214*ROcp15_314-OMcp15_314*ROcp15_214)+qd(16)*(OMcp15_215*ROcp15_915-OMcp15_315*ROcp15_815);
    OPcp15_216 = OPcp15_213+ROcp15_214*qdd(15)+ROcp15_513*qdd(14)+ROcp15_815*qdd(16)-qd(14)*(OMcp15_113*ROcp15_613-OMcp15_313*ROcp15_413)-qd(15)*(...
 OMcp15_114*ROcp15_314-OMcp15_314*ROcp15_114)-qd(16)*(OMcp15_115*ROcp15_915-OMcp15_315*ROcp15_715);
    OPcp15_316 = OPcp15_313+ROcp15_314*qdd(15)+ROcp15_613*qdd(14)+ROcp15_915*qdd(16)+qd(14)*(OMcp15_113*ROcp15_513-OMcp15_213*ROcp15_413)+qd(15)*(...
 OMcp15_114*ROcp15_214-OMcp15_214*ROcp15_114)+qd(16)*(OMcp15_115*ROcp15_815-OMcp15_215*ROcp15_715);

% = = Block_1_0_0_16_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp15_114;
    sens.P(2) = POcp15_214;
    sens.P(3) = POcp15_314;
    sens.R(1,1) = ROcp15_116;
    sens.R(1,2) = ROcp15_216;
    sens.R(1,3) = ROcp15_316;
    sens.R(2,1) = ROcp15_416;
    sens.R(2,2) = ROcp15_516;
    sens.R(2,3) = ROcp15_616;
    sens.R(3,1) = ROcp15_715;
    sens.R(3,2) = ROcp15_815;
    sens.R(3,3) = ROcp15_915;
    sens.V(1) = VIcp15_114;
    sens.V(2) = VIcp15_214;
    sens.V(3) = VIcp15_314;
    sens.OM(1) = OMcp15_116;
    sens.OM(2) = OMcp15_216;
    sens.OM(3) = OMcp15_316;
    sens.A(1) = ACcp15_114;
    sens.A(2) = ACcp15_214;
    sens.A(3) = ACcp15_314;
    sens.OMP(1) = OPcp15_116;
    sens.OMP(2) = OPcp15_216;
    sens.OMP(3) = OPcp15_316;
 
% 
case 17, 


% = = Block_1_0_0_17_0_1 = = 
 
% Sensor Kinematics 


    ROcp16_45 = -S4*C5;
    ROcp16_55 = C4*C5;
    ROcp16_75 = S4*S5;
    ROcp16_85 = -C4*S5;
    ROcp16_16 = -(ROcp16_75*S6-C4*C6);
    ROcp16_26 = -(ROcp16_85*S6-S4*C6);
    ROcp16_36 = -C5*S6;
    ROcp16_76 = ROcp16_75*C6+C4*S6;
    ROcp16_86 = ROcp16_85*C6+S4*S6;
    ROcp16_96 = C5*C6;
    OMcp16_15 = qd(5)*C4;
    OMcp16_25 = qd(5)*S4;
    OMcp16_16 = OMcp16_15+ROcp16_45*qd(6);
    OMcp16_26 = OMcp16_25+ROcp16_55*qd(6);
    OMcp16_36 = qd(4)+qd(6)*S5;
    OPcp16_16 = ROcp16_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp16_25*S5-ROcp16_55*qd(4));
    OPcp16_26 = ROcp16_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp16_15*S5-ROcp16_45*qd(4));
    OPcp16_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_17_0_4 = = 
 
% Sensor Kinematics 


    ROcp16_413 = ROcp16_45*C13+ROcp16_76*S13;
    ROcp16_513 = ROcp16_55*C13+ROcp16_86*S13;
    ROcp16_613 = ROcp16_96*S13+C13*S5;
    ROcp16_713 = -(ROcp16_45*S13-ROcp16_76*C13);
    ROcp16_813 = -(ROcp16_55*S13-ROcp16_86*C13);
    ROcp16_913 = ROcp16_96*C13-S13*S5;
    ROcp16_114 = ROcp16_16*C14-ROcp16_713*S14;
    ROcp16_214 = ROcp16_26*C14-ROcp16_813*S14;
    ROcp16_314 = ROcp16_36*C14-ROcp16_913*S14;
    ROcp16_714 = ROcp16_16*S14+ROcp16_713*C14;
    ROcp16_814 = ROcp16_26*S14+ROcp16_813*C14;
    ROcp16_914 = ROcp16_36*S14+ROcp16_913*C14;
    ROcp16_415 = ROcp16_413*C15+ROcp16_714*S15;
    ROcp16_515 = ROcp16_513*C15+ROcp16_814*S15;
    ROcp16_615 = ROcp16_613*C15+ROcp16_914*S15;
    ROcp16_715 = -(ROcp16_413*S15-ROcp16_714*C15);
    ROcp16_815 = -(ROcp16_513*S15-ROcp16_814*C15);
    ROcp16_915 = -(ROcp16_613*S15-ROcp16_914*C15);
    ROcp16_116 = ROcp16_114*C16+ROcp16_415*S16;
    ROcp16_216 = ROcp16_214*C16+ROcp16_515*S16;
    ROcp16_316 = ROcp16_314*C16+ROcp16_615*S16;
    ROcp16_416 = -(ROcp16_114*S16-ROcp16_415*C16);
    ROcp16_516 = -(ROcp16_214*S16-ROcp16_515*C16);
    ROcp16_616 = -(ROcp16_314*S16-ROcp16_615*C16);
    ROcp16_117 = ROcp16_116*C17-ROcp16_715*S17;
    ROcp16_217 = ROcp16_216*C17-ROcp16_815*S17;
    ROcp16_317 = ROcp16_316*C17-ROcp16_915*S17;
    ROcp16_717 = ROcp16_116*S17+ROcp16_715*C17;
    ROcp16_817 = ROcp16_216*S17+ROcp16_815*C17;
    ROcp16_917 = ROcp16_316*S17+ROcp16_915*C17;
    RLcp16_113 = ROcp16_16*s.dpt(1,2)+ROcp16_45*s.dpt(2,2)+ROcp16_76*s.dpt(3,2);
    RLcp16_213 = ROcp16_26*s.dpt(1,2)+ROcp16_55*s.dpt(2,2)+ROcp16_86*s.dpt(3,2);
    RLcp16_313 = ROcp16_36*s.dpt(1,2)+ROcp16_96*s.dpt(3,2)+s.dpt(2,2)*S5;
    OMcp16_113 = OMcp16_16+ROcp16_16*qd(13);
    OMcp16_213 = OMcp16_26+ROcp16_26*qd(13);
    OMcp16_313 = OMcp16_36+ROcp16_36*qd(13);
    ORcp16_113 = OMcp16_26*RLcp16_313-OMcp16_36*RLcp16_213;
    ORcp16_213 = -(OMcp16_16*RLcp16_313-OMcp16_36*RLcp16_113);
    ORcp16_313 = OMcp16_16*RLcp16_213-OMcp16_26*RLcp16_113;
    OPcp16_113 = OPcp16_16+ROcp16_16*qdd(13)+qd(13)*(OMcp16_26*ROcp16_36-OMcp16_36*ROcp16_26);
    OPcp16_213 = OPcp16_26+ROcp16_26*qdd(13)-qd(13)*(OMcp16_16*ROcp16_36-OMcp16_36*ROcp16_16);
    OPcp16_313 = OPcp16_36+ROcp16_36*qdd(13)+qd(13)*(OMcp16_16*ROcp16_26-OMcp16_26*ROcp16_16);
    RLcp16_114 = ROcp16_413*s.dpt(2,25);
    RLcp16_214 = ROcp16_513*s.dpt(2,25);
    RLcp16_314 = ROcp16_613*s.dpt(2,25);
    OMcp16_114 = OMcp16_113+ROcp16_413*qd(14);
    OMcp16_214 = OMcp16_213+ROcp16_513*qd(14);
    OMcp16_314 = OMcp16_313+ROcp16_613*qd(14);
    ORcp16_114 = OMcp16_213*RLcp16_314-OMcp16_313*RLcp16_214;
    ORcp16_214 = -(OMcp16_113*RLcp16_314-OMcp16_313*RLcp16_114);
    ORcp16_314 = OMcp16_113*RLcp16_214-OMcp16_213*RLcp16_114;
    OMcp16_115 = OMcp16_114+ROcp16_114*qd(15);
    OMcp16_215 = OMcp16_214+ROcp16_214*qd(15);
    OMcp16_315 = OMcp16_314+ROcp16_314*qd(15);
    OMcp16_116 = OMcp16_115+ROcp16_715*qd(16);
    OMcp16_216 = OMcp16_215+ROcp16_815*qd(16);
    OMcp16_316 = OMcp16_315+ROcp16_915*qd(16);
    OPcp16_116 = OPcp16_113+ROcp16_114*qdd(15)+ROcp16_413*qdd(14)+ROcp16_715*qdd(16)+qd(14)*(OMcp16_213*ROcp16_613-OMcp16_313*ROcp16_513)+qd(15)*(...
 OMcp16_214*ROcp16_314-OMcp16_314*ROcp16_214)+qd(16)*(OMcp16_215*ROcp16_915-OMcp16_315*ROcp16_815);
    OPcp16_216 = OPcp16_213+ROcp16_214*qdd(15)+ROcp16_513*qdd(14)+ROcp16_815*qdd(16)-qd(14)*(OMcp16_113*ROcp16_613-OMcp16_313*ROcp16_413)-qd(15)*(...
 OMcp16_114*ROcp16_314-OMcp16_314*ROcp16_114)-qd(16)*(OMcp16_115*ROcp16_915-OMcp16_315*ROcp16_715);
    OPcp16_316 = OPcp16_313+ROcp16_314*qdd(15)+ROcp16_613*qdd(14)+ROcp16_915*qdd(16)+qd(14)*(OMcp16_113*ROcp16_513-OMcp16_213*ROcp16_413)+qd(15)*(...
 OMcp16_114*ROcp16_214-OMcp16_214*ROcp16_114)+qd(16)*(OMcp16_115*ROcp16_815-OMcp16_215*ROcp16_715);
    RLcp16_117 = ROcp16_715*s.dpt(3,28);
    RLcp16_217 = ROcp16_815*s.dpt(3,28);
    RLcp16_317 = ROcp16_915*s.dpt(3,28);
    POcp16_117 = RLcp16_113+RLcp16_114+RLcp16_117+q(1);
    POcp16_217 = RLcp16_213+RLcp16_214+RLcp16_217+q(2);
    POcp16_317 = RLcp16_313+RLcp16_314+RLcp16_317+q(3);
    OMcp16_117 = OMcp16_116+ROcp16_416*qd(17);
    OMcp16_217 = OMcp16_216+ROcp16_516*qd(17);
    OMcp16_317 = OMcp16_316+ROcp16_616*qd(17);
    ORcp16_117 = OMcp16_216*RLcp16_317-OMcp16_316*RLcp16_217;
    ORcp16_217 = -(OMcp16_116*RLcp16_317-OMcp16_316*RLcp16_117);
    ORcp16_317 = OMcp16_116*RLcp16_217-OMcp16_216*RLcp16_117;
    VIcp16_117 = ORcp16_113+ORcp16_114+ORcp16_117+qd(1);
    VIcp16_217 = ORcp16_213+ORcp16_214+ORcp16_217+qd(2);
    VIcp16_317 = ORcp16_313+ORcp16_314+ORcp16_317+qd(3);
    OPcp16_117 = OPcp16_116+ROcp16_416*qdd(17)+qd(17)*(OMcp16_216*ROcp16_616-OMcp16_316*ROcp16_516);
    OPcp16_217 = OPcp16_216+ROcp16_516*qdd(17)-qd(17)*(OMcp16_116*ROcp16_616-OMcp16_316*ROcp16_416);
    OPcp16_317 = OPcp16_316+ROcp16_616*qdd(17)+qd(17)*(OMcp16_116*ROcp16_516-OMcp16_216*ROcp16_416);
    ACcp16_117 = qdd(1)+OMcp16_213*ORcp16_314+OMcp16_216*ORcp16_317+OMcp16_26*ORcp16_313-OMcp16_313*ORcp16_214-OMcp16_316*ORcp16_217-OMcp16_36*...
 ORcp16_213+OPcp16_213*RLcp16_314+OPcp16_216*RLcp16_317+OPcp16_26*RLcp16_313-OPcp16_313*RLcp16_214-OPcp16_316*RLcp16_217-OPcp16_36*RLcp16_213;
    ACcp16_217 = qdd(2)-OMcp16_113*ORcp16_314-OMcp16_116*ORcp16_317-OMcp16_16*ORcp16_313+OMcp16_313*ORcp16_114+OMcp16_316*ORcp16_117+OMcp16_36*...
 ORcp16_113-OPcp16_113*RLcp16_314-OPcp16_116*RLcp16_317-OPcp16_16*RLcp16_313+OPcp16_313*RLcp16_114+OPcp16_316*RLcp16_117+OPcp16_36*RLcp16_113;
    ACcp16_317 = qdd(3)+OMcp16_113*ORcp16_214+OMcp16_116*ORcp16_217+OMcp16_16*ORcp16_213-OMcp16_213*ORcp16_114-OMcp16_216*ORcp16_117-OMcp16_26*...
 ORcp16_113+OPcp16_113*RLcp16_214+OPcp16_116*RLcp16_217+OPcp16_16*RLcp16_213-OPcp16_213*RLcp16_114-OPcp16_216*RLcp16_117-OPcp16_26*RLcp16_113;

% = = Block_1_0_0_17_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp16_117;
    sens.P(2) = POcp16_217;
    sens.P(3) = POcp16_317;
    sens.R(1,1) = ROcp16_117;
    sens.R(1,2) = ROcp16_217;
    sens.R(1,3) = ROcp16_317;
    sens.R(2,1) = ROcp16_416;
    sens.R(2,2) = ROcp16_516;
    sens.R(2,3) = ROcp16_616;
    sens.R(3,1) = ROcp16_717;
    sens.R(3,2) = ROcp16_817;
    sens.R(3,3) = ROcp16_917;
    sens.V(1) = VIcp16_117;
    sens.V(2) = VIcp16_217;
    sens.V(3) = VIcp16_317;
    sens.OM(1) = OMcp16_117;
    sens.OM(2) = OMcp16_217;
    sens.OM(3) = OMcp16_317;
    sens.A(1) = ACcp16_117;
    sens.A(2) = ACcp16_217;
    sens.A(3) = ACcp16_317;
    sens.OMP(1) = OPcp16_117;
    sens.OMP(2) = OPcp16_217;
    sens.OMP(3) = OPcp16_317;
 
% 
case 18, 


% = = Block_1_0_0_18_0_1 = = 
 
% Sensor Kinematics 


    ROcp17_45 = -S4*C5;
    ROcp17_55 = C4*C5;
    ROcp17_75 = S4*S5;
    ROcp17_85 = -C4*S5;
    ROcp17_16 = -(ROcp17_75*S6-C4*C6);
    ROcp17_26 = -(ROcp17_85*S6-S4*C6);
    ROcp17_36 = -C5*S6;
    ROcp17_76 = ROcp17_75*C6+C4*S6;
    ROcp17_86 = ROcp17_85*C6+S4*S6;
    ROcp17_96 = C5*C6;
    OMcp17_15 = qd(5)*C4;
    OMcp17_25 = qd(5)*S4;
    OMcp17_16 = OMcp17_15+ROcp17_45*qd(6);
    OMcp17_26 = OMcp17_25+ROcp17_55*qd(6);
    OMcp17_36 = qd(4)+qd(6)*S5;
    OPcp17_16 = ROcp17_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp17_25*S5-ROcp17_55*qd(4));
    OPcp17_26 = ROcp17_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp17_15*S5-ROcp17_45*qd(4));
    OPcp17_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_18_0_5 = = 
 
% Sensor Kinematics 


    ROcp17_418 = ROcp17_45*C18+ROcp17_76*S18;
    ROcp17_518 = ROcp17_55*C18+ROcp17_86*S18;
    ROcp17_618 = ROcp17_96*S18+C18*S5;
    ROcp17_718 = -(ROcp17_45*S18-ROcp17_76*C18);
    ROcp17_818 = -(ROcp17_55*S18-ROcp17_86*C18);
    ROcp17_918 = ROcp17_96*C18-S18*S5;
    RLcp17_118 = ROcp17_45*s.dpt(2,3);
    RLcp17_218 = ROcp17_55*s.dpt(2,3);
    RLcp17_318 = s.dpt(2,3)*S5;
    POcp17_118 = RLcp17_118+q(1);
    POcp17_218 = RLcp17_218+q(2);
    POcp17_318 = RLcp17_318+q(3);
    OMcp17_118 = OMcp17_16+ROcp17_16*qd(18);
    OMcp17_218 = OMcp17_26+ROcp17_26*qd(18);
    OMcp17_318 = OMcp17_36+ROcp17_36*qd(18);
    ORcp17_118 = OMcp17_26*RLcp17_318-OMcp17_36*RLcp17_218;
    ORcp17_218 = -(OMcp17_16*RLcp17_318-OMcp17_36*RLcp17_118);
    ORcp17_318 = OMcp17_16*RLcp17_218-OMcp17_26*RLcp17_118;
    VIcp17_118 = ORcp17_118+qd(1);
    VIcp17_218 = ORcp17_218+qd(2);
    VIcp17_318 = ORcp17_318+qd(3);
    OPcp17_118 = OPcp17_16+ROcp17_16*qdd(18)+qd(18)*(OMcp17_26*ROcp17_36-OMcp17_36*ROcp17_26);
    OPcp17_218 = OPcp17_26+ROcp17_26*qdd(18)-qd(18)*(OMcp17_16*ROcp17_36-OMcp17_36*ROcp17_16);
    OPcp17_318 = OPcp17_36+ROcp17_36*qdd(18)+qd(18)*(OMcp17_16*ROcp17_26-OMcp17_26*ROcp17_16);
    ACcp17_118 = qdd(1)+OMcp17_26*ORcp17_318-OMcp17_36*ORcp17_218+OPcp17_26*RLcp17_318-OPcp17_36*RLcp17_218;
    ACcp17_218 = qdd(2)-OMcp17_16*ORcp17_318+OMcp17_36*ORcp17_118-OPcp17_16*RLcp17_318+OPcp17_36*RLcp17_118;
    ACcp17_318 = qdd(3)+OMcp17_16*ORcp17_218-OMcp17_26*ORcp17_118+OPcp17_16*RLcp17_218-OPcp17_26*RLcp17_118;

% = = Block_1_0_0_18_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp17_118;
    sens.P(2) = POcp17_218;
    sens.P(3) = POcp17_318;
    sens.R(1,1) = ROcp17_16;
    sens.R(1,2) = ROcp17_26;
    sens.R(1,3) = ROcp17_36;
    sens.R(2,1) = ROcp17_418;
    sens.R(2,2) = ROcp17_518;
    sens.R(2,3) = ROcp17_618;
    sens.R(3,1) = ROcp17_718;
    sens.R(3,2) = ROcp17_818;
    sens.R(3,3) = ROcp17_918;
    sens.V(1) = VIcp17_118;
    sens.V(2) = VIcp17_218;
    sens.V(3) = VIcp17_318;
    sens.OM(1) = OMcp17_118;
    sens.OM(2) = OMcp17_218;
    sens.OM(3) = OMcp17_318;
    sens.A(1) = ACcp17_118;
    sens.A(2) = ACcp17_218;
    sens.A(3) = ACcp17_318;
    sens.OMP(1) = OPcp17_118;
    sens.OMP(2) = OPcp17_218;
    sens.OMP(3) = OPcp17_318;
 
% 
case 19, 


% = = Block_1_0_0_19_0_1 = = 
 
% Sensor Kinematics 


    ROcp18_45 = -S4*C5;
    ROcp18_55 = C4*C5;
    ROcp18_75 = S4*S5;
    ROcp18_85 = -C4*S5;
    ROcp18_16 = -(ROcp18_75*S6-C4*C6);
    ROcp18_26 = -(ROcp18_85*S6-S4*C6);
    ROcp18_36 = -C5*S6;
    ROcp18_76 = ROcp18_75*C6+C4*S6;
    ROcp18_86 = ROcp18_85*C6+S4*S6;
    ROcp18_96 = C5*C6;
    OMcp18_15 = qd(5)*C4;
    OMcp18_25 = qd(5)*S4;
    OMcp18_16 = OMcp18_15+ROcp18_45*qd(6);
    OMcp18_26 = OMcp18_25+ROcp18_55*qd(6);
    OMcp18_36 = qd(4)+qd(6)*S5;
    OPcp18_16 = ROcp18_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp18_25*S5-ROcp18_55*qd(4));
    OPcp18_26 = ROcp18_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp18_15*S5-ROcp18_45*qd(4));
    OPcp18_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_19_0_6 = = 
 
% Sensor Kinematics 


    ROcp18_419 = ROcp18_45*C19+ROcp18_76*S19;
    ROcp18_519 = ROcp18_55*C19+ROcp18_86*S19;
    ROcp18_619 = ROcp18_96*S19+C19*S5;
    ROcp18_719 = -(ROcp18_45*S19-ROcp18_76*C19);
    ROcp18_819 = -(ROcp18_55*S19-ROcp18_86*C19);
    ROcp18_919 = ROcp18_96*C19-S19*S5;
    RLcp18_119 = ROcp18_45*s.dpt(2,4);
    RLcp18_219 = ROcp18_55*s.dpt(2,4);
    RLcp18_319 = s.dpt(2,4)*S5;
    POcp18_119 = RLcp18_119+q(1);
    POcp18_219 = RLcp18_219+q(2);
    POcp18_319 = RLcp18_319+q(3);
    OMcp18_119 = OMcp18_16+ROcp18_16*qd(19);
    OMcp18_219 = OMcp18_26+ROcp18_26*qd(19);
    OMcp18_319 = OMcp18_36+ROcp18_36*qd(19);
    ORcp18_119 = OMcp18_26*RLcp18_319-OMcp18_36*RLcp18_219;
    ORcp18_219 = -(OMcp18_16*RLcp18_319-OMcp18_36*RLcp18_119);
    ORcp18_319 = OMcp18_16*RLcp18_219-OMcp18_26*RLcp18_119;
    VIcp18_119 = ORcp18_119+qd(1);
    VIcp18_219 = ORcp18_219+qd(2);
    VIcp18_319 = ORcp18_319+qd(3);
    OPcp18_119 = OPcp18_16+ROcp18_16*qdd(19)+qd(19)*(OMcp18_26*ROcp18_36-OMcp18_36*ROcp18_26);
    OPcp18_219 = OPcp18_26+ROcp18_26*qdd(19)-qd(19)*(OMcp18_16*ROcp18_36-OMcp18_36*ROcp18_16);
    OPcp18_319 = OPcp18_36+ROcp18_36*qdd(19)+qd(19)*(OMcp18_16*ROcp18_26-OMcp18_26*ROcp18_16);
    ACcp18_119 = qdd(1)+OMcp18_26*ORcp18_319-OMcp18_36*ORcp18_219+OPcp18_26*RLcp18_319-OPcp18_36*RLcp18_219;
    ACcp18_219 = qdd(2)-OMcp18_16*ORcp18_319+OMcp18_36*ORcp18_119-OPcp18_16*RLcp18_319+OPcp18_36*RLcp18_119;
    ACcp18_319 = qdd(3)+OMcp18_16*ORcp18_219-OMcp18_26*ORcp18_119+OPcp18_16*RLcp18_219-OPcp18_26*RLcp18_119;

% = = Block_1_0_0_19_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp18_119;
    sens.P(2) = POcp18_219;
    sens.P(3) = POcp18_319;
    sens.R(1,1) = ROcp18_16;
    sens.R(1,2) = ROcp18_26;
    sens.R(1,3) = ROcp18_36;
    sens.R(2,1) = ROcp18_419;
    sens.R(2,2) = ROcp18_519;
    sens.R(2,3) = ROcp18_619;
    sens.R(3,1) = ROcp18_719;
    sens.R(3,2) = ROcp18_819;
    sens.R(3,3) = ROcp18_919;
    sens.V(1) = VIcp18_119;
    sens.V(2) = VIcp18_219;
    sens.V(3) = VIcp18_319;
    sens.OM(1) = OMcp18_119;
    sens.OM(2) = OMcp18_219;
    sens.OM(3) = OMcp18_319;
    sens.A(1) = ACcp18_119;
    sens.A(2) = ACcp18_219;
    sens.A(3) = ACcp18_319;
    sens.OMP(1) = OPcp18_119;
    sens.OMP(2) = OPcp18_219;
    sens.OMP(3) = OPcp18_319;
 
% 
case 20, 


% = = Block_1_0_0_20_0_1 = = 
 
% Sensor Kinematics 


    ROcp19_45 = -S4*C5;
    ROcp19_55 = C4*C5;
    ROcp19_75 = S4*S5;
    ROcp19_85 = -C4*S5;
    ROcp19_16 = -(ROcp19_75*S6-C4*C6);
    ROcp19_26 = -(ROcp19_85*S6-S4*C6);
    ROcp19_36 = -C5*S6;
    ROcp19_76 = ROcp19_75*C6+C4*S6;
    ROcp19_86 = ROcp19_85*C6+S4*S6;
    ROcp19_96 = C5*C6;
    OMcp19_15 = qd(5)*C4;
    OMcp19_25 = qd(5)*S4;
    OMcp19_16 = OMcp19_15+ROcp19_45*qd(6);
    OMcp19_26 = OMcp19_25+ROcp19_55*qd(6);
    OMcp19_36 = qd(4)+qd(6)*S5;
    OPcp19_16 = ROcp19_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp19_25*S5-ROcp19_55*qd(4));
    OPcp19_26 = ROcp19_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp19_15*S5-ROcp19_45*qd(4));
    OPcp19_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_20_0_7 = = 
 
% Sensor Kinematics 


    ROcp19_420 = ROcp19_45*C20+ROcp19_76*S20;
    ROcp19_520 = ROcp19_55*C20+ROcp19_86*S20;
    ROcp19_620 = ROcp19_96*S20+C20*S5;
    ROcp19_720 = -(ROcp19_45*S20-ROcp19_76*C20);
    ROcp19_820 = -(ROcp19_55*S20-ROcp19_86*C20);
    ROcp19_920 = ROcp19_96*C20-S20*S5;
    RLcp19_120 = ROcp19_16*s.dpt(1,5)+ROcp19_45*s.dpt(2,5);
    RLcp19_220 = ROcp19_26*s.dpt(1,5)+ROcp19_55*s.dpt(2,5);
    RLcp19_320 = ROcp19_36*s.dpt(1,5)+s.dpt(2,5)*S5;
    POcp19_120 = RLcp19_120+q(1);
    POcp19_220 = RLcp19_220+q(2);
    POcp19_320 = RLcp19_320+q(3);
    OMcp19_120 = OMcp19_16+ROcp19_16*qd(20);
    OMcp19_220 = OMcp19_26+ROcp19_26*qd(20);
    OMcp19_320 = OMcp19_36+ROcp19_36*qd(20);
    ORcp19_120 = OMcp19_26*RLcp19_320-OMcp19_36*RLcp19_220;
    ORcp19_220 = -(OMcp19_16*RLcp19_320-OMcp19_36*RLcp19_120);
    ORcp19_320 = OMcp19_16*RLcp19_220-OMcp19_26*RLcp19_120;
    VIcp19_120 = ORcp19_120+qd(1);
    VIcp19_220 = ORcp19_220+qd(2);
    VIcp19_320 = ORcp19_320+qd(3);
    OPcp19_120 = OPcp19_16+ROcp19_16*qdd(20)+qd(20)*(OMcp19_26*ROcp19_36-OMcp19_36*ROcp19_26);
    OPcp19_220 = OPcp19_26+ROcp19_26*qdd(20)-qd(20)*(OMcp19_16*ROcp19_36-OMcp19_36*ROcp19_16);
    OPcp19_320 = OPcp19_36+ROcp19_36*qdd(20)+qd(20)*(OMcp19_16*ROcp19_26-OMcp19_26*ROcp19_16);
    ACcp19_120 = qdd(1)+OMcp19_26*ORcp19_320-OMcp19_36*ORcp19_220+OPcp19_26*RLcp19_320-OPcp19_36*RLcp19_220;
    ACcp19_220 = qdd(2)-OMcp19_16*ORcp19_320+OMcp19_36*ORcp19_120-OPcp19_16*RLcp19_320+OPcp19_36*RLcp19_120;
    ACcp19_320 = qdd(3)+OMcp19_16*ORcp19_220-OMcp19_26*ORcp19_120+OPcp19_16*RLcp19_220-OPcp19_26*RLcp19_120;

% = = Block_1_0_0_20_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp19_120;
    sens.P(2) = POcp19_220;
    sens.P(3) = POcp19_320;
    sens.R(1,1) = ROcp19_16;
    sens.R(1,2) = ROcp19_26;
    sens.R(1,3) = ROcp19_36;
    sens.R(2,1) = ROcp19_420;
    sens.R(2,2) = ROcp19_520;
    sens.R(2,3) = ROcp19_620;
    sens.R(3,1) = ROcp19_720;
    sens.R(3,2) = ROcp19_820;
    sens.R(3,3) = ROcp19_920;
    sens.V(1) = VIcp19_120;
    sens.V(2) = VIcp19_220;
    sens.V(3) = VIcp19_320;
    sens.OM(1) = OMcp19_120;
    sens.OM(2) = OMcp19_220;
    sens.OM(3) = OMcp19_320;
    sens.A(1) = ACcp19_120;
    sens.A(2) = ACcp19_220;
    sens.A(3) = ACcp19_320;
    sens.OMP(1) = OPcp19_120;
    sens.OMP(2) = OPcp19_220;
    sens.OMP(3) = OPcp19_320;
 
% 
case 21, 


% = = Block_1_0_0_21_0_1 = = 
 
% Sensor Kinematics 


    ROcp20_45 = -S4*C5;
    ROcp20_55 = C4*C5;
    ROcp20_75 = S4*S5;
    ROcp20_85 = -C4*S5;
    ROcp20_16 = -(ROcp20_75*S6-C4*C6);
    ROcp20_26 = -(ROcp20_85*S6-S4*C6);
    ROcp20_36 = -C5*S6;
    ROcp20_76 = ROcp20_75*C6+C4*S6;
    ROcp20_86 = ROcp20_85*C6+S4*S6;
    ROcp20_96 = C5*C6;
    OMcp20_15 = qd(5)*C4;
    OMcp20_25 = qd(5)*S4;
    OMcp20_16 = OMcp20_15+ROcp20_45*qd(6);
    OMcp20_26 = OMcp20_25+ROcp20_55*qd(6);
    OMcp20_36 = qd(4)+qd(6)*S5;
    OPcp20_16 = ROcp20_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp20_25*S5-ROcp20_55*qd(4));
    OPcp20_26 = ROcp20_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp20_15*S5-ROcp20_45*qd(4));
    OPcp20_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_21_0_8 = = 
 
% Sensor Kinematics 


    ROcp20_421 = ROcp20_45*C21+ROcp20_76*S21;
    ROcp20_521 = ROcp20_55*C21+ROcp20_86*S21;
    ROcp20_621 = ROcp20_96*S21+C21*S5;
    ROcp20_721 = -(ROcp20_45*S21-ROcp20_76*C21);
    ROcp20_821 = -(ROcp20_55*S21-ROcp20_86*C21);
    ROcp20_921 = ROcp20_96*C21-S21*S5;
    RLcp20_121 = ROcp20_16*s.dpt(1,7)+ROcp20_45*s.dpt(2,7);
    RLcp20_221 = ROcp20_26*s.dpt(1,7)+ROcp20_55*s.dpt(2,7);
    RLcp20_321 = ROcp20_36*s.dpt(1,7)+s.dpt(2,7)*S5;
    POcp20_121 = RLcp20_121+q(1);
    POcp20_221 = RLcp20_221+q(2);
    POcp20_321 = RLcp20_321+q(3);
    OMcp20_121 = OMcp20_16+ROcp20_16*qd(21);
    OMcp20_221 = OMcp20_26+ROcp20_26*qd(21);
    OMcp20_321 = OMcp20_36+ROcp20_36*qd(21);
    ORcp20_121 = OMcp20_26*RLcp20_321-OMcp20_36*RLcp20_221;
    ORcp20_221 = -(OMcp20_16*RLcp20_321-OMcp20_36*RLcp20_121);
    ORcp20_321 = OMcp20_16*RLcp20_221-OMcp20_26*RLcp20_121;
    VIcp20_121 = ORcp20_121+qd(1);
    VIcp20_221 = ORcp20_221+qd(2);
    VIcp20_321 = ORcp20_321+qd(3);
    OPcp20_121 = OPcp20_16+ROcp20_16*qdd(21)+qd(21)*(OMcp20_26*ROcp20_36-OMcp20_36*ROcp20_26);
    OPcp20_221 = OPcp20_26+ROcp20_26*qdd(21)-qd(21)*(OMcp20_16*ROcp20_36-OMcp20_36*ROcp20_16);
    OPcp20_321 = OPcp20_36+ROcp20_36*qdd(21)+qd(21)*(OMcp20_16*ROcp20_26-OMcp20_26*ROcp20_16);
    ACcp20_121 = qdd(1)+OMcp20_26*ORcp20_321-OMcp20_36*ORcp20_221+OPcp20_26*RLcp20_321-OPcp20_36*RLcp20_221;
    ACcp20_221 = qdd(2)-OMcp20_16*ORcp20_321+OMcp20_36*ORcp20_121-OPcp20_16*RLcp20_321+OPcp20_36*RLcp20_121;
    ACcp20_321 = qdd(3)+OMcp20_16*ORcp20_221-OMcp20_26*ORcp20_121+OPcp20_16*RLcp20_221-OPcp20_26*RLcp20_121;

% = = Block_1_0_0_21_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp20_121;
    sens.P(2) = POcp20_221;
    sens.P(3) = POcp20_321;
    sens.R(1,1) = ROcp20_16;
    sens.R(1,2) = ROcp20_26;
    sens.R(1,3) = ROcp20_36;
    sens.R(2,1) = ROcp20_421;
    sens.R(2,2) = ROcp20_521;
    sens.R(2,3) = ROcp20_621;
    sens.R(3,1) = ROcp20_721;
    sens.R(3,2) = ROcp20_821;
    sens.R(3,3) = ROcp20_921;
    sens.V(1) = VIcp20_121;
    sens.V(2) = VIcp20_221;
    sens.V(3) = VIcp20_321;
    sens.OM(1) = OMcp20_121;
    sens.OM(2) = OMcp20_221;
    sens.OM(3) = OMcp20_321;
    sens.A(1) = ACcp20_121;
    sens.A(2) = ACcp20_221;
    sens.A(3) = ACcp20_321;
    sens.OMP(1) = OPcp20_121;
    sens.OMP(2) = OPcp20_221;
    sens.OMP(3) = OPcp20_321;
 
% 
case 22, 


% = = Block_1_0_0_22_0_1 = = 
 
% Sensor Kinematics 


    ROcp21_45 = -S4*C5;
    ROcp21_55 = C4*C5;
    ROcp21_75 = S4*S5;
    ROcp21_85 = -C4*S5;
    ROcp21_16 = -(ROcp21_75*S6-C4*C6);
    ROcp21_26 = -(ROcp21_85*S6-S4*C6);
    ROcp21_76 = ROcp21_75*C6+C4*S6;
    ROcp21_86 = ROcp21_85*C6+S4*S6;
    OMcp21_15 = qd(5)*C4;
    OMcp21_25 = qd(5)*S4;
    OMcp21_16 = OMcp21_15+ROcp21_45*qd(6);
    OMcp21_26 = OMcp21_25+ROcp21_55*qd(6);
    OMcp21_36 = qd(4)+qd(6)*S5;
    OPcp21_16 = ROcp21_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp21_25*S5-ROcp21_55*qd(4));
    OPcp21_26 = ROcp21_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp21_15*S5-ROcp21_45*qd(4));
    OPcp21_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_22_0_9 = = 
 
% Sensor Kinematics 


    ROcp21_122 = ROcp21_16*C22-ROcp21_76*S22;
    ROcp21_222 = ROcp21_26*C22-ROcp21_86*S22;
    ROcp21_322 = -C5*S22p6;
    ROcp21_722 = ROcp21_16*S22+ROcp21_76*C22;
    ROcp21_822 = ROcp21_26*S22+ROcp21_86*C22;
    ROcp21_922 = C5*C22p6;
    RLcp21_122 = ROcp21_16*s.dpt(1,8)+ROcp21_76*s.dpt(3,8);
    RLcp21_222 = ROcp21_26*s.dpt(1,8)+ROcp21_86*s.dpt(3,8);
    RLcp21_322 = -C5*(s.dpt(1,8)*S6-s.dpt(3,8)*C6);
    POcp21_122 = RLcp21_122+q(1);
    POcp21_222 = RLcp21_222+q(2);
    POcp21_322 = RLcp21_322+q(3);
    OMcp21_122 = OMcp21_16+ROcp21_45*qd(22);
    OMcp21_222 = OMcp21_26+ROcp21_55*qd(22);
    OMcp21_322 = OMcp21_36+qd(22)*S5;
    ORcp21_122 = OMcp21_26*RLcp21_322-OMcp21_36*RLcp21_222;
    ORcp21_222 = -(OMcp21_16*RLcp21_322-OMcp21_36*RLcp21_122);
    ORcp21_322 = OMcp21_16*RLcp21_222-OMcp21_26*RLcp21_122;
    VIcp21_122 = ORcp21_122+qd(1);
    VIcp21_222 = ORcp21_222+qd(2);
    VIcp21_322 = ORcp21_322+qd(3);
    OPcp21_122 = OPcp21_16+ROcp21_45*qdd(22)+qd(22)*(OMcp21_26*S5-OMcp21_36*ROcp21_55);
    OPcp21_222 = OPcp21_26+ROcp21_55*qdd(22)-qd(22)*(OMcp21_16*S5-OMcp21_36*ROcp21_45);
    OPcp21_322 = OPcp21_36+qdd(22)*S5+qd(22)*(OMcp21_16*ROcp21_55-OMcp21_26*ROcp21_45);
    ACcp21_122 = qdd(1)+OMcp21_26*ORcp21_322-OMcp21_36*ORcp21_222+OPcp21_26*RLcp21_322-OPcp21_36*RLcp21_222;
    ACcp21_222 = qdd(2)-OMcp21_16*ORcp21_322+OMcp21_36*ORcp21_122-OPcp21_16*RLcp21_322+OPcp21_36*RLcp21_122;
    ACcp21_322 = qdd(3)+OMcp21_16*ORcp21_222-OMcp21_26*ORcp21_122+OPcp21_16*RLcp21_222-OPcp21_26*RLcp21_122;

% = = Block_1_0_0_22_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp21_122;
    sens.P(2) = POcp21_222;
    sens.P(3) = POcp21_322;
    sens.R(1,1) = ROcp21_122;
    sens.R(1,2) = ROcp21_222;
    sens.R(1,3) = ROcp21_322;
    sens.R(2,1) = ROcp21_45;
    sens.R(2,2) = ROcp21_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp21_722;
    sens.R(3,2) = ROcp21_822;
    sens.R(3,3) = ROcp21_922;
    sens.V(1) = VIcp21_122;
    sens.V(2) = VIcp21_222;
    sens.V(3) = VIcp21_322;
    sens.OM(1) = OMcp21_122;
    sens.OM(2) = OMcp21_222;
    sens.OM(3) = OMcp21_322;
    sens.A(1) = ACcp21_122;
    sens.A(2) = ACcp21_222;
    sens.A(3) = ACcp21_322;
    sens.OMP(1) = OPcp21_122;
    sens.OMP(2) = OPcp21_222;
    sens.OMP(3) = OPcp21_322;
 
% 
case 23, 


% = = Block_1_0_0_23_0_1 = = 
 
% Sensor Kinematics 


    ROcp22_45 = -S4*C5;
    ROcp22_55 = C4*C5;
    ROcp22_75 = S4*S5;
    ROcp22_85 = -C4*S5;
    ROcp22_16 = -(ROcp22_75*S6-C4*C6);
    ROcp22_26 = -(ROcp22_85*S6-S4*C6);
    ROcp22_76 = ROcp22_75*C6+C4*S6;
    ROcp22_86 = ROcp22_85*C6+S4*S6;
    OMcp22_15 = qd(5)*C4;
    OMcp22_25 = qd(5)*S4;
    OMcp22_16 = OMcp22_15+ROcp22_45*qd(6);
    OMcp22_26 = OMcp22_25+ROcp22_55*qd(6);
    OMcp22_36 = qd(4)+qd(6)*S5;
    OPcp22_16 = ROcp22_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp22_25*S5-ROcp22_55*qd(4));
    OPcp22_26 = ROcp22_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp22_15*S5-ROcp22_45*qd(4));
    OPcp22_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_23_0_9 = = 
 
% Sensor Kinematics 


    ROcp22_122 = ROcp22_16*C22-ROcp22_76*S22;
    ROcp22_222 = ROcp22_26*C22-ROcp22_86*S22;
    ROcp22_322 = -C5*S22p6;
    ROcp22_722 = ROcp22_16*S22+ROcp22_76*C22;
    ROcp22_822 = ROcp22_26*S22+ROcp22_86*C22;
    ROcp22_922 = C5*C22p6;
    RLcp22_122 = ROcp22_16*s.dpt(1,8)+ROcp22_76*s.dpt(3,8);
    RLcp22_222 = ROcp22_26*s.dpt(1,8)+ROcp22_86*s.dpt(3,8);
    RLcp22_322 = -C5*(s.dpt(1,8)*S6-s.dpt(3,8)*C6);
    OMcp22_122 = OMcp22_16+ROcp22_45*qd(22);
    OMcp22_222 = OMcp22_26+ROcp22_55*qd(22);
    OMcp22_322 = OMcp22_36+qd(22)*S5;
    ORcp22_122 = OMcp22_26*RLcp22_322-OMcp22_36*RLcp22_222;
    ORcp22_222 = -(OMcp22_16*RLcp22_322-OMcp22_36*RLcp22_122);
    ORcp22_322 = OMcp22_16*RLcp22_222-OMcp22_26*RLcp22_122;
    OPcp22_122 = OPcp22_16+ROcp22_45*qdd(22)+qd(22)*(OMcp22_26*S5-OMcp22_36*ROcp22_55);
    OPcp22_222 = OPcp22_26+ROcp22_55*qdd(22)-qd(22)*(OMcp22_16*S5-OMcp22_36*ROcp22_45);
    OPcp22_322 = OPcp22_36+qdd(22)*S5+qd(22)*(OMcp22_16*ROcp22_55-OMcp22_26*ROcp22_45);

% = = Block_1_0_0_23_0_10 = = 
 
% Sensor Kinematics 


    ROcp22_423 = ROcp22_45*C23+ROcp22_722*S23;
    ROcp22_523 = ROcp22_55*C23+ROcp22_822*S23;
    ROcp22_623 = ROcp22_922*S23+C23*S5;
    ROcp22_723 = -(ROcp22_45*S23-ROcp22_722*C23);
    ROcp22_823 = -(ROcp22_55*S23-ROcp22_822*C23);
    ROcp22_923 = ROcp22_922*C23-S23*S5;
    RLcp22_123 = ROcp22_122*s.dpt(1,39)+ROcp22_45*s.dpt(2,39);
    RLcp22_223 = ROcp22_222*s.dpt(1,39)+ROcp22_55*s.dpt(2,39);
    RLcp22_323 = ROcp22_322*s.dpt(1,39)+s.dpt(2,39)*S5;
    POcp22_123 = RLcp22_122+RLcp22_123+q(1);
    POcp22_223 = RLcp22_222+RLcp22_223+q(2);
    POcp22_323 = RLcp22_322+RLcp22_323+q(3);
    OMcp22_123 = OMcp22_122+ROcp22_122*qd(23);
    OMcp22_223 = OMcp22_222+ROcp22_222*qd(23);
    OMcp22_323 = OMcp22_322+ROcp22_322*qd(23);
    ORcp22_123 = OMcp22_222*RLcp22_323-OMcp22_322*RLcp22_223;
    ORcp22_223 = -(OMcp22_122*RLcp22_323-OMcp22_322*RLcp22_123);
    ORcp22_323 = OMcp22_122*RLcp22_223-OMcp22_222*RLcp22_123;
    VIcp22_123 = ORcp22_122+ORcp22_123+qd(1);
    VIcp22_223 = ORcp22_222+ORcp22_223+qd(2);
    VIcp22_323 = ORcp22_322+ORcp22_323+qd(3);
    OPcp22_123 = OPcp22_122+ROcp22_122*qdd(23)+qd(23)*(OMcp22_222*ROcp22_322-OMcp22_322*ROcp22_222);
    OPcp22_223 = OPcp22_222+ROcp22_222*qdd(23)-qd(23)*(OMcp22_122*ROcp22_322-OMcp22_322*ROcp22_122);
    OPcp22_323 = OPcp22_322+ROcp22_322*qdd(23)+qd(23)*(OMcp22_122*ROcp22_222-OMcp22_222*ROcp22_122);
    ACcp22_123 = qdd(1)+OMcp22_222*ORcp22_323+OMcp22_26*ORcp22_322-OMcp22_322*ORcp22_223-OMcp22_36*ORcp22_222+OPcp22_222*RLcp22_323+OPcp22_26*...
 RLcp22_322-OPcp22_322*RLcp22_223-OPcp22_36*RLcp22_222;
    ACcp22_223 = qdd(2)-OMcp22_122*ORcp22_323-OMcp22_16*ORcp22_322+OMcp22_322*ORcp22_123+OMcp22_36*ORcp22_122-OPcp22_122*RLcp22_323-OPcp22_16*...
 RLcp22_322+OPcp22_322*RLcp22_123+OPcp22_36*RLcp22_122;
    ACcp22_323 = qdd(3)+OMcp22_122*ORcp22_223+OMcp22_16*ORcp22_222-OMcp22_222*ORcp22_123-OMcp22_26*ORcp22_122+OPcp22_122*RLcp22_223+OPcp22_16*...
 RLcp22_222-OPcp22_222*RLcp22_123-OPcp22_26*RLcp22_122;

% = = Block_1_0_0_23_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp22_123;
    sens.P(2) = POcp22_223;
    sens.P(3) = POcp22_323;
    sens.R(1,1) = ROcp22_122;
    sens.R(1,2) = ROcp22_222;
    sens.R(1,3) = ROcp22_322;
    sens.R(2,1) = ROcp22_423;
    sens.R(2,2) = ROcp22_523;
    sens.R(2,3) = ROcp22_623;
    sens.R(3,1) = ROcp22_723;
    sens.R(3,2) = ROcp22_823;
    sens.R(3,3) = ROcp22_923;
    sens.V(1) = VIcp22_123;
    sens.V(2) = VIcp22_223;
    sens.V(3) = VIcp22_323;
    sens.OM(1) = OMcp22_123;
    sens.OM(2) = OMcp22_223;
    sens.OM(3) = OMcp22_323;
    sens.A(1) = ACcp22_123;
    sens.A(2) = ACcp22_223;
    sens.A(3) = ACcp22_323;
    sens.OMP(1) = OPcp22_123;
    sens.OMP(2) = OPcp22_223;
    sens.OMP(3) = OPcp22_323;
 
% 
case 24, 


% = = Block_1_0_0_24_0_1 = = 
 
% Sensor Kinematics 


    ROcp23_45 = -S4*C5;
    ROcp23_55 = C4*C5;
    ROcp23_75 = S4*S5;
    ROcp23_85 = -C4*S5;
    ROcp23_16 = -(ROcp23_75*S6-C4*C6);
    ROcp23_26 = -(ROcp23_85*S6-S4*C6);
    ROcp23_76 = ROcp23_75*C6+C4*S6;
    ROcp23_86 = ROcp23_85*C6+S4*S6;
    OMcp23_15 = qd(5)*C4;
    OMcp23_25 = qd(5)*S4;
    OMcp23_16 = OMcp23_15+ROcp23_45*qd(6);
    OMcp23_26 = OMcp23_25+ROcp23_55*qd(6);
    OMcp23_36 = qd(4)+qd(6)*S5;
    OPcp23_16 = ROcp23_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp23_25*S5-ROcp23_55*qd(4));
    OPcp23_26 = ROcp23_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp23_15*S5-ROcp23_45*qd(4));
    OPcp23_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_24_0_9 = = 
 
% Sensor Kinematics 


    ROcp23_122 = ROcp23_16*C22-ROcp23_76*S22;
    ROcp23_222 = ROcp23_26*C22-ROcp23_86*S22;
    ROcp23_322 = -C5*S22p6;
    ROcp23_722 = ROcp23_16*S22+ROcp23_76*C22;
    ROcp23_822 = ROcp23_26*S22+ROcp23_86*C22;
    ROcp23_922 = C5*C22p6;
    RLcp23_122 = ROcp23_16*s.dpt(1,8)+ROcp23_76*s.dpt(3,8);
    RLcp23_222 = ROcp23_26*s.dpt(1,8)+ROcp23_86*s.dpt(3,8);
    RLcp23_322 = -C5*(s.dpt(1,8)*S6-s.dpt(3,8)*C6);
    OMcp23_122 = OMcp23_16+ROcp23_45*qd(22);
    OMcp23_222 = OMcp23_26+ROcp23_55*qd(22);
    OMcp23_322 = OMcp23_36+qd(22)*S5;
    ORcp23_122 = OMcp23_26*RLcp23_322-OMcp23_36*RLcp23_222;
    ORcp23_222 = -(OMcp23_16*RLcp23_322-OMcp23_36*RLcp23_122);
    ORcp23_322 = OMcp23_16*RLcp23_222-OMcp23_26*RLcp23_122;
    OPcp23_122 = OPcp23_16+ROcp23_45*qdd(22)+qd(22)*(OMcp23_26*S5-OMcp23_36*ROcp23_55);
    OPcp23_222 = OPcp23_26+ROcp23_55*qdd(22)-qd(22)*(OMcp23_16*S5-OMcp23_36*ROcp23_45);
    OPcp23_322 = OPcp23_36+qdd(22)*S5+qd(22)*(OMcp23_16*ROcp23_55-OMcp23_26*ROcp23_45);

% = = Block_1_0_0_24_0_10 = = 
 
% Sensor Kinematics 


    ROcp23_423 = ROcp23_45*C23+ROcp23_722*S23;
    ROcp23_523 = ROcp23_55*C23+ROcp23_822*S23;
    ROcp23_623 = ROcp23_922*S23+C23*S5;
    ROcp23_723 = -(ROcp23_45*S23-ROcp23_722*C23);
    ROcp23_823 = -(ROcp23_55*S23-ROcp23_822*C23);
    ROcp23_923 = ROcp23_922*C23-S23*S5;
    ROcp23_124 = ROcp23_122*C24-ROcp23_723*S24;
    ROcp23_224 = ROcp23_222*C24-ROcp23_823*S24;
    ROcp23_324 = ROcp23_322*C24-ROcp23_923*S24;
    ROcp23_724 = ROcp23_122*S24+ROcp23_723*C24;
    ROcp23_824 = ROcp23_222*S24+ROcp23_823*C24;
    ROcp23_924 = ROcp23_322*S24+ROcp23_923*C24;
    RLcp23_123 = ROcp23_122*s.dpt(1,39)+ROcp23_45*s.dpt(2,39);
    RLcp23_223 = ROcp23_222*s.dpt(1,39)+ROcp23_55*s.dpt(2,39);
    RLcp23_323 = ROcp23_322*s.dpt(1,39)+s.dpt(2,39)*S5;
    POcp23_123 = RLcp23_122+RLcp23_123+q(1);
    POcp23_223 = RLcp23_222+RLcp23_223+q(2);
    POcp23_323 = RLcp23_322+RLcp23_323+q(3);
    OMcp23_123 = OMcp23_122+ROcp23_122*qd(23);
    OMcp23_223 = OMcp23_222+ROcp23_222*qd(23);
    OMcp23_323 = OMcp23_322+ROcp23_322*qd(23);
    ORcp23_123 = OMcp23_222*RLcp23_323-OMcp23_322*RLcp23_223;
    ORcp23_223 = -(OMcp23_122*RLcp23_323-OMcp23_322*RLcp23_123);
    ORcp23_323 = OMcp23_122*RLcp23_223-OMcp23_222*RLcp23_123;
    VIcp23_123 = ORcp23_122+ORcp23_123+qd(1);
    VIcp23_223 = ORcp23_222+ORcp23_223+qd(2);
    VIcp23_323 = ORcp23_322+ORcp23_323+qd(3);
    ACcp23_123 = qdd(1)+OMcp23_222*ORcp23_323+OMcp23_26*ORcp23_322-OMcp23_322*ORcp23_223-OMcp23_36*ORcp23_222+OPcp23_222*RLcp23_323+OPcp23_26*...
 RLcp23_322-OPcp23_322*RLcp23_223-OPcp23_36*RLcp23_222;
    ACcp23_223 = qdd(2)-OMcp23_122*ORcp23_323-OMcp23_16*ORcp23_322+OMcp23_322*ORcp23_123+OMcp23_36*ORcp23_122-OPcp23_122*RLcp23_323-OPcp23_16*...
 RLcp23_322+OPcp23_322*RLcp23_123+OPcp23_36*RLcp23_122;
    ACcp23_323 = qdd(3)+OMcp23_122*ORcp23_223+OMcp23_16*ORcp23_222-OMcp23_222*ORcp23_123-OMcp23_26*ORcp23_122+OPcp23_122*RLcp23_223+OPcp23_16*...
 RLcp23_222-OPcp23_222*RLcp23_123-OPcp23_26*RLcp23_122;
    OMcp23_124 = OMcp23_123+ROcp23_423*qd(24);
    OMcp23_224 = OMcp23_223+ROcp23_523*qd(24);
    OMcp23_324 = OMcp23_323+ROcp23_623*qd(24);
    OPcp23_124 = OPcp23_122+ROcp23_122*qdd(23)+ROcp23_423*qdd(24)+qd(23)*(OMcp23_222*ROcp23_322-OMcp23_322*ROcp23_222)+qd(24)*(OMcp23_223*...
 ROcp23_623-OMcp23_323*ROcp23_523);
    OPcp23_224 = OPcp23_222+ROcp23_222*qdd(23)+ROcp23_523*qdd(24)-qd(23)*(OMcp23_122*ROcp23_322-OMcp23_322*ROcp23_122)-qd(24)*(OMcp23_123*...
 ROcp23_623-OMcp23_323*ROcp23_423);
    OPcp23_324 = OPcp23_322+ROcp23_322*qdd(23)+ROcp23_623*qdd(24)+qd(23)*(OMcp23_122*ROcp23_222-OMcp23_222*ROcp23_122)+qd(24)*(OMcp23_123*...
 ROcp23_523-OMcp23_223*ROcp23_423);

% = = Block_1_0_0_24_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp23_123;
    sens.P(2) = POcp23_223;
    sens.P(3) = POcp23_323;
    sens.R(1,1) = ROcp23_124;
    sens.R(1,2) = ROcp23_224;
    sens.R(1,3) = ROcp23_324;
    sens.R(2,1) = ROcp23_423;
    sens.R(2,2) = ROcp23_523;
    sens.R(2,3) = ROcp23_623;
    sens.R(3,1) = ROcp23_724;
    sens.R(3,2) = ROcp23_824;
    sens.R(3,3) = ROcp23_924;
    sens.V(1) = VIcp23_123;
    sens.V(2) = VIcp23_223;
    sens.V(3) = VIcp23_323;
    sens.OM(1) = OMcp23_124;
    sens.OM(2) = OMcp23_224;
    sens.OM(3) = OMcp23_324;
    sens.A(1) = ACcp23_123;
    sens.A(2) = ACcp23_223;
    sens.A(3) = ACcp23_323;
    sens.OMP(1) = OPcp23_124;
    sens.OMP(2) = OPcp23_224;
    sens.OMP(3) = OPcp23_324;
 
% 
case 25, 


% = = Block_1_0_0_25_0_1 = = 
 
% Sensor Kinematics 


    ROcp24_45 = -S4*C5;
    ROcp24_55 = C4*C5;
    ROcp24_75 = S4*S5;
    ROcp24_85 = -C4*S5;
    ROcp24_16 = -(ROcp24_75*S6-C4*C6);
    ROcp24_26 = -(ROcp24_85*S6-S4*C6);
    ROcp24_76 = ROcp24_75*C6+C4*S6;
    ROcp24_86 = ROcp24_85*C6+S4*S6;
    OMcp24_15 = qd(5)*C4;
    OMcp24_25 = qd(5)*S4;
    OMcp24_16 = OMcp24_15+ROcp24_45*qd(6);
    OMcp24_26 = OMcp24_25+ROcp24_55*qd(6);
    OMcp24_36 = qd(4)+qd(6)*S5;
    OPcp24_16 = ROcp24_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp24_25*S5-ROcp24_55*qd(4));
    OPcp24_26 = ROcp24_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp24_15*S5-ROcp24_45*qd(4));
    OPcp24_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_25_0_9 = = 
 
% Sensor Kinematics 


    ROcp24_122 = ROcp24_16*C22-ROcp24_76*S22;
    ROcp24_222 = ROcp24_26*C22-ROcp24_86*S22;
    ROcp24_722 = ROcp24_16*S22+ROcp24_76*C22;
    ROcp24_822 = ROcp24_26*S22+ROcp24_86*C22;
    RLcp24_122 = ROcp24_16*s.dpt(1,8)+ROcp24_76*s.dpt(3,8);
    RLcp24_222 = ROcp24_26*s.dpt(1,8)+ROcp24_86*s.dpt(3,8);
    RLcp24_322 = -C5*(s.dpt(1,8)*S6-s.dpt(3,8)*C6);
    POcp24_122 = RLcp24_122+q(1);
    POcp24_222 = RLcp24_222+q(2);
    POcp24_322 = RLcp24_322+q(3);
    OMcp24_122 = OMcp24_16+ROcp24_45*qd(22);
    OMcp24_222 = OMcp24_26+ROcp24_55*qd(22);
    OMcp24_322 = OMcp24_36+qd(22)*S5;
    ORcp24_122 = OMcp24_26*RLcp24_322-OMcp24_36*RLcp24_222;
    ORcp24_222 = -(OMcp24_16*RLcp24_322-OMcp24_36*RLcp24_122);
    ORcp24_322 = OMcp24_16*RLcp24_222-OMcp24_26*RLcp24_122;
    VIcp24_122 = ORcp24_122+qd(1);
    VIcp24_222 = ORcp24_222+qd(2);
    VIcp24_322 = ORcp24_322+qd(3);
    ACcp24_122 = qdd(1)+OMcp24_26*ORcp24_322-OMcp24_36*ORcp24_222+OPcp24_26*RLcp24_322-OPcp24_36*RLcp24_222;
    ACcp24_222 = qdd(2)-OMcp24_16*ORcp24_322+OMcp24_36*ORcp24_122-OPcp24_16*RLcp24_322+OPcp24_36*RLcp24_122;
    ACcp24_322 = qdd(3)+OMcp24_16*ORcp24_222-OMcp24_26*ORcp24_122+OPcp24_16*RLcp24_222-OPcp24_26*RLcp24_122;

% = = Block_1_0_0_25_0_11 = = 
 
% Sensor Kinematics 


    ROcp24_125 = ROcp24_122*C25-ROcp24_722*S25;
    ROcp24_225 = ROcp24_222*C25-ROcp24_822*S25;
    ROcp24_325 = -C5*S25p22p6;
    ROcp24_725 = ROcp24_122*S25+ROcp24_722*C25;
    ROcp24_825 = ROcp24_222*S25+ROcp24_822*C25;
    ROcp24_925 = C5*C25p22p6;
    OMcp24_125 = OMcp24_122+ROcp24_45*qd(25);
    OMcp24_225 = OMcp24_222+ROcp24_55*qd(25);
    OMcp24_325 = OMcp24_322+qd(25)*S5;
    OPcp24_125 = OPcp24_16+ROcp24_45*qdd(22)+ROcp24_45*qdd(25)+qd(22)*(OMcp24_26*S5-OMcp24_36*ROcp24_55)+qd(25)*(OMcp24_222*S5-OMcp24_322*ROcp24_55...
 );
    OPcp24_225 = OPcp24_26+ROcp24_55*qdd(22)+ROcp24_55*qdd(25)-qd(22)*(OMcp24_16*S5-OMcp24_36*ROcp24_45)-qd(25)*(OMcp24_122*S5-OMcp24_322*ROcp24_45...
 );
    OPcp24_325 = OPcp24_36+qdd(22)*S5+qdd(25)*S5+qd(22)*(OMcp24_16*ROcp24_55-OMcp24_26*ROcp24_45)+qd(25)*(OMcp24_122*ROcp24_55-OMcp24_222*ROcp24_45...
 );

% = = Block_1_0_0_25_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp24_122;
    sens.P(2) = POcp24_222;
    sens.P(3) = POcp24_322;
    sens.R(1,1) = ROcp24_125;
    sens.R(1,2) = ROcp24_225;
    sens.R(1,3) = ROcp24_325;
    sens.R(2,1) = ROcp24_45;
    sens.R(2,2) = ROcp24_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp24_725;
    sens.R(3,2) = ROcp24_825;
    sens.R(3,3) = ROcp24_925;
    sens.V(1) = VIcp24_122;
    sens.V(2) = VIcp24_222;
    sens.V(3) = VIcp24_322;
    sens.OM(1) = OMcp24_125;
    sens.OM(2) = OMcp24_225;
    sens.OM(3) = OMcp24_325;
    sens.A(1) = ACcp24_122;
    sens.A(2) = ACcp24_222;
    sens.A(3) = ACcp24_322;
    sens.OMP(1) = OPcp24_125;
    sens.OMP(2) = OPcp24_225;
    sens.OMP(3) = OPcp24_325;
 
% 
case 26, 


% = = Block_1_0_0_26_0_1 = = 
 
% Sensor Kinematics 


    ROcp25_45 = -S4*C5;
    ROcp25_55 = C4*C5;
    ROcp25_75 = S4*S5;
    ROcp25_85 = -C4*S5;
    ROcp25_16 = -(ROcp25_75*S6-C4*C6);
    ROcp25_26 = -(ROcp25_85*S6-S4*C6);
    ROcp25_76 = ROcp25_75*C6+C4*S6;
    ROcp25_86 = ROcp25_85*C6+S4*S6;
    OMcp25_15 = qd(5)*C4;
    OMcp25_25 = qd(5)*S4;
    OMcp25_16 = OMcp25_15+ROcp25_45*qd(6);
    OMcp25_26 = OMcp25_25+ROcp25_55*qd(6);
    OMcp25_36 = qd(4)+qd(6)*S5;
    OPcp25_16 = ROcp25_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp25_25*S5-ROcp25_55*qd(4));
    OPcp25_26 = ROcp25_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp25_15*S5-ROcp25_45*qd(4));
    OPcp25_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_26_0_9 = = 
 
% Sensor Kinematics 


    ROcp25_122 = ROcp25_16*C22-ROcp25_76*S22;
    ROcp25_222 = ROcp25_26*C22-ROcp25_86*S22;
    ROcp25_722 = ROcp25_16*S22+ROcp25_76*C22;
    ROcp25_822 = ROcp25_26*S22+ROcp25_86*C22;
    RLcp25_122 = ROcp25_16*s.dpt(1,8)+ROcp25_76*s.dpt(3,8);
    RLcp25_222 = ROcp25_26*s.dpt(1,8)+ROcp25_86*s.dpt(3,8);
    RLcp25_322 = -C5*(s.dpt(1,8)*S6-s.dpt(3,8)*C6);
    OMcp25_122 = OMcp25_16+ROcp25_45*qd(22);
    OMcp25_222 = OMcp25_26+ROcp25_55*qd(22);
    OMcp25_322 = OMcp25_36+qd(22)*S5;
    ORcp25_122 = OMcp25_26*RLcp25_322-OMcp25_36*RLcp25_222;
    ORcp25_222 = -(OMcp25_16*RLcp25_322-OMcp25_36*RLcp25_122);
    ORcp25_322 = OMcp25_16*RLcp25_222-OMcp25_26*RLcp25_122;

% = = Block_1_0_0_26_0_11 = = 
 
% Sensor Kinematics 


    ROcp25_125 = ROcp25_122*C25-ROcp25_722*S25;
    ROcp25_225 = ROcp25_222*C25-ROcp25_822*S25;
    ROcp25_325 = -C5*S25p22p6;
    ROcp25_725 = ROcp25_122*S25+ROcp25_722*C25;
    ROcp25_825 = ROcp25_222*S25+ROcp25_822*C25;
    ROcp25_925 = C5*C25p22p6;
    ROcp25_426 = ROcp25_45*C26+ROcp25_725*S26;
    ROcp25_526 = ROcp25_55*C26+ROcp25_825*S26;
    ROcp25_626 = ROcp25_925*S26+C26*S5;
    ROcp25_726 = -(ROcp25_45*S26-ROcp25_725*C26);
    ROcp25_826 = -(ROcp25_55*S26-ROcp25_825*C26);
    ROcp25_926 = ROcp25_925*C26-S26*S5;
    OMcp25_125 = OMcp25_122+ROcp25_45*qd(25);
    OMcp25_225 = OMcp25_222+ROcp25_55*qd(25);
    OMcp25_325 = OMcp25_322+qd(25)*S5;
    OPcp25_125 = OPcp25_16+ROcp25_45*qdd(22)+ROcp25_45*qdd(25)+qd(22)*(OMcp25_26*S5-OMcp25_36*ROcp25_55)+qd(25)*(OMcp25_222*S5-OMcp25_322*ROcp25_55...
 );
    OPcp25_225 = OPcp25_26+ROcp25_55*qdd(22)+ROcp25_55*qdd(25)-qd(22)*(OMcp25_16*S5-OMcp25_36*ROcp25_45)-qd(25)*(OMcp25_122*S5-OMcp25_322*ROcp25_45...
 );
    OPcp25_325 = OPcp25_36+qdd(22)*S5+qdd(25)*S5+qd(22)*(OMcp25_16*ROcp25_55-OMcp25_26*ROcp25_45)+qd(25)*(OMcp25_122*ROcp25_55-OMcp25_222*ROcp25_45...
 );
    RLcp25_126 = ROcp25_125*s.dpt(1,42)+ROcp25_45*s.dpt(2,42);
    RLcp25_226 = ROcp25_225*s.dpt(1,42)+ROcp25_55*s.dpt(2,42);
    RLcp25_326 = ROcp25_325*s.dpt(1,42)+s.dpt(2,42)*S5;
    POcp25_126 = RLcp25_122+RLcp25_126+q(1);
    POcp25_226 = RLcp25_222+RLcp25_226+q(2);
    POcp25_326 = RLcp25_322+RLcp25_326+q(3);
    OMcp25_126 = OMcp25_125+ROcp25_125*qd(26);
    OMcp25_226 = OMcp25_225+ROcp25_225*qd(26);
    OMcp25_326 = OMcp25_325+ROcp25_325*qd(26);
    ORcp25_126 = OMcp25_225*RLcp25_326-OMcp25_325*RLcp25_226;
    ORcp25_226 = -(OMcp25_125*RLcp25_326-OMcp25_325*RLcp25_126);
    ORcp25_326 = OMcp25_125*RLcp25_226-OMcp25_225*RLcp25_126;
    VIcp25_126 = ORcp25_122+ORcp25_126+qd(1);
    VIcp25_226 = ORcp25_222+ORcp25_226+qd(2);
    VIcp25_326 = ORcp25_322+ORcp25_326+qd(3);
    OPcp25_126 = OPcp25_125+ROcp25_125*qdd(26)+qd(26)*(OMcp25_225*ROcp25_325-OMcp25_325*ROcp25_225);
    OPcp25_226 = OPcp25_225+ROcp25_225*qdd(26)-qd(26)*(OMcp25_125*ROcp25_325-OMcp25_325*ROcp25_125);
    OPcp25_326 = OPcp25_325+ROcp25_325*qdd(26)+qd(26)*(OMcp25_125*ROcp25_225-OMcp25_225*ROcp25_125);
    ACcp25_126 = qdd(1)+OMcp25_225*ORcp25_326+OMcp25_26*ORcp25_322-OMcp25_325*ORcp25_226-OMcp25_36*ORcp25_222+OPcp25_225*RLcp25_326+OPcp25_26*...
 RLcp25_322-OPcp25_325*RLcp25_226-OPcp25_36*RLcp25_222;
    ACcp25_226 = qdd(2)-OMcp25_125*ORcp25_326-OMcp25_16*ORcp25_322+OMcp25_325*ORcp25_126+OMcp25_36*ORcp25_122-OPcp25_125*RLcp25_326-OPcp25_16*...
 RLcp25_322+OPcp25_325*RLcp25_126+OPcp25_36*RLcp25_122;
    ACcp25_326 = qdd(3)+OMcp25_125*ORcp25_226+OMcp25_16*ORcp25_222-OMcp25_225*ORcp25_126-OMcp25_26*ORcp25_122+OPcp25_125*RLcp25_226+OPcp25_16*...
 RLcp25_222-OPcp25_225*RLcp25_126-OPcp25_26*RLcp25_122;

% = = Block_1_0_0_26_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp25_126;
    sens.P(2) = POcp25_226;
    sens.P(3) = POcp25_326;
    sens.R(1,1) = ROcp25_125;
    sens.R(1,2) = ROcp25_225;
    sens.R(1,3) = ROcp25_325;
    sens.R(2,1) = ROcp25_426;
    sens.R(2,2) = ROcp25_526;
    sens.R(2,3) = ROcp25_626;
    sens.R(3,1) = ROcp25_726;
    sens.R(3,2) = ROcp25_826;
    sens.R(3,3) = ROcp25_926;
    sens.V(1) = VIcp25_126;
    sens.V(2) = VIcp25_226;
    sens.V(3) = VIcp25_326;
    sens.OM(1) = OMcp25_126;
    sens.OM(2) = OMcp25_226;
    sens.OM(3) = OMcp25_326;
    sens.A(1) = ACcp25_126;
    sens.A(2) = ACcp25_226;
    sens.A(3) = ACcp25_326;
    sens.OMP(1) = OPcp25_126;
    sens.OMP(2) = OPcp25_226;
    sens.OMP(3) = OPcp25_326;
 
% 
case 27, 


% = = Block_1_0_0_27_0_1 = = 
 
% Sensor Kinematics 


    ROcp26_45 = -S4*C5;
    ROcp26_55 = C4*C5;
    ROcp26_75 = S4*S5;
    ROcp26_85 = -C4*S5;
    ROcp26_16 = -(ROcp26_75*S6-C4*C6);
    ROcp26_26 = -(ROcp26_85*S6-S4*C6);
    ROcp26_76 = ROcp26_75*C6+C4*S6;
    ROcp26_86 = ROcp26_85*C6+S4*S6;
    OMcp26_15 = qd(5)*C4;
    OMcp26_25 = qd(5)*S4;
    OMcp26_16 = OMcp26_15+ROcp26_45*qd(6);
    OMcp26_26 = OMcp26_25+ROcp26_55*qd(6);
    OMcp26_36 = qd(4)+qd(6)*S5;
    OPcp26_16 = ROcp26_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp26_25*S5-ROcp26_55*qd(4));
    OPcp26_26 = ROcp26_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp26_15*S5-ROcp26_45*qd(4));
    OPcp26_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_27_0_9 = = 
 
% Sensor Kinematics 


    ROcp26_122 = ROcp26_16*C22-ROcp26_76*S22;
    ROcp26_222 = ROcp26_26*C22-ROcp26_86*S22;
    ROcp26_722 = ROcp26_16*S22+ROcp26_76*C22;
    ROcp26_822 = ROcp26_26*S22+ROcp26_86*C22;
    RLcp26_122 = ROcp26_16*s.dpt(1,8)+ROcp26_76*s.dpt(3,8);
    RLcp26_222 = ROcp26_26*s.dpt(1,8)+ROcp26_86*s.dpt(3,8);
    RLcp26_322 = -C5*(s.dpt(1,8)*S6-s.dpt(3,8)*C6);
    OMcp26_122 = OMcp26_16+ROcp26_45*qd(22);
    OMcp26_222 = OMcp26_26+ROcp26_55*qd(22);
    OMcp26_322 = OMcp26_36+qd(22)*S5;
    ORcp26_122 = OMcp26_26*RLcp26_322-OMcp26_36*RLcp26_222;
    ORcp26_222 = -(OMcp26_16*RLcp26_322-OMcp26_36*RLcp26_122);
    ORcp26_322 = OMcp26_16*RLcp26_222-OMcp26_26*RLcp26_122;

% = = Block_1_0_0_27_0_11 = = 
 
% Sensor Kinematics 


    ROcp26_125 = ROcp26_122*C25-ROcp26_722*S25;
    ROcp26_225 = ROcp26_222*C25-ROcp26_822*S25;
    ROcp26_325 = -C5*S25p22p6;
    ROcp26_725 = ROcp26_122*S25+ROcp26_722*C25;
    ROcp26_825 = ROcp26_222*S25+ROcp26_822*C25;
    ROcp26_925 = C5*C25p22p6;
    ROcp26_426 = ROcp26_45*C26+ROcp26_725*S26;
    ROcp26_526 = ROcp26_55*C26+ROcp26_825*S26;
    ROcp26_626 = ROcp26_925*S26+C26*S5;
    ROcp26_726 = -(ROcp26_45*S26-ROcp26_725*C26);
    ROcp26_826 = -(ROcp26_55*S26-ROcp26_825*C26);
    ROcp26_926 = ROcp26_925*C26-S26*S5;
    ROcp26_127 = ROcp26_125*C27-ROcp26_726*S27;
    ROcp26_227 = ROcp26_225*C27-ROcp26_826*S27;
    ROcp26_327 = ROcp26_325*C27-ROcp26_926*S27;
    ROcp26_727 = ROcp26_125*S27+ROcp26_726*C27;
    ROcp26_827 = ROcp26_225*S27+ROcp26_826*C27;
    ROcp26_927 = ROcp26_325*S27+ROcp26_926*C27;
    OMcp26_125 = OMcp26_122+ROcp26_45*qd(25);
    OMcp26_225 = OMcp26_222+ROcp26_55*qd(25);
    OMcp26_325 = OMcp26_322+qd(25)*S5;
    OPcp26_125 = OPcp26_16+ROcp26_45*qdd(22)+ROcp26_45*qdd(25)+qd(22)*(OMcp26_26*S5-OMcp26_36*ROcp26_55)+qd(25)*(OMcp26_222*S5-OMcp26_322*ROcp26_55...
 );
    OPcp26_225 = OPcp26_26+ROcp26_55*qdd(22)+ROcp26_55*qdd(25)-qd(22)*(OMcp26_16*S5-OMcp26_36*ROcp26_45)-qd(25)*(OMcp26_122*S5-OMcp26_322*ROcp26_45...
 );
    OPcp26_325 = OPcp26_36+qdd(22)*S5+qdd(25)*S5+qd(22)*(OMcp26_16*ROcp26_55-OMcp26_26*ROcp26_45)+qd(25)*(OMcp26_122*ROcp26_55-OMcp26_222*ROcp26_45...
 );
    RLcp26_126 = ROcp26_125*s.dpt(1,42)+ROcp26_45*s.dpt(2,42);
    RLcp26_226 = ROcp26_225*s.dpt(1,42)+ROcp26_55*s.dpt(2,42);
    RLcp26_326 = ROcp26_325*s.dpt(1,42)+s.dpt(2,42)*S5;
    POcp26_126 = RLcp26_122+RLcp26_126+q(1);
    POcp26_226 = RLcp26_222+RLcp26_226+q(2);
    POcp26_326 = RLcp26_322+RLcp26_326+q(3);
    OMcp26_126 = OMcp26_125+ROcp26_125*qd(26);
    OMcp26_226 = OMcp26_225+ROcp26_225*qd(26);
    OMcp26_326 = OMcp26_325+ROcp26_325*qd(26);
    ORcp26_126 = OMcp26_225*RLcp26_326-OMcp26_325*RLcp26_226;
    ORcp26_226 = -(OMcp26_125*RLcp26_326-OMcp26_325*RLcp26_126);
    ORcp26_326 = OMcp26_125*RLcp26_226-OMcp26_225*RLcp26_126;
    VIcp26_126 = ORcp26_122+ORcp26_126+qd(1);
    VIcp26_226 = ORcp26_222+ORcp26_226+qd(2);
    VIcp26_326 = ORcp26_322+ORcp26_326+qd(3);
    ACcp26_126 = qdd(1)+OMcp26_225*ORcp26_326+OMcp26_26*ORcp26_322-OMcp26_325*ORcp26_226-OMcp26_36*ORcp26_222+OPcp26_225*RLcp26_326+OPcp26_26*...
 RLcp26_322-OPcp26_325*RLcp26_226-OPcp26_36*RLcp26_222;
    ACcp26_226 = qdd(2)-OMcp26_125*ORcp26_326-OMcp26_16*ORcp26_322+OMcp26_325*ORcp26_126+OMcp26_36*ORcp26_122-OPcp26_125*RLcp26_326-OPcp26_16*...
 RLcp26_322+OPcp26_325*RLcp26_126+OPcp26_36*RLcp26_122;
    ACcp26_326 = qdd(3)+OMcp26_125*ORcp26_226+OMcp26_16*ORcp26_222-OMcp26_225*ORcp26_126-OMcp26_26*ORcp26_122+OPcp26_125*RLcp26_226+OPcp26_16*...
 RLcp26_222-OPcp26_225*RLcp26_126-OPcp26_26*RLcp26_122;
    OMcp26_127 = OMcp26_126+ROcp26_426*qd(27);
    OMcp26_227 = OMcp26_226+ROcp26_526*qd(27);
    OMcp26_327 = OMcp26_326+ROcp26_626*qd(27);
    OPcp26_127 = OPcp26_125+ROcp26_125*qdd(26)+ROcp26_426*qdd(27)+qd(26)*(OMcp26_225*ROcp26_325-OMcp26_325*ROcp26_225)+qd(27)*(OMcp26_226*...
 ROcp26_626-OMcp26_326*ROcp26_526);
    OPcp26_227 = OPcp26_225+ROcp26_225*qdd(26)+ROcp26_526*qdd(27)-qd(26)*(OMcp26_125*ROcp26_325-OMcp26_325*ROcp26_125)-qd(27)*(OMcp26_126*...
 ROcp26_626-OMcp26_326*ROcp26_426);
    OPcp26_327 = OPcp26_325+ROcp26_325*qdd(26)+ROcp26_626*qdd(27)+qd(26)*(OMcp26_125*ROcp26_225-OMcp26_225*ROcp26_125)+qd(27)*(OMcp26_126*...
 ROcp26_526-OMcp26_226*ROcp26_426);

% = = Block_1_0_0_27_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp26_126;
    sens.P(2) = POcp26_226;
    sens.P(3) = POcp26_326;
    sens.R(1,1) = ROcp26_127;
    sens.R(1,2) = ROcp26_227;
    sens.R(1,3) = ROcp26_327;
    sens.R(2,1) = ROcp26_426;
    sens.R(2,2) = ROcp26_526;
    sens.R(2,3) = ROcp26_626;
    sens.R(3,1) = ROcp26_727;
    sens.R(3,2) = ROcp26_827;
    sens.R(3,3) = ROcp26_927;
    sens.V(1) = VIcp26_126;
    sens.V(2) = VIcp26_226;
    sens.V(3) = VIcp26_326;
    sens.OM(1) = OMcp26_127;
    sens.OM(2) = OMcp26_227;
    sens.OM(3) = OMcp26_327;
    sens.A(1) = ACcp26_126;
    sens.A(2) = ACcp26_226;
    sens.A(3) = ACcp26_326;
    sens.OMP(1) = OPcp26_127;
    sens.OMP(2) = OPcp26_227;
    sens.OMP(3) = OPcp26_327;
 
% 
case 28, 


% = = Block_1_0_0_28_0_1 = = 
 
% Sensor Kinematics 


    ROcp27_45 = -S4*C5;
    ROcp27_55 = C4*C5;
    ROcp27_75 = S4*S5;
    ROcp27_85 = -C4*S5;
    ROcp27_16 = -(ROcp27_75*S6-C4*C6);
    ROcp27_26 = -(ROcp27_85*S6-S4*C6);
    ROcp27_36 = -C5*S6;
    ROcp27_76 = ROcp27_75*C6+C4*S6;
    ROcp27_86 = ROcp27_85*C6+S4*S6;
    ROcp27_96 = C5*C6;
    OMcp27_15 = qd(5)*C4;
    OMcp27_25 = qd(5)*S4;
    OMcp27_16 = OMcp27_15+ROcp27_45*qd(6);
    OMcp27_26 = OMcp27_25+ROcp27_55*qd(6);
    OMcp27_36 = qd(4)+qd(6)*S5;
    OPcp27_16 = ROcp27_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp27_25*S5-ROcp27_55*qd(4));
    OPcp27_26 = ROcp27_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp27_15*S5-ROcp27_45*qd(4));
    OPcp27_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_28_0_12 = = 
 
% Sensor Kinematics 


    RLcp27_128 = ROcp27_16*s.dpt(1,9)+ROcp27_45*q(28)+ROcp27_76*s.dpt(3,9);
    RLcp27_228 = ROcp27_26*s.dpt(1,9)+ROcp27_55*q(28)+ROcp27_86*s.dpt(3,9);
    RLcp27_328 = ROcp27_36*s.dpt(1,9)+ROcp27_96*s.dpt(3,9)+q(28)*S5;
    POcp27_128 = RLcp27_128+q(1);
    POcp27_228 = RLcp27_228+q(2);
    POcp27_328 = RLcp27_328+q(3);
    ORcp27_128 = OMcp27_26*RLcp27_328-OMcp27_36*RLcp27_228;
    ORcp27_228 = -(OMcp27_16*RLcp27_328-OMcp27_36*RLcp27_128);
    ORcp27_328 = OMcp27_16*RLcp27_228-OMcp27_26*RLcp27_128;
    VIcp27_128 = ORcp27_128+qd(1)+ROcp27_45*qd(28);
    VIcp27_228 = ORcp27_228+qd(2)+ROcp27_55*qd(28);
    VIcp27_328 = ORcp27_328+qd(3)+qd(28)*S5;
    ACcp27_128 = qdd(1)+OMcp27_26*ORcp27_328-OMcp27_36*ORcp27_228+OPcp27_26*RLcp27_328-OPcp27_36*RLcp27_228+ROcp27_45*qdd(28)+(2.0)*qd(28)*(OMcp27_26*S5...
 -OMcp27_36*ROcp27_55);
    ACcp27_228 = qdd(2)-OMcp27_16*ORcp27_328+OMcp27_36*ORcp27_128-OPcp27_16*RLcp27_328+OPcp27_36*RLcp27_128+ROcp27_55*qdd(28)-(2.0)*qd(28)*(OMcp27_16*S5...
 -OMcp27_36*ROcp27_45);
    ACcp27_328 = qdd(3)+OMcp27_16*ORcp27_228-OMcp27_26*ORcp27_128+OPcp27_16*RLcp27_228-OPcp27_26*RLcp27_128+qdd(28)*S5+(2.0)*qd(28)*(OMcp27_16*ROcp27_55...
 -OMcp27_26*ROcp27_45);

% = = Block_1_0_0_28_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp27_128;
    sens.P(2) = POcp27_228;
    sens.P(3) = POcp27_328;
    sens.R(1,1) = ROcp27_16;
    sens.R(1,2) = ROcp27_26;
    sens.R(1,3) = ROcp27_36;
    sens.R(2,1) = ROcp27_45;
    sens.R(2,2) = ROcp27_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp27_76;
    sens.R(3,2) = ROcp27_86;
    sens.R(3,3) = ROcp27_96;
    sens.V(1) = VIcp27_128;
    sens.V(2) = VIcp27_228;
    sens.V(3) = VIcp27_328;
    sens.OM(1) = OMcp27_16;
    sens.OM(2) = OMcp27_26;
    sens.OM(3) = OMcp27_36;
    sens.A(1) = ACcp27_128;
    sens.A(2) = ACcp27_228;
    sens.A(3) = ACcp27_328;
    sens.OMP(1) = OPcp27_16;
    sens.OMP(2) = OPcp27_26;
    sens.OMP(3) = OPcp27_36;
 
% 
case 29, 


% = = Block_1_0_0_29_0_1 = = 
 
% Sensor Kinematics 


    ROcp28_45 = -S4*C5;
    ROcp28_55 = C4*C5;
    ROcp28_75 = S4*S5;
    ROcp28_85 = -C4*S5;
    ROcp28_16 = -(ROcp28_75*S6-C4*C6);
    ROcp28_26 = -(ROcp28_85*S6-S4*C6);
    ROcp28_36 = -C5*S6;
    ROcp28_76 = ROcp28_75*C6+C4*S6;
    ROcp28_86 = ROcp28_85*C6+S4*S6;
    ROcp28_96 = C5*C6;
    OMcp28_15 = qd(5)*C4;
    OMcp28_25 = qd(5)*S4;
    OMcp28_16 = OMcp28_15+ROcp28_45*qd(6);
    OMcp28_26 = OMcp28_25+ROcp28_55*qd(6);
    OMcp28_36 = qd(4)+qd(6)*S5;
    OPcp28_16 = ROcp28_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp28_25*S5-ROcp28_55*qd(4));
    OPcp28_26 = ROcp28_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp28_15*S5-ROcp28_45*qd(4));
    OPcp28_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_29_0_12 = = 
 
% Sensor Kinematics 


    RLcp28_128 = ROcp28_16*s.dpt(1,9)+ROcp28_45*q(28)+ROcp28_76*s.dpt(3,9);
    RLcp28_228 = ROcp28_26*s.dpt(1,9)+ROcp28_55*q(28)+ROcp28_86*s.dpt(3,9);
    RLcp28_328 = ROcp28_36*s.dpt(1,9)+ROcp28_96*s.dpt(3,9)+q(28)*S5;
    ORcp28_128 = OMcp28_26*RLcp28_328-OMcp28_36*RLcp28_228;
    ORcp28_228 = -(OMcp28_16*RLcp28_328-OMcp28_36*RLcp28_128);
    ORcp28_328 = OMcp28_16*RLcp28_228-OMcp28_26*RLcp28_128;

% = = Block_1_0_0_29_0_13 = = 
 
% Sensor Kinematics 


    ROcp28_429 = ROcp28_45*C29+ROcp28_76*S29;
    ROcp28_529 = ROcp28_55*C29+ROcp28_86*S29;
    ROcp28_629 = ROcp28_96*S29+C29*S5;
    ROcp28_729 = -(ROcp28_45*S29-ROcp28_76*C29);
    ROcp28_829 = -(ROcp28_55*S29-ROcp28_86*C29);
    ROcp28_929 = ROcp28_96*C29-S29*S5;
    RLcp28_129 = ROcp28_45*s.dpt(2,44);
    RLcp28_229 = ROcp28_55*s.dpt(2,44);
    RLcp28_329 = s.dpt(2,44)*S5;
    POcp28_129 = RLcp28_128+RLcp28_129+q(1);
    POcp28_229 = RLcp28_228+RLcp28_229+q(2);
    POcp28_329 = RLcp28_328+RLcp28_329+q(3);
    OMcp28_129 = OMcp28_16+ROcp28_16*qd(29);
    OMcp28_229 = OMcp28_26+ROcp28_26*qd(29);
    OMcp28_329 = OMcp28_36+ROcp28_36*qd(29);
    ORcp28_129 = OMcp28_26*RLcp28_329-OMcp28_36*RLcp28_229;
    ORcp28_229 = -(OMcp28_16*RLcp28_329-OMcp28_36*RLcp28_129);
    ORcp28_329 = OMcp28_16*RLcp28_229-OMcp28_26*RLcp28_129;
    VIcp28_129 = ORcp28_128+ORcp28_129+qd(1)+ROcp28_45*qd(28);
    VIcp28_229 = ORcp28_228+ORcp28_229+qd(2)+ROcp28_55*qd(28);
    VIcp28_329 = ORcp28_328+ORcp28_329+qd(3)+qd(28)*S5;
    OPcp28_129 = OPcp28_16+ROcp28_16*qdd(29)+qd(29)*(OMcp28_26*ROcp28_36-OMcp28_36*ROcp28_26);
    OPcp28_229 = OPcp28_26+ROcp28_26*qdd(29)-qd(29)*(OMcp28_16*ROcp28_36-OMcp28_36*ROcp28_16);
    OPcp28_329 = OPcp28_36+ROcp28_36*qdd(29)+qd(29)*(OMcp28_16*ROcp28_26-OMcp28_26*ROcp28_16);
    ACcp28_129 = qdd(1)+OMcp28_26*ORcp28_328+OMcp28_26*ORcp28_329-OMcp28_36*ORcp28_228-OMcp28_36*ORcp28_229+OPcp28_26*RLcp28_328+OPcp28_26*...
 RLcp28_329-OPcp28_36*RLcp28_228-OPcp28_36*RLcp28_229+ROcp28_45*qdd(28)+(2.0)*qd(28)*(OMcp28_26*S5-OMcp28_36*ROcp28_55);
    ACcp28_229 = qdd(2)-OMcp28_16*ORcp28_328-OMcp28_16*ORcp28_329+OMcp28_36*ORcp28_128+OMcp28_36*ORcp28_129-OPcp28_16*RLcp28_328-OPcp28_16*...
 RLcp28_329+OPcp28_36*RLcp28_128+OPcp28_36*RLcp28_129+ROcp28_55*qdd(28)-(2.0)*qd(28)*(OMcp28_16*S5-OMcp28_36*ROcp28_45);
    ACcp28_329 = qdd(3)+OMcp28_16*ORcp28_228+OMcp28_16*ORcp28_229-OMcp28_26*ORcp28_128-OMcp28_26*ORcp28_129+OPcp28_16*RLcp28_228+OPcp28_16*...
 RLcp28_229-OPcp28_26*RLcp28_128-OPcp28_26*RLcp28_129+qdd(28)*S5+(2.0)*qd(28)*(OMcp28_16*ROcp28_55-OMcp28_26*ROcp28_45);

% = = Block_1_0_0_29_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp28_129;
    sens.P(2) = POcp28_229;
    sens.P(3) = POcp28_329;
    sens.R(1,1) = ROcp28_16;
    sens.R(1,2) = ROcp28_26;
    sens.R(1,3) = ROcp28_36;
    sens.R(2,1) = ROcp28_429;
    sens.R(2,2) = ROcp28_529;
    sens.R(2,3) = ROcp28_629;
    sens.R(3,1) = ROcp28_729;
    sens.R(3,2) = ROcp28_829;
    sens.R(3,3) = ROcp28_929;
    sens.V(1) = VIcp28_129;
    sens.V(2) = VIcp28_229;
    sens.V(3) = VIcp28_329;
    sens.OM(1) = OMcp28_129;
    sens.OM(2) = OMcp28_229;
    sens.OM(3) = OMcp28_329;
    sens.A(1) = ACcp28_129;
    sens.A(2) = ACcp28_229;
    sens.A(3) = ACcp28_329;
    sens.OMP(1) = OPcp28_129;
    sens.OMP(2) = OPcp28_229;
    sens.OMP(3) = OPcp28_329;
 
% 
case 30, 


% = = Block_1_0_0_30_0_1 = = 
 
% Sensor Kinematics 


    ROcp29_45 = -S4*C5;
    ROcp29_55 = C4*C5;
    ROcp29_75 = S4*S5;
    ROcp29_85 = -C4*S5;
    ROcp29_16 = -(ROcp29_75*S6-C4*C6);
    ROcp29_26 = -(ROcp29_85*S6-S4*C6);
    ROcp29_36 = -C5*S6;
    ROcp29_76 = ROcp29_75*C6+C4*S6;
    ROcp29_86 = ROcp29_85*C6+S4*S6;
    ROcp29_96 = C5*C6;
    OMcp29_15 = qd(5)*C4;
    OMcp29_25 = qd(5)*S4;
    OMcp29_16 = OMcp29_15+ROcp29_45*qd(6);
    OMcp29_26 = OMcp29_25+ROcp29_55*qd(6);
    OMcp29_36 = qd(4)+qd(6)*S5;
    OPcp29_16 = ROcp29_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp29_25*S5-ROcp29_55*qd(4));
    OPcp29_26 = ROcp29_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp29_15*S5-ROcp29_45*qd(4));
    OPcp29_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_30_0_12 = = 
 
% Sensor Kinematics 


    RLcp29_128 = ROcp29_16*s.dpt(1,9)+ROcp29_45*q(28)+ROcp29_76*s.dpt(3,9);
    RLcp29_228 = ROcp29_26*s.dpt(1,9)+ROcp29_55*q(28)+ROcp29_86*s.dpt(3,9);
    RLcp29_328 = ROcp29_36*s.dpt(1,9)+ROcp29_96*s.dpt(3,9)+q(28)*S5;
    ORcp29_128 = OMcp29_26*RLcp29_328-OMcp29_36*RLcp29_228;
    ORcp29_228 = -(OMcp29_16*RLcp29_328-OMcp29_36*RLcp29_128);
    ORcp29_328 = OMcp29_16*RLcp29_228-OMcp29_26*RLcp29_128;

% = = Block_1_0_0_30_0_13 = = 
 
% Sensor Kinematics 


    ROcp29_429 = ROcp29_45*C29+ROcp29_76*S29;
    ROcp29_529 = ROcp29_55*C29+ROcp29_86*S29;
    ROcp29_629 = ROcp29_96*S29+C29*S5;
    ROcp29_729 = -(ROcp29_45*S29-ROcp29_76*C29);
    ROcp29_829 = -(ROcp29_55*S29-ROcp29_86*C29);
    ROcp29_929 = ROcp29_96*C29-S29*S5;
    ROcp29_130 = ROcp29_16*C30+ROcp29_429*S30;
    ROcp29_230 = ROcp29_26*C30+ROcp29_529*S30;
    ROcp29_330 = ROcp29_36*C30+ROcp29_629*S30;
    ROcp29_430 = -(ROcp29_16*S30-ROcp29_429*C30);
    ROcp29_530 = -(ROcp29_26*S30-ROcp29_529*C30);
    ROcp29_630 = -(ROcp29_36*S30-ROcp29_629*C30);
    RLcp29_129 = ROcp29_45*s.dpt(2,44);
    RLcp29_229 = ROcp29_55*s.dpt(2,44);
    RLcp29_329 = s.dpt(2,44)*S5;
    POcp29_129 = RLcp29_128+RLcp29_129+q(1);
    POcp29_229 = RLcp29_228+RLcp29_229+q(2);
    POcp29_329 = RLcp29_328+RLcp29_329+q(3);
    OMcp29_129 = OMcp29_16+ROcp29_16*qd(29);
    OMcp29_229 = OMcp29_26+ROcp29_26*qd(29);
    OMcp29_329 = OMcp29_36+ROcp29_36*qd(29);
    ORcp29_129 = OMcp29_26*RLcp29_329-OMcp29_36*RLcp29_229;
    ORcp29_229 = -(OMcp29_16*RLcp29_329-OMcp29_36*RLcp29_129);
    ORcp29_329 = OMcp29_16*RLcp29_229-OMcp29_26*RLcp29_129;
    VIcp29_129 = ORcp29_128+ORcp29_129+qd(1)+ROcp29_45*qd(28);
    VIcp29_229 = ORcp29_228+ORcp29_229+qd(2)+ROcp29_55*qd(28);
    VIcp29_329 = ORcp29_328+ORcp29_329+qd(3)+qd(28)*S5;
    ACcp29_129 = qdd(1)+OMcp29_26*ORcp29_328+OMcp29_26*ORcp29_329-OMcp29_36*ORcp29_228-OMcp29_36*ORcp29_229+OPcp29_26*RLcp29_328+OPcp29_26*...
 RLcp29_329-OPcp29_36*RLcp29_228-OPcp29_36*RLcp29_229+ROcp29_45*qdd(28)+(2.0)*qd(28)*(OMcp29_26*S5-OMcp29_36*ROcp29_55);
    ACcp29_229 = qdd(2)-OMcp29_16*ORcp29_328-OMcp29_16*ORcp29_329+OMcp29_36*ORcp29_128+OMcp29_36*ORcp29_129-OPcp29_16*RLcp29_328-OPcp29_16*...
 RLcp29_329+OPcp29_36*RLcp29_128+OPcp29_36*RLcp29_129+ROcp29_55*qdd(28)-(2.0)*qd(28)*(OMcp29_16*S5-OMcp29_36*ROcp29_45);
    ACcp29_329 = qdd(3)+OMcp29_16*ORcp29_228+OMcp29_16*ORcp29_229-OMcp29_26*ORcp29_128-OMcp29_26*ORcp29_129+OPcp29_16*RLcp29_228+OPcp29_16*...
 RLcp29_229-OPcp29_26*RLcp29_128-OPcp29_26*RLcp29_129+qdd(28)*S5+(2.0)*qd(28)*(OMcp29_16*ROcp29_55-OMcp29_26*ROcp29_45);
    OMcp29_130 = OMcp29_129+ROcp29_729*qd(30);
    OMcp29_230 = OMcp29_229+ROcp29_829*qd(30);
    OMcp29_330 = OMcp29_329+ROcp29_929*qd(30);
    OPcp29_130 = OPcp29_16+ROcp29_16*qdd(29)+ROcp29_729*qdd(30)+qd(29)*(OMcp29_26*ROcp29_36-OMcp29_36*ROcp29_26)+qd(30)*(OMcp29_229*ROcp29_929-...
 OMcp29_329*ROcp29_829);
    OPcp29_230 = OPcp29_26+ROcp29_26*qdd(29)+ROcp29_829*qdd(30)-qd(29)*(OMcp29_16*ROcp29_36-OMcp29_36*ROcp29_16)-qd(30)*(OMcp29_129*ROcp29_929-...
 OMcp29_329*ROcp29_729);
    OPcp29_330 = OPcp29_36+ROcp29_36*qdd(29)+ROcp29_929*qdd(30)+qd(29)*(OMcp29_16*ROcp29_26-OMcp29_26*ROcp29_16)+qd(30)*(OMcp29_129*ROcp29_829-...
 OMcp29_229*ROcp29_729);

% = = Block_1_0_0_30_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp29_129;
    sens.P(2) = POcp29_229;
    sens.P(3) = POcp29_329;
    sens.R(1,1) = ROcp29_130;
    sens.R(1,2) = ROcp29_230;
    sens.R(1,3) = ROcp29_330;
    sens.R(2,1) = ROcp29_430;
    sens.R(2,2) = ROcp29_530;
    sens.R(2,3) = ROcp29_630;
    sens.R(3,1) = ROcp29_729;
    sens.R(3,2) = ROcp29_829;
    sens.R(3,3) = ROcp29_929;
    sens.V(1) = VIcp29_129;
    sens.V(2) = VIcp29_229;
    sens.V(3) = VIcp29_329;
    sens.OM(1) = OMcp29_130;
    sens.OM(2) = OMcp29_230;
    sens.OM(3) = OMcp29_330;
    sens.A(1) = ACcp29_129;
    sens.A(2) = ACcp29_229;
    sens.A(3) = ACcp29_329;
    sens.OMP(1) = OPcp29_130;
    sens.OMP(2) = OPcp29_230;
    sens.OMP(3) = OPcp29_330;
 
% 
case 31, 


% = = Block_1_0_0_31_0_1 = = 
 
% Sensor Kinematics 


    ROcp30_45 = -S4*C5;
    ROcp30_55 = C4*C5;
    ROcp30_75 = S4*S5;
    ROcp30_85 = -C4*S5;
    ROcp30_16 = -(ROcp30_75*S6-C4*C6);
    ROcp30_26 = -(ROcp30_85*S6-S4*C6);
    ROcp30_36 = -C5*S6;
    ROcp30_76 = ROcp30_75*C6+C4*S6;
    ROcp30_86 = ROcp30_85*C6+S4*S6;
    ROcp30_96 = C5*C6;
    OMcp30_15 = qd(5)*C4;
    OMcp30_25 = qd(5)*S4;
    OMcp30_16 = OMcp30_15+ROcp30_45*qd(6);
    OMcp30_26 = OMcp30_25+ROcp30_55*qd(6);
    OMcp30_36 = qd(4)+qd(6)*S5;
    OPcp30_16 = ROcp30_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp30_25*S5-ROcp30_55*qd(4));
    OPcp30_26 = ROcp30_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp30_15*S5-ROcp30_45*qd(4));
    OPcp30_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_31_0_12 = = 
 
% Sensor Kinematics 


    RLcp30_128 = ROcp30_16*s.dpt(1,9)+ROcp30_45*q(28)+ROcp30_76*s.dpt(3,9);
    RLcp30_228 = ROcp30_26*s.dpt(1,9)+ROcp30_55*q(28)+ROcp30_86*s.dpt(3,9);
    RLcp30_328 = ROcp30_36*s.dpt(1,9)+ROcp30_96*s.dpt(3,9)+q(28)*S5;
    ORcp30_128 = OMcp30_26*RLcp30_328-OMcp30_36*RLcp30_228;
    ORcp30_228 = -(OMcp30_16*RLcp30_328-OMcp30_36*RLcp30_128);
    ORcp30_328 = OMcp30_16*RLcp30_228-OMcp30_26*RLcp30_128;

% = = Block_1_0_0_31_0_14 = = 
 
% Sensor Kinematics 


    ROcp30_431 = ROcp30_45*C31+ROcp30_76*S31;
    ROcp30_531 = ROcp30_55*C31+ROcp30_86*S31;
    ROcp30_631 = ROcp30_96*S31+C31*S5;
    ROcp30_731 = -(ROcp30_45*S31-ROcp30_76*C31);
    ROcp30_831 = -(ROcp30_55*S31-ROcp30_86*C31);
    ROcp30_931 = ROcp30_96*C31-S31*S5;
    RLcp30_131 = ROcp30_45*s.dpt(2,45);
    RLcp30_231 = ROcp30_55*s.dpt(2,45);
    RLcp30_331 = s.dpt(2,45)*S5;
    POcp30_131 = RLcp30_128+RLcp30_131+q(1);
    POcp30_231 = RLcp30_228+RLcp30_231+q(2);
    POcp30_331 = RLcp30_328+RLcp30_331+q(3);
    OMcp30_131 = OMcp30_16+ROcp30_16*qd(31);
    OMcp30_231 = OMcp30_26+ROcp30_26*qd(31);
    OMcp30_331 = OMcp30_36+ROcp30_36*qd(31);
    ORcp30_131 = OMcp30_26*RLcp30_331-OMcp30_36*RLcp30_231;
    ORcp30_231 = -(OMcp30_16*RLcp30_331-OMcp30_36*RLcp30_131);
    ORcp30_331 = OMcp30_16*RLcp30_231-OMcp30_26*RLcp30_131;
    VIcp30_131 = ORcp30_128+ORcp30_131+qd(1)+ROcp30_45*qd(28);
    VIcp30_231 = ORcp30_228+ORcp30_231+qd(2)+ROcp30_55*qd(28);
    VIcp30_331 = ORcp30_328+ORcp30_331+qd(3)+qd(28)*S5;
    OPcp30_131 = OPcp30_16+ROcp30_16*qdd(31)+qd(31)*(OMcp30_26*ROcp30_36-OMcp30_36*ROcp30_26);
    OPcp30_231 = OPcp30_26+ROcp30_26*qdd(31)-qd(31)*(OMcp30_16*ROcp30_36-OMcp30_36*ROcp30_16);
    OPcp30_331 = OPcp30_36+ROcp30_36*qdd(31)+qd(31)*(OMcp30_16*ROcp30_26-OMcp30_26*ROcp30_16);
    ACcp30_131 = qdd(1)+OMcp30_26*ORcp30_328+OMcp30_26*ORcp30_331-OMcp30_36*ORcp30_228-OMcp30_36*ORcp30_231+OPcp30_26*RLcp30_328+OPcp30_26*...
 RLcp30_331-OPcp30_36*RLcp30_228-OPcp30_36*RLcp30_231+ROcp30_45*qdd(28)+(2.0)*qd(28)*(OMcp30_26*S5-OMcp30_36*ROcp30_55);
    ACcp30_231 = qdd(2)-OMcp30_16*ORcp30_328-OMcp30_16*ORcp30_331+OMcp30_36*ORcp30_128+OMcp30_36*ORcp30_131-OPcp30_16*RLcp30_328-OPcp30_16*...
 RLcp30_331+OPcp30_36*RLcp30_128+OPcp30_36*RLcp30_131+ROcp30_55*qdd(28)-(2.0)*qd(28)*(OMcp30_16*S5-OMcp30_36*ROcp30_45);
    ACcp30_331 = qdd(3)+OMcp30_16*ORcp30_228+OMcp30_16*ORcp30_231-OMcp30_26*ORcp30_128-OMcp30_26*ORcp30_131+OPcp30_16*RLcp30_228+OPcp30_16*...
 RLcp30_231-OPcp30_26*RLcp30_128-OPcp30_26*RLcp30_131+qdd(28)*S5+(2.0)*qd(28)*(OMcp30_16*ROcp30_55-OMcp30_26*ROcp30_45);

% = = Block_1_0_0_31_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp30_131;
    sens.P(2) = POcp30_231;
    sens.P(3) = POcp30_331;
    sens.R(1,1) = ROcp30_16;
    sens.R(1,2) = ROcp30_26;
    sens.R(1,3) = ROcp30_36;
    sens.R(2,1) = ROcp30_431;
    sens.R(2,2) = ROcp30_531;
    sens.R(2,3) = ROcp30_631;
    sens.R(3,1) = ROcp30_731;
    sens.R(3,2) = ROcp30_831;
    sens.R(3,3) = ROcp30_931;
    sens.V(1) = VIcp30_131;
    sens.V(2) = VIcp30_231;
    sens.V(3) = VIcp30_331;
    sens.OM(1) = OMcp30_131;
    sens.OM(2) = OMcp30_231;
    sens.OM(3) = OMcp30_331;
    sens.A(1) = ACcp30_131;
    sens.A(2) = ACcp30_231;
    sens.A(3) = ACcp30_331;
    sens.OMP(1) = OPcp30_131;
    sens.OMP(2) = OPcp30_231;
    sens.OMP(3) = OPcp30_331;
 
% 
case 32, 


% = = Block_1_0_0_32_0_1 = = 
 
% Sensor Kinematics 


    ROcp31_45 = -S4*C5;
    ROcp31_55 = C4*C5;
    ROcp31_75 = S4*S5;
    ROcp31_85 = -C4*S5;
    ROcp31_16 = -(ROcp31_75*S6-C4*C6);
    ROcp31_26 = -(ROcp31_85*S6-S4*C6);
    ROcp31_36 = -C5*S6;
    ROcp31_76 = ROcp31_75*C6+C4*S6;
    ROcp31_86 = ROcp31_85*C6+S4*S6;
    ROcp31_96 = C5*C6;
    OMcp31_15 = qd(5)*C4;
    OMcp31_25 = qd(5)*S4;
    OMcp31_16 = OMcp31_15+ROcp31_45*qd(6);
    OMcp31_26 = OMcp31_25+ROcp31_55*qd(6);
    OMcp31_36 = qd(4)+qd(6)*S5;
    OPcp31_16 = ROcp31_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp31_25*S5-ROcp31_55*qd(4));
    OPcp31_26 = ROcp31_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp31_15*S5-ROcp31_45*qd(4));
    OPcp31_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_32_0_12 = = 
 
% Sensor Kinematics 


    RLcp31_128 = ROcp31_16*s.dpt(1,9)+ROcp31_45*q(28)+ROcp31_76*s.dpt(3,9);
    RLcp31_228 = ROcp31_26*s.dpt(1,9)+ROcp31_55*q(28)+ROcp31_86*s.dpt(3,9);
    RLcp31_328 = ROcp31_36*s.dpt(1,9)+ROcp31_96*s.dpt(3,9)+q(28)*S5;
    ORcp31_128 = OMcp31_26*RLcp31_328-OMcp31_36*RLcp31_228;
    ORcp31_228 = -(OMcp31_16*RLcp31_328-OMcp31_36*RLcp31_128);
    ORcp31_328 = OMcp31_16*RLcp31_228-OMcp31_26*RLcp31_128;

% = = Block_1_0_0_32_0_14 = = 
 
% Sensor Kinematics 


    ROcp31_431 = ROcp31_45*C31+ROcp31_76*S31;
    ROcp31_531 = ROcp31_55*C31+ROcp31_86*S31;
    ROcp31_631 = ROcp31_96*S31+C31*S5;
    ROcp31_731 = -(ROcp31_45*S31-ROcp31_76*C31);
    ROcp31_831 = -(ROcp31_55*S31-ROcp31_86*C31);
    ROcp31_931 = ROcp31_96*C31-S31*S5;
    ROcp31_132 = ROcp31_16*C32+ROcp31_431*S32;
    ROcp31_232 = ROcp31_26*C32+ROcp31_531*S32;
    ROcp31_332 = ROcp31_36*C32+ROcp31_631*S32;
    ROcp31_432 = -(ROcp31_16*S32-ROcp31_431*C32);
    ROcp31_532 = -(ROcp31_26*S32-ROcp31_531*C32);
    ROcp31_632 = -(ROcp31_36*S32-ROcp31_631*C32);
    RLcp31_131 = ROcp31_45*s.dpt(2,45);
    RLcp31_231 = ROcp31_55*s.dpt(2,45);
    RLcp31_331 = s.dpt(2,45)*S5;
    POcp31_131 = RLcp31_128+RLcp31_131+q(1);
    POcp31_231 = RLcp31_228+RLcp31_231+q(2);
    POcp31_331 = RLcp31_328+RLcp31_331+q(3);
    OMcp31_131 = OMcp31_16+ROcp31_16*qd(31);
    OMcp31_231 = OMcp31_26+ROcp31_26*qd(31);
    OMcp31_331 = OMcp31_36+ROcp31_36*qd(31);
    ORcp31_131 = OMcp31_26*RLcp31_331-OMcp31_36*RLcp31_231;
    ORcp31_231 = -(OMcp31_16*RLcp31_331-OMcp31_36*RLcp31_131);
    ORcp31_331 = OMcp31_16*RLcp31_231-OMcp31_26*RLcp31_131;
    VIcp31_131 = ORcp31_128+ORcp31_131+qd(1)+ROcp31_45*qd(28);
    VIcp31_231 = ORcp31_228+ORcp31_231+qd(2)+ROcp31_55*qd(28);
    VIcp31_331 = ORcp31_328+ORcp31_331+qd(3)+qd(28)*S5;
    ACcp31_131 = qdd(1)+OMcp31_26*ORcp31_328+OMcp31_26*ORcp31_331-OMcp31_36*ORcp31_228-OMcp31_36*ORcp31_231+OPcp31_26*RLcp31_328+OPcp31_26*...
 RLcp31_331-OPcp31_36*RLcp31_228-OPcp31_36*RLcp31_231+ROcp31_45*qdd(28)+(2.0)*qd(28)*(OMcp31_26*S5-OMcp31_36*ROcp31_55);
    ACcp31_231 = qdd(2)-OMcp31_16*ORcp31_328-OMcp31_16*ORcp31_331+OMcp31_36*ORcp31_128+OMcp31_36*ORcp31_131-OPcp31_16*RLcp31_328-OPcp31_16*...
 RLcp31_331+OPcp31_36*RLcp31_128+OPcp31_36*RLcp31_131+ROcp31_55*qdd(28)-(2.0)*qd(28)*(OMcp31_16*S5-OMcp31_36*ROcp31_45);
    ACcp31_331 = qdd(3)+OMcp31_16*ORcp31_228+OMcp31_16*ORcp31_231-OMcp31_26*ORcp31_128-OMcp31_26*ORcp31_131+OPcp31_16*RLcp31_228+OPcp31_16*...
 RLcp31_231-OPcp31_26*RLcp31_128-OPcp31_26*RLcp31_131+qdd(28)*S5+(2.0)*qd(28)*(OMcp31_16*ROcp31_55-OMcp31_26*ROcp31_45);
    OMcp31_132 = OMcp31_131+ROcp31_731*qd(32);
    OMcp31_232 = OMcp31_231+ROcp31_831*qd(32);
    OMcp31_332 = OMcp31_331+ROcp31_931*qd(32);
    OPcp31_132 = OPcp31_16+ROcp31_16*qdd(31)+ROcp31_731*qdd(32)+qd(31)*(OMcp31_26*ROcp31_36-OMcp31_36*ROcp31_26)+qd(32)*(OMcp31_231*ROcp31_931-...
 OMcp31_331*ROcp31_831);
    OPcp31_232 = OPcp31_26+ROcp31_26*qdd(31)+ROcp31_831*qdd(32)-qd(31)*(OMcp31_16*ROcp31_36-OMcp31_36*ROcp31_16)-qd(32)*(OMcp31_131*ROcp31_931-...
 OMcp31_331*ROcp31_731);
    OPcp31_332 = OPcp31_36+ROcp31_36*qdd(31)+ROcp31_931*qdd(32)+qd(31)*(OMcp31_16*ROcp31_26-OMcp31_26*ROcp31_16)+qd(32)*(OMcp31_131*ROcp31_831-...
 OMcp31_231*ROcp31_731);

% = = Block_1_0_0_32_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp31_131;
    sens.P(2) = POcp31_231;
    sens.P(3) = POcp31_331;
    sens.R(1,1) = ROcp31_132;
    sens.R(1,2) = ROcp31_232;
    sens.R(1,3) = ROcp31_332;
    sens.R(2,1) = ROcp31_432;
    sens.R(2,2) = ROcp31_532;
    sens.R(2,3) = ROcp31_632;
    sens.R(3,1) = ROcp31_731;
    sens.R(3,2) = ROcp31_831;
    sens.R(3,3) = ROcp31_931;
    sens.V(1) = VIcp31_131;
    sens.V(2) = VIcp31_231;
    sens.V(3) = VIcp31_331;
    sens.OM(1) = OMcp31_132;
    sens.OM(2) = OMcp31_232;
    sens.OM(3) = OMcp31_332;
    sens.A(1) = ACcp31_131;
    sens.A(2) = ACcp31_231;
    sens.A(3) = ACcp31_331;
    sens.OMP(1) = OPcp31_132;
    sens.OMP(2) = OPcp31_232;
    sens.OMP(3) = OPcp31_332;
 
% 
case 33, 


% = = Block_1_0_0_33_0_1 = = 
 
% Sensor Kinematics 


    ROcp32_45 = -S4*C5;
    ROcp32_55 = C4*C5;
    ROcp32_75 = S4*S5;
    ROcp32_85 = -C4*S5;
    ROcp32_16 = -(ROcp32_75*S6-C4*C6);
    ROcp32_26 = -(ROcp32_85*S6-S4*C6);
    ROcp32_36 = -C5*S6;
    ROcp32_76 = ROcp32_75*C6+C4*S6;
    ROcp32_86 = ROcp32_85*C6+S4*S6;
    ROcp32_96 = C5*C6;
    OMcp32_15 = qd(5)*C4;
    OMcp32_25 = qd(5)*S4;
    OMcp32_16 = OMcp32_15+ROcp32_45*qd(6);
    OMcp32_26 = OMcp32_25+ROcp32_55*qd(6);
    OMcp32_36 = qd(4)+qd(6)*S5;
    OPcp32_16 = ROcp32_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp32_25*S5-ROcp32_55*qd(4));
    OPcp32_26 = ROcp32_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp32_15*S5-ROcp32_45*qd(4));
    OPcp32_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_33_0_12 = = 
 
% Sensor Kinematics 


    RLcp32_128 = ROcp32_16*s.dpt(1,9)+ROcp32_45*q(28)+ROcp32_76*s.dpt(3,9);
    RLcp32_228 = ROcp32_26*s.dpt(1,9)+ROcp32_55*q(28)+ROcp32_86*s.dpt(3,9);
    RLcp32_328 = ROcp32_36*s.dpt(1,9)+ROcp32_96*s.dpt(3,9)+q(28)*S5;
    ORcp32_128 = OMcp32_26*RLcp32_328-OMcp32_36*RLcp32_228;
    ORcp32_228 = -(OMcp32_16*RLcp32_328-OMcp32_36*RLcp32_128);
    ORcp32_328 = OMcp32_16*RLcp32_228-OMcp32_26*RLcp32_128;

% = = Block_1_0_0_33_0_15 = = 
 
% Sensor Kinematics 


    RLcp32_133 = Dz332*ROcp32_45;
    RLcp32_233 = Dz332*ROcp32_55;
    RLcp32_333 = Dz332*S5;
    POcp32_133 = RLcp32_128+RLcp32_133+q(1);
    POcp32_233 = RLcp32_228+RLcp32_233+q(2);
    POcp32_333 = RLcp32_328+RLcp32_333+q(3);
    ORcp32_133 = OMcp32_26*RLcp32_333-OMcp32_36*RLcp32_233;
    ORcp32_233 = -(OMcp32_16*RLcp32_333-OMcp32_36*RLcp32_133);
    ORcp32_333 = OMcp32_16*RLcp32_233-OMcp32_26*RLcp32_133;
    VIcp32_133 = ORcp32_128+ORcp32_133+qd(1)+ROcp32_45*qd(28)+ROcp32_45*qd(33);
    VIcp32_233 = ORcp32_228+ORcp32_233+qd(2)+ROcp32_55*qd(28)+ROcp32_55*qd(33);
    VIcp32_333 = ORcp32_328+ORcp32_333+qd(3)+qd(28)*S5+qd(33)*S5;
    ACcp32_133 = qdd(1)+OMcp32_26*ORcp32_328+OMcp32_26*ORcp32_333-OMcp32_36*ORcp32_228-OMcp32_36*ORcp32_233+OPcp32_26*RLcp32_328+OPcp32_26*...
 RLcp32_333-OPcp32_36*RLcp32_228-OPcp32_36*RLcp32_233+ROcp32_45*qdd(28)+ROcp32_45*qdd(33)+(2.0)*qd(28)*(OMcp32_26*S5-OMcp32_36*ROcp32_55)+(2.0)*qd(33)*(...
 OMcp32_26*S5-OMcp32_36*ROcp32_55);
    ACcp32_233 = qdd(2)-OMcp32_16*ORcp32_328-OMcp32_16*ORcp32_333+OMcp32_36*ORcp32_128+OMcp32_36*ORcp32_133-OPcp32_16*RLcp32_328-OPcp32_16*...
 RLcp32_333+OPcp32_36*RLcp32_128+OPcp32_36*RLcp32_133+ROcp32_55*qdd(28)+ROcp32_55*qdd(33)-(2.0)*qd(28)*(OMcp32_16*S5-OMcp32_36*ROcp32_45)-(2.0)*qd(33)*(...
 OMcp32_16*S5-OMcp32_36*ROcp32_45);
    ACcp32_333 = qdd(3)+OMcp32_16*ORcp32_228+OMcp32_16*ORcp32_233-OMcp32_26*ORcp32_128-OMcp32_26*ORcp32_133+OPcp32_16*RLcp32_228+OPcp32_16*...
 RLcp32_233-OPcp32_26*RLcp32_128-OPcp32_26*RLcp32_133+qdd(28)*S5+qdd(33)*S5+(2.0)*qd(28)*(OMcp32_16*ROcp32_55-OMcp32_26*ROcp32_45)+(2.0)*qd(33)*(OMcp32_16*...
 ROcp32_55-OMcp32_26*ROcp32_45);

% = = Block_1_0_0_33_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp32_133;
    sens.P(2) = POcp32_233;
    sens.P(3) = POcp32_333;
    sens.R(1,1) = ROcp32_16;
    sens.R(1,2) = ROcp32_26;
    sens.R(1,3) = ROcp32_36;
    sens.R(2,1) = ROcp32_45;
    sens.R(2,2) = ROcp32_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp32_76;
    sens.R(3,2) = ROcp32_86;
    sens.R(3,3) = ROcp32_96;
    sens.V(1) = VIcp32_133;
    sens.V(2) = VIcp32_233;
    sens.V(3) = VIcp32_333;
    sens.OM(1) = OMcp32_16;
    sens.OM(2) = OMcp32_26;
    sens.OM(3) = OMcp32_36;
    sens.A(1) = ACcp32_133;
    sens.A(2) = ACcp32_233;
    sens.A(3) = ACcp32_333;
    sens.OMP(1) = OPcp32_16;
    sens.OMP(2) = OPcp32_26;
    sens.OMP(3) = OPcp32_36;
 
% 
case 34, 


% = = Block_1_0_0_34_0_1 = = 
 
% Sensor Kinematics 


    ROcp33_45 = -S4*C5;
    ROcp33_55 = C4*C5;
    ROcp33_75 = S4*S5;
    ROcp33_85 = -C4*S5;
    ROcp33_16 = -(ROcp33_75*S6-C4*C6);
    ROcp33_26 = -(ROcp33_85*S6-S4*C6);
    ROcp33_36 = -C5*S6;
    ROcp33_76 = ROcp33_75*C6+C4*S6;
    ROcp33_86 = ROcp33_85*C6+S4*S6;
    ROcp33_96 = C5*C6;
    OMcp33_15 = qd(5)*C4;
    OMcp33_25 = qd(5)*S4;
    OMcp33_16 = OMcp33_15+ROcp33_45*qd(6);
    OMcp33_26 = OMcp33_25+ROcp33_55*qd(6);
    OMcp33_36 = qd(4)+qd(6)*S5;
    OPcp33_16 = ROcp33_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp33_25*S5-ROcp33_55*qd(4));
    OPcp33_26 = ROcp33_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp33_15*S5-ROcp33_45*qd(4));
    OPcp33_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_34_0_12 = = 
 
% Sensor Kinematics 


    RLcp33_128 = ROcp33_16*s.dpt(1,9)+ROcp33_45*q(28)+ROcp33_76*s.dpt(3,9);
    RLcp33_228 = ROcp33_26*s.dpt(1,9)+ROcp33_55*q(28)+ROcp33_86*s.dpt(3,9);
    RLcp33_328 = ROcp33_36*s.dpt(1,9)+ROcp33_96*s.dpt(3,9)+q(28)*S5;
    ORcp33_128 = OMcp33_26*RLcp33_328-OMcp33_36*RLcp33_228;
    ORcp33_228 = -(OMcp33_16*RLcp33_328-OMcp33_36*RLcp33_128);
    ORcp33_328 = OMcp33_16*RLcp33_228-OMcp33_26*RLcp33_128;

% = = Block_1_0_0_34_0_15 = = 
 
% Sensor Kinematics 


    ROcp33_134 = ROcp33_16*C34+ROcp33_45*S34;
    ROcp33_234 = ROcp33_26*C34+ROcp33_55*S34;
    ROcp33_334 = ROcp33_36*C34+S34*S5;
    ROcp33_434 = -(ROcp33_16*S34-ROcp33_45*C34);
    ROcp33_534 = -(ROcp33_26*S34-ROcp33_55*C34);
    ROcp33_634 = -(ROcp33_36*S34-C34*S5);
    RLcp33_133 = Dz332*ROcp33_45;
    RLcp33_233 = Dz332*ROcp33_55;
    RLcp33_333 = Dz332*S5;
    POcp33_133 = RLcp33_128+RLcp33_133+q(1);
    POcp33_233 = RLcp33_228+RLcp33_233+q(2);
    POcp33_333 = RLcp33_328+RLcp33_333+q(3);
    ORcp33_133 = OMcp33_26*RLcp33_333-OMcp33_36*RLcp33_233;
    ORcp33_233 = -(OMcp33_16*RLcp33_333-OMcp33_36*RLcp33_133);
    ORcp33_333 = OMcp33_16*RLcp33_233-OMcp33_26*RLcp33_133;
    VIcp33_133 = ORcp33_128+ORcp33_133+qd(1)+ROcp33_45*qd(28)+ROcp33_45*qd(33);
    VIcp33_233 = ORcp33_228+ORcp33_233+qd(2)+ROcp33_55*qd(28)+ROcp33_55*qd(33);
    VIcp33_333 = ORcp33_328+ORcp33_333+qd(3)+qd(28)*S5+qd(33)*S5;
    ACcp33_133 = qdd(1)+OMcp33_26*ORcp33_328+OMcp33_26*ORcp33_333-OMcp33_36*ORcp33_228-OMcp33_36*ORcp33_233+OPcp33_26*RLcp33_328+OPcp33_26*...
 RLcp33_333-OPcp33_36*RLcp33_228-OPcp33_36*RLcp33_233+ROcp33_45*qdd(28)+ROcp33_45*qdd(33)+(2.0)*qd(28)*(OMcp33_26*S5-OMcp33_36*ROcp33_55)+(2.0)*qd(33)*(...
 OMcp33_26*S5-OMcp33_36*ROcp33_55);
    ACcp33_233 = qdd(2)-OMcp33_16*ORcp33_328-OMcp33_16*ORcp33_333+OMcp33_36*ORcp33_128+OMcp33_36*ORcp33_133-OPcp33_16*RLcp33_328-OPcp33_16*...
 RLcp33_333+OPcp33_36*RLcp33_128+OPcp33_36*RLcp33_133+ROcp33_55*qdd(28)+ROcp33_55*qdd(33)-(2.0)*qd(28)*(OMcp33_16*S5-OMcp33_36*ROcp33_45)-(2.0)*qd(33)*(...
 OMcp33_16*S5-OMcp33_36*ROcp33_45);
    ACcp33_333 = qdd(3)+OMcp33_16*ORcp33_228+OMcp33_16*ORcp33_233-OMcp33_26*ORcp33_128-OMcp33_26*ORcp33_133+OPcp33_16*RLcp33_228+OPcp33_16*...
 RLcp33_233-OPcp33_26*RLcp33_128-OPcp33_26*RLcp33_133+qdd(28)*S5+qdd(33)*S5+(2.0)*qd(28)*(OMcp33_16*ROcp33_55-OMcp33_26*ROcp33_45)+(2.0)*qd(33)*(OMcp33_16*...
 ROcp33_55-OMcp33_26*ROcp33_45);
    OMcp33_134 = OMcp33_16+ROcp33_76*qd(34);
    OMcp33_234 = OMcp33_26+ROcp33_86*qd(34);
    OMcp33_334 = OMcp33_36+ROcp33_96*qd(34);
    OPcp33_134 = OPcp33_16+ROcp33_76*qdd(34)+qd(34)*(OMcp33_26*ROcp33_96-OMcp33_36*ROcp33_86);
    OPcp33_234 = OPcp33_26+ROcp33_86*qdd(34)-qd(34)*(OMcp33_16*ROcp33_96-OMcp33_36*ROcp33_76);
    OPcp33_334 = OPcp33_36+ROcp33_96*qdd(34)+qd(34)*(OMcp33_16*ROcp33_86-OMcp33_26*ROcp33_76);

% = = Block_1_0_0_34_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp33_133;
    sens.P(2) = POcp33_233;
    sens.P(3) = POcp33_333;
    sens.R(1,1) = ROcp33_134;
    sens.R(1,2) = ROcp33_234;
    sens.R(1,3) = ROcp33_334;
    sens.R(2,1) = ROcp33_434;
    sens.R(2,2) = ROcp33_534;
    sens.R(2,3) = ROcp33_634;
    sens.R(3,1) = ROcp33_76;
    sens.R(3,2) = ROcp33_86;
    sens.R(3,3) = ROcp33_96;
    sens.V(1) = VIcp33_133;
    sens.V(2) = VIcp33_233;
    sens.V(3) = VIcp33_333;
    sens.OM(1) = OMcp33_134;
    sens.OM(2) = OMcp33_234;
    sens.OM(3) = OMcp33_334;
    sens.A(1) = ACcp33_133;
    sens.A(2) = ACcp33_233;
    sens.A(3) = ACcp33_333;
    sens.OMP(1) = OPcp33_134;
    sens.OMP(2) = OPcp33_234;
    sens.OMP(3) = OPcp33_334;
 
% 
case 35, 


% = = Block_1_0_0_35_0_1 = = 
 
% Sensor Kinematics 


    ROcp34_45 = -S4*C5;
    ROcp34_55 = C4*C5;
    ROcp34_75 = S4*S5;
    ROcp34_85 = -C4*S5;
    ROcp34_16 = -(ROcp34_75*S6-C4*C6);
    ROcp34_26 = -(ROcp34_85*S6-S4*C6);
    ROcp34_36 = -C5*S6;
    ROcp34_76 = ROcp34_75*C6+C4*S6;
    ROcp34_86 = ROcp34_85*C6+S4*S6;
    ROcp34_96 = C5*C6;
    OMcp34_15 = qd(5)*C4;
    OMcp34_25 = qd(5)*S4;
    OMcp34_16 = OMcp34_15+ROcp34_45*qd(6);
    OMcp34_26 = OMcp34_25+ROcp34_55*qd(6);
    OMcp34_36 = qd(4)+qd(6)*S5;
    OPcp34_16 = ROcp34_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp34_25*S5-ROcp34_55*qd(4));
    OPcp34_26 = ROcp34_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp34_15*S5-ROcp34_45*qd(4));
    OPcp34_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_35_0_16 = = 
 
% Sensor Kinematics 


    ROcp34_435 = ROcp34_45*C35+ROcp34_76*S35;
    ROcp34_535 = ROcp34_55*C35+ROcp34_86*S35;
    ROcp34_635 = ROcp34_96*S35+C35*S5;
    ROcp34_735 = -(ROcp34_45*S35-ROcp34_76*C35);
    ROcp34_835 = -(ROcp34_55*S35-ROcp34_86*C35);
    ROcp34_935 = ROcp34_96*C35-S35*S5;
    RLcp34_135 = ROcp34_16*s.dpt(1,12)+ROcp34_45*s.dpt(2,12)+ROcp34_76*s.dpt(3,12);
    RLcp34_235 = ROcp34_26*s.dpt(1,12)+ROcp34_55*s.dpt(2,12)+ROcp34_86*s.dpt(3,12);
    RLcp34_335 = ROcp34_36*s.dpt(1,12)+ROcp34_96*s.dpt(3,12)+s.dpt(2,12)*S5;
    POcp34_135 = RLcp34_135+q(1);
    POcp34_235 = RLcp34_235+q(2);
    POcp34_335 = RLcp34_335+q(3);
    OMcp34_135 = OMcp34_16+ROcp34_16*qd(35);
    OMcp34_235 = OMcp34_26+ROcp34_26*qd(35);
    OMcp34_335 = OMcp34_36+ROcp34_36*qd(35);
    ORcp34_135 = OMcp34_26*RLcp34_335-OMcp34_36*RLcp34_235;
    ORcp34_235 = -(OMcp34_16*RLcp34_335-OMcp34_36*RLcp34_135);
    ORcp34_335 = OMcp34_16*RLcp34_235-OMcp34_26*RLcp34_135;
    VIcp34_135 = ORcp34_135+qd(1);
    VIcp34_235 = ORcp34_235+qd(2);
    VIcp34_335 = ORcp34_335+qd(3);
    OPcp34_135 = OPcp34_16+ROcp34_16*qdd(35)+qd(35)*(OMcp34_26*ROcp34_36-OMcp34_36*ROcp34_26);
    OPcp34_235 = OPcp34_26+ROcp34_26*qdd(35)-qd(35)*(OMcp34_16*ROcp34_36-OMcp34_36*ROcp34_16);
    OPcp34_335 = OPcp34_36+ROcp34_36*qdd(35)+qd(35)*(OMcp34_16*ROcp34_26-OMcp34_26*ROcp34_16);
    ACcp34_135 = qdd(1)+OMcp34_26*ORcp34_335-OMcp34_36*ORcp34_235+OPcp34_26*RLcp34_335-OPcp34_36*RLcp34_235;
    ACcp34_235 = qdd(2)-OMcp34_16*ORcp34_335+OMcp34_36*ORcp34_135-OPcp34_16*RLcp34_335+OPcp34_36*RLcp34_135;
    ACcp34_335 = qdd(3)+OMcp34_16*ORcp34_235-OMcp34_26*ORcp34_135+OPcp34_16*RLcp34_235-OPcp34_26*RLcp34_135;

% = = Block_1_0_0_35_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp34_135;
    sens.P(2) = POcp34_235;
    sens.P(3) = POcp34_335;
    sens.R(1,1) = ROcp34_16;
    sens.R(1,2) = ROcp34_26;
    sens.R(1,3) = ROcp34_36;
    sens.R(2,1) = ROcp34_435;
    sens.R(2,2) = ROcp34_535;
    sens.R(2,3) = ROcp34_635;
    sens.R(3,1) = ROcp34_735;
    sens.R(3,2) = ROcp34_835;
    sens.R(3,3) = ROcp34_935;
    sens.V(1) = VIcp34_135;
    sens.V(2) = VIcp34_235;
    sens.V(3) = VIcp34_335;
    sens.OM(1) = OMcp34_135;
    sens.OM(2) = OMcp34_235;
    sens.OM(3) = OMcp34_335;
    sens.A(1) = ACcp34_135;
    sens.A(2) = ACcp34_235;
    sens.A(3) = ACcp34_335;
    sens.OMP(1) = OPcp34_135;
    sens.OMP(2) = OPcp34_235;
    sens.OMP(3) = OPcp34_335;
 
% 
case 36, 


% = = Block_1_0_0_36_0_1 = = 
 
% Sensor Kinematics 


    ROcp35_45 = -S4*C5;
    ROcp35_55 = C4*C5;
    ROcp35_75 = S4*S5;
    ROcp35_85 = -C4*S5;
    ROcp35_16 = -(ROcp35_75*S6-C4*C6);
    ROcp35_26 = -(ROcp35_85*S6-S4*C6);
    ROcp35_36 = -C5*S6;
    ROcp35_76 = ROcp35_75*C6+C4*S6;
    ROcp35_86 = ROcp35_85*C6+S4*S6;
    ROcp35_96 = C5*C6;
    OMcp35_15 = qd(5)*C4;
    OMcp35_25 = qd(5)*S4;
    OMcp35_16 = OMcp35_15+ROcp35_45*qd(6);
    OMcp35_26 = OMcp35_25+ROcp35_55*qd(6);
    OMcp35_36 = qd(4)+qd(6)*S5;
    OPcp35_16 = ROcp35_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp35_25*S5-ROcp35_55*qd(4));
    OPcp35_26 = ROcp35_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp35_15*S5-ROcp35_45*qd(4));
    OPcp35_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_36_0_16 = = 
 
% Sensor Kinematics 


    ROcp35_435 = ROcp35_45*C35+ROcp35_76*S35;
    ROcp35_535 = ROcp35_55*C35+ROcp35_86*S35;
    ROcp35_635 = ROcp35_96*S35+C35*S5;
    ROcp35_735 = -(ROcp35_45*S35-ROcp35_76*C35);
    ROcp35_835 = -(ROcp35_55*S35-ROcp35_86*C35);
    ROcp35_935 = ROcp35_96*C35-S35*S5;
    ROcp35_436 = ROcp35_435*C36+ROcp35_735*S36;
    ROcp35_536 = ROcp35_535*C36+ROcp35_835*S36;
    ROcp35_636 = ROcp35_635*C36+ROcp35_935*S36;
    ROcp35_736 = -(ROcp35_435*S36-ROcp35_735*C36);
    ROcp35_836 = -(ROcp35_535*S36-ROcp35_835*C36);
    ROcp35_936 = -(ROcp35_635*S36-ROcp35_935*C36);
    RLcp35_135 = ROcp35_16*s.dpt(1,12)+ROcp35_45*s.dpt(2,12)+ROcp35_76*s.dpt(3,12);
    RLcp35_235 = ROcp35_26*s.dpt(1,12)+ROcp35_55*s.dpt(2,12)+ROcp35_86*s.dpt(3,12);
    RLcp35_335 = ROcp35_36*s.dpt(1,12)+ROcp35_96*s.dpt(3,12)+s.dpt(2,12)*S5;
    OMcp35_135 = OMcp35_16+ROcp35_16*qd(35);
    OMcp35_235 = OMcp35_26+ROcp35_26*qd(35);
    OMcp35_335 = OMcp35_36+ROcp35_36*qd(35);
    ORcp35_135 = OMcp35_26*RLcp35_335-OMcp35_36*RLcp35_235;
    ORcp35_235 = -(OMcp35_16*RLcp35_335-OMcp35_36*RLcp35_135);
    ORcp35_335 = OMcp35_16*RLcp35_235-OMcp35_26*RLcp35_135;
    OPcp35_135 = OPcp35_16+ROcp35_16*qdd(35)+qd(35)*(OMcp35_26*ROcp35_36-OMcp35_36*ROcp35_26);
    OPcp35_235 = OPcp35_26+ROcp35_26*qdd(35)-qd(35)*(OMcp35_16*ROcp35_36-OMcp35_36*ROcp35_16);
    OPcp35_335 = OPcp35_36+ROcp35_36*qdd(35)+qd(35)*(OMcp35_16*ROcp35_26-OMcp35_26*ROcp35_16);
    RLcp35_136 = ROcp35_435*s.dpt(2,49);
    RLcp35_236 = ROcp35_535*s.dpt(2,49);
    RLcp35_336 = ROcp35_635*s.dpt(2,49);
    POcp35_136 = RLcp35_135+RLcp35_136+q(1);
    POcp35_236 = RLcp35_235+RLcp35_236+q(2);
    POcp35_336 = RLcp35_335+RLcp35_336+q(3);
    OMcp35_136 = OMcp35_135+ROcp35_16*qd(36);
    OMcp35_236 = OMcp35_235+ROcp35_26*qd(36);
    OMcp35_336 = OMcp35_335+ROcp35_36*qd(36);
    ORcp35_136 = OMcp35_235*RLcp35_336-OMcp35_335*RLcp35_236;
    ORcp35_236 = -(OMcp35_135*RLcp35_336-OMcp35_335*RLcp35_136);
    ORcp35_336 = OMcp35_135*RLcp35_236-OMcp35_235*RLcp35_136;
    VIcp35_136 = ORcp35_135+ORcp35_136+qd(1);
    VIcp35_236 = ORcp35_235+ORcp35_236+qd(2);
    VIcp35_336 = ORcp35_335+ORcp35_336+qd(3);
    OPcp35_136 = OPcp35_135+ROcp35_16*qdd(36)+qd(36)*(OMcp35_235*ROcp35_36-OMcp35_335*ROcp35_26);
    OPcp35_236 = OPcp35_235+ROcp35_26*qdd(36)-qd(36)*(OMcp35_135*ROcp35_36-OMcp35_335*ROcp35_16);
    OPcp35_336 = OPcp35_335+ROcp35_36*qdd(36)+qd(36)*(OMcp35_135*ROcp35_26-OMcp35_235*ROcp35_16);
    ACcp35_136 = qdd(1)+OMcp35_235*ORcp35_336+OMcp35_26*ORcp35_335-OMcp35_335*ORcp35_236-OMcp35_36*ORcp35_235+OPcp35_235*RLcp35_336+OPcp35_26*...
 RLcp35_335-OPcp35_335*RLcp35_236-OPcp35_36*RLcp35_235;
    ACcp35_236 = qdd(2)-OMcp35_135*ORcp35_336-OMcp35_16*ORcp35_335+OMcp35_335*ORcp35_136+OMcp35_36*ORcp35_135-OPcp35_135*RLcp35_336-OPcp35_16*...
 RLcp35_335+OPcp35_335*RLcp35_136+OPcp35_36*RLcp35_135;
    ACcp35_336 = qdd(3)+OMcp35_135*ORcp35_236+OMcp35_16*ORcp35_235-OMcp35_235*ORcp35_136-OMcp35_26*ORcp35_135+OPcp35_135*RLcp35_236+OPcp35_16*...
 RLcp35_235-OPcp35_235*RLcp35_136-OPcp35_26*RLcp35_135;

% = = Block_1_0_0_36_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp35_136;
    sens.P(2) = POcp35_236;
    sens.P(3) = POcp35_336;
    sens.R(1,1) = ROcp35_16;
    sens.R(1,2) = ROcp35_26;
    sens.R(1,3) = ROcp35_36;
    sens.R(2,1) = ROcp35_436;
    sens.R(2,2) = ROcp35_536;
    sens.R(2,3) = ROcp35_636;
    sens.R(3,1) = ROcp35_736;
    sens.R(3,2) = ROcp35_836;
    sens.R(3,3) = ROcp35_936;
    sens.V(1) = VIcp35_136;
    sens.V(2) = VIcp35_236;
    sens.V(3) = VIcp35_336;
    sens.OM(1) = OMcp35_136;
    sens.OM(2) = OMcp35_236;
    sens.OM(3) = OMcp35_336;
    sens.A(1) = ACcp35_136;
    sens.A(2) = ACcp35_236;
    sens.A(3) = ACcp35_336;
    sens.OMP(1) = OPcp35_136;
    sens.OMP(2) = OPcp35_236;
    sens.OMP(3) = OPcp35_336;
 
% 
case 37, 


% = = Block_1_0_0_37_0_1 = = 
 
% Sensor Kinematics 


    ROcp36_45 = -S4*C5;
    ROcp36_55 = C4*C5;
    ROcp36_75 = S4*S5;
    ROcp36_85 = -C4*S5;
    ROcp36_16 = -(ROcp36_75*S6-C4*C6);
    ROcp36_26 = -(ROcp36_85*S6-S4*C6);
    ROcp36_36 = -C5*S6;
    ROcp36_76 = ROcp36_75*C6+C4*S6;
    ROcp36_86 = ROcp36_85*C6+S4*S6;
    ROcp36_96 = C5*C6;
    OMcp36_15 = qd(5)*C4;
    OMcp36_25 = qd(5)*S4;
    OMcp36_16 = OMcp36_15+ROcp36_45*qd(6);
    OMcp36_26 = OMcp36_25+ROcp36_55*qd(6);
    OMcp36_36 = qd(4)+qd(6)*S5;
    OPcp36_16 = ROcp36_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp36_25*S5-ROcp36_55*qd(4));
    OPcp36_26 = ROcp36_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp36_15*S5-ROcp36_45*qd(4));
    OPcp36_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_37_0_16 = = 
 
% Sensor Kinematics 


    ROcp36_435 = ROcp36_45*C35+ROcp36_76*S35;
    ROcp36_535 = ROcp36_55*C35+ROcp36_86*S35;
    ROcp36_635 = ROcp36_96*S35+C35*S5;
    ROcp36_735 = -(ROcp36_45*S35-ROcp36_76*C35);
    ROcp36_835 = -(ROcp36_55*S35-ROcp36_86*C35);
    ROcp36_935 = ROcp36_96*C35-S35*S5;
    ROcp36_436 = ROcp36_435*C36+ROcp36_735*S36;
    ROcp36_536 = ROcp36_535*C36+ROcp36_835*S36;
    ROcp36_636 = ROcp36_635*C36+ROcp36_935*S36;
    ROcp36_736 = -(ROcp36_435*S36-ROcp36_735*C36);
    ROcp36_836 = -(ROcp36_535*S36-ROcp36_835*C36);
    ROcp36_936 = -(ROcp36_635*S36-ROcp36_935*C36);
    ROcp36_137 = ROcp36_16*C37-ROcp36_736*S37;
    ROcp36_237 = ROcp36_26*C37-ROcp36_836*S37;
    ROcp36_337 = ROcp36_36*C37-ROcp36_936*S37;
    ROcp36_737 = ROcp36_16*S37+ROcp36_736*C37;
    ROcp36_837 = ROcp36_26*S37+ROcp36_836*C37;
    ROcp36_937 = ROcp36_36*S37+ROcp36_936*C37;
    RLcp36_135 = ROcp36_16*s.dpt(1,12)+ROcp36_45*s.dpt(2,12)+ROcp36_76*s.dpt(3,12);
    RLcp36_235 = ROcp36_26*s.dpt(1,12)+ROcp36_55*s.dpt(2,12)+ROcp36_86*s.dpt(3,12);
    RLcp36_335 = ROcp36_36*s.dpt(1,12)+ROcp36_96*s.dpt(3,12)+s.dpt(2,12)*S5;
    OMcp36_135 = OMcp36_16+ROcp36_16*qd(35);
    OMcp36_235 = OMcp36_26+ROcp36_26*qd(35);
    OMcp36_335 = OMcp36_36+ROcp36_36*qd(35);
    ORcp36_135 = OMcp36_26*RLcp36_335-OMcp36_36*RLcp36_235;
    ORcp36_235 = -(OMcp36_16*RLcp36_335-OMcp36_36*RLcp36_135);
    ORcp36_335 = OMcp36_16*RLcp36_235-OMcp36_26*RLcp36_135;
    OPcp36_135 = OPcp36_16+ROcp36_16*qdd(35)+qd(35)*(OMcp36_26*ROcp36_36-OMcp36_36*ROcp36_26);
    OPcp36_235 = OPcp36_26+ROcp36_26*qdd(35)-qd(35)*(OMcp36_16*ROcp36_36-OMcp36_36*ROcp36_16);
    OPcp36_335 = OPcp36_36+ROcp36_36*qdd(35)+qd(35)*(OMcp36_16*ROcp36_26-OMcp36_26*ROcp36_16);
    RLcp36_136 = ROcp36_435*s.dpt(2,49);
    RLcp36_236 = ROcp36_535*s.dpt(2,49);
    RLcp36_336 = ROcp36_635*s.dpt(2,49);
    OMcp36_136 = OMcp36_135+ROcp36_16*qd(36);
    OMcp36_236 = OMcp36_235+ROcp36_26*qd(36);
    OMcp36_336 = OMcp36_335+ROcp36_36*qd(36);
    ORcp36_136 = OMcp36_235*RLcp36_336-OMcp36_335*RLcp36_236;
    ORcp36_236 = -(OMcp36_135*RLcp36_336-OMcp36_335*RLcp36_136);
    ORcp36_336 = OMcp36_135*RLcp36_236-OMcp36_235*RLcp36_136;
    OPcp36_136 = OPcp36_135+ROcp36_16*qdd(36)+qd(36)*(OMcp36_235*ROcp36_36-OMcp36_335*ROcp36_26);
    OPcp36_236 = OPcp36_235+ROcp36_26*qdd(36)-qd(36)*(OMcp36_135*ROcp36_36-OMcp36_335*ROcp36_16);
    OPcp36_336 = OPcp36_335+ROcp36_36*qdd(36)+qd(36)*(OMcp36_135*ROcp36_26-OMcp36_235*ROcp36_16);
    RLcp36_137 = ROcp36_736*s.dpt(3,52);
    RLcp36_237 = ROcp36_836*s.dpt(3,52);
    RLcp36_337 = ROcp36_936*s.dpt(3,52);
    POcp36_137 = RLcp36_135+RLcp36_136+RLcp36_137+q(1);
    POcp36_237 = RLcp36_235+RLcp36_236+RLcp36_237+q(2);
    POcp36_337 = RLcp36_335+RLcp36_336+RLcp36_337+q(3);
    OMcp36_137 = OMcp36_136+ROcp36_436*qd(37);
    OMcp36_237 = OMcp36_236+ROcp36_536*qd(37);
    OMcp36_337 = OMcp36_336+ROcp36_636*qd(37);
    ORcp36_137 = OMcp36_236*RLcp36_337-OMcp36_336*RLcp36_237;
    ORcp36_237 = -(OMcp36_136*RLcp36_337-OMcp36_336*RLcp36_137);
    ORcp36_337 = OMcp36_136*RLcp36_237-OMcp36_236*RLcp36_137;
    VIcp36_137 = ORcp36_135+ORcp36_136+ORcp36_137+qd(1);
    VIcp36_237 = ORcp36_235+ORcp36_236+ORcp36_237+qd(2);
    VIcp36_337 = ORcp36_335+ORcp36_336+ORcp36_337+qd(3);
    OPcp36_137 = OPcp36_136+ROcp36_436*qdd(37)+qd(37)*(OMcp36_236*ROcp36_636-OMcp36_336*ROcp36_536);
    OPcp36_237 = OPcp36_236+ROcp36_536*qdd(37)-qd(37)*(OMcp36_136*ROcp36_636-OMcp36_336*ROcp36_436);
    OPcp36_337 = OPcp36_336+ROcp36_636*qdd(37)+qd(37)*(OMcp36_136*ROcp36_536-OMcp36_236*ROcp36_436);
    ACcp36_137 = qdd(1)+OMcp36_235*ORcp36_336+OMcp36_236*ORcp36_337+OMcp36_26*ORcp36_335-OMcp36_335*ORcp36_236-OMcp36_336*ORcp36_237-OMcp36_36*...
 ORcp36_235+OPcp36_235*RLcp36_336+OPcp36_236*RLcp36_337+OPcp36_26*RLcp36_335-OPcp36_335*RLcp36_236-OPcp36_336*RLcp36_237-OPcp36_36*RLcp36_235;
    ACcp36_237 = qdd(2)-OMcp36_135*ORcp36_336-OMcp36_136*ORcp36_337-OMcp36_16*ORcp36_335+OMcp36_335*ORcp36_136+OMcp36_336*ORcp36_137+OMcp36_36*...
 ORcp36_135-OPcp36_135*RLcp36_336-OPcp36_136*RLcp36_337-OPcp36_16*RLcp36_335+OPcp36_335*RLcp36_136+OPcp36_336*RLcp36_137+OPcp36_36*RLcp36_135;
    ACcp36_337 = qdd(3)+OMcp36_135*ORcp36_236+OMcp36_136*ORcp36_237+OMcp36_16*ORcp36_235-OMcp36_235*ORcp36_136-OMcp36_236*ORcp36_137-OMcp36_26*...
 ORcp36_135+OPcp36_135*RLcp36_236+OPcp36_136*RLcp36_237+OPcp36_16*RLcp36_235-OPcp36_235*RLcp36_136-OPcp36_236*RLcp36_137-OPcp36_26*RLcp36_135;

% = = Block_1_0_0_37_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp36_137;
    sens.P(2) = POcp36_237;
    sens.P(3) = POcp36_337;
    sens.R(1,1) = ROcp36_137;
    sens.R(1,2) = ROcp36_237;
    sens.R(1,3) = ROcp36_337;
    sens.R(2,1) = ROcp36_436;
    sens.R(2,2) = ROcp36_536;
    sens.R(2,3) = ROcp36_636;
    sens.R(3,1) = ROcp36_737;
    sens.R(3,2) = ROcp36_837;
    sens.R(3,3) = ROcp36_937;
    sens.V(1) = VIcp36_137;
    sens.V(2) = VIcp36_237;
    sens.V(3) = VIcp36_337;
    sens.OM(1) = OMcp36_137;
    sens.OM(2) = OMcp36_237;
    sens.OM(3) = OMcp36_337;
    sens.A(1) = ACcp36_137;
    sens.A(2) = ACcp36_237;
    sens.A(3) = ACcp36_337;
    sens.OMP(1) = OPcp36_137;
    sens.OMP(2) = OPcp36_237;
    sens.OMP(3) = OPcp36_337;
 
% 
case 38, 


% = = Block_1_0_0_38_0_1 = = 
 
% Sensor Kinematics 


    ROcp37_45 = -S4*C5;
    ROcp37_55 = C4*C5;
    ROcp37_75 = S4*S5;
    ROcp37_85 = -C4*S5;
    ROcp37_16 = -(ROcp37_75*S6-C4*C6);
    ROcp37_26 = -(ROcp37_85*S6-S4*C6);
    ROcp37_36 = -C5*S6;
    ROcp37_76 = ROcp37_75*C6+C4*S6;
    ROcp37_86 = ROcp37_85*C6+S4*S6;
    ROcp37_96 = C5*C6;
    OMcp37_15 = qd(5)*C4;
    OMcp37_25 = qd(5)*S4;
    OMcp37_16 = OMcp37_15+ROcp37_45*qd(6);
    OMcp37_26 = OMcp37_25+ROcp37_55*qd(6);
    OMcp37_36 = qd(4)+qd(6)*S5;
    OPcp37_16 = ROcp37_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp37_25*S5-ROcp37_55*qd(4));
    OPcp37_26 = ROcp37_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp37_15*S5-ROcp37_45*qd(4));
    OPcp37_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_38_0_17 = = 
 
% Sensor Kinematics 


    ROcp37_438 = ROcp37_45*C38+ROcp37_76*S38;
    ROcp37_538 = ROcp37_55*C38+ROcp37_86*S38;
    ROcp37_638 = ROcp37_96*S38+C38*S5;
    ROcp37_738 = -(ROcp37_45*S38-ROcp37_76*C38);
    ROcp37_838 = -(ROcp37_55*S38-ROcp37_86*C38);
    ROcp37_938 = ROcp37_96*C38-S38*S5;
    RLcp37_138 = ROcp37_16*s.dpt(1,13)+ROcp37_45*s.dpt(2,13)+ROcp37_76*s.dpt(3,13);
    RLcp37_238 = ROcp37_26*s.dpt(1,13)+ROcp37_55*s.dpt(2,13)+ROcp37_86*s.dpt(3,13);
    RLcp37_338 = ROcp37_36*s.dpt(1,13)+ROcp37_96*s.dpt(3,13)+s.dpt(2,13)*S5;
    POcp37_138 = RLcp37_138+q(1);
    POcp37_238 = RLcp37_238+q(2);
    POcp37_338 = RLcp37_338+q(3);
    OMcp37_138 = OMcp37_16+ROcp37_16*qd(38);
    OMcp37_238 = OMcp37_26+ROcp37_26*qd(38);
    OMcp37_338 = OMcp37_36+ROcp37_36*qd(38);
    ORcp37_138 = OMcp37_26*RLcp37_338-OMcp37_36*RLcp37_238;
    ORcp37_238 = -(OMcp37_16*RLcp37_338-OMcp37_36*RLcp37_138);
    ORcp37_338 = OMcp37_16*RLcp37_238-OMcp37_26*RLcp37_138;
    VIcp37_138 = ORcp37_138+qd(1);
    VIcp37_238 = ORcp37_238+qd(2);
    VIcp37_338 = ORcp37_338+qd(3);
    OPcp37_138 = OPcp37_16+ROcp37_16*qdd(38)+qd(38)*(OMcp37_26*ROcp37_36-OMcp37_36*ROcp37_26);
    OPcp37_238 = OPcp37_26+ROcp37_26*qdd(38)-qd(38)*(OMcp37_16*ROcp37_36-OMcp37_36*ROcp37_16);
    OPcp37_338 = OPcp37_36+ROcp37_36*qdd(38)+qd(38)*(OMcp37_16*ROcp37_26-OMcp37_26*ROcp37_16);
    ACcp37_138 = qdd(1)+OMcp37_26*ORcp37_338-OMcp37_36*ORcp37_238+OPcp37_26*RLcp37_338-OPcp37_36*RLcp37_238;
    ACcp37_238 = qdd(2)-OMcp37_16*ORcp37_338+OMcp37_36*ORcp37_138-OPcp37_16*RLcp37_338+OPcp37_36*RLcp37_138;
    ACcp37_338 = qdd(3)+OMcp37_16*ORcp37_238-OMcp37_26*ORcp37_138+OPcp37_16*RLcp37_238-OPcp37_26*RLcp37_138;

% = = Block_1_0_0_38_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp37_138;
    sens.P(2) = POcp37_238;
    sens.P(3) = POcp37_338;
    sens.R(1,1) = ROcp37_16;
    sens.R(1,2) = ROcp37_26;
    sens.R(1,3) = ROcp37_36;
    sens.R(2,1) = ROcp37_438;
    sens.R(2,2) = ROcp37_538;
    sens.R(2,3) = ROcp37_638;
    sens.R(3,1) = ROcp37_738;
    sens.R(3,2) = ROcp37_838;
    sens.R(3,3) = ROcp37_938;
    sens.V(1) = VIcp37_138;
    sens.V(2) = VIcp37_238;
    sens.V(3) = VIcp37_338;
    sens.OM(1) = OMcp37_138;
    sens.OM(2) = OMcp37_238;
    sens.OM(3) = OMcp37_338;
    sens.A(1) = ACcp37_138;
    sens.A(2) = ACcp37_238;
    sens.A(3) = ACcp37_338;
    sens.OMP(1) = OPcp37_138;
    sens.OMP(2) = OPcp37_238;
    sens.OMP(3) = OPcp37_338;
 
% 
case 39, 


% = = Block_1_0_0_39_0_1 = = 
 
% Sensor Kinematics 


    ROcp38_45 = -S4*C5;
    ROcp38_55 = C4*C5;
    ROcp38_75 = S4*S5;
    ROcp38_85 = -C4*S5;
    ROcp38_16 = -(ROcp38_75*S6-C4*C6);
    ROcp38_26 = -(ROcp38_85*S6-S4*C6);
    ROcp38_36 = -C5*S6;
    ROcp38_76 = ROcp38_75*C6+C4*S6;
    ROcp38_86 = ROcp38_85*C6+S4*S6;
    ROcp38_96 = C5*C6;
    OMcp38_15 = qd(5)*C4;
    OMcp38_25 = qd(5)*S4;
    OMcp38_16 = OMcp38_15+ROcp38_45*qd(6);
    OMcp38_26 = OMcp38_25+ROcp38_55*qd(6);
    OMcp38_36 = qd(4)+qd(6)*S5;
    OPcp38_16 = ROcp38_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp38_25*S5-ROcp38_55*qd(4));
    OPcp38_26 = ROcp38_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp38_15*S5-ROcp38_45*qd(4));
    OPcp38_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_39_0_17 = = 
 
% Sensor Kinematics 


    ROcp38_438 = ROcp38_45*C38+ROcp38_76*S38;
    ROcp38_538 = ROcp38_55*C38+ROcp38_86*S38;
    ROcp38_638 = ROcp38_96*S38+C38*S5;
    ROcp38_738 = -(ROcp38_45*S38-ROcp38_76*C38);
    ROcp38_838 = -(ROcp38_55*S38-ROcp38_86*C38);
    ROcp38_938 = ROcp38_96*C38-S38*S5;
    ROcp38_439 = ROcp38_438*C39+ROcp38_738*S39;
    ROcp38_539 = ROcp38_538*C39+ROcp38_838*S39;
    ROcp38_639 = ROcp38_638*C39+ROcp38_938*S39;
    ROcp38_739 = -(ROcp38_438*S39-ROcp38_738*C39);
    ROcp38_839 = -(ROcp38_538*S39-ROcp38_838*C39);
    ROcp38_939 = -(ROcp38_638*S39-ROcp38_938*C39);
    RLcp38_138 = ROcp38_16*s.dpt(1,13)+ROcp38_45*s.dpt(2,13)+ROcp38_76*s.dpt(3,13);
    RLcp38_238 = ROcp38_26*s.dpt(1,13)+ROcp38_55*s.dpt(2,13)+ROcp38_86*s.dpt(3,13);
    RLcp38_338 = ROcp38_36*s.dpt(1,13)+ROcp38_96*s.dpt(3,13)+s.dpt(2,13)*S5;
    OMcp38_138 = OMcp38_16+ROcp38_16*qd(38);
    OMcp38_238 = OMcp38_26+ROcp38_26*qd(38);
    OMcp38_338 = OMcp38_36+ROcp38_36*qd(38);
    ORcp38_138 = OMcp38_26*RLcp38_338-OMcp38_36*RLcp38_238;
    ORcp38_238 = -(OMcp38_16*RLcp38_338-OMcp38_36*RLcp38_138);
    ORcp38_338 = OMcp38_16*RLcp38_238-OMcp38_26*RLcp38_138;
    OPcp38_138 = OPcp38_16+ROcp38_16*qdd(38)+qd(38)*(OMcp38_26*ROcp38_36-OMcp38_36*ROcp38_26);
    OPcp38_238 = OPcp38_26+ROcp38_26*qdd(38)-qd(38)*(OMcp38_16*ROcp38_36-OMcp38_36*ROcp38_16);
    OPcp38_338 = OPcp38_36+ROcp38_36*qdd(38)+qd(38)*(OMcp38_16*ROcp38_26-OMcp38_26*ROcp38_16);
    RLcp38_139 = ROcp38_438*s.dpt(2,54);
    RLcp38_239 = ROcp38_538*s.dpt(2,54);
    RLcp38_339 = ROcp38_638*s.dpt(2,54);
    POcp38_139 = RLcp38_138+RLcp38_139+q(1);
    POcp38_239 = RLcp38_238+RLcp38_239+q(2);
    POcp38_339 = RLcp38_338+RLcp38_339+q(3);
    OMcp38_139 = OMcp38_138+ROcp38_16*qd(39);
    OMcp38_239 = OMcp38_238+ROcp38_26*qd(39);
    OMcp38_339 = OMcp38_338+ROcp38_36*qd(39);
    ORcp38_139 = OMcp38_238*RLcp38_339-OMcp38_338*RLcp38_239;
    ORcp38_239 = -(OMcp38_138*RLcp38_339-OMcp38_338*RLcp38_139);
    ORcp38_339 = OMcp38_138*RLcp38_239-OMcp38_238*RLcp38_139;
    VIcp38_139 = ORcp38_138+ORcp38_139+qd(1);
    VIcp38_239 = ORcp38_238+ORcp38_239+qd(2);
    VIcp38_339 = ORcp38_338+ORcp38_339+qd(3);
    OPcp38_139 = OPcp38_138+ROcp38_16*qdd(39)+qd(39)*(OMcp38_238*ROcp38_36-OMcp38_338*ROcp38_26);
    OPcp38_239 = OPcp38_238+ROcp38_26*qdd(39)-qd(39)*(OMcp38_138*ROcp38_36-OMcp38_338*ROcp38_16);
    OPcp38_339 = OPcp38_338+ROcp38_36*qdd(39)+qd(39)*(OMcp38_138*ROcp38_26-OMcp38_238*ROcp38_16);
    ACcp38_139 = qdd(1)+OMcp38_238*ORcp38_339+OMcp38_26*ORcp38_338-OMcp38_338*ORcp38_239-OMcp38_36*ORcp38_238+OPcp38_238*RLcp38_339+OPcp38_26*...
 RLcp38_338-OPcp38_338*RLcp38_239-OPcp38_36*RLcp38_238;
    ACcp38_239 = qdd(2)-OMcp38_138*ORcp38_339-OMcp38_16*ORcp38_338+OMcp38_338*ORcp38_139+OMcp38_36*ORcp38_138-OPcp38_138*RLcp38_339-OPcp38_16*...
 RLcp38_338+OPcp38_338*RLcp38_139+OPcp38_36*RLcp38_138;
    ACcp38_339 = qdd(3)+OMcp38_138*ORcp38_239+OMcp38_16*ORcp38_238-OMcp38_238*ORcp38_139-OMcp38_26*ORcp38_138+OPcp38_138*RLcp38_239+OPcp38_16*...
 RLcp38_238-OPcp38_238*RLcp38_139-OPcp38_26*RLcp38_138;

% = = Block_1_0_0_39_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp38_139;
    sens.P(2) = POcp38_239;
    sens.P(3) = POcp38_339;
    sens.R(1,1) = ROcp38_16;
    sens.R(1,2) = ROcp38_26;
    sens.R(1,3) = ROcp38_36;
    sens.R(2,1) = ROcp38_439;
    sens.R(2,2) = ROcp38_539;
    sens.R(2,3) = ROcp38_639;
    sens.R(3,1) = ROcp38_739;
    sens.R(3,2) = ROcp38_839;
    sens.R(3,3) = ROcp38_939;
    sens.V(1) = VIcp38_139;
    sens.V(2) = VIcp38_239;
    sens.V(3) = VIcp38_339;
    sens.OM(1) = OMcp38_139;
    sens.OM(2) = OMcp38_239;
    sens.OM(3) = OMcp38_339;
    sens.A(1) = ACcp38_139;
    sens.A(2) = ACcp38_239;
    sens.A(3) = ACcp38_339;
    sens.OMP(1) = OPcp38_139;
    sens.OMP(2) = OPcp38_239;
    sens.OMP(3) = OPcp38_339;
 
% 
case 40, 


% = = Block_1_0_0_40_0_1 = = 
 
% Sensor Kinematics 


    ROcp39_45 = -S4*C5;
    ROcp39_55 = C4*C5;
    ROcp39_75 = S4*S5;
    ROcp39_85 = -C4*S5;
    ROcp39_16 = -(ROcp39_75*S6-C4*C6);
    ROcp39_26 = -(ROcp39_85*S6-S4*C6);
    ROcp39_36 = -C5*S6;
    ROcp39_76 = ROcp39_75*C6+C4*S6;
    ROcp39_86 = ROcp39_85*C6+S4*S6;
    ROcp39_96 = C5*C6;
    OMcp39_15 = qd(5)*C4;
    OMcp39_25 = qd(5)*S4;
    OMcp39_16 = OMcp39_15+ROcp39_45*qd(6);
    OMcp39_26 = OMcp39_25+ROcp39_55*qd(6);
    OMcp39_36 = qd(4)+qd(6)*S5;
    OPcp39_16 = ROcp39_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp39_25*S5-ROcp39_55*qd(4));
    OPcp39_26 = ROcp39_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp39_15*S5-ROcp39_45*qd(4));
    OPcp39_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_40_0_17 = = 
 
% Sensor Kinematics 


    ROcp39_438 = ROcp39_45*C38+ROcp39_76*S38;
    ROcp39_538 = ROcp39_55*C38+ROcp39_86*S38;
    ROcp39_638 = ROcp39_96*S38+C38*S5;
    ROcp39_738 = -(ROcp39_45*S38-ROcp39_76*C38);
    ROcp39_838 = -(ROcp39_55*S38-ROcp39_86*C38);
    ROcp39_938 = ROcp39_96*C38-S38*S5;
    ROcp39_439 = ROcp39_438*C39+ROcp39_738*S39;
    ROcp39_539 = ROcp39_538*C39+ROcp39_838*S39;
    ROcp39_639 = ROcp39_638*C39+ROcp39_938*S39;
    ROcp39_739 = -(ROcp39_438*S39-ROcp39_738*C39);
    ROcp39_839 = -(ROcp39_538*S39-ROcp39_838*C39);
    ROcp39_939 = -(ROcp39_638*S39-ROcp39_938*C39);
    ROcp39_140 = ROcp39_16*C40-ROcp39_739*S40;
    ROcp39_240 = ROcp39_26*C40-ROcp39_839*S40;
    ROcp39_340 = ROcp39_36*C40-ROcp39_939*S40;
    ROcp39_740 = ROcp39_16*S40+ROcp39_739*C40;
    ROcp39_840 = ROcp39_26*S40+ROcp39_839*C40;
    ROcp39_940 = ROcp39_36*S40+ROcp39_939*C40;
    RLcp39_138 = ROcp39_16*s.dpt(1,13)+ROcp39_45*s.dpt(2,13)+ROcp39_76*s.dpt(3,13);
    RLcp39_238 = ROcp39_26*s.dpt(1,13)+ROcp39_55*s.dpt(2,13)+ROcp39_86*s.dpt(3,13);
    RLcp39_338 = ROcp39_36*s.dpt(1,13)+ROcp39_96*s.dpt(3,13)+s.dpt(2,13)*S5;
    OMcp39_138 = OMcp39_16+ROcp39_16*qd(38);
    OMcp39_238 = OMcp39_26+ROcp39_26*qd(38);
    OMcp39_338 = OMcp39_36+ROcp39_36*qd(38);
    ORcp39_138 = OMcp39_26*RLcp39_338-OMcp39_36*RLcp39_238;
    ORcp39_238 = -(OMcp39_16*RLcp39_338-OMcp39_36*RLcp39_138);
    ORcp39_338 = OMcp39_16*RLcp39_238-OMcp39_26*RLcp39_138;
    OPcp39_138 = OPcp39_16+ROcp39_16*qdd(38)+qd(38)*(OMcp39_26*ROcp39_36-OMcp39_36*ROcp39_26);
    OPcp39_238 = OPcp39_26+ROcp39_26*qdd(38)-qd(38)*(OMcp39_16*ROcp39_36-OMcp39_36*ROcp39_16);
    OPcp39_338 = OPcp39_36+ROcp39_36*qdd(38)+qd(38)*(OMcp39_16*ROcp39_26-OMcp39_26*ROcp39_16);
    RLcp39_139 = ROcp39_438*s.dpt(2,54);
    RLcp39_239 = ROcp39_538*s.dpt(2,54);
    RLcp39_339 = ROcp39_638*s.dpt(2,54);
    OMcp39_139 = OMcp39_138+ROcp39_16*qd(39);
    OMcp39_239 = OMcp39_238+ROcp39_26*qd(39);
    OMcp39_339 = OMcp39_338+ROcp39_36*qd(39);
    ORcp39_139 = OMcp39_238*RLcp39_339-OMcp39_338*RLcp39_239;
    ORcp39_239 = -(OMcp39_138*RLcp39_339-OMcp39_338*RLcp39_139);
    ORcp39_339 = OMcp39_138*RLcp39_239-OMcp39_238*RLcp39_139;
    OPcp39_139 = OPcp39_138+ROcp39_16*qdd(39)+qd(39)*(OMcp39_238*ROcp39_36-OMcp39_338*ROcp39_26);
    OPcp39_239 = OPcp39_238+ROcp39_26*qdd(39)-qd(39)*(OMcp39_138*ROcp39_36-OMcp39_338*ROcp39_16);
    OPcp39_339 = OPcp39_338+ROcp39_36*qdd(39)+qd(39)*(OMcp39_138*ROcp39_26-OMcp39_238*ROcp39_16);
    RLcp39_140 = ROcp39_739*s.dpt(3,56);
    RLcp39_240 = ROcp39_839*s.dpt(3,56);
    RLcp39_340 = ROcp39_939*s.dpt(3,56);
    POcp39_140 = RLcp39_138+RLcp39_139+RLcp39_140+q(1);
    POcp39_240 = RLcp39_238+RLcp39_239+RLcp39_240+q(2);
    POcp39_340 = RLcp39_338+RLcp39_339+RLcp39_340+q(3);
    OMcp39_140 = OMcp39_139+ROcp39_439*qd(40);
    OMcp39_240 = OMcp39_239+ROcp39_539*qd(40);
    OMcp39_340 = OMcp39_339+ROcp39_639*qd(40);
    ORcp39_140 = OMcp39_239*RLcp39_340-OMcp39_339*RLcp39_240;
    ORcp39_240 = -(OMcp39_139*RLcp39_340-OMcp39_339*RLcp39_140);
    ORcp39_340 = OMcp39_139*RLcp39_240-OMcp39_239*RLcp39_140;
    VIcp39_140 = ORcp39_138+ORcp39_139+ORcp39_140+qd(1);
    VIcp39_240 = ORcp39_238+ORcp39_239+ORcp39_240+qd(2);
    VIcp39_340 = ORcp39_338+ORcp39_339+ORcp39_340+qd(3);
    OPcp39_140 = OPcp39_139+ROcp39_439*qdd(40)+qd(40)*(OMcp39_239*ROcp39_639-OMcp39_339*ROcp39_539);
    OPcp39_240 = OPcp39_239+ROcp39_539*qdd(40)-qd(40)*(OMcp39_139*ROcp39_639-OMcp39_339*ROcp39_439);
    OPcp39_340 = OPcp39_339+ROcp39_639*qdd(40)+qd(40)*(OMcp39_139*ROcp39_539-OMcp39_239*ROcp39_439);
    ACcp39_140 = qdd(1)+OMcp39_238*ORcp39_339+OMcp39_239*ORcp39_340+OMcp39_26*ORcp39_338-OMcp39_338*ORcp39_239-OMcp39_339*ORcp39_240-OMcp39_36*...
 ORcp39_238+OPcp39_238*RLcp39_339+OPcp39_239*RLcp39_340+OPcp39_26*RLcp39_338-OPcp39_338*RLcp39_239-OPcp39_339*RLcp39_240-OPcp39_36*RLcp39_238;
    ACcp39_240 = qdd(2)-OMcp39_138*ORcp39_339-OMcp39_139*ORcp39_340-OMcp39_16*ORcp39_338+OMcp39_338*ORcp39_139+OMcp39_339*ORcp39_140+OMcp39_36*...
 ORcp39_138-OPcp39_138*RLcp39_339-OPcp39_139*RLcp39_340-OPcp39_16*RLcp39_338+OPcp39_338*RLcp39_139+OPcp39_339*RLcp39_140+OPcp39_36*RLcp39_138;
    ACcp39_340 = qdd(3)+OMcp39_138*ORcp39_239+OMcp39_139*ORcp39_240+OMcp39_16*ORcp39_238-OMcp39_238*ORcp39_139-OMcp39_239*ORcp39_140-OMcp39_26*...
 ORcp39_138+OPcp39_138*RLcp39_239+OPcp39_139*RLcp39_240+OPcp39_16*RLcp39_238-OPcp39_238*RLcp39_139-OPcp39_239*RLcp39_140-OPcp39_26*RLcp39_138;

% = = Block_1_0_0_40_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp39_140;
    sens.P(2) = POcp39_240;
    sens.P(3) = POcp39_340;
    sens.R(1,1) = ROcp39_140;
    sens.R(1,2) = ROcp39_240;
    sens.R(1,3) = ROcp39_340;
    sens.R(2,1) = ROcp39_439;
    sens.R(2,2) = ROcp39_539;
    sens.R(2,3) = ROcp39_639;
    sens.R(3,1) = ROcp39_740;
    sens.R(3,2) = ROcp39_840;
    sens.R(3,3) = ROcp39_940;
    sens.V(1) = VIcp39_140;
    sens.V(2) = VIcp39_240;
    sens.V(3) = VIcp39_340;
    sens.OM(1) = OMcp39_140;
    sens.OM(2) = OMcp39_240;
    sens.OM(3) = OMcp39_340;
    sens.A(1) = ACcp39_140;
    sens.A(2) = ACcp39_240;
    sens.A(3) = ACcp39_340;
    sens.OMP(1) = OPcp39_140;
    sens.OMP(2) = OPcp39_240;
    sens.OMP(3) = OPcp39_340;
 
% 
case 41, 


% = = Block_1_0_0_41_0_1 = = 
 
% Sensor Kinematics 


    ROcp40_45 = -S4*C5;
    ROcp40_55 = C4*C5;
    ROcp40_75 = S4*S5;
    ROcp40_85 = -C4*S5;
    ROcp40_16 = -(ROcp40_75*S6-C4*C6);
    ROcp40_26 = -(ROcp40_85*S6-S4*C6);
    ROcp40_76 = ROcp40_75*C6+C4*S6;
    ROcp40_86 = ROcp40_85*C6+S4*S6;
    OMcp40_15 = qd(5)*C4;
    OMcp40_25 = qd(5)*S4;
    OMcp40_16 = OMcp40_15+ROcp40_45*qd(6);
    OMcp40_26 = OMcp40_25+ROcp40_55*qd(6);
    OMcp40_36 = qd(4)+qd(6)*S5;
    OPcp40_16 = ROcp40_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp40_25*S5-ROcp40_55*qd(4));
    OPcp40_26 = ROcp40_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp40_15*S5-ROcp40_45*qd(4));
    OPcp40_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_41_0_18 = = 
 
% Sensor Kinematics 


    ROcp40_141 = ROcp40_16*C41-ROcp40_76*S41;
    ROcp40_241 = ROcp40_26*C41-ROcp40_86*S41;
    ROcp40_341 = -C5*S41p6;
    ROcp40_741 = ROcp40_16*S41+ROcp40_76*C41;
    ROcp40_841 = ROcp40_26*S41+ROcp40_86*C41;
    ROcp40_941 = C5*C41p6;
    RLcp40_141 = ROcp40_16*s.dpt(1,15)+ROcp40_76*s.dpt(3,15);
    RLcp40_241 = ROcp40_26*s.dpt(1,15)+ROcp40_86*s.dpt(3,15);
    RLcp40_341 = -C5*(s.dpt(1,15)*S6-s.dpt(3,15)*C6);
    POcp40_141 = RLcp40_141+q(1);
    POcp40_241 = RLcp40_241+q(2);
    POcp40_341 = RLcp40_341+q(3);
    OMcp40_141 = OMcp40_16+ROcp40_45*qd(41);
    OMcp40_241 = OMcp40_26+ROcp40_55*qd(41);
    OMcp40_341 = OMcp40_36+qd(41)*S5;
    ORcp40_141 = OMcp40_26*RLcp40_341-OMcp40_36*RLcp40_241;
    ORcp40_241 = -(OMcp40_16*RLcp40_341-OMcp40_36*RLcp40_141);
    ORcp40_341 = OMcp40_16*RLcp40_241-OMcp40_26*RLcp40_141;
    VIcp40_141 = ORcp40_141+qd(1);
    VIcp40_241 = ORcp40_241+qd(2);
    VIcp40_341 = ORcp40_341+qd(3);
    OPcp40_141 = OPcp40_16+ROcp40_45*qdd(41)+qd(41)*(OMcp40_26*S5-OMcp40_36*ROcp40_55);
    OPcp40_241 = OPcp40_26+ROcp40_55*qdd(41)-qd(41)*(OMcp40_16*S5-OMcp40_36*ROcp40_45);
    OPcp40_341 = OPcp40_36+qdd(41)*S5+qd(41)*(OMcp40_16*ROcp40_55-OMcp40_26*ROcp40_45);
    ACcp40_141 = qdd(1)+OMcp40_26*ORcp40_341-OMcp40_36*ORcp40_241+OPcp40_26*RLcp40_341-OPcp40_36*RLcp40_241;
    ACcp40_241 = qdd(2)-OMcp40_16*ORcp40_341+OMcp40_36*ORcp40_141-OPcp40_16*RLcp40_341+OPcp40_36*RLcp40_141;
    ACcp40_341 = qdd(3)+OMcp40_16*ORcp40_241-OMcp40_26*ORcp40_141+OPcp40_16*RLcp40_241-OPcp40_26*RLcp40_141;

% = = Block_1_0_0_41_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp40_141;
    sens.P(2) = POcp40_241;
    sens.P(3) = POcp40_341;
    sens.R(1,1) = ROcp40_141;
    sens.R(1,2) = ROcp40_241;
    sens.R(1,3) = ROcp40_341;
    sens.R(2,1) = ROcp40_45;
    sens.R(2,2) = ROcp40_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp40_741;
    sens.R(3,2) = ROcp40_841;
    sens.R(3,3) = ROcp40_941;
    sens.V(1) = VIcp40_141;
    sens.V(2) = VIcp40_241;
    sens.V(3) = VIcp40_341;
    sens.OM(1) = OMcp40_141;
    sens.OM(2) = OMcp40_241;
    sens.OM(3) = OMcp40_341;
    sens.A(1) = ACcp40_141;
    sens.A(2) = ACcp40_241;
    sens.A(3) = ACcp40_341;
    sens.OMP(1) = OPcp40_141;
    sens.OMP(2) = OPcp40_241;
    sens.OMP(3) = OPcp40_341;
 
% 
case 42, 


% = = Block_1_0_0_42_0_1 = = 
 
% Sensor Kinematics 


    ROcp41_45 = -S4*C5;
    ROcp41_55 = C4*C5;
    ROcp41_75 = S4*S5;
    ROcp41_85 = -C4*S5;
    ROcp41_16 = -(ROcp41_75*S6-C4*C6);
    ROcp41_26 = -(ROcp41_85*S6-S4*C6);
    ROcp41_76 = ROcp41_75*C6+C4*S6;
    ROcp41_86 = ROcp41_85*C6+S4*S6;
    OMcp41_15 = qd(5)*C4;
    OMcp41_25 = qd(5)*S4;
    OMcp41_16 = OMcp41_15+ROcp41_45*qd(6);
    OMcp41_26 = OMcp41_25+ROcp41_55*qd(6);
    OMcp41_36 = qd(4)+qd(6)*S5;
    OPcp41_16 = ROcp41_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp41_25*S5-ROcp41_55*qd(4));
    OPcp41_26 = ROcp41_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp41_15*S5-ROcp41_45*qd(4));
    OPcp41_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_42_0_18 = = 
 
% Sensor Kinematics 


    ROcp41_141 = ROcp41_16*C41-ROcp41_76*S41;
    ROcp41_241 = ROcp41_26*C41-ROcp41_86*S41;
    ROcp41_741 = ROcp41_16*S41+ROcp41_76*C41;
    ROcp41_841 = ROcp41_26*S41+ROcp41_86*C41;
    RLcp41_141 = ROcp41_16*s.dpt(1,15)+ROcp41_76*s.dpt(3,15);
    RLcp41_241 = ROcp41_26*s.dpt(1,15)+ROcp41_86*s.dpt(3,15);
    RLcp41_341 = -C5*(s.dpt(1,15)*S6-s.dpt(3,15)*C6);
    POcp41_141 = RLcp41_141+q(1);
    POcp41_241 = RLcp41_241+q(2);
    POcp41_341 = RLcp41_341+q(3);
    OMcp41_141 = OMcp41_16+ROcp41_45*qd(41);
    OMcp41_241 = OMcp41_26+ROcp41_55*qd(41);
    OMcp41_341 = OMcp41_36+qd(41)*S5;
    ORcp41_141 = OMcp41_26*RLcp41_341-OMcp41_36*RLcp41_241;
    ORcp41_241 = -(OMcp41_16*RLcp41_341-OMcp41_36*RLcp41_141);
    ORcp41_341 = OMcp41_16*RLcp41_241-OMcp41_26*RLcp41_141;
    VIcp41_141 = ORcp41_141+qd(1);
    VIcp41_241 = ORcp41_241+qd(2);
    VIcp41_341 = ORcp41_341+qd(3);
    ACcp41_141 = qdd(1)+OMcp41_26*ORcp41_341-OMcp41_36*ORcp41_241+OPcp41_26*RLcp41_341-OPcp41_36*RLcp41_241;
    ACcp41_241 = qdd(2)-OMcp41_16*ORcp41_341+OMcp41_36*ORcp41_141-OPcp41_16*RLcp41_341+OPcp41_36*RLcp41_141;
    ACcp41_341 = qdd(3)+OMcp41_16*ORcp41_241-OMcp41_26*ORcp41_141+OPcp41_16*RLcp41_241-OPcp41_26*RLcp41_141;

% = = Block_1_0_0_42_0_19 = = 
 
% Sensor Kinematics 


    ROcp41_142 = ROcp41_141*C42-ROcp41_741*S42;
    ROcp41_242 = ROcp41_241*C42-ROcp41_841*S42;
    ROcp41_342 = -C5*S42p41p6;
    ROcp41_742 = ROcp41_141*S42+ROcp41_741*C42;
    ROcp41_842 = ROcp41_241*S42+ROcp41_841*C42;
    ROcp41_942 = C5*C42p41p6;
    OMcp41_142 = OMcp41_141+ROcp41_45*qd(42);
    OMcp41_242 = OMcp41_241+ROcp41_55*qd(42);
    OMcp41_342 = OMcp41_341+qd(42)*S5;
    OPcp41_142 = OPcp41_16+ROcp41_45*qdd(41)+ROcp41_45*qdd(42)+qd(41)*(OMcp41_26*S5-OMcp41_36*ROcp41_55)+qd(42)*(OMcp41_241*S5-OMcp41_341*ROcp41_55...
 );
    OPcp41_242 = OPcp41_26+ROcp41_55*qdd(41)+ROcp41_55*qdd(42)-qd(41)*(OMcp41_16*S5-OMcp41_36*ROcp41_45)-qd(42)*(OMcp41_141*S5-OMcp41_341*ROcp41_45...
 );
    OPcp41_342 = OPcp41_36+qdd(41)*S5+qdd(42)*S5+qd(41)*(OMcp41_16*ROcp41_55-OMcp41_26*ROcp41_45)+qd(42)*(OMcp41_141*ROcp41_55-OMcp41_241*ROcp41_45...
 );

% = = Block_1_0_0_42_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp41_141;
    sens.P(2) = POcp41_241;
    sens.P(3) = POcp41_341;
    sens.R(1,1) = ROcp41_142;
    sens.R(1,2) = ROcp41_242;
    sens.R(1,3) = ROcp41_342;
    sens.R(2,1) = ROcp41_45;
    sens.R(2,2) = ROcp41_55;
    sens.R(2,3) = S5;
    sens.R(3,1) = ROcp41_742;
    sens.R(3,2) = ROcp41_842;
    sens.R(3,3) = ROcp41_942;
    sens.V(1) = VIcp41_141;
    sens.V(2) = VIcp41_241;
    sens.V(3) = VIcp41_341;
    sens.OM(1) = OMcp41_142;
    sens.OM(2) = OMcp41_242;
    sens.OM(3) = OMcp41_342;
    sens.A(1) = ACcp41_141;
    sens.A(2) = ACcp41_241;
    sens.A(3) = ACcp41_341;
    sens.OMP(1) = OPcp41_142;
    sens.OMP(2) = OPcp41_242;
    sens.OMP(3) = OPcp41_342;
 
% 
case 43, 


% = = Block_1_0_0_43_0_1 = = 
 
% Sensor Kinematics 


    ROcp42_45 = -S4*C5;
    ROcp42_55 = C4*C5;
    ROcp42_75 = S4*S5;
    ROcp42_85 = -C4*S5;
    ROcp42_16 = -(ROcp42_75*S6-C4*C6);
    ROcp42_26 = -(ROcp42_85*S6-S4*C6);
    ROcp42_76 = ROcp42_75*C6+C4*S6;
    ROcp42_86 = ROcp42_85*C6+S4*S6;
    OMcp42_15 = qd(5)*C4;
    OMcp42_25 = qd(5)*S4;
    OMcp42_16 = OMcp42_15+ROcp42_45*qd(6);
    OMcp42_26 = OMcp42_25+ROcp42_55*qd(6);
    OMcp42_36 = qd(4)+qd(6)*S5;
    OPcp42_16 = ROcp42_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp42_25*S5-ROcp42_55*qd(4));
    OPcp42_26 = ROcp42_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp42_15*S5-ROcp42_45*qd(4));
    OPcp42_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_43_0_18 = = 
 
% Sensor Kinematics 


    ROcp42_141 = ROcp42_16*C41-ROcp42_76*S41;
    ROcp42_241 = ROcp42_26*C41-ROcp42_86*S41;
    ROcp42_741 = ROcp42_16*S41+ROcp42_76*C41;
    ROcp42_841 = ROcp42_26*S41+ROcp42_86*C41;
    RLcp42_141 = ROcp42_16*s.dpt(1,15)+ROcp42_76*s.dpt(3,15);
    RLcp42_241 = ROcp42_26*s.dpt(1,15)+ROcp42_86*s.dpt(3,15);
    RLcp42_341 = -C5*(s.dpt(1,15)*S6-s.dpt(3,15)*C6);
    OMcp42_141 = OMcp42_16+ROcp42_45*qd(41);
    OMcp42_241 = OMcp42_26+ROcp42_55*qd(41);
    OMcp42_341 = OMcp42_36+qd(41)*S5;
    ORcp42_141 = OMcp42_26*RLcp42_341-OMcp42_36*RLcp42_241;
    ORcp42_241 = -(OMcp42_16*RLcp42_341-OMcp42_36*RLcp42_141);
    ORcp42_341 = OMcp42_16*RLcp42_241-OMcp42_26*RLcp42_141;

% = = Block_1_0_0_43_0_19 = = 
 
% Sensor Kinematics 


    ROcp42_142 = ROcp42_141*C42-ROcp42_741*S42;
    ROcp42_242 = ROcp42_241*C42-ROcp42_841*S42;
    ROcp42_342 = -C5*S42p41p6;
    ROcp42_742 = ROcp42_141*S42+ROcp42_741*C42;
    ROcp42_842 = ROcp42_241*S42+ROcp42_841*C42;
    ROcp42_942 = C5*C42p41p6;
    ROcp42_443 = ROcp42_45*C43+ROcp42_742*S43;
    ROcp42_543 = ROcp42_55*C43+ROcp42_842*S43;
    ROcp42_643 = ROcp42_942*S43+C43*S5;
    ROcp42_743 = -(ROcp42_45*S43-ROcp42_742*C43);
    ROcp42_843 = -(ROcp42_55*S43-ROcp42_842*C43);
    ROcp42_943 = ROcp42_942*C43-S43*S5;
    OMcp42_142 = OMcp42_141+ROcp42_45*qd(42);
    OMcp42_242 = OMcp42_241+ROcp42_55*qd(42);
    OMcp42_342 = OMcp42_341+qd(42)*S5;
    OPcp42_142 = OPcp42_16+ROcp42_45*qdd(41)+ROcp42_45*qdd(42)+qd(41)*(OMcp42_26*S5-OMcp42_36*ROcp42_55)+qd(42)*(OMcp42_241*S5-OMcp42_341*ROcp42_55...
 );
    OPcp42_242 = OPcp42_26+ROcp42_55*qdd(41)+ROcp42_55*qdd(42)-qd(41)*(OMcp42_16*S5-OMcp42_36*ROcp42_45)-qd(42)*(OMcp42_141*S5-OMcp42_341*ROcp42_45...
 );
    OPcp42_342 = OPcp42_36+qdd(41)*S5+qdd(42)*S5+qd(41)*(OMcp42_16*ROcp42_55-OMcp42_26*ROcp42_45)+qd(42)*(OMcp42_141*ROcp42_55-OMcp42_241*ROcp42_45...
 );
    RLcp42_143 = ROcp42_142*s.dpt(1,61)+ROcp42_45*s.dpt(2,61);
    RLcp42_243 = ROcp42_242*s.dpt(1,61)+ROcp42_55*s.dpt(2,61);
    RLcp42_343 = ROcp42_342*s.dpt(1,61)+s.dpt(2,61)*S5;
    POcp42_143 = RLcp42_141+RLcp42_143+q(1);
    POcp42_243 = RLcp42_241+RLcp42_243+q(2);
    POcp42_343 = RLcp42_341+RLcp42_343+q(3);
    OMcp42_143 = OMcp42_142+ROcp42_142*qd(43);
    OMcp42_243 = OMcp42_242+ROcp42_242*qd(43);
    OMcp42_343 = OMcp42_342+ROcp42_342*qd(43);
    ORcp42_143 = OMcp42_242*RLcp42_343-OMcp42_342*RLcp42_243;
    ORcp42_243 = -(OMcp42_142*RLcp42_343-OMcp42_342*RLcp42_143);
    ORcp42_343 = OMcp42_142*RLcp42_243-OMcp42_242*RLcp42_143;
    VIcp42_143 = ORcp42_141+ORcp42_143+qd(1);
    VIcp42_243 = ORcp42_241+ORcp42_243+qd(2);
    VIcp42_343 = ORcp42_341+ORcp42_343+qd(3);
    OPcp42_143 = OPcp42_142+ROcp42_142*qdd(43)+qd(43)*(OMcp42_242*ROcp42_342-OMcp42_342*ROcp42_242);
    OPcp42_243 = OPcp42_242+ROcp42_242*qdd(43)-qd(43)*(OMcp42_142*ROcp42_342-OMcp42_342*ROcp42_142);
    OPcp42_343 = OPcp42_342+ROcp42_342*qdd(43)+qd(43)*(OMcp42_142*ROcp42_242-OMcp42_242*ROcp42_142);
    ACcp42_143 = qdd(1)+OMcp42_242*ORcp42_343+OMcp42_26*ORcp42_341-OMcp42_342*ORcp42_243-OMcp42_36*ORcp42_241+OPcp42_242*RLcp42_343+OPcp42_26*...
 RLcp42_341-OPcp42_342*RLcp42_243-OPcp42_36*RLcp42_241;
    ACcp42_243 = qdd(2)-OMcp42_142*ORcp42_343-OMcp42_16*ORcp42_341+OMcp42_342*ORcp42_143+OMcp42_36*ORcp42_141-OPcp42_142*RLcp42_343-OPcp42_16*...
 RLcp42_341+OPcp42_342*RLcp42_143+OPcp42_36*RLcp42_141;
    ACcp42_343 = qdd(3)+OMcp42_142*ORcp42_243+OMcp42_16*ORcp42_241-OMcp42_242*ORcp42_143-OMcp42_26*ORcp42_141+OPcp42_142*RLcp42_243+OPcp42_16*...
 RLcp42_241-OPcp42_242*RLcp42_143-OPcp42_26*RLcp42_141;

% = = Block_1_0_0_43_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp42_143;
    sens.P(2) = POcp42_243;
    sens.P(3) = POcp42_343;
    sens.R(1,1) = ROcp42_142;
    sens.R(1,2) = ROcp42_242;
    sens.R(1,3) = ROcp42_342;
    sens.R(2,1) = ROcp42_443;
    sens.R(2,2) = ROcp42_543;
    sens.R(2,3) = ROcp42_643;
    sens.R(3,1) = ROcp42_743;
    sens.R(3,2) = ROcp42_843;
    sens.R(3,3) = ROcp42_943;
    sens.V(1) = VIcp42_143;
    sens.V(2) = VIcp42_243;
    sens.V(3) = VIcp42_343;
    sens.OM(1) = OMcp42_143;
    sens.OM(2) = OMcp42_243;
    sens.OM(3) = OMcp42_343;
    sens.A(1) = ACcp42_143;
    sens.A(2) = ACcp42_243;
    sens.A(3) = ACcp42_343;
    sens.OMP(1) = OPcp42_143;
    sens.OMP(2) = OPcp42_243;
    sens.OMP(3) = OPcp42_343;
 
% 
case 44, 


% = = Block_1_0_0_44_0_1 = = 
 
% Sensor Kinematics 


    ROcp43_45 = -S4*C5;
    ROcp43_55 = C4*C5;
    ROcp43_75 = S4*S5;
    ROcp43_85 = -C4*S5;
    ROcp43_16 = -(ROcp43_75*S6-C4*C6);
    ROcp43_26 = -(ROcp43_85*S6-S4*C6);
    ROcp43_76 = ROcp43_75*C6+C4*S6;
    ROcp43_86 = ROcp43_85*C6+S4*S6;
    OMcp43_15 = qd(5)*C4;
    OMcp43_25 = qd(5)*S4;
    OMcp43_16 = OMcp43_15+ROcp43_45*qd(6);
    OMcp43_26 = OMcp43_25+ROcp43_55*qd(6);
    OMcp43_36 = qd(4)+qd(6)*S5;
    OPcp43_16 = ROcp43_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp43_25*S5-ROcp43_55*qd(4));
    OPcp43_26 = ROcp43_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp43_15*S5-ROcp43_45*qd(4));
    OPcp43_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_44_0_18 = = 
 
% Sensor Kinematics 


    ROcp43_141 = ROcp43_16*C41-ROcp43_76*S41;
    ROcp43_241 = ROcp43_26*C41-ROcp43_86*S41;
    ROcp43_741 = ROcp43_16*S41+ROcp43_76*C41;
    ROcp43_841 = ROcp43_26*S41+ROcp43_86*C41;
    RLcp43_141 = ROcp43_16*s.dpt(1,15)+ROcp43_76*s.dpt(3,15);
    RLcp43_241 = ROcp43_26*s.dpt(1,15)+ROcp43_86*s.dpt(3,15);
    RLcp43_341 = -C5*(s.dpt(1,15)*S6-s.dpt(3,15)*C6);
    OMcp43_141 = OMcp43_16+ROcp43_45*qd(41);
    OMcp43_241 = OMcp43_26+ROcp43_55*qd(41);
    OMcp43_341 = OMcp43_36+qd(41)*S5;
    ORcp43_141 = OMcp43_26*RLcp43_341-OMcp43_36*RLcp43_241;
    ORcp43_241 = -(OMcp43_16*RLcp43_341-OMcp43_36*RLcp43_141);
    ORcp43_341 = OMcp43_16*RLcp43_241-OMcp43_26*RLcp43_141;

% = = Block_1_0_0_44_0_19 = = 
 
% Sensor Kinematics 


    ROcp43_142 = ROcp43_141*C42-ROcp43_741*S42;
    ROcp43_242 = ROcp43_241*C42-ROcp43_841*S42;
    ROcp43_342 = -C5*S42p41p6;
    ROcp43_742 = ROcp43_141*S42+ROcp43_741*C42;
    ROcp43_842 = ROcp43_241*S42+ROcp43_841*C42;
    ROcp43_942 = C5*C42p41p6;
    ROcp43_443 = ROcp43_45*C43+ROcp43_742*S43;
    ROcp43_543 = ROcp43_55*C43+ROcp43_842*S43;
    ROcp43_643 = ROcp43_942*S43+C43*S5;
    ROcp43_743 = -(ROcp43_45*S43-ROcp43_742*C43);
    ROcp43_843 = -(ROcp43_55*S43-ROcp43_842*C43);
    ROcp43_943 = ROcp43_942*C43-S43*S5;
    ROcp43_144 = ROcp43_142*C44-ROcp43_743*S44;
    ROcp43_244 = ROcp43_242*C44-ROcp43_843*S44;
    ROcp43_344 = ROcp43_342*C44-ROcp43_943*S44;
    ROcp43_744 = ROcp43_142*S44+ROcp43_743*C44;
    ROcp43_844 = ROcp43_242*S44+ROcp43_843*C44;
    ROcp43_944 = ROcp43_342*S44+ROcp43_943*C44;
    OMcp43_142 = OMcp43_141+ROcp43_45*qd(42);
    OMcp43_242 = OMcp43_241+ROcp43_55*qd(42);
    OMcp43_342 = OMcp43_341+qd(42)*S5;
    OPcp43_142 = OPcp43_16+ROcp43_45*qdd(41)+ROcp43_45*qdd(42)+qd(41)*(OMcp43_26*S5-OMcp43_36*ROcp43_55)+qd(42)*(OMcp43_241*S5-OMcp43_341*ROcp43_55...
 );
    OPcp43_242 = OPcp43_26+ROcp43_55*qdd(41)+ROcp43_55*qdd(42)-qd(41)*(OMcp43_16*S5-OMcp43_36*ROcp43_45)-qd(42)*(OMcp43_141*S5-OMcp43_341*ROcp43_45...
 );
    OPcp43_342 = OPcp43_36+qdd(41)*S5+qdd(42)*S5+qd(41)*(OMcp43_16*ROcp43_55-OMcp43_26*ROcp43_45)+qd(42)*(OMcp43_141*ROcp43_55-OMcp43_241*ROcp43_45...
 );
    RLcp43_143 = ROcp43_142*s.dpt(1,61)+ROcp43_45*s.dpt(2,61);
    RLcp43_243 = ROcp43_242*s.dpt(1,61)+ROcp43_55*s.dpt(2,61);
    RLcp43_343 = ROcp43_342*s.dpt(1,61)+s.dpt(2,61)*S5;
    POcp43_143 = RLcp43_141+RLcp43_143+q(1);
    POcp43_243 = RLcp43_241+RLcp43_243+q(2);
    POcp43_343 = RLcp43_341+RLcp43_343+q(3);
    OMcp43_143 = OMcp43_142+ROcp43_142*qd(43);
    OMcp43_243 = OMcp43_242+ROcp43_242*qd(43);
    OMcp43_343 = OMcp43_342+ROcp43_342*qd(43);
    ORcp43_143 = OMcp43_242*RLcp43_343-OMcp43_342*RLcp43_243;
    ORcp43_243 = -(OMcp43_142*RLcp43_343-OMcp43_342*RLcp43_143);
    ORcp43_343 = OMcp43_142*RLcp43_243-OMcp43_242*RLcp43_143;
    VIcp43_143 = ORcp43_141+ORcp43_143+qd(1);
    VIcp43_243 = ORcp43_241+ORcp43_243+qd(2);
    VIcp43_343 = ORcp43_341+ORcp43_343+qd(3);
    ACcp43_143 = qdd(1)+OMcp43_242*ORcp43_343+OMcp43_26*ORcp43_341-OMcp43_342*ORcp43_243-OMcp43_36*ORcp43_241+OPcp43_242*RLcp43_343+OPcp43_26*...
 RLcp43_341-OPcp43_342*RLcp43_243-OPcp43_36*RLcp43_241;
    ACcp43_243 = qdd(2)-OMcp43_142*ORcp43_343-OMcp43_16*ORcp43_341+OMcp43_342*ORcp43_143+OMcp43_36*ORcp43_141-OPcp43_142*RLcp43_343-OPcp43_16*...
 RLcp43_341+OPcp43_342*RLcp43_143+OPcp43_36*RLcp43_141;
    ACcp43_343 = qdd(3)+OMcp43_142*ORcp43_243+OMcp43_16*ORcp43_241-OMcp43_242*ORcp43_143-OMcp43_26*ORcp43_141+OPcp43_142*RLcp43_243+OPcp43_16*...
 RLcp43_241-OPcp43_242*RLcp43_143-OPcp43_26*RLcp43_141;
    OMcp43_144 = OMcp43_143+ROcp43_443*qd(44);
    OMcp43_244 = OMcp43_243+ROcp43_543*qd(44);
    OMcp43_344 = OMcp43_343+ROcp43_643*qd(44);
    OPcp43_144 = OPcp43_142+ROcp43_142*qdd(43)+ROcp43_443*qdd(44)+qd(43)*(OMcp43_242*ROcp43_342-OMcp43_342*ROcp43_242)+qd(44)*(OMcp43_243*...
 ROcp43_643-OMcp43_343*ROcp43_543);
    OPcp43_244 = OPcp43_242+ROcp43_242*qdd(43)+ROcp43_543*qdd(44)-qd(43)*(OMcp43_142*ROcp43_342-OMcp43_342*ROcp43_142)-qd(44)*(OMcp43_143*...
 ROcp43_643-OMcp43_343*ROcp43_443);
    OPcp43_344 = OPcp43_342+ROcp43_342*qdd(43)+ROcp43_643*qdd(44)+qd(43)*(OMcp43_142*ROcp43_242-OMcp43_242*ROcp43_142)+qd(44)*(OMcp43_143*...
 ROcp43_543-OMcp43_243*ROcp43_443);

% = = Block_1_0_0_44_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp43_143;
    sens.P(2) = POcp43_243;
    sens.P(3) = POcp43_343;
    sens.R(1,1) = ROcp43_144;
    sens.R(1,2) = ROcp43_244;
    sens.R(1,3) = ROcp43_344;
    sens.R(2,1) = ROcp43_443;
    sens.R(2,2) = ROcp43_543;
    sens.R(2,3) = ROcp43_643;
    sens.R(3,1) = ROcp43_744;
    sens.R(3,2) = ROcp43_844;
    sens.R(3,3) = ROcp43_944;
    sens.V(1) = VIcp43_143;
    sens.V(2) = VIcp43_243;
    sens.V(3) = VIcp43_343;
    sens.OM(1) = OMcp43_144;
    sens.OM(2) = OMcp43_244;
    sens.OM(3) = OMcp43_344;
    sens.A(1) = ACcp43_143;
    sens.A(2) = ACcp43_243;
    sens.A(3) = ACcp43_343;
    sens.OMP(1) = OPcp43_144;
    sens.OMP(2) = OPcp43_244;
    sens.OMP(3) = OPcp43_344;
 
% 
case 45, 


% = = Block_1_0_0_45_0_1 = = 
 
% Sensor Kinematics 


    ROcp44_45 = -S4*C5;
    ROcp44_55 = C4*C5;
    ROcp44_75 = S4*S5;
    ROcp44_85 = -C4*S5;
    ROcp44_16 = -(ROcp44_75*S6-C4*C6);
    ROcp44_26 = -(ROcp44_85*S6-S4*C6);
    ROcp44_76 = ROcp44_75*C6+C4*S6;
    ROcp44_86 = ROcp44_85*C6+S4*S6;
    OMcp44_15 = qd(5)*C4;
    OMcp44_25 = qd(5)*S4;
    OMcp44_16 = OMcp44_15+ROcp44_45*qd(6);
    OMcp44_26 = OMcp44_25+ROcp44_55*qd(6);
    OMcp44_36 = qd(4)+qd(6)*S5;
    OPcp44_16 = ROcp44_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp44_25*S5-ROcp44_55*qd(4));
    OPcp44_26 = ROcp44_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp44_15*S5-ROcp44_45*qd(4));
    OPcp44_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_45_0_18 = = 
 
% Sensor Kinematics 


    ROcp44_141 = ROcp44_16*C41-ROcp44_76*S41;
    ROcp44_241 = ROcp44_26*C41-ROcp44_86*S41;
    ROcp44_341 = -C5*S41p6;
    ROcp44_741 = ROcp44_16*S41+ROcp44_76*C41;
    ROcp44_841 = ROcp44_26*S41+ROcp44_86*C41;
    ROcp44_941 = C5*C41p6;
    RLcp44_141 = ROcp44_16*s.dpt(1,15)+ROcp44_76*s.dpt(3,15);
    RLcp44_241 = ROcp44_26*s.dpt(1,15)+ROcp44_86*s.dpt(3,15);
    RLcp44_341 = -C5*(s.dpt(1,15)*S6-s.dpt(3,15)*C6);
    OMcp44_141 = OMcp44_16+ROcp44_45*qd(41);
    OMcp44_241 = OMcp44_26+ROcp44_55*qd(41);
    OMcp44_341 = OMcp44_36+qd(41)*S5;
    ORcp44_141 = OMcp44_26*RLcp44_341-OMcp44_36*RLcp44_241;
    ORcp44_241 = -(OMcp44_16*RLcp44_341-OMcp44_36*RLcp44_141);
    ORcp44_341 = OMcp44_16*RLcp44_241-OMcp44_26*RLcp44_141;
    OPcp44_141 = OPcp44_16+ROcp44_45*qdd(41)+qd(41)*(OMcp44_26*S5-OMcp44_36*ROcp44_55);
    OPcp44_241 = OPcp44_26+ROcp44_55*qdd(41)-qd(41)*(OMcp44_16*S5-OMcp44_36*ROcp44_45);
    OPcp44_341 = OPcp44_36+qdd(41)*S5+qd(41)*(OMcp44_16*ROcp44_55-OMcp44_26*ROcp44_45);

% = = Block_1_0_0_45_0_20 = = 
 
% Sensor Kinematics 


    ROcp44_445 = ROcp44_45*C45+ROcp44_741*S45;
    ROcp44_545 = ROcp44_55*C45+ROcp44_841*S45;
    ROcp44_645 = ROcp44_941*S45+C45*S5;
    ROcp44_745 = -(ROcp44_45*S45-ROcp44_741*C45);
    ROcp44_845 = -(ROcp44_55*S45-ROcp44_841*C45);
    ROcp44_945 = ROcp44_941*C45-S45*S5;
    RLcp44_145 = ROcp44_141*s.dpt(1,60)+ROcp44_45*s.dpt(2,60);
    RLcp44_245 = ROcp44_241*s.dpt(1,60)+ROcp44_55*s.dpt(2,60);
    RLcp44_345 = ROcp44_341*s.dpt(1,60)+s.dpt(2,60)*S5;
    POcp44_145 = RLcp44_141+RLcp44_145+q(1);
    POcp44_245 = RLcp44_241+RLcp44_245+q(2);
    POcp44_345 = RLcp44_341+RLcp44_345+q(3);
    OMcp44_145 = OMcp44_141+ROcp44_141*qd(45);
    OMcp44_245 = OMcp44_241+ROcp44_241*qd(45);
    OMcp44_345 = OMcp44_341+ROcp44_341*qd(45);
    ORcp44_145 = OMcp44_241*RLcp44_345-OMcp44_341*RLcp44_245;
    ORcp44_245 = -(OMcp44_141*RLcp44_345-OMcp44_341*RLcp44_145);
    ORcp44_345 = OMcp44_141*RLcp44_245-OMcp44_241*RLcp44_145;
    VIcp44_145 = ORcp44_141+ORcp44_145+qd(1);
    VIcp44_245 = ORcp44_241+ORcp44_245+qd(2);
    VIcp44_345 = ORcp44_341+ORcp44_345+qd(3);
    OPcp44_145 = OPcp44_141+ROcp44_141*qdd(45)+qd(45)*(OMcp44_241*ROcp44_341-OMcp44_341*ROcp44_241);
    OPcp44_245 = OPcp44_241+ROcp44_241*qdd(45)-qd(45)*(OMcp44_141*ROcp44_341-OMcp44_341*ROcp44_141);
    OPcp44_345 = OPcp44_341+ROcp44_341*qdd(45)+qd(45)*(OMcp44_141*ROcp44_241-OMcp44_241*ROcp44_141);
    ACcp44_145 = qdd(1)+OMcp44_241*ORcp44_345+OMcp44_26*ORcp44_341-OMcp44_341*ORcp44_245-OMcp44_36*ORcp44_241+OPcp44_241*RLcp44_345+OPcp44_26*...
 RLcp44_341-OPcp44_341*RLcp44_245-OPcp44_36*RLcp44_241;
    ACcp44_245 = qdd(2)-OMcp44_141*ORcp44_345-OMcp44_16*ORcp44_341+OMcp44_341*ORcp44_145+OMcp44_36*ORcp44_141-OPcp44_141*RLcp44_345-OPcp44_16*...
 RLcp44_341+OPcp44_341*RLcp44_145+OPcp44_36*RLcp44_141;
    ACcp44_345 = qdd(3)+OMcp44_141*ORcp44_245+OMcp44_16*ORcp44_241-OMcp44_241*ORcp44_145-OMcp44_26*ORcp44_141+OPcp44_141*RLcp44_245+OPcp44_16*...
 RLcp44_241-OPcp44_241*RLcp44_145-OPcp44_26*RLcp44_141;

% = = Block_1_0_0_45_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp44_145;
    sens.P(2) = POcp44_245;
    sens.P(3) = POcp44_345;
    sens.R(1,1) = ROcp44_141;
    sens.R(1,2) = ROcp44_241;
    sens.R(1,3) = ROcp44_341;
    sens.R(2,1) = ROcp44_445;
    sens.R(2,2) = ROcp44_545;
    sens.R(2,3) = ROcp44_645;
    sens.R(3,1) = ROcp44_745;
    sens.R(3,2) = ROcp44_845;
    sens.R(3,3) = ROcp44_945;
    sens.V(1) = VIcp44_145;
    sens.V(2) = VIcp44_245;
    sens.V(3) = VIcp44_345;
    sens.OM(1) = OMcp44_145;
    sens.OM(2) = OMcp44_245;
    sens.OM(3) = OMcp44_345;
    sens.A(1) = ACcp44_145;
    sens.A(2) = ACcp44_245;
    sens.A(3) = ACcp44_345;
    sens.OMP(1) = OPcp44_145;
    sens.OMP(2) = OPcp44_245;
    sens.OMP(3) = OPcp44_345;
 
% 
case 46, 


% = = Block_1_0_0_46_0_1 = = 
 
% Sensor Kinematics 


    ROcp45_45 = -S4*C5;
    ROcp45_55 = C4*C5;
    ROcp45_75 = S4*S5;
    ROcp45_85 = -C4*S5;
    ROcp45_16 = -(ROcp45_75*S6-C4*C6);
    ROcp45_26 = -(ROcp45_85*S6-S4*C6);
    ROcp45_76 = ROcp45_75*C6+C4*S6;
    ROcp45_86 = ROcp45_85*C6+S4*S6;
    OMcp45_15 = qd(5)*C4;
    OMcp45_25 = qd(5)*S4;
    OMcp45_16 = OMcp45_15+ROcp45_45*qd(6);
    OMcp45_26 = OMcp45_25+ROcp45_55*qd(6);
    OMcp45_36 = qd(4)+qd(6)*S5;
    OPcp45_16 = ROcp45_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp45_25*S5-ROcp45_55*qd(4));
    OPcp45_26 = ROcp45_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp45_15*S5-ROcp45_45*qd(4));
    OPcp45_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_46_0_18 = = 
 
% Sensor Kinematics 


    ROcp45_141 = ROcp45_16*C41-ROcp45_76*S41;
    ROcp45_241 = ROcp45_26*C41-ROcp45_86*S41;
    ROcp45_341 = -C5*S41p6;
    ROcp45_741 = ROcp45_16*S41+ROcp45_76*C41;
    ROcp45_841 = ROcp45_26*S41+ROcp45_86*C41;
    ROcp45_941 = C5*C41p6;
    RLcp45_141 = ROcp45_16*s.dpt(1,15)+ROcp45_76*s.dpt(3,15);
    RLcp45_241 = ROcp45_26*s.dpt(1,15)+ROcp45_86*s.dpt(3,15);
    RLcp45_341 = -C5*(s.dpt(1,15)*S6-s.dpt(3,15)*C6);
    OMcp45_141 = OMcp45_16+ROcp45_45*qd(41);
    OMcp45_241 = OMcp45_26+ROcp45_55*qd(41);
    OMcp45_341 = OMcp45_36+qd(41)*S5;
    ORcp45_141 = OMcp45_26*RLcp45_341-OMcp45_36*RLcp45_241;
    ORcp45_241 = -(OMcp45_16*RLcp45_341-OMcp45_36*RLcp45_141);
    ORcp45_341 = OMcp45_16*RLcp45_241-OMcp45_26*RLcp45_141;
    OPcp45_141 = OPcp45_16+ROcp45_45*qdd(41)+qd(41)*(OMcp45_26*S5-OMcp45_36*ROcp45_55);
    OPcp45_241 = OPcp45_26+ROcp45_55*qdd(41)-qd(41)*(OMcp45_16*S5-OMcp45_36*ROcp45_45);
    OPcp45_341 = OPcp45_36+qdd(41)*S5+qd(41)*(OMcp45_16*ROcp45_55-OMcp45_26*ROcp45_45);

% = = Block_1_0_0_46_0_20 = = 
 
% Sensor Kinematics 


    ROcp45_445 = ROcp45_45*C45+ROcp45_741*S45;
    ROcp45_545 = ROcp45_55*C45+ROcp45_841*S45;
    ROcp45_645 = ROcp45_941*S45+C45*S5;
    ROcp45_745 = -(ROcp45_45*S45-ROcp45_741*C45);
    ROcp45_845 = -(ROcp45_55*S45-ROcp45_841*C45);
    ROcp45_945 = ROcp45_941*C45-S45*S5;
    ROcp45_146 = ROcp45_141*C46-ROcp45_745*S46;
    ROcp45_246 = ROcp45_241*C46-ROcp45_845*S46;
    ROcp45_346 = ROcp45_341*C46-ROcp45_945*S46;
    ROcp45_746 = ROcp45_141*S46+ROcp45_745*C46;
    ROcp45_846 = ROcp45_241*S46+ROcp45_845*C46;
    ROcp45_946 = ROcp45_341*S46+ROcp45_945*C46;
    RLcp45_145 = ROcp45_141*s.dpt(1,60)+ROcp45_45*s.dpt(2,60);
    RLcp45_245 = ROcp45_241*s.dpt(1,60)+ROcp45_55*s.dpt(2,60);
    RLcp45_345 = ROcp45_341*s.dpt(1,60)+s.dpt(2,60)*S5;
    POcp45_145 = RLcp45_141+RLcp45_145+q(1);
    POcp45_245 = RLcp45_241+RLcp45_245+q(2);
    POcp45_345 = RLcp45_341+RLcp45_345+q(3);
    OMcp45_145 = OMcp45_141+ROcp45_141*qd(45);
    OMcp45_245 = OMcp45_241+ROcp45_241*qd(45);
    OMcp45_345 = OMcp45_341+ROcp45_341*qd(45);
    ORcp45_145 = OMcp45_241*RLcp45_345-OMcp45_341*RLcp45_245;
    ORcp45_245 = -(OMcp45_141*RLcp45_345-OMcp45_341*RLcp45_145);
    ORcp45_345 = OMcp45_141*RLcp45_245-OMcp45_241*RLcp45_145;
    VIcp45_145 = ORcp45_141+ORcp45_145+qd(1);
    VIcp45_245 = ORcp45_241+ORcp45_245+qd(2);
    VIcp45_345 = ORcp45_341+ORcp45_345+qd(3);
    ACcp45_145 = qdd(1)+OMcp45_241*ORcp45_345+OMcp45_26*ORcp45_341-OMcp45_341*ORcp45_245-OMcp45_36*ORcp45_241+OPcp45_241*RLcp45_345+OPcp45_26*...
 RLcp45_341-OPcp45_341*RLcp45_245-OPcp45_36*RLcp45_241;
    ACcp45_245 = qdd(2)-OMcp45_141*ORcp45_345-OMcp45_16*ORcp45_341+OMcp45_341*ORcp45_145+OMcp45_36*ORcp45_141-OPcp45_141*RLcp45_345-OPcp45_16*...
 RLcp45_341+OPcp45_341*RLcp45_145+OPcp45_36*RLcp45_141;
    ACcp45_345 = qdd(3)+OMcp45_141*ORcp45_245+OMcp45_16*ORcp45_241-OMcp45_241*ORcp45_145-OMcp45_26*ORcp45_141+OPcp45_141*RLcp45_245+OPcp45_16*...
 RLcp45_241-OPcp45_241*RLcp45_145-OPcp45_26*RLcp45_141;
    OMcp45_146 = OMcp45_145+ROcp45_445*qd(46);
    OMcp45_246 = OMcp45_245+ROcp45_545*qd(46);
    OMcp45_346 = OMcp45_345+ROcp45_645*qd(46);
    OPcp45_146 = OPcp45_141+ROcp45_141*qdd(45)+ROcp45_445*qdd(46)+qd(45)*(OMcp45_241*ROcp45_341-OMcp45_341*ROcp45_241)+qd(46)*(OMcp45_245*...
 ROcp45_645-OMcp45_345*ROcp45_545);
    OPcp45_246 = OPcp45_241+ROcp45_241*qdd(45)+ROcp45_545*qdd(46)-qd(45)*(OMcp45_141*ROcp45_341-OMcp45_341*ROcp45_141)-qd(46)*(OMcp45_145*...
 ROcp45_645-OMcp45_345*ROcp45_445);
    OPcp45_346 = OPcp45_341+ROcp45_341*qdd(45)+ROcp45_645*qdd(46)+qd(45)*(OMcp45_141*ROcp45_241-OMcp45_241*ROcp45_141)+qd(46)*(OMcp45_145*...
 ROcp45_545-OMcp45_245*ROcp45_445);

% = = Block_1_0_0_46_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp45_145;
    sens.P(2) = POcp45_245;
    sens.P(3) = POcp45_345;
    sens.R(1,1) = ROcp45_146;
    sens.R(1,2) = ROcp45_246;
    sens.R(1,3) = ROcp45_346;
    sens.R(2,1) = ROcp45_445;
    sens.R(2,2) = ROcp45_545;
    sens.R(2,3) = ROcp45_645;
    sens.R(3,1) = ROcp45_746;
    sens.R(3,2) = ROcp45_846;
    sens.R(3,3) = ROcp45_946;
    sens.V(1) = VIcp45_145;
    sens.V(2) = VIcp45_245;
    sens.V(3) = VIcp45_345;
    sens.OM(1) = OMcp45_146;
    sens.OM(2) = OMcp45_246;
    sens.OM(3) = OMcp45_346;
    sens.A(1) = ACcp45_145;
    sens.A(2) = ACcp45_245;
    sens.A(3) = ACcp45_345;
    sens.OMP(1) = OPcp45_146;
    sens.OMP(2) = OPcp45_246;
    sens.OMP(3) = OPcp45_346;
 
% 
case 47, 


% = = Block_1_0_0_47_0_1 = = 
 
% Sensor Kinematics 


    ROcp46_45 = -S4*C5;
    ROcp46_55 = C4*C5;
    ROcp46_75 = S4*S5;
    ROcp46_85 = -C4*S5;
    ROcp46_16 = -(ROcp46_75*S6-C4*C6);
    ROcp46_26 = -(ROcp46_85*S6-S4*C6);
    ROcp46_36 = -C5*S6;
    ROcp46_76 = ROcp46_75*C6+C4*S6;
    ROcp46_86 = ROcp46_85*C6+S4*S6;
    ROcp46_96 = C5*C6;
    OMcp46_15 = qd(5)*C4;
    OMcp46_25 = qd(5)*S4;
    OMcp46_16 = OMcp46_15+ROcp46_45*qd(6);
    OMcp46_26 = OMcp46_25+ROcp46_55*qd(6);
    OMcp46_36 = qd(4)+qd(6)*S5;
    OPcp46_16 = ROcp46_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp46_25*S5-ROcp46_55*qd(4));
    OPcp46_26 = ROcp46_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp46_15*S5-ROcp46_45*qd(4));
    OPcp46_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_47_0_21 = = 
 
% Sensor Kinematics 


    ROcp46_447 = ROcp46_45*C47+ROcp46_76*S47;
    ROcp46_547 = ROcp46_55*C47+ROcp46_86*S47;
    ROcp46_647 = ROcp46_96*S47+C47*S5;
    ROcp46_747 = -(ROcp46_45*S47-ROcp46_76*C47);
    ROcp46_847 = -(ROcp46_55*S47-ROcp46_86*C47);
    ROcp46_947 = ROcp46_96*C47-S47*S5;
    RLcp46_147 = ROcp46_16*s.dpt(1,17)+ROcp46_45*s.dpt(2,17)+ROcp46_76*s.dpt(3,17);
    RLcp46_247 = ROcp46_26*s.dpt(1,17)+ROcp46_55*s.dpt(2,17)+ROcp46_86*s.dpt(3,17);
    RLcp46_347 = ROcp46_36*s.dpt(1,17)+ROcp46_96*s.dpt(3,17)+s.dpt(2,17)*S5;
    POcp46_147 = RLcp46_147+q(1);
    POcp46_247 = RLcp46_247+q(2);
    POcp46_347 = RLcp46_347+q(3);
    OMcp46_147 = OMcp46_16+ROcp46_16*qd(47);
    OMcp46_247 = OMcp46_26+ROcp46_26*qd(47);
    OMcp46_347 = OMcp46_36+ROcp46_36*qd(47);
    ORcp46_147 = OMcp46_26*RLcp46_347-OMcp46_36*RLcp46_247;
    ORcp46_247 = -(OMcp46_16*RLcp46_347-OMcp46_36*RLcp46_147);
    ORcp46_347 = OMcp46_16*RLcp46_247-OMcp46_26*RLcp46_147;
    VIcp46_147 = ORcp46_147+qd(1);
    VIcp46_247 = ORcp46_247+qd(2);
    VIcp46_347 = ORcp46_347+qd(3);
    OPcp46_147 = OPcp46_16+ROcp46_16*qdd(47)+qd(47)*(OMcp46_26*ROcp46_36-OMcp46_36*ROcp46_26);
    OPcp46_247 = OPcp46_26+ROcp46_26*qdd(47)-qd(47)*(OMcp46_16*ROcp46_36-OMcp46_36*ROcp46_16);
    OPcp46_347 = OPcp46_36+ROcp46_36*qdd(47)+qd(47)*(OMcp46_16*ROcp46_26-OMcp46_26*ROcp46_16);
    ACcp46_147 = qdd(1)+OMcp46_26*ORcp46_347-OMcp46_36*ORcp46_247+OPcp46_26*RLcp46_347-OPcp46_36*RLcp46_247;
    ACcp46_247 = qdd(2)-OMcp46_16*ORcp46_347+OMcp46_36*ORcp46_147-OPcp46_16*RLcp46_347+OPcp46_36*RLcp46_147;
    ACcp46_347 = qdd(3)+OMcp46_16*ORcp46_247-OMcp46_26*ORcp46_147+OPcp46_16*RLcp46_247-OPcp46_26*RLcp46_147;

% = = Block_1_0_0_47_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp46_147;
    sens.P(2) = POcp46_247;
    sens.P(3) = POcp46_347;
    sens.R(1,1) = ROcp46_16;
    sens.R(1,2) = ROcp46_26;
    sens.R(1,3) = ROcp46_36;
    sens.R(2,1) = ROcp46_447;
    sens.R(2,2) = ROcp46_547;
    sens.R(2,3) = ROcp46_647;
    sens.R(3,1) = ROcp46_747;
    sens.R(3,2) = ROcp46_847;
    sens.R(3,3) = ROcp46_947;
    sens.V(1) = VIcp46_147;
    sens.V(2) = VIcp46_247;
    sens.V(3) = VIcp46_347;
    sens.OM(1) = OMcp46_147;
    sens.OM(2) = OMcp46_247;
    sens.OM(3) = OMcp46_347;
    sens.A(1) = ACcp46_147;
    sens.A(2) = ACcp46_247;
    sens.A(3) = ACcp46_347;
    sens.OMP(1) = OPcp46_147;
    sens.OMP(2) = OPcp46_247;
    sens.OMP(3) = OPcp46_347;
 
% 
case 48, 


% = = Block_1_0_0_48_0_1 = = 
 
% Sensor Kinematics 


    ROcp47_45 = -S4*C5;
    ROcp47_55 = C4*C5;
    ROcp47_75 = S4*S5;
    ROcp47_85 = -C4*S5;
    ROcp47_16 = -(ROcp47_75*S6-C4*C6);
    ROcp47_26 = -(ROcp47_85*S6-S4*C6);
    ROcp47_36 = -C5*S6;
    ROcp47_76 = ROcp47_75*C6+C4*S6;
    ROcp47_86 = ROcp47_85*C6+S4*S6;
    ROcp47_96 = C5*C6;
    OMcp47_15 = qd(5)*C4;
    OMcp47_25 = qd(5)*S4;
    OMcp47_16 = OMcp47_15+ROcp47_45*qd(6);
    OMcp47_26 = OMcp47_25+ROcp47_55*qd(6);
    OMcp47_36 = qd(4)+qd(6)*S5;
    OPcp47_16 = ROcp47_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp47_25*S5-ROcp47_55*qd(4));
    OPcp47_26 = ROcp47_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp47_15*S5-ROcp47_45*qd(4));
    OPcp47_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_48_0_21 = = 
 
% Sensor Kinematics 


    ROcp47_447 = ROcp47_45*C47+ROcp47_76*S47;
    ROcp47_547 = ROcp47_55*C47+ROcp47_86*S47;
    ROcp47_647 = ROcp47_96*S47+C47*S5;
    ROcp47_747 = -(ROcp47_45*S47-ROcp47_76*C47);
    ROcp47_847 = -(ROcp47_55*S47-ROcp47_86*C47);
    ROcp47_947 = ROcp47_96*C47-S47*S5;
    ROcp47_448 = ROcp47_447*C48+ROcp47_747*S48;
    ROcp47_548 = ROcp47_547*C48+ROcp47_847*S48;
    ROcp47_648 = ROcp47_647*C48+ROcp47_947*S48;
    ROcp47_748 = -(ROcp47_447*S48-ROcp47_747*C48);
    ROcp47_848 = -(ROcp47_547*S48-ROcp47_847*C48);
    ROcp47_948 = -(ROcp47_647*S48-ROcp47_947*C48);
    RLcp47_147 = ROcp47_16*s.dpt(1,17)+ROcp47_45*s.dpt(2,17)+ROcp47_76*s.dpt(3,17);
    RLcp47_247 = ROcp47_26*s.dpt(1,17)+ROcp47_55*s.dpt(2,17)+ROcp47_86*s.dpt(3,17);
    RLcp47_347 = ROcp47_36*s.dpt(1,17)+ROcp47_96*s.dpt(3,17)+s.dpt(2,17)*S5;
    POcp47_147 = RLcp47_147+q(1);
    POcp47_247 = RLcp47_247+q(2);
    POcp47_347 = RLcp47_347+q(3);
    OMcp47_147 = OMcp47_16+ROcp47_16*qd(47);
    OMcp47_247 = OMcp47_26+ROcp47_26*qd(47);
    OMcp47_347 = OMcp47_36+ROcp47_36*qd(47);
    ORcp47_147 = OMcp47_26*RLcp47_347-OMcp47_36*RLcp47_247;
    ORcp47_247 = -(OMcp47_16*RLcp47_347-OMcp47_36*RLcp47_147);
    ORcp47_347 = OMcp47_16*RLcp47_247-OMcp47_26*RLcp47_147;
    VIcp47_147 = ORcp47_147+qd(1);
    VIcp47_247 = ORcp47_247+qd(2);
    VIcp47_347 = ORcp47_347+qd(3);
    ACcp47_147 = qdd(1)+OMcp47_26*ORcp47_347-OMcp47_36*ORcp47_247+OPcp47_26*RLcp47_347-OPcp47_36*RLcp47_247;
    ACcp47_247 = qdd(2)-OMcp47_16*ORcp47_347+OMcp47_36*ORcp47_147-OPcp47_16*RLcp47_347+OPcp47_36*RLcp47_147;
    ACcp47_347 = qdd(3)+OMcp47_16*ORcp47_247-OMcp47_26*ORcp47_147+OPcp47_16*RLcp47_247-OPcp47_26*RLcp47_147;
    OMcp47_148 = OMcp47_147+ROcp47_16*qd(48);
    OMcp47_248 = OMcp47_247+ROcp47_26*qd(48);
    OMcp47_348 = OMcp47_347+ROcp47_36*qd(48);
    OPcp47_148 = OPcp47_16+ROcp47_16*qdd(47)+ROcp47_16*qdd(48)+qd(47)*(OMcp47_26*ROcp47_36-OMcp47_36*ROcp47_26)+qd(48)*(OMcp47_247*ROcp47_36-...
 OMcp47_347*ROcp47_26);
    OPcp47_248 = OPcp47_26+ROcp47_26*qdd(47)+ROcp47_26*qdd(48)-qd(47)*(OMcp47_16*ROcp47_36-OMcp47_36*ROcp47_16)-qd(48)*(OMcp47_147*ROcp47_36-...
 OMcp47_347*ROcp47_16);
    OPcp47_348 = OPcp47_36+ROcp47_36*qdd(47)+ROcp47_36*qdd(48)+qd(47)*(OMcp47_16*ROcp47_26-OMcp47_26*ROcp47_16)+qd(48)*(OMcp47_147*ROcp47_26-...
 OMcp47_247*ROcp47_16);

% = = Block_1_0_0_48_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp47_147;
    sens.P(2) = POcp47_247;
    sens.P(3) = POcp47_347;
    sens.R(1,1) = ROcp47_16;
    sens.R(1,2) = ROcp47_26;
    sens.R(1,3) = ROcp47_36;
    sens.R(2,1) = ROcp47_448;
    sens.R(2,2) = ROcp47_548;
    sens.R(2,3) = ROcp47_648;
    sens.R(3,1) = ROcp47_748;
    sens.R(3,2) = ROcp47_848;
    sens.R(3,3) = ROcp47_948;
    sens.V(1) = VIcp47_147;
    sens.V(2) = VIcp47_247;
    sens.V(3) = VIcp47_347;
    sens.OM(1) = OMcp47_148;
    sens.OM(2) = OMcp47_248;
    sens.OM(3) = OMcp47_348;
    sens.A(1) = ACcp47_147;
    sens.A(2) = ACcp47_247;
    sens.A(3) = ACcp47_347;
    sens.OMP(1) = OPcp47_148;
    sens.OMP(2) = OPcp47_248;
    sens.OMP(3) = OPcp47_348;
 
% 
case 49, 


% = = Block_1_0_0_49_0_1 = = 
 
% Sensor Kinematics 


    ROcp48_45 = -S4*C5;
    ROcp48_55 = C4*C5;
    ROcp48_75 = S4*S5;
    ROcp48_85 = -C4*S5;
    ROcp48_16 = -(ROcp48_75*S6-C4*C6);
    ROcp48_26 = -(ROcp48_85*S6-S4*C6);
    ROcp48_36 = -C5*S6;
    ROcp48_76 = ROcp48_75*C6+C4*S6;
    ROcp48_86 = ROcp48_85*C6+S4*S6;
    ROcp48_96 = C5*C6;
    OMcp48_15 = qd(5)*C4;
    OMcp48_25 = qd(5)*S4;
    OMcp48_16 = OMcp48_15+ROcp48_45*qd(6);
    OMcp48_26 = OMcp48_25+ROcp48_55*qd(6);
    OMcp48_36 = qd(4)+qd(6)*S5;
    OPcp48_16 = ROcp48_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp48_25*S5-ROcp48_55*qd(4));
    OPcp48_26 = ROcp48_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp48_15*S5-ROcp48_45*qd(4));
    OPcp48_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_49_0_21 = = 
 
% Sensor Kinematics 


    ROcp48_447 = ROcp48_45*C47+ROcp48_76*S47;
    ROcp48_547 = ROcp48_55*C47+ROcp48_86*S47;
    ROcp48_647 = ROcp48_96*S47+C47*S5;
    ROcp48_747 = -(ROcp48_45*S47-ROcp48_76*C47);
    ROcp48_847 = -(ROcp48_55*S47-ROcp48_86*C47);
    ROcp48_947 = ROcp48_96*C47-S47*S5;
    ROcp48_448 = ROcp48_447*C48+ROcp48_747*S48;
    ROcp48_548 = ROcp48_547*C48+ROcp48_847*S48;
    ROcp48_648 = ROcp48_647*C48+ROcp48_947*S48;
    ROcp48_748 = -(ROcp48_447*S48-ROcp48_747*C48);
    ROcp48_848 = -(ROcp48_547*S48-ROcp48_847*C48);
    ROcp48_948 = -(ROcp48_647*S48-ROcp48_947*C48);
    ROcp48_449 = ROcp48_448*C49+ROcp48_748*S49;
    ROcp48_549 = ROcp48_548*C49+ROcp48_848*S49;
    ROcp48_649 = ROcp48_648*C49+ROcp48_948*S49;
    ROcp48_749 = -(ROcp48_448*S49-ROcp48_748*C49);
    ROcp48_849 = -(ROcp48_548*S49-ROcp48_848*C49);
    ROcp48_949 = -(ROcp48_648*S49-ROcp48_948*C49);
    RLcp48_147 = ROcp48_16*s.dpt(1,17)+ROcp48_45*s.dpt(2,17)+ROcp48_76*s.dpt(3,17);
    RLcp48_247 = ROcp48_26*s.dpt(1,17)+ROcp48_55*s.dpt(2,17)+ROcp48_86*s.dpt(3,17);
    RLcp48_347 = ROcp48_36*s.dpt(1,17)+ROcp48_96*s.dpt(3,17)+s.dpt(2,17)*S5;
    POcp48_147 = RLcp48_147+q(1);
    POcp48_247 = RLcp48_247+q(2);
    POcp48_347 = RLcp48_347+q(3);
    OMcp48_147 = OMcp48_16+ROcp48_16*qd(47);
    OMcp48_247 = OMcp48_26+ROcp48_26*qd(47);
    OMcp48_347 = OMcp48_36+ROcp48_36*qd(47);
    ORcp48_147 = OMcp48_26*RLcp48_347-OMcp48_36*RLcp48_247;
    ORcp48_247 = -(OMcp48_16*RLcp48_347-OMcp48_36*RLcp48_147);
    ORcp48_347 = OMcp48_16*RLcp48_247-OMcp48_26*RLcp48_147;
    VIcp48_147 = ORcp48_147+qd(1);
    VIcp48_247 = ORcp48_247+qd(2);
    VIcp48_347 = ORcp48_347+qd(3);
    ACcp48_147 = qdd(1)+OMcp48_26*ORcp48_347-OMcp48_36*ORcp48_247+OPcp48_26*RLcp48_347-OPcp48_36*RLcp48_247;
    ACcp48_247 = qdd(2)-OMcp48_16*ORcp48_347+OMcp48_36*ORcp48_147-OPcp48_16*RLcp48_347+OPcp48_36*RLcp48_147;
    ACcp48_347 = qdd(3)+OMcp48_16*ORcp48_247-OMcp48_26*ORcp48_147+OPcp48_16*RLcp48_247-OPcp48_26*RLcp48_147;
    OMcp48_148 = OMcp48_147+ROcp48_16*qd(48);
    OMcp48_248 = OMcp48_247+ROcp48_26*qd(48);
    OMcp48_348 = OMcp48_347+ROcp48_36*qd(48);
    OMcp48_149 = OMcp48_148+ROcp48_16*qd(49);
    OMcp48_249 = OMcp48_248+ROcp48_26*qd(49);
    OMcp48_349 = OMcp48_348+ROcp48_36*qd(49);
    OPcp48_149 = OPcp48_16+ROcp48_16*qdd(47)+ROcp48_16*qdd(48)+ROcp48_16*qdd(49)+qd(47)*(OMcp48_26*ROcp48_36-OMcp48_36*ROcp48_26)+qd(48)*(...
 OMcp48_247*ROcp48_36-OMcp48_347*ROcp48_26)+qd(49)*(OMcp48_248*ROcp48_36-OMcp48_348*ROcp48_26);
    OPcp48_249 = OPcp48_26+ROcp48_26*qdd(47)+ROcp48_26*qdd(48)+ROcp48_26*qdd(49)-qd(47)*(OMcp48_16*ROcp48_36-OMcp48_36*ROcp48_16)-qd(48)*(...
 OMcp48_147*ROcp48_36-OMcp48_347*ROcp48_16)-qd(49)*(OMcp48_148*ROcp48_36-OMcp48_348*ROcp48_16);
    OPcp48_349 = OPcp48_36+ROcp48_36*qdd(47)+ROcp48_36*qdd(48)+ROcp48_36*qdd(49)+qd(47)*(OMcp48_16*ROcp48_26-OMcp48_26*ROcp48_16)+qd(48)*(...
 OMcp48_147*ROcp48_26-OMcp48_247*ROcp48_16)+qd(49)*(OMcp48_148*ROcp48_26-OMcp48_248*ROcp48_16);

% = = Block_1_0_0_49_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp48_147;
    sens.P(2) = POcp48_247;
    sens.P(3) = POcp48_347;
    sens.R(1,1) = ROcp48_16;
    sens.R(1,2) = ROcp48_26;
    sens.R(1,3) = ROcp48_36;
    sens.R(2,1) = ROcp48_449;
    sens.R(2,2) = ROcp48_549;
    sens.R(2,3) = ROcp48_649;
    sens.R(3,1) = ROcp48_749;
    sens.R(3,2) = ROcp48_849;
    sens.R(3,3) = ROcp48_949;
    sens.V(1) = VIcp48_147;
    sens.V(2) = VIcp48_247;
    sens.V(3) = VIcp48_347;
    sens.OM(1) = OMcp48_149;
    sens.OM(2) = OMcp48_249;
    sens.OM(3) = OMcp48_349;
    sens.A(1) = ACcp48_147;
    sens.A(2) = ACcp48_247;
    sens.A(3) = ACcp48_347;
    sens.OMP(1) = OPcp48_149;
    sens.OMP(2) = OPcp48_249;
    sens.OMP(3) = OPcp48_349;
 
% 
case 50, 


% = = Block_1_0_0_50_0_1 = = 
 
% Sensor Kinematics 


    ROcp49_45 = -S4*C5;
    ROcp49_55 = C4*C5;
    ROcp49_75 = S4*S5;
    ROcp49_85 = -C4*S5;
    ROcp49_16 = -(ROcp49_75*S6-C4*C6);
    ROcp49_26 = -(ROcp49_85*S6-S4*C6);
    ROcp49_36 = -C5*S6;
    ROcp49_76 = ROcp49_75*C6+C4*S6;
    ROcp49_86 = ROcp49_85*C6+S4*S6;
    ROcp49_96 = C5*C6;
    OMcp49_15 = qd(5)*C4;
    OMcp49_25 = qd(5)*S4;
    OMcp49_16 = OMcp49_15+ROcp49_45*qd(6);
    OMcp49_26 = OMcp49_25+ROcp49_55*qd(6);
    OMcp49_36 = qd(4)+qd(6)*S5;
    OPcp49_16 = ROcp49_45*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp49_25*S5-ROcp49_55*qd(4));
    OPcp49_26 = ROcp49_55*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp49_15*S5-ROcp49_45*qd(4));
    OPcp49_36 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;

% = = Block_1_0_0_50_0_21 = = 
 
% Sensor Kinematics 


    ROcp49_447 = ROcp49_45*C47+ROcp49_76*S47;
    ROcp49_547 = ROcp49_55*C47+ROcp49_86*S47;
    ROcp49_647 = ROcp49_96*S47+C47*S5;
    ROcp49_747 = -(ROcp49_45*S47-ROcp49_76*C47);
    ROcp49_847 = -(ROcp49_55*S47-ROcp49_86*C47);
    ROcp49_947 = ROcp49_96*C47-S47*S5;
    ROcp49_448 = ROcp49_447*C48+ROcp49_747*S48;
    ROcp49_548 = ROcp49_547*C48+ROcp49_847*S48;
    ROcp49_648 = ROcp49_647*C48+ROcp49_947*S48;
    ROcp49_748 = -(ROcp49_447*S48-ROcp49_747*C48);
    ROcp49_848 = -(ROcp49_547*S48-ROcp49_847*C48);
    ROcp49_948 = -(ROcp49_647*S48-ROcp49_947*C48);
    ROcp49_449 = ROcp49_448*C49+ROcp49_748*S49;
    ROcp49_549 = ROcp49_548*C49+ROcp49_848*S49;
    ROcp49_649 = ROcp49_648*C49+ROcp49_948*S49;
    ROcp49_749 = -(ROcp49_448*S49-ROcp49_748*C49);
    ROcp49_849 = -(ROcp49_548*S49-ROcp49_848*C49);
    ROcp49_949 = -(ROcp49_648*S49-ROcp49_948*C49);
    ROcp49_450 = ROcp49_449*C50+ROcp49_749*S50;
    ROcp49_550 = ROcp49_549*C50+ROcp49_849*S50;
    ROcp49_650 = ROcp49_649*C50+ROcp49_949*S50;
    ROcp49_750 = -(ROcp49_449*S50-ROcp49_749*C50);
    ROcp49_850 = -(ROcp49_549*S50-ROcp49_849*C50);
    ROcp49_950 = -(ROcp49_649*S50-ROcp49_949*C50);
    RLcp49_147 = ROcp49_16*s.dpt(1,17)+ROcp49_45*s.dpt(2,17)+ROcp49_76*s.dpt(3,17);
    RLcp49_247 = ROcp49_26*s.dpt(1,17)+ROcp49_55*s.dpt(2,17)+ROcp49_86*s.dpt(3,17);
    RLcp49_347 = ROcp49_36*s.dpt(1,17)+ROcp49_96*s.dpt(3,17)+s.dpt(2,17)*S5;
    POcp49_147 = RLcp49_147+q(1);
    POcp49_247 = RLcp49_247+q(2);
    POcp49_347 = RLcp49_347+q(3);
    OMcp49_147 = OMcp49_16+ROcp49_16*qd(47);
    OMcp49_247 = OMcp49_26+ROcp49_26*qd(47);
    OMcp49_347 = OMcp49_36+ROcp49_36*qd(47);
    ORcp49_147 = OMcp49_26*RLcp49_347-OMcp49_36*RLcp49_247;
    ORcp49_247 = -(OMcp49_16*RLcp49_347-OMcp49_36*RLcp49_147);
    ORcp49_347 = OMcp49_16*RLcp49_247-OMcp49_26*RLcp49_147;
    VIcp49_147 = ORcp49_147+qd(1);
    VIcp49_247 = ORcp49_247+qd(2);
    VIcp49_347 = ORcp49_347+qd(3);
    ACcp49_147 = qdd(1)+OMcp49_26*ORcp49_347-OMcp49_36*ORcp49_247+OPcp49_26*RLcp49_347-OPcp49_36*RLcp49_247;
    ACcp49_247 = qdd(2)-OMcp49_16*ORcp49_347+OMcp49_36*ORcp49_147-OPcp49_16*RLcp49_347+OPcp49_36*RLcp49_147;
    ACcp49_347 = qdd(3)+OMcp49_16*ORcp49_247-OMcp49_26*ORcp49_147+OPcp49_16*RLcp49_247-OPcp49_26*RLcp49_147;
    OMcp49_148 = OMcp49_147+ROcp49_16*qd(48);
    OMcp49_248 = OMcp49_247+ROcp49_26*qd(48);
    OMcp49_348 = OMcp49_347+ROcp49_36*qd(48);
    OMcp49_149 = OMcp49_148+ROcp49_16*qd(49);
    OMcp49_249 = OMcp49_248+ROcp49_26*qd(49);
    OMcp49_349 = OMcp49_348+ROcp49_36*qd(49);
    OMcp49_150 = OMcp49_149+ROcp49_16*qd(50);
    OMcp49_250 = OMcp49_249+ROcp49_26*qd(50);
    OMcp49_350 = OMcp49_349+ROcp49_36*qd(50);
    OPcp49_150 = OPcp49_16+ROcp49_16*qdd(47)+ROcp49_16*qdd(48)+ROcp49_16*qdd(49)+ROcp49_16*qdd(50)+qd(47)*(OMcp49_26*ROcp49_36-OMcp49_36*ROcp49_26)...
 +qd(48)*(OMcp49_247*ROcp49_36-OMcp49_347*ROcp49_26)+qd(49)*(OMcp49_248*ROcp49_36-OMcp49_348*ROcp49_26)+qd(50)*(OMcp49_249*ROcp49_36-OMcp49_349*...
 ROcp49_26);
    OPcp49_250 = OPcp49_26+ROcp49_26*qdd(47)+ROcp49_26*qdd(48)+ROcp49_26*qdd(49)+ROcp49_26*qdd(50)-qd(47)*(OMcp49_16*ROcp49_36-OMcp49_36*ROcp49_16)...
 -qd(48)*(OMcp49_147*ROcp49_36-OMcp49_347*ROcp49_16)-qd(49)*(OMcp49_148*ROcp49_36-OMcp49_348*ROcp49_16)-qd(50)*(OMcp49_149*ROcp49_36-OMcp49_349*...
 ROcp49_16);
    OPcp49_350 = OPcp49_36+ROcp49_36*qdd(47)+ROcp49_36*qdd(48)+ROcp49_36*qdd(49)+ROcp49_36*qdd(50)+qd(47)*(OMcp49_16*ROcp49_26-OMcp49_26*ROcp49_16)...
 +qd(48)*(OMcp49_147*ROcp49_26-OMcp49_247*ROcp49_16)+qd(49)*(OMcp49_148*ROcp49_26-OMcp49_248*ROcp49_16)+qd(50)*(OMcp49_149*ROcp49_26-OMcp49_249*...
 ROcp49_16);

% = = Block_1_0_0_50_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp49_147;
    sens.P(2) = POcp49_247;
    sens.P(3) = POcp49_347;
    sens.R(1,1) = ROcp49_16;
    sens.R(1,2) = ROcp49_26;
    sens.R(1,3) = ROcp49_36;
    sens.R(2,1) = ROcp49_450;
    sens.R(2,2) = ROcp49_550;
    sens.R(2,3) = ROcp49_650;
    sens.R(3,1) = ROcp49_750;
    sens.R(3,2) = ROcp49_850;
    sens.R(3,3) = ROcp49_950;
    sens.V(1) = VIcp49_147;
    sens.V(2) = VIcp49_247;
    sens.V(3) = VIcp49_347;
    sens.OM(1) = OMcp49_150;
    sens.OM(2) = OMcp49_250;
    sens.OM(3) = OMcp49_350;
    sens.A(1) = ACcp49_147;
    sens.A(2) = ACcp49_247;
    sens.A(3) = ACcp49_347;
    sens.OMP(1) = OPcp49_150;
    sens.OMP(2) = OPcp49_250;
    sens.OMP(3) = OPcp49_350;

end


% ====== END Task 1 ====== 

  

