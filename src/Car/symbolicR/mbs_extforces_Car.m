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
%	==> Generation Date : Thu Apr  1 15:04:08 2021
%
%	==> Project name : Car
%	==> using XML input file 
%
%	==> Number of joints : 50
%
%	==> Function : F19 : External Forces
%	==> Flops complexity : 1708
%
%	==> Generation Time :  0.030 seconds
%	==> Post-Processing :  0.040 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,50);
 trq = zeros(3,50);

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

% = = Block_0_0_1_1_0_1 = = 
 
% Sensor Kinematics 


  ROcp4_45 = -S4*C5;
  ROcp4_55 = C4*C5;
  ROcp4_75 = S4*S5;
  ROcp4_85 = -C4*S5;
  ROcp4_16 = -(ROcp4_75*S6-C4*C6);
  ROcp4_26 = -(ROcp4_85*S6-S4*C6);
  ROcp4_36 = -C5*S6;
  ROcp4_76 = ROcp4_75*C6+C4*S6;
  ROcp4_86 = ROcp4_85*C6+S4*S6;
  ROcp4_96 = C5*C6;
  OMcp4_15 = qd(5)*C4;
  OMcp4_25 = qd(5)*S4;
  OMcp4_16 = OMcp4_15+qd(6)*ROcp4_45;
  OMcp4_26 = OMcp4_25+qd(6)*ROcp4_55;
  OMcp4_36 = qd(4)+qd(6)*S5;
  OPcp4_16 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp4_55-OMcp4_25*S5)-qdd(5)*C4-qdd(6)*ROcp4_45);
  OPcp4_26 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp4_45-OMcp4_15*S5)+qdd(5)*S4+qdd(6)*ROcp4_55;
  OPcp4_36 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;

% = = Block_0_0_1_1_0_3 = = 
 
% Sensor Kinematics 


  ROcp4_48 = ROcp4_45*C8+ROcp4_76*S8;
  ROcp4_58 = ROcp4_55*C8+ROcp4_86*S8;
  ROcp4_68 = ROcp4_96*S8+S5*C8;
  ROcp4_78 = -(ROcp4_45*S8-ROcp4_76*C8);
  ROcp4_88 = -(ROcp4_55*S8-ROcp4_86*C8);
  ROcp4_98 = ROcp4_96*C8-S5*S8;
  ROcp4_19 = ROcp4_16*C9-ROcp4_78*S9;
  ROcp4_29 = ROcp4_26*C9-ROcp4_88*S9;
  ROcp4_39 = ROcp4_36*C9-ROcp4_98*S9;
  ROcp4_79 = ROcp4_16*S9+ROcp4_78*C9;
  ROcp4_89 = ROcp4_26*S9+ROcp4_88*C9;
  ROcp4_99 = ROcp4_36*S9+ROcp4_98*C9;
  ROcp4_410 = ROcp4_48*C10+ROcp4_79*S10;
  ROcp4_510 = ROcp4_58*C10+ROcp4_89*S10;
  ROcp4_610 = ROcp4_68*C10+ROcp4_99*S10;
  ROcp4_710 = -(ROcp4_48*S10-ROcp4_79*C10);
  ROcp4_810 = -(ROcp4_58*S10-ROcp4_89*C10);
  ROcp4_910 = -(ROcp4_68*S10-ROcp4_99*C10);
  ROcp4_111 = ROcp4_19*C11+ROcp4_410*S11;
  ROcp4_211 = ROcp4_29*C11+ROcp4_510*S11;
  ROcp4_311 = ROcp4_39*C11+ROcp4_610*S11;
  ROcp4_411 = -(ROcp4_19*S11-ROcp4_410*C11);
  ROcp4_511 = -(ROcp4_29*S11-ROcp4_510*C11);
  ROcp4_611 = -(ROcp4_39*S11-ROcp4_610*C11);
  ROcp4_112 = ROcp4_111*C12-ROcp4_710*S12;
  ROcp4_212 = ROcp4_211*C12-ROcp4_810*S12;
  ROcp4_312 = ROcp4_311*C12-ROcp4_910*S12;
  ROcp4_712 = ROcp4_111*S12+ROcp4_710*C12;
  ROcp4_812 = ROcp4_211*S12+ROcp4_810*C12;
  ROcp4_912 = ROcp4_311*S12+ROcp4_910*C12;
  RLcp4_18 = ROcp4_16*s.dpt(1,1)+ROcp4_45*s.dpt(2,1)+ROcp4_76*s.dpt(3,1);
  RLcp4_28 = ROcp4_26*s.dpt(1,1)+ROcp4_55*s.dpt(2,1)+ROcp4_86*s.dpt(3,1);
  RLcp4_38 = ROcp4_36*s.dpt(1,1)+ROcp4_96*s.dpt(3,1)+s.dpt(2,1)*S5;
  OMcp4_18 = OMcp4_16+qd(8)*ROcp4_16;
  OMcp4_28 = OMcp4_26+qd(8)*ROcp4_26;
  OMcp4_38 = OMcp4_36+qd(8)*ROcp4_36;
  ORcp4_18 = OMcp4_26*RLcp4_38-OMcp4_36*RLcp4_28;
  ORcp4_28 = -(OMcp4_16*RLcp4_38-OMcp4_36*RLcp4_18);
  ORcp4_38 = OMcp4_16*RLcp4_28-OMcp4_26*RLcp4_18;
  OPcp4_18 = OPcp4_16+qd(8)*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26)+qdd(8)*ROcp4_16;
  OPcp4_28 = OPcp4_26-qd(8)*(OMcp4_16*ROcp4_36-OMcp4_36*ROcp4_16)+qdd(8)*ROcp4_26;
  OPcp4_38 = OPcp4_36+qd(8)*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16)+qdd(8)*ROcp4_36;
  RLcp4_19 = ROcp4_48*s.dpt(2,18);
  RLcp4_29 = ROcp4_58*s.dpt(2,18);
  RLcp4_39 = ROcp4_68*s.dpt(2,18);
  OMcp4_19 = OMcp4_18+qd(9)*ROcp4_48;
  OMcp4_29 = OMcp4_28+qd(9)*ROcp4_58;
  OMcp4_39 = OMcp4_38+qd(9)*ROcp4_68;
  ORcp4_19 = OMcp4_28*RLcp4_39-OMcp4_38*RLcp4_29;
  ORcp4_29 = -(OMcp4_18*RLcp4_39-OMcp4_38*RLcp4_19);
  ORcp4_39 = OMcp4_18*RLcp4_29-OMcp4_28*RLcp4_19;
  OMcp4_110 = OMcp4_19+qd(10)*ROcp4_19;
  OMcp4_210 = OMcp4_29+qd(10)*ROcp4_29;
  OMcp4_310 = OMcp4_39+qd(10)*ROcp4_39;
  OMcp4_111 = OMcp4_110+qd(11)*ROcp4_710;
  OMcp4_211 = OMcp4_210+qd(11)*ROcp4_810;
  OMcp4_311 = OMcp4_310+qd(11)*ROcp4_910;
  OPcp4_111 = OPcp4_18+qd(10)*(OMcp4_29*ROcp4_39-OMcp4_39*ROcp4_29)+qd(11)*(OMcp4_210*ROcp4_910-OMcp4_310*ROcp4_810)+qd(9)*(OMcp4_28*ROcp4_68-...
 OMcp4_38*ROcp4_58)+qdd(10)*ROcp4_19+qdd(11)*ROcp4_710+qdd(9)*ROcp4_48;
  OPcp4_211 = OPcp4_28-qd(10)*(OMcp4_19*ROcp4_39-OMcp4_39*ROcp4_19)-qd(11)*(OMcp4_110*ROcp4_910-OMcp4_310*ROcp4_710)-qd(9)*(OMcp4_18*ROcp4_68-...
 OMcp4_38*ROcp4_48)+qdd(10)*ROcp4_29+qdd(11)*ROcp4_810+qdd(9)*ROcp4_58;
  OPcp4_311 = OPcp4_38+qd(10)*(OMcp4_19*ROcp4_29-OMcp4_29*ROcp4_19)+qd(11)*(OMcp4_110*ROcp4_810-OMcp4_210*ROcp4_710)+qd(9)*(OMcp4_18*ROcp4_58-...
 OMcp4_28*ROcp4_48)+qdd(10)*ROcp4_39+qdd(11)*ROcp4_910+qdd(9)*ROcp4_68;
  RLcp4_112 = ROcp4_710*s.dpt(3,22);
  RLcp4_212 = ROcp4_810*s.dpt(3,22);
  RLcp4_312 = ROcp4_910*s.dpt(3,22);
  ORcp4_112 = OMcp4_211*RLcp4_312-OMcp4_311*RLcp4_212;
  ORcp4_212 = -(OMcp4_111*RLcp4_312-OMcp4_311*RLcp4_112);
  ORcp4_312 = OMcp4_111*RLcp4_212-OMcp4_211*RLcp4_112;
  PxF1(1) = q(1)+RLcp4_112+RLcp4_18+RLcp4_19;
  PxF1(2) = q(2)+RLcp4_212+RLcp4_28+RLcp4_29;
  PxF1(3) = q(3)+RLcp4_312+RLcp4_38+RLcp4_39;
  RxF1(1,1) = ROcp4_112;
  RxF1(1,2) = ROcp4_212;
  RxF1(1,3) = ROcp4_312;
  RxF1(2,1) = ROcp4_411;
  RxF1(2,2) = ROcp4_511;
  RxF1(2,3) = ROcp4_611;
  RxF1(3,1) = ROcp4_712;
  RxF1(3,2) = ROcp4_812;
  RxF1(3,3) = ROcp4_912;
  VxF1(1) = qd(1)+ORcp4_112+ORcp4_18+ORcp4_19;
  VxF1(2) = qd(2)+ORcp4_212+ORcp4_28+ORcp4_29;
  VxF1(3) = qd(3)+ORcp4_312+ORcp4_38+ORcp4_39;
  OMxF1(1) = OMcp4_111+qd(12)*ROcp4_411;
  OMxF1(2) = OMcp4_211+qd(12)*ROcp4_511;
  OMxF1(3) = OMcp4_311+qd(12)*ROcp4_611;
  AxF1(1) = qdd(1)+OMcp4_211*ORcp4_312+OMcp4_26*ORcp4_38+OMcp4_28*ORcp4_39-OMcp4_311*ORcp4_212-OMcp4_36*ORcp4_28-OMcp4_38*ORcp4_29+OPcp4_211*...
 RLcp4_312+OPcp4_26*RLcp4_38+OPcp4_28*RLcp4_39-OPcp4_311*RLcp4_212-OPcp4_36*RLcp4_28-OPcp4_38*RLcp4_29;
  AxF1(2) = qdd(2)-OMcp4_111*ORcp4_312-OMcp4_16*ORcp4_38-OMcp4_18*ORcp4_39+OMcp4_311*ORcp4_112+OMcp4_36*ORcp4_18+OMcp4_38*ORcp4_19-OPcp4_111*...
 RLcp4_312-OPcp4_16*RLcp4_38-OPcp4_18*RLcp4_39+OPcp4_311*RLcp4_112+OPcp4_36*RLcp4_18+OPcp4_38*RLcp4_19;
  AxF1(3) = qdd(3)+OMcp4_111*ORcp4_212+OMcp4_16*ORcp4_28+OMcp4_18*ORcp4_29-OMcp4_211*ORcp4_112-OMcp4_26*ORcp4_18-OMcp4_28*ORcp4_19+OPcp4_111*...
 RLcp4_212+OPcp4_16*RLcp4_28+OPcp4_18*RLcp4_29-OPcp4_211*RLcp4_112-OPcp4_26*RLcp4_18-OPcp4_28*RLcp4_19;
  OMPxF1(1) = OPcp4_111+qd(12)*(OMcp4_211*ROcp4_611-OMcp4_311*ROcp4_511)+qdd(12)*ROcp4_411;
  OMPxF1(2) = OPcp4_211-qd(12)*(OMcp4_111*ROcp4_611-OMcp4_311*ROcp4_411)+qdd(12)*ROcp4_511;
  OMPxF1(3) = OPcp4_311+qd(12)*(OMcp4_111*ROcp4_511-OMcp4_211*ROcp4_411)+qdd(12)*ROcp4_611;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_112*SWr1(1)+ROcp4_212*SWr1(2)+ROcp4_312*SWr1(3);
  xfrc25 = ROcp4_411*SWr1(1)+ROcp4_511*SWr1(2)+ROcp4_611*SWr1(3);
  xfrc35 = ROcp4_712*SWr1(1)+ROcp4_812*SWr1(2)+ROcp4_912*SWr1(3);
  frc(1,12) = s.frc(1,12)+xfrc15;
  frc(2,12) = s.frc(2,12)+xfrc25;
  frc(3,12) = s.frc(3,12)+xfrc35;
  xtrq15 = ROcp4_112*SWr1(4)+ROcp4_212*SWr1(5)+ROcp4_312*SWr1(6);
  xtrq25 = ROcp4_411*SWr1(4)+ROcp4_511*SWr1(5)+ROcp4_611*SWr1(6);
  xtrq35 = ROcp4_712*SWr1(4)+ROcp4_812*SWr1(5)+ROcp4_912*SWr1(6);
  trq(1,12) = s.trq(1,12)+xtrq15-xfrc25*SWr1(9)+xfrc35*SWr1(8);
  trq(2,12) = s.trq(2,12)+xtrq25+xfrc15*SWr1(9)-xfrc35*SWr1(7);
  trq(3,12) = s.trq(3,12)+xtrq35-xfrc15*SWr1(8)+xfrc25*SWr1(7);

% = = Block_0_0_1_2_0_1 = = 
 
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
  OMcp5_16 = OMcp5_15+qd(6)*ROcp5_45;
  OMcp5_26 = OMcp5_25+qd(6)*ROcp5_55;
  OMcp5_36 = qd(4)+qd(6)*S5;
  OPcp5_16 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp5_55-OMcp5_25*S5)-qdd(5)*C4-qdd(6)*ROcp5_45);
  OPcp5_26 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp5_45-OMcp5_15*S5)+qdd(5)*S4+qdd(6)*ROcp5_55;
  OPcp5_36 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;

% = = Block_0_0_1_2_0_4 = = 
 
% Sensor Kinematics 


  ROcp5_413 = ROcp5_45*C13+ROcp5_76*S13;
  ROcp5_513 = ROcp5_55*C13+ROcp5_86*S13;
  ROcp5_613 = ROcp5_96*S13+C13*S5;
  ROcp5_713 = -(ROcp5_45*S13-ROcp5_76*C13);
  ROcp5_813 = -(ROcp5_55*S13-ROcp5_86*C13);
  ROcp5_913 = ROcp5_96*C13-S13*S5;
  ROcp5_114 = ROcp5_16*C14-ROcp5_713*S14;
  ROcp5_214 = ROcp5_26*C14-ROcp5_813*S14;
  ROcp5_314 = ROcp5_36*C14-ROcp5_913*S14;
  ROcp5_714 = ROcp5_16*S14+ROcp5_713*C14;
  ROcp5_814 = ROcp5_26*S14+ROcp5_813*C14;
  ROcp5_914 = ROcp5_36*S14+ROcp5_913*C14;
  ROcp5_415 = ROcp5_413*C15+ROcp5_714*S15;
  ROcp5_515 = ROcp5_513*C15+ROcp5_814*S15;
  ROcp5_615 = ROcp5_613*C15+ROcp5_914*S15;
  ROcp5_715 = -(ROcp5_413*S15-ROcp5_714*C15);
  ROcp5_815 = -(ROcp5_513*S15-ROcp5_814*C15);
  ROcp5_915 = -(ROcp5_613*S15-ROcp5_914*C15);
  ROcp5_116 = ROcp5_114*C16+ROcp5_415*S16;
  ROcp5_216 = ROcp5_214*C16+ROcp5_515*S16;
  ROcp5_316 = ROcp5_314*C16+ROcp5_615*S16;
  ROcp5_416 = -(ROcp5_114*S16-ROcp5_415*C16);
  ROcp5_516 = -(ROcp5_214*S16-ROcp5_515*C16);
  ROcp5_616 = -(ROcp5_314*S16-ROcp5_615*C16);
  ROcp5_117 = ROcp5_116*C17-ROcp5_715*S17;
  ROcp5_217 = ROcp5_216*C17-ROcp5_815*S17;
  ROcp5_317 = ROcp5_316*C17-ROcp5_915*S17;
  ROcp5_717 = ROcp5_116*S17+ROcp5_715*C17;
  ROcp5_817 = ROcp5_216*S17+ROcp5_815*C17;
  ROcp5_917 = ROcp5_316*S17+ROcp5_915*C17;
  RLcp5_113 = ROcp5_16*s.dpt(1,2)+ROcp5_45*s.dpt(2,2)+ROcp5_76*s.dpt(3,2);
  RLcp5_213 = ROcp5_26*s.dpt(1,2)+ROcp5_55*s.dpt(2,2)+ROcp5_86*s.dpt(3,2);
  RLcp5_313 = ROcp5_36*s.dpt(1,2)+ROcp5_96*s.dpt(3,2)+s.dpt(2,2)*S5;
  OMcp5_113 = OMcp5_16+qd(13)*ROcp5_16;
  OMcp5_213 = OMcp5_26+qd(13)*ROcp5_26;
  OMcp5_313 = OMcp5_36+qd(13)*ROcp5_36;
  ORcp5_113 = OMcp5_26*RLcp5_313-OMcp5_36*RLcp5_213;
  ORcp5_213 = -(OMcp5_16*RLcp5_313-OMcp5_36*RLcp5_113);
  ORcp5_313 = OMcp5_16*RLcp5_213-OMcp5_26*RLcp5_113;
  OPcp5_113 = OPcp5_16+qd(13)*(OMcp5_26*ROcp5_36-OMcp5_36*ROcp5_26)+qdd(13)*ROcp5_16;
  OPcp5_213 = OPcp5_26-qd(13)*(OMcp5_16*ROcp5_36-OMcp5_36*ROcp5_16)+qdd(13)*ROcp5_26;
  OPcp5_313 = OPcp5_36+qd(13)*(OMcp5_16*ROcp5_26-OMcp5_26*ROcp5_16)+qdd(13)*ROcp5_36;
  RLcp5_114 = ROcp5_413*s.dpt(2,25);
  RLcp5_214 = ROcp5_513*s.dpt(2,25);
  RLcp5_314 = ROcp5_613*s.dpt(2,25);
  OMcp5_114 = OMcp5_113+qd(14)*ROcp5_413;
  OMcp5_214 = OMcp5_213+qd(14)*ROcp5_513;
  OMcp5_314 = OMcp5_313+qd(14)*ROcp5_613;
  ORcp5_114 = OMcp5_213*RLcp5_314-OMcp5_313*RLcp5_214;
  ORcp5_214 = -(OMcp5_113*RLcp5_314-OMcp5_313*RLcp5_114);
  ORcp5_314 = OMcp5_113*RLcp5_214-OMcp5_213*RLcp5_114;
  OMcp5_115 = OMcp5_114+qd(15)*ROcp5_114;
  OMcp5_215 = OMcp5_214+qd(15)*ROcp5_214;
  OMcp5_315 = OMcp5_314+qd(15)*ROcp5_314;
  OMcp5_116 = OMcp5_115+qd(16)*ROcp5_715;
  OMcp5_216 = OMcp5_215+qd(16)*ROcp5_815;
  OMcp5_316 = OMcp5_315+qd(16)*ROcp5_915;
  OPcp5_116 = OPcp5_113+qd(14)*(OMcp5_213*ROcp5_613-OMcp5_313*ROcp5_513)+qd(15)*(OMcp5_214*ROcp5_314-OMcp5_314*ROcp5_214)+qd(16)*(OMcp5_215*...
 ROcp5_915-OMcp5_315*ROcp5_815)+qdd(14)*ROcp5_413+qdd(15)*ROcp5_114+qdd(16)*ROcp5_715;
  OPcp5_216 = OPcp5_213-qd(14)*(OMcp5_113*ROcp5_613-OMcp5_313*ROcp5_413)-qd(15)*(OMcp5_114*ROcp5_314-OMcp5_314*ROcp5_114)-qd(16)*(OMcp5_115*...
 ROcp5_915-OMcp5_315*ROcp5_715)+qdd(14)*ROcp5_513+qdd(15)*ROcp5_214+qdd(16)*ROcp5_815;
  OPcp5_316 = OPcp5_313+qd(14)*(OMcp5_113*ROcp5_513-OMcp5_213*ROcp5_413)+qd(15)*(OMcp5_114*ROcp5_214-OMcp5_214*ROcp5_114)+qd(16)*(OMcp5_115*...
 ROcp5_815-OMcp5_215*ROcp5_715)+qdd(14)*ROcp5_613+qdd(15)*ROcp5_314+qdd(16)*ROcp5_915;
  RLcp5_117 = ROcp5_715*s.dpt(3,28);
  RLcp5_217 = ROcp5_815*s.dpt(3,28);
  RLcp5_317 = ROcp5_915*s.dpt(3,28);
  ORcp5_117 = OMcp5_216*RLcp5_317-OMcp5_316*RLcp5_217;
  ORcp5_217 = -(OMcp5_116*RLcp5_317-OMcp5_316*RLcp5_117);
  ORcp5_317 = OMcp5_116*RLcp5_217-OMcp5_216*RLcp5_117;
  PxF2(1) = q(1)+RLcp5_113+RLcp5_114+RLcp5_117;
  PxF2(2) = q(2)+RLcp5_213+RLcp5_214+RLcp5_217;
  PxF2(3) = q(3)+RLcp5_313+RLcp5_314+RLcp5_317;
  RxF2(1,1) = ROcp5_117;
  RxF2(1,2) = ROcp5_217;
  RxF2(1,3) = ROcp5_317;
  RxF2(2,1) = ROcp5_416;
  RxF2(2,2) = ROcp5_516;
  RxF2(2,3) = ROcp5_616;
  RxF2(3,1) = ROcp5_717;
  RxF2(3,2) = ROcp5_817;
  RxF2(3,3) = ROcp5_917;
  VxF2(1) = qd(1)+ORcp5_113+ORcp5_114+ORcp5_117;
  VxF2(2) = qd(2)+ORcp5_213+ORcp5_214+ORcp5_217;
  VxF2(3) = qd(3)+ORcp5_313+ORcp5_314+ORcp5_317;
  OMxF2(1) = OMcp5_116+qd(17)*ROcp5_416;
  OMxF2(2) = OMcp5_216+qd(17)*ROcp5_516;
  OMxF2(3) = OMcp5_316+qd(17)*ROcp5_616;
  AxF2(1) = qdd(1)+OMcp5_213*ORcp5_314+OMcp5_216*ORcp5_317+OMcp5_26*ORcp5_313-OMcp5_313*ORcp5_214-OMcp5_316*ORcp5_217-OMcp5_36*ORcp5_213+...
 OPcp5_213*RLcp5_314+OPcp5_216*RLcp5_317+OPcp5_26*RLcp5_313-OPcp5_313*RLcp5_214-OPcp5_316*RLcp5_217-OPcp5_36*RLcp5_213;
  AxF2(2) = qdd(2)-OMcp5_113*ORcp5_314-OMcp5_116*ORcp5_317-OMcp5_16*ORcp5_313+OMcp5_313*ORcp5_114+OMcp5_316*ORcp5_117+OMcp5_36*ORcp5_113-...
 OPcp5_113*RLcp5_314-OPcp5_116*RLcp5_317-OPcp5_16*RLcp5_313+OPcp5_313*RLcp5_114+OPcp5_316*RLcp5_117+OPcp5_36*RLcp5_113;
  AxF2(3) = qdd(3)+OMcp5_113*ORcp5_214+OMcp5_116*ORcp5_217+OMcp5_16*ORcp5_213-OMcp5_213*ORcp5_114-OMcp5_216*ORcp5_117-OMcp5_26*ORcp5_113+...
 OPcp5_113*RLcp5_214+OPcp5_116*RLcp5_217+OPcp5_16*RLcp5_213-OPcp5_213*RLcp5_114-OPcp5_216*RLcp5_117-OPcp5_26*RLcp5_113;
  OMPxF2(1) = OPcp5_116+qd(17)*(OMcp5_216*ROcp5_616-OMcp5_316*ROcp5_516)+qdd(17)*ROcp5_416;
  OMPxF2(2) = OPcp5_216-qd(17)*(OMcp5_116*ROcp5_616-OMcp5_316*ROcp5_416)+qdd(17)*ROcp5_516;
  OMPxF2(3) = OPcp5_316+qd(17)*(OMcp5_116*ROcp5_516-OMcp5_216*ROcp5_416)+qdd(17)*ROcp5_616;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = ROcp5_117*SWr2(1)+ROcp5_217*SWr2(2)+ROcp5_317*SWr2(3);
  xfrc26 = ROcp5_416*SWr2(1)+ROcp5_516*SWr2(2)+ROcp5_616*SWr2(3);
  xfrc36 = ROcp5_717*SWr2(1)+ROcp5_817*SWr2(2)+ROcp5_917*SWr2(3);
  frc(1,17) = s.frc(1,17)+xfrc16;
  frc(2,17) = s.frc(2,17)+xfrc26;
  frc(3,17) = s.frc(3,17)+xfrc36;
  xtrq16 = ROcp5_117*SWr2(4)+ROcp5_217*SWr2(5)+ROcp5_317*SWr2(6);
  xtrq26 = ROcp5_416*SWr2(4)+ROcp5_516*SWr2(5)+ROcp5_616*SWr2(6);
  xtrq36 = ROcp5_717*SWr2(4)+ROcp5_817*SWr2(5)+ROcp5_917*SWr2(6);
  trq(1,17) = s.trq(1,17)+xtrq16-xfrc26*SWr2(9)+xfrc36*SWr2(8);
  trq(2,17) = s.trq(2,17)+xtrq26+xfrc16*SWr2(9)-xfrc36*SWr2(7);
  trq(3,17) = s.trq(3,17)+xtrq36-xfrc16*SWr2(8)+xfrc26*SWr2(7);

% = = Block_0_0_1_3_0_1 = = 
 
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
  OMcp6_16 = OMcp6_15+qd(6)*ROcp6_45;
  OMcp6_26 = OMcp6_25+qd(6)*ROcp6_55;
  OMcp6_36 = qd(4)+qd(6)*S5;
  OPcp6_16 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp6_55-OMcp6_25*S5)-qdd(5)*C4-qdd(6)*ROcp6_45);
  OPcp6_26 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp6_45-OMcp6_15*S5)+qdd(5)*S4+qdd(6)*ROcp6_55;
  OPcp6_36 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;

% = = Block_0_0_1_3_0_16 = = 
 
% Sensor Kinematics 


  ROcp6_435 = ROcp6_45*C35+ROcp6_76*S35;
  ROcp6_535 = ROcp6_55*C35+ROcp6_86*S35;
  ROcp6_635 = ROcp6_96*S35+C35*S5;
  ROcp6_735 = -(ROcp6_45*S35-ROcp6_76*C35);
  ROcp6_835 = -(ROcp6_55*S35-ROcp6_86*C35);
  ROcp6_935 = ROcp6_96*C35-S35*S5;
  ROcp6_436 = ROcp6_435*C36+ROcp6_735*S36;
  ROcp6_536 = ROcp6_535*C36+ROcp6_835*S36;
  ROcp6_636 = ROcp6_635*C36+ROcp6_935*S36;
  ROcp6_736 = -(ROcp6_435*S36-ROcp6_735*C36);
  ROcp6_836 = -(ROcp6_535*S36-ROcp6_835*C36);
  ROcp6_936 = -(ROcp6_635*S36-ROcp6_935*C36);
  ROcp6_137 = ROcp6_16*C37-ROcp6_736*S37;
  ROcp6_237 = ROcp6_26*C37-ROcp6_836*S37;
  ROcp6_337 = ROcp6_36*C37-ROcp6_936*S37;
  ROcp6_737 = ROcp6_16*S37+ROcp6_736*C37;
  ROcp6_837 = ROcp6_26*S37+ROcp6_836*C37;
  ROcp6_937 = ROcp6_36*S37+ROcp6_936*C37;
  RLcp6_135 = ROcp6_16*s.dpt(1,12)+ROcp6_45*s.dpt(2,12)+ROcp6_76*s.dpt(3,12);
  RLcp6_235 = ROcp6_26*s.dpt(1,12)+ROcp6_55*s.dpt(2,12)+ROcp6_86*s.dpt(3,12);
  RLcp6_335 = ROcp6_36*s.dpt(1,12)+ROcp6_96*s.dpt(3,12)+s.dpt(2,12)*S5;
  OMcp6_135 = OMcp6_16+qd(35)*ROcp6_16;
  OMcp6_235 = OMcp6_26+qd(35)*ROcp6_26;
  OMcp6_335 = OMcp6_36+qd(35)*ROcp6_36;
  ORcp6_135 = OMcp6_26*RLcp6_335-OMcp6_36*RLcp6_235;
  ORcp6_235 = -(OMcp6_16*RLcp6_335-OMcp6_36*RLcp6_135);
  ORcp6_335 = OMcp6_16*RLcp6_235-OMcp6_26*RLcp6_135;
  OPcp6_135 = OPcp6_16+qd(35)*(OMcp6_26*ROcp6_36-OMcp6_36*ROcp6_26)+qdd(35)*ROcp6_16;
  OPcp6_235 = OPcp6_26-qd(35)*(OMcp6_16*ROcp6_36-OMcp6_36*ROcp6_16)+qdd(35)*ROcp6_26;
  OPcp6_335 = OPcp6_36+qd(35)*(OMcp6_16*ROcp6_26-OMcp6_26*ROcp6_16)+qdd(35)*ROcp6_36;
  RLcp6_136 = ROcp6_435*s.dpt(2,49);
  RLcp6_236 = ROcp6_535*s.dpt(2,49);
  RLcp6_336 = ROcp6_635*s.dpt(2,49);
  OMcp6_136 = OMcp6_135+qd(36)*ROcp6_16;
  OMcp6_236 = OMcp6_235+qd(36)*ROcp6_26;
  OMcp6_336 = OMcp6_335+qd(36)*ROcp6_36;
  ORcp6_136 = OMcp6_235*RLcp6_336-OMcp6_335*RLcp6_236;
  ORcp6_236 = -(OMcp6_135*RLcp6_336-OMcp6_335*RLcp6_136);
  ORcp6_336 = OMcp6_135*RLcp6_236-OMcp6_235*RLcp6_136;
  OPcp6_136 = OPcp6_135+qd(36)*(OMcp6_235*ROcp6_36-OMcp6_335*ROcp6_26)+qdd(36)*ROcp6_16;
  OPcp6_236 = OPcp6_235-qd(36)*(OMcp6_135*ROcp6_36-OMcp6_335*ROcp6_16)+qdd(36)*ROcp6_26;
  OPcp6_336 = OPcp6_335+qd(36)*(OMcp6_135*ROcp6_26-OMcp6_235*ROcp6_16)+qdd(36)*ROcp6_36;
  RLcp6_137 = ROcp6_736*s.dpt(3,52);
  RLcp6_237 = ROcp6_836*s.dpt(3,52);
  RLcp6_337 = ROcp6_936*s.dpt(3,52);
  ORcp6_137 = OMcp6_236*RLcp6_337-OMcp6_336*RLcp6_237;
  ORcp6_237 = -(OMcp6_136*RLcp6_337-OMcp6_336*RLcp6_137);
  ORcp6_337 = OMcp6_136*RLcp6_237-OMcp6_236*RLcp6_137;
  PxF3(1) = q(1)+RLcp6_135+RLcp6_136+RLcp6_137;
  PxF3(2) = q(2)+RLcp6_235+RLcp6_236+RLcp6_237;
  PxF3(3) = q(3)+RLcp6_335+RLcp6_336+RLcp6_337;
  RxF3(1,1) = ROcp6_137;
  RxF3(1,2) = ROcp6_237;
  RxF3(1,3) = ROcp6_337;
  RxF3(2,1) = ROcp6_436;
  RxF3(2,2) = ROcp6_536;
  RxF3(2,3) = ROcp6_636;
  RxF3(3,1) = ROcp6_737;
  RxF3(3,2) = ROcp6_837;
  RxF3(3,3) = ROcp6_937;
  VxF3(1) = qd(1)+ORcp6_135+ORcp6_136+ORcp6_137;
  VxF3(2) = qd(2)+ORcp6_235+ORcp6_236+ORcp6_237;
  VxF3(3) = qd(3)+ORcp6_335+ORcp6_336+ORcp6_337;
  OMxF3(1) = OMcp6_136+qd(37)*ROcp6_436;
  OMxF3(2) = OMcp6_236+qd(37)*ROcp6_536;
  OMxF3(3) = OMcp6_336+qd(37)*ROcp6_636;
  AxF3(1) = qdd(1)+OMcp6_235*ORcp6_336+OMcp6_236*ORcp6_337+OMcp6_26*ORcp6_335-OMcp6_335*ORcp6_236-OMcp6_336*ORcp6_237-OMcp6_36*ORcp6_235+...
 OPcp6_235*RLcp6_336+OPcp6_236*RLcp6_337+OPcp6_26*RLcp6_335-OPcp6_335*RLcp6_236-OPcp6_336*RLcp6_237-OPcp6_36*RLcp6_235;
  AxF3(2) = qdd(2)-OMcp6_135*ORcp6_336-OMcp6_136*ORcp6_337-OMcp6_16*ORcp6_335+OMcp6_335*ORcp6_136+OMcp6_336*ORcp6_137+OMcp6_36*ORcp6_135-...
 OPcp6_135*RLcp6_336-OPcp6_136*RLcp6_337-OPcp6_16*RLcp6_335+OPcp6_335*RLcp6_136+OPcp6_336*RLcp6_137+OPcp6_36*RLcp6_135;
  AxF3(3) = qdd(3)+OMcp6_135*ORcp6_236+OMcp6_136*ORcp6_237+OMcp6_16*ORcp6_235-OMcp6_235*ORcp6_136-OMcp6_236*ORcp6_137-OMcp6_26*ORcp6_135+...
 OPcp6_135*RLcp6_236+OPcp6_136*RLcp6_237+OPcp6_16*RLcp6_235-OPcp6_235*RLcp6_136-OPcp6_236*RLcp6_137-OPcp6_26*RLcp6_135;
  OMPxF3(1) = OPcp6_136+qd(37)*(OMcp6_236*ROcp6_636-OMcp6_336*ROcp6_536)+qdd(37)*ROcp6_436;
  OMPxF3(2) = OPcp6_236-qd(37)*(OMcp6_136*ROcp6_636-OMcp6_336*ROcp6_436)+qdd(37)*ROcp6_536;
  OMPxF3(3) = OPcp6_336+qd(37)*(OMcp6_136*ROcp6_536-OMcp6_236*ROcp6_436)+qdd(37)*ROcp6_636;
 
% Sensor Forces Computation 

  SWr3 = usrfun.fext(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc17 = ROcp6_137*SWr3(1)+ROcp6_237*SWr3(2)+ROcp6_337*SWr3(3);
  xfrc27 = ROcp6_436*SWr3(1)+ROcp6_536*SWr3(2)+ROcp6_636*SWr3(3);
  xfrc37 = ROcp6_737*SWr3(1)+ROcp6_837*SWr3(2)+ROcp6_937*SWr3(3);
  frc(1,37) = s.frc(1,37)+xfrc17;
  frc(2,37) = s.frc(2,37)+xfrc27;
  frc(3,37) = s.frc(3,37)+xfrc37;
  xtrq17 = ROcp6_137*SWr3(4)+ROcp6_237*SWr3(5)+ROcp6_337*SWr3(6);
  xtrq27 = ROcp6_436*SWr3(4)+ROcp6_536*SWr3(5)+ROcp6_636*SWr3(6);
  xtrq37 = ROcp6_737*SWr3(4)+ROcp6_837*SWr3(5)+ROcp6_937*SWr3(6);
  trq(1,37) = s.trq(1,37)+xtrq17-xfrc27*SWr3(9)+xfrc37*SWr3(8);
  trq(2,37) = s.trq(2,37)+xtrq27+xfrc17*SWr3(9)-xfrc37*SWr3(7);
  trq(3,37) = s.trq(3,37)+xtrq37-xfrc17*SWr3(8)+xfrc27*SWr3(7);

% = = Block_0_0_1_4_0_1 = = 
 
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
  OMcp7_16 = OMcp7_15+qd(6)*ROcp7_45;
  OMcp7_26 = OMcp7_25+qd(6)*ROcp7_55;
  OMcp7_36 = qd(4)+qd(6)*S5;
  OPcp7_16 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp7_55-OMcp7_25*S5)-qdd(5)*C4-qdd(6)*ROcp7_45);
  OPcp7_26 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp7_45-OMcp7_15*S5)+qdd(5)*S4+qdd(6)*ROcp7_55;
  OPcp7_36 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;

% = = Block_0_0_1_4_0_17 = = 
 
% Sensor Kinematics 


  ROcp7_438 = ROcp7_45*C38+ROcp7_76*S38;
  ROcp7_538 = ROcp7_55*C38+ROcp7_86*S38;
  ROcp7_638 = ROcp7_96*S38+C38*S5;
  ROcp7_738 = -(ROcp7_45*S38-ROcp7_76*C38);
  ROcp7_838 = -(ROcp7_55*S38-ROcp7_86*C38);
  ROcp7_938 = ROcp7_96*C38-S38*S5;
  ROcp7_439 = ROcp7_438*C39+ROcp7_738*S39;
  ROcp7_539 = ROcp7_538*C39+ROcp7_838*S39;
  ROcp7_639 = ROcp7_638*C39+ROcp7_938*S39;
  ROcp7_739 = -(ROcp7_438*S39-ROcp7_738*C39);
  ROcp7_839 = -(ROcp7_538*S39-ROcp7_838*C39);
  ROcp7_939 = -(ROcp7_638*S39-ROcp7_938*C39);
  ROcp7_140 = ROcp7_16*C40-ROcp7_739*S40;
  ROcp7_240 = ROcp7_26*C40-ROcp7_839*S40;
  ROcp7_340 = ROcp7_36*C40-ROcp7_939*S40;
  ROcp7_740 = ROcp7_16*S40+ROcp7_739*C40;
  ROcp7_840 = ROcp7_26*S40+ROcp7_839*C40;
  ROcp7_940 = ROcp7_36*S40+ROcp7_939*C40;
  RLcp7_138 = ROcp7_16*s.dpt(1,13)+ROcp7_45*s.dpt(2,13)+ROcp7_76*s.dpt(3,13);
  RLcp7_238 = ROcp7_26*s.dpt(1,13)+ROcp7_55*s.dpt(2,13)+ROcp7_86*s.dpt(3,13);
  RLcp7_338 = ROcp7_36*s.dpt(1,13)+ROcp7_96*s.dpt(3,13)+s.dpt(2,13)*S5;
  OMcp7_138 = OMcp7_16+qd(38)*ROcp7_16;
  OMcp7_238 = OMcp7_26+qd(38)*ROcp7_26;
  OMcp7_338 = OMcp7_36+qd(38)*ROcp7_36;
  ORcp7_138 = OMcp7_26*RLcp7_338-OMcp7_36*RLcp7_238;
  ORcp7_238 = -(OMcp7_16*RLcp7_338-OMcp7_36*RLcp7_138);
  ORcp7_338 = OMcp7_16*RLcp7_238-OMcp7_26*RLcp7_138;
  OPcp7_138 = OPcp7_16+qd(38)*(OMcp7_26*ROcp7_36-OMcp7_36*ROcp7_26)+qdd(38)*ROcp7_16;
  OPcp7_238 = OPcp7_26-qd(38)*(OMcp7_16*ROcp7_36-OMcp7_36*ROcp7_16)+qdd(38)*ROcp7_26;
  OPcp7_338 = OPcp7_36+qd(38)*(OMcp7_16*ROcp7_26-OMcp7_26*ROcp7_16)+qdd(38)*ROcp7_36;
  RLcp7_139 = ROcp7_438*s.dpt(2,54);
  RLcp7_239 = ROcp7_538*s.dpt(2,54);
  RLcp7_339 = ROcp7_638*s.dpt(2,54);
  OMcp7_139 = OMcp7_138+qd(39)*ROcp7_16;
  OMcp7_239 = OMcp7_238+qd(39)*ROcp7_26;
  OMcp7_339 = OMcp7_338+qd(39)*ROcp7_36;
  ORcp7_139 = OMcp7_238*RLcp7_339-OMcp7_338*RLcp7_239;
  ORcp7_239 = -(OMcp7_138*RLcp7_339-OMcp7_338*RLcp7_139);
  ORcp7_339 = OMcp7_138*RLcp7_239-OMcp7_238*RLcp7_139;
  OPcp7_139 = OPcp7_138+qd(39)*(OMcp7_238*ROcp7_36-OMcp7_338*ROcp7_26)+qdd(39)*ROcp7_16;
  OPcp7_239 = OPcp7_238-qd(39)*(OMcp7_138*ROcp7_36-OMcp7_338*ROcp7_16)+qdd(39)*ROcp7_26;
  OPcp7_339 = OPcp7_338+qd(39)*(OMcp7_138*ROcp7_26-OMcp7_238*ROcp7_16)+qdd(39)*ROcp7_36;
  RLcp7_140 = ROcp7_739*s.dpt(3,56);
  RLcp7_240 = ROcp7_839*s.dpt(3,56);
  RLcp7_340 = ROcp7_939*s.dpt(3,56);
  ORcp7_140 = OMcp7_239*RLcp7_340-OMcp7_339*RLcp7_240;
  ORcp7_240 = -(OMcp7_139*RLcp7_340-OMcp7_339*RLcp7_140);
  ORcp7_340 = OMcp7_139*RLcp7_240-OMcp7_239*RLcp7_140;
  PxF4(1) = q(1)+RLcp7_138+RLcp7_139+RLcp7_140;
  PxF4(2) = q(2)+RLcp7_238+RLcp7_239+RLcp7_240;
  PxF4(3) = q(3)+RLcp7_338+RLcp7_339+RLcp7_340;
  RxF4(1,1) = ROcp7_140;
  RxF4(1,2) = ROcp7_240;
  RxF4(1,3) = ROcp7_340;
  RxF4(2,1) = ROcp7_439;
  RxF4(2,2) = ROcp7_539;
  RxF4(2,3) = ROcp7_639;
  RxF4(3,1) = ROcp7_740;
  RxF4(3,2) = ROcp7_840;
  RxF4(3,3) = ROcp7_940;
  VxF4(1) = qd(1)+ORcp7_138+ORcp7_139+ORcp7_140;
  VxF4(2) = qd(2)+ORcp7_238+ORcp7_239+ORcp7_240;
  VxF4(3) = qd(3)+ORcp7_338+ORcp7_339+ORcp7_340;
  OMxF4(1) = OMcp7_139+qd(40)*ROcp7_439;
  OMxF4(2) = OMcp7_239+qd(40)*ROcp7_539;
  OMxF4(3) = OMcp7_339+qd(40)*ROcp7_639;
  AxF4(1) = qdd(1)+OMcp7_238*ORcp7_339+OMcp7_239*ORcp7_340+OMcp7_26*ORcp7_338-OMcp7_338*ORcp7_239-OMcp7_339*ORcp7_240-OMcp7_36*ORcp7_238+...
 OPcp7_238*RLcp7_339+OPcp7_239*RLcp7_340+OPcp7_26*RLcp7_338-OPcp7_338*RLcp7_239-OPcp7_339*RLcp7_240-OPcp7_36*RLcp7_238;
  AxF4(2) = qdd(2)-OMcp7_138*ORcp7_339-OMcp7_139*ORcp7_340-OMcp7_16*ORcp7_338+OMcp7_338*ORcp7_139+OMcp7_339*ORcp7_140+OMcp7_36*ORcp7_138-...
 OPcp7_138*RLcp7_339-OPcp7_139*RLcp7_340-OPcp7_16*RLcp7_338+OPcp7_338*RLcp7_139+OPcp7_339*RLcp7_140+OPcp7_36*RLcp7_138;
  AxF4(3) = qdd(3)+OMcp7_138*ORcp7_239+OMcp7_139*ORcp7_240+OMcp7_16*ORcp7_238-OMcp7_238*ORcp7_139-OMcp7_239*ORcp7_140-OMcp7_26*ORcp7_138+...
 OPcp7_138*RLcp7_239+OPcp7_139*RLcp7_240+OPcp7_16*RLcp7_238-OPcp7_238*RLcp7_139-OPcp7_239*RLcp7_140-OPcp7_26*RLcp7_138;
  OMPxF4(1) = OPcp7_139+qd(40)*(OMcp7_239*ROcp7_639-OMcp7_339*ROcp7_539)+qdd(40)*ROcp7_439;
  OMPxF4(2) = OPcp7_239-qd(40)*(OMcp7_139*ROcp7_639-OMcp7_339*ROcp7_439)+qdd(40)*ROcp7_539;
  OMPxF4(3) = OPcp7_339+qd(40)*(OMcp7_139*ROcp7_539-OMcp7_239*ROcp7_439)+qdd(40)*ROcp7_639;
 
% Sensor Forces Computation 

  SWr4 = usrfun.fext(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc18 = ROcp7_140*SWr4(1)+ROcp7_240*SWr4(2)+ROcp7_340*SWr4(3);
  xfrc28 = ROcp7_439*SWr4(1)+ROcp7_539*SWr4(2)+ROcp7_639*SWr4(3);
  xfrc38 = ROcp7_740*SWr4(1)+ROcp7_840*SWr4(2)+ROcp7_940*SWr4(3);
  frc(1,40) = s.frc(1,40)+xfrc18;
  frc(2,40) = s.frc(2,40)+xfrc28;
  frc(3,40) = s.frc(3,40)+xfrc38;
  xtrq18 = ROcp7_140*SWr4(4)+ROcp7_240*SWr4(5)+ROcp7_340*SWr4(6);
  xtrq28 = ROcp7_439*SWr4(4)+ROcp7_539*SWr4(5)+ROcp7_639*SWr4(6);
  xtrq38 = ROcp7_740*SWr4(4)+ROcp7_840*SWr4(5)+ROcp7_940*SWr4(6);
  trq(1,40) = s.trq(1,40)+xtrq18-xfrc28*SWr4(9)+xfrc38*SWr4(8);
  trq(2,40) = s.trq(2,40)+xtrq28+xfrc18*SWr4(9)-xfrc38*SWr4(7);
  trq(3,40) = s.trq(3,40)+xtrq38-xfrc18*SWr4(8)+xfrc28*SWr4(7);

% = = Block_0_0_1_4_1_0 = = 
 
% Symbolic Outputs  

  frc(1,6) = s.frc(1,6);
  frc(2,6) = s.frc(2,6);
  frc(3,6) = s.frc(3,6);
  frc(1,7) = s.frc(1,7);
  frc(2,7) = s.frc(2,7);
  frc(3,7) = s.frc(3,7);
  frc(1,8) = s.frc(1,8);
  frc(2,8) = s.frc(2,8);
  frc(3,8) = s.frc(3,8);
  frc(1,11) = s.frc(1,11);
  frc(2,11) = s.frc(2,11);
  frc(3,11) = s.frc(3,11);
  frc(1,13) = s.frc(1,13);
  frc(2,13) = s.frc(2,13);
  frc(3,13) = s.frc(3,13);
  frc(1,16) = s.frc(1,16);
  frc(2,16) = s.frc(2,16);
  frc(3,16) = s.frc(3,16);
  frc(1,18) = s.frc(1,18);
  frc(2,18) = s.frc(2,18);
  frc(3,18) = s.frc(3,18);
  frc(1,19) = s.frc(1,19);
  frc(2,19) = s.frc(2,19);
  frc(3,19) = s.frc(3,19);
  frc(1,20) = s.frc(1,20);
  frc(2,20) = s.frc(2,20);
  frc(3,20) = s.frc(3,20);
  frc(1,21) = s.frc(1,21);
  frc(2,21) = s.frc(2,21);
  frc(3,21) = s.frc(3,21);
  frc(1,22) = s.frc(1,22);
  frc(2,22) = s.frc(2,22);
  frc(3,22) = s.frc(3,22);
  frc(1,24) = s.frc(1,24);
  frc(2,24) = s.frc(2,24);
  frc(3,24) = s.frc(3,24);
  frc(1,25) = s.frc(1,25);
  frc(2,25) = s.frc(2,25);
  frc(3,25) = s.frc(3,25);
  frc(1,27) = s.frc(1,27);
  frc(2,27) = s.frc(2,27);
  frc(3,27) = s.frc(3,27);
  frc(1,28) = s.frc(1,28);
  frc(2,28) = s.frc(2,28);
  frc(3,28) = s.frc(3,28);
  frc(1,30) = s.frc(1,30);
  frc(2,30) = s.frc(2,30);
  frc(3,30) = s.frc(3,30);
  frc(1,32) = s.frc(1,32);
  frc(2,32) = s.frc(2,32);
  frc(3,32) = s.frc(3,32);
  frc(1,34) = s.frc(1,34);
  frc(2,34) = s.frc(2,34);
  frc(3,34) = s.frc(3,34);
  frc(1,35) = s.frc(1,35);
  frc(2,35) = s.frc(2,35);
  frc(3,35) = s.frc(3,35);
  frc(1,36) = s.frc(1,36);
  frc(2,36) = s.frc(2,36);
  frc(3,36) = s.frc(3,36);
  frc(1,38) = s.frc(1,38);
  frc(2,38) = s.frc(2,38);
  frc(3,38) = s.frc(3,38);
  frc(1,39) = s.frc(1,39);
  frc(2,39) = s.frc(2,39);
  frc(3,39) = s.frc(3,39);
  frc(1,41) = s.frc(1,41);
  frc(2,41) = s.frc(2,41);
  frc(3,41) = s.frc(3,41);
  frc(1,42) = s.frc(1,42);
  frc(2,42) = s.frc(2,42);
  frc(3,42) = s.frc(3,42);
  frc(1,44) = s.frc(1,44);
  frc(2,44) = s.frc(2,44);
  frc(3,44) = s.frc(3,44);
  frc(1,46) = s.frc(1,46);
  frc(2,46) = s.frc(2,46);
  frc(3,46) = s.frc(3,46);
  frc(1,50) = s.frc(1,50);
  frc(2,50) = s.frc(2,50);
  frc(3,50) = s.frc(3,50);
  trq(1,6) = s.trq(1,6);
  trq(2,6) = s.trq(2,6);
  trq(3,6) = s.trq(3,6);
  trq(1,7) = s.trq(1,7);
  trq(2,7) = s.trq(2,7);
  trq(3,7) = s.trq(3,7);
  trq(1,8) = s.trq(1,8);
  trq(2,8) = s.trq(2,8);
  trq(3,8) = s.trq(3,8);
  trq(1,11) = s.trq(1,11);
  trq(2,11) = s.trq(2,11);
  trq(3,11) = s.trq(3,11);
  trq(1,13) = s.trq(1,13);
  trq(2,13) = s.trq(2,13);
  trq(3,13) = s.trq(3,13);
  trq(1,16) = s.trq(1,16);
  trq(2,16) = s.trq(2,16);
  trq(3,16) = s.trq(3,16);
  trq(1,18) = s.trq(1,18);
  trq(2,18) = s.trq(2,18);
  trq(3,18) = s.trq(3,18);
  trq(1,19) = s.trq(1,19);
  trq(2,19) = s.trq(2,19);
  trq(3,19) = s.trq(3,19);
  trq(1,20) = s.trq(1,20);
  trq(2,20) = s.trq(2,20);
  trq(3,20) = s.trq(3,20);
  trq(1,21) = s.trq(1,21);
  trq(2,21) = s.trq(2,21);
  trq(3,21) = s.trq(3,21);
  trq(1,22) = s.trq(1,22);
  trq(2,22) = s.trq(2,22);
  trq(3,22) = s.trq(3,22);
  trq(1,24) = s.trq(1,24);
  trq(2,24) = s.trq(2,24);
  trq(3,24) = s.trq(3,24);
  trq(1,25) = s.trq(1,25);
  trq(2,25) = s.trq(2,25);
  trq(3,25) = s.trq(3,25);
  trq(1,27) = s.trq(1,27);
  trq(2,27) = s.trq(2,27);
  trq(3,27) = s.trq(3,27);
  trq(1,28) = s.trq(1,28);
  trq(2,28) = s.trq(2,28);
  trq(3,28) = s.trq(3,28);
  trq(1,30) = s.trq(1,30);
  trq(2,30) = s.trq(2,30);
  trq(3,30) = s.trq(3,30);
  trq(1,32) = s.trq(1,32);
  trq(2,32) = s.trq(2,32);
  trq(3,32) = s.trq(3,32);
  trq(1,34) = s.trq(1,34);
  trq(2,34) = s.trq(2,34);
  trq(3,34) = s.trq(3,34);
  trq(1,35) = s.trq(1,35);
  trq(2,35) = s.trq(2,35);
  trq(3,35) = s.trq(3,35);
  trq(1,36) = s.trq(1,36);
  trq(2,36) = s.trq(2,36);
  trq(3,36) = s.trq(3,36);
  trq(1,38) = s.trq(1,38);
  trq(2,38) = s.trq(2,38);
  trq(3,38) = s.trq(3,38);
  trq(1,39) = s.trq(1,39);
  trq(2,39) = s.trq(2,39);
  trq(3,39) = s.trq(3,39);
  trq(1,41) = s.trq(1,41);
  trq(2,41) = s.trq(2,41);
  trq(3,41) = s.trq(3,41);
  trq(1,42) = s.trq(1,42);
  trq(2,42) = s.trq(2,42);
  trq(3,42) = s.trq(3,42);
  trq(1,44) = s.trq(1,44);
  trq(2,44) = s.trq(2,44);
  trq(3,44) = s.trq(3,44);
  trq(1,46) = s.trq(1,46);
  trq(2,46) = s.trq(2,46);
  trq(3,46) = s.trq(3,46);
  trq(1,50) = s.trq(1,50);
  trq(2,50) = s.trq(2,50);
  trq(3,50) = s.trq(3,50);

% ====== END Task 0 ====== 

  

