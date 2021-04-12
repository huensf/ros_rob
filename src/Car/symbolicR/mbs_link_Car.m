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
%	==> Function : F 7 : Point to point Link Forces (frc,trq,Flnk) 
%	==> Flops complexity : 172
%
%	==> Generation Time :  0.010 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [frc,trq,Flnk,Z,Zd] = link(s,tsim,usrfun)

 frc = zeros(3,50);
 trq = zeros(3,50);
 Flnk = zeros(4,1);
 Z = zeros(4,1);
 Zd = zeros(4,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

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

% = = Block_0_1_0_0_1_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk1_272 = s.dpt(2,32)*C18;
  RLlnk1_372 = s.dpt(2,32)*S18;

% = = Block_0_1_0_0_2_6 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk3_274 = s.dpt(2,34)*C19;
  RLlnk3_374 = s.dpt(2,34)*S19;

% = = Block_0_1_0_0_3_7 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk5_276 = s.dpt(2,36)*C20;
  RLlnk5_376 = s.dpt(2,36)*S20;

% = = Block_0_1_0_0_4_8 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk6_277 = s.dpt(2,38)*C21;
  RLlnk6_377 = s.dpt(2,38)*S21;

% = = Block_0_1_0_1_1_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk21 = RLlnk1_272-s.dpt(2,11)+s.dpt(2,3);
  Plnk31 = RLlnk1_372-s.dpt(3,11);
  Z1 = sqrt(Plnk21*Plnk21+Plnk31*Plnk31);
  e21 = Plnk21/Z1;
  e31 = Plnk31/Z1;
  Zd1 = qd(18)*(RLlnk1_272*e31-RLlnk1_372*e21);
 
% Link Force Computation 

  Flink1 = usrfun.flink(Z1,Zd1,s,tsim,1);

% = = Block_0_1_0_1_2_6 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk22 = RLlnk3_274-s.dpt(2,14)+s.dpt(2,4);
  Plnk32 = RLlnk3_374-s.dpt(3,14);
  Z2 = sqrt(Plnk22*Plnk22+Plnk32*Plnk32);
  e22 = Plnk22/Z2;
  e32 = Plnk32/Z2;
  Zd2 = qd(19)*(RLlnk3_274*e32-RLlnk3_374*e22);
 
% Link Force Computation 

  Flink2 = usrfun.flink(Z2,Zd2,s,tsim,2);

% = = Block_0_1_0_1_3_7 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk13 = s.dpt(1,5)-s.dpt(1,6);
  Plnk23 = RLlnk5_276+s.dpt(2,5)-s.dpt(2,6);
  Plnk33 = RLlnk5_376-s.dpt(3,6);
  Z3 = sqrt(Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33);
  e13 = Plnk13/Z3;
  e23 = Plnk23/Z3;
  e33 = Plnk33/Z3;
  Zd3 = qd(20)*(RLlnk5_276*e33-RLlnk5_376*e23);
 
% Link Force Computation 

  Flink3 = usrfun.flink(Z3,Zd3,s,tsim,3);

% = = Block_0_1_0_1_4_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk14 = s.dpt(1,10)-s.dpt(1,7);
  Plnk24 = -(RLlnk6_277-s.dpt(2,10)+s.dpt(2,7));
  Plnk34 = -(RLlnk6_377-s.dpt(3,10));
  Z4 = sqrt(Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34);
  e14 = Plnk14/Z4;
  e24 = Plnk24/Z4;
  e34 = Plnk34/Z4;
  Zd4 = -qd(21)*(RLlnk6_277*e34-RLlnk6_377*e24);
 
% Link Force Computation 

  Flink4 = usrfun.flink(Z4,Zd4,s,tsim,4);

% = = Block_0_1_0_2_2_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk21 = Flink1*e21;
  fPlnk31 = Flink1*e31;

% = = Block_0_1_0_2_2_5 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk21 = Flink1*(e21*C18+e31*S18);
  fSlnk31 = -Flink1*(e21*S18-e31*C18);
  frc(2,18) = s.frc(2,18)-fSlnk21;
  frc(3,18) = s.frc(3,18)-fSlnk31;
  trq(1,18) = s.trq(1,18)-fSlnk31*s.dpt(2,32);

% = = Block_0_1_0_2_3_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk22 = Flink2*e22;
  fPlnk32 = Flink2*e32;

% = = Block_0_1_0_2_3_6 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk22 = Flink2*(e22*C19+e32*S19);
  fSlnk32 = -Flink2*(e22*S19-e32*C19);
  frc(2,19) = s.frc(2,19)-fSlnk22;
  frc(3,19) = s.frc(3,19)-fSlnk32;
  trq(1,19) = s.trq(1,19)-fSlnk32*s.dpt(2,34);

% = = Block_0_1_0_2_4_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk13 = Flink3*e13;
  fPlnk23 = Flink3*e23;
  fPlnk33 = Flink3*e33;
  s.frc(1,6) = s.frc(1,6)+fPlnk13;
  s.frc(2,6) = s.frc(2,6)+fPlnk21+fPlnk22+fPlnk23;
  s.frc(3,6) = s.frc(3,6)+fPlnk31+fPlnk32+fPlnk33;
  s.trq(1,6) = s.trq(1,6)-fPlnk21*(s.dpt(3,11)-s.l(3,6))-fPlnk22*(s.dpt(3,14)-s.l(3,6))-fPlnk23*(s.dpt(3,6)-s.l(3,6))+fPlnk31*s.dpt(2,11)+fPlnk32...
 *s.dpt(2,14)+fPlnk33*s.dpt(2,6);
  s.trq(2,6) = s.trq(2,6)+fPlnk13*(s.dpt(3,6)-s.l(3,6))+fPlnk31*s.l(1,6)+fPlnk32*s.l(1,6)-fPlnk33*(s.dpt(1,6)-s.l(1,6));
  s.trq(3,6) = s.trq(3,6)-fPlnk13*s.dpt(2,6)-fPlnk21*s.l(1,6)-fPlnk22*s.l(1,6)+fPlnk23*(s.dpt(1,6)-s.l(1,6));

% = = Block_0_1_0_2_4_7 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk13 = Flink3*e13;
  fSlnk23 = Flink3*(e23*C20+e33*S20);
  fSlnk33 = -Flink3*(e23*S20-e33*C20);
  frc(1,20) = s.frc(1,20)-fSlnk13;
  frc(2,20) = s.frc(2,20)-fSlnk23;
  frc(3,20) = s.frc(3,20)-fSlnk33;
  trq(1,20) = s.trq(1,20)-fSlnk33*s.dpt(2,36);
  trq(3,20) = s.trq(3,20)+fSlnk13*s.dpt(2,36);

% = = Block_0_1_0_2_5_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk14 = Flink4*e14;
  fSlnk24 = Flink4*e24;
  fSlnk34 = Flink4*e34;
  frc(1,6) = -(fSlnk14-s.frc(1,6));
  frc(2,6) = -(fSlnk24-s.frc(2,6));
  frc(3,6) = -(fSlnk34-s.frc(3,6));
  trq(1,6) = s.trq(1,6)+fSlnk24*(s.dpt(3,10)-s.l(3,6))-fSlnk34*s.dpt(2,10);
  trq(2,6) = s.trq(2,6)-fSlnk14*(s.dpt(3,10)-s.l(3,6))+fSlnk34*(s.dpt(1,10)-s.l(1,6));
  trq(3,6) = s.trq(3,6)+fSlnk14*s.dpt(2,10)-fSlnk24*(s.dpt(1,10)-s.l(1,6));

% = = Block_0_1_0_2_5_8 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk14 = Flink4*e14;
  fPlnk24 = Flink4*(e24*C21+e34*S21);
  fPlnk34 = -Flink4*(e24*S21-e34*C21);
  frc(1,21) = s.frc(1,21)+fPlnk14;
  frc(2,21) = s.frc(2,21)+fPlnk24;
  frc(3,21) = s.frc(3,21)+fPlnk34;
  trq(1,21) = s.trq(1,21)+fPlnk34*s.dpt(2,38);
  trq(3,21) = s.trq(3,21)-fPlnk14*s.dpt(2,38);

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  frc(1,7) = s.frc(1,7);
  frc(2,7) = s.frc(2,7);
  frc(3,7) = s.frc(3,7);
  frc(1,8) = s.frc(1,8);
  frc(2,8) = s.frc(2,8);
  frc(3,8) = s.frc(3,8);
  frc(1,11) = s.frc(1,11);
  frc(2,11) = s.frc(2,11);
  frc(3,11) = s.frc(3,11);
  frc(1,12) = s.frc(1,12);
  frc(2,12) = s.frc(2,12);
  frc(3,12) = s.frc(3,12);
  frc(1,13) = s.frc(1,13);
  frc(2,13) = s.frc(2,13);
  frc(3,13) = s.frc(3,13);
  frc(1,16) = s.frc(1,16);
  frc(2,16) = s.frc(2,16);
  frc(3,16) = s.frc(3,16);
  frc(1,17) = s.frc(1,17);
  frc(2,17) = s.frc(2,17);
  frc(3,17) = s.frc(3,17);
  frc(1,18) = s.frc(1,18);
  frc(1,19) = s.frc(1,19);
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
  frc(1,37) = s.frc(1,37);
  frc(2,37) = s.frc(2,37);
  frc(3,37) = s.frc(3,37);
  frc(1,38) = s.frc(1,38);
  frc(2,38) = s.frc(2,38);
  frc(3,38) = s.frc(3,38);
  frc(1,39) = s.frc(1,39);
  frc(2,39) = s.frc(2,39);
  frc(3,39) = s.frc(3,39);
  frc(1,40) = s.frc(1,40);
  frc(2,40) = s.frc(2,40);
  frc(3,40) = s.frc(3,40);
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
  trq(1,7) = s.trq(1,7);
  trq(2,7) = s.trq(2,7);
  trq(3,7) = s.trq(3,7);
  trq(1,8) = s.trq(1,8);
  trq(2,8) = s.trq(2,8);
  trq(3,8) = s.trq(3,8);
  trq(1,11) = s.trq(1,11);
  trq(2,11) = s.trq(2,11);
  trq(3,11) = s.trq(3,11);
  trq(1,12) = s.trq(1,12);
  trq(2,12) = s.trq(2,12);
  trq(3,12) = s.trq(3,12);
  trq(1,13) = s.trq(1,13);
  trq(2,13) = s.trq(2,13);
  trq(3,13) = s.trq(3,13);
  trq(1,16) = s.trq(1,16);
  trq(2,16) = s.trq(2,16);
  trq(3,16) = s.trq(3,16);
  trq(1,17) = s.trq(1,17);
  trq(2,17) = s.trq(2,17);
  trq(3,17) = s.trq(3,17);
  trq(2,18) = s.trq(2,18);
  trq(3,18) = s.trq(3,18);
  trq(2,19) = s.trq(2,19);
  trq(3,19) = s.trq(3,19);
  trq(2,20) = s.trq(2,20);
  trq(2,21) = s.trq(2,21);
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
  trq(1,37) = s.trq(1,37);
  trq(2,37) = s.trq(2,37);
  trq(3,37) = s.trq(3,37);
  trq(1,38) = s.trq(1,38);
  trq(2,38) = s.trq(2,38);
  trq(3,38) = s.trq(3,38);
  trq(1,39) = s.trq(1,39);
  trq(2,39) = s.trq(2,39);
  trq(3,39) = s.trq(3,39);
  trq(1,40) = s.trq(1,40);
  trq(2,40) = s.trq(2,40);
  trq(3,40) = s.trq(3,40);
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
  Flnk(1) = Flink1;
  Flnk(2) = Flink2;
  Flnk(3) = Flink3;
  Flnk(4) = Flink4;
  Z(1) = Z1;
  Z(2) = Z2;
  Z(3) = Z3;
  Z(4) = Z4;
  Zd(1) = Zd1;
  Zd(2) = Zd2;
  Zd(3) = Zd3;
  Zd(4) = Zd4;

% ====== END Task 0 ====== 

  

