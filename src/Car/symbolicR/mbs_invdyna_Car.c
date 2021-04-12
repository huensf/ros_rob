//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Tue Dec  8 10:41:21 2020
//
//	==> Project name : Car
//	==> using XML input file 
//
//	==> Number of joints : 50
//
//	==> Function : F 2 : Inverse Dynamics : RNEA
//	==> Flops complexity : 2540
//
//	==> Generation Time :  0.050 seconds
//	==> Post-Processing :  0.040 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)

// double Qq[50];
{ 
 
#include "mbs_invdyna_Car.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);
  C11 = cos(q[11]);
  S11 = sin(q[11]);
  C12 = cos(q[12]);
  S12 = sin(q[12]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);
  C17 = cos(q[17]);
  S17 = sin(q[17]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C18 = cos(q[18]);
  S18 = sin(q[18]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C20 = cos(q[20]);
  S20 = sin(q[20]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C22 = cos(q[22]);
  S22 = sin(q[22]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);

// = = Block_0_0_0_0_0_11 = = 
 
// Trigonometric Variables  

  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);

// = = Block_0_0_0_0_0_13 = = 
 
// Trigonometric Variables  

  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);

// = = Block_0_0_0_0_0_14 = = 
 
// Trigonometric Variables  

  C31 = cos(q[31]);
  S31 = sin(q[31]);
  C32 = cos(q[32]);
  S32 = sin(q[32]);

// = = Block_0_0_0_0_0_15 = = 
 
// Augmented Joint Position Vectors   

  Dz332 = q[33]+s->dpt[2][46];
 
// Trigonometric Variables  

  C34 = cos(q[34]);
  S34 = sin(q[34]);

// = = Block_0_0_0_0_0_16 = = 
 
// Trigonometric Variables  

  C35 = cos(q[35]);
  S35 = sin(q[35]);
  C36 = cos(q[36]);
  S36 = sin(q[36]);
  C37 = cos(q[37]);
  S37 = sin(q[37]);

// = = Block_0_0_0_0_0_17 = = 
 
// Trigonometric Variables  

  C38 = cos(q[38]);
  S38 = sin(q[38]);
  C39 = cos(q[39]);
  S39 = sin(q[39]);
  C40 = cos(q[40]);
  S40 = sin(q[40]);

// = = Block_0_0_0_0_0_18 = = 
 
// Trigonometric Variables  

  C41 = cos(q[41]);
  S41 = sin(q[41]);

// = = Block_0_0_0_0_0_19 = = 
 
// Trigonometric Variables  

  C42 = cos(q[42]);
  S42 = sin(q[42]);
  C43 = cos(q[43]);
  S43 = sin(q[43]);
  C44 = cos(q[44]);
  S44 = sin(q[44]);

// = = Block_0_0_0_0_0_20 = = 
 
// Trigonometric Variables  

  C45 = cos(q[45]);
  S45 = sin(q[45]);
  C46 = cos(q[46]);
  S46 = sin(q[46]);

// = = Block_0_0_0_0_0_21 = = 
 
// Trigonometric Variables  

  C47 = cos(q[47]);
  S47 = sin(q[47]);
  C48 = cos(q[48]);
  S48 = sin(q[48]);
  C49 = cos(q[49]);
  S49 = sin(q[49]);
  C50 = cos(q[50]);
  S50 = sin(q[50]);

// = = Block_0_1_0_0_0_0 = = 
 
// Forward Kinematics 

  ALPHA33 = qdd[3]-s->g[3];
  ALPHA14 = qdd[1]*C4+qdd[2]*S4;
  ALPHA24 = -(qdd[1]*S4-qdd[2]*C4);
  OM35 = qd[4]*C5;
  OMp35 = -(qd[4]*qd[5]*S5-qdd[4]*C5);
  ALPHA25 = ALPHA24*C5+ALPHA33*S5;
  ALPHA35 = -(ALPHA24*S5-ALPHA33*C5);
  OM16 = qd[5]*C6-OM35*S6;
  OM26 = qd[6]+qd[4]*S5;
  OM36 = qd[5]*S6+OM35*C6;
  OMp16 = C6*(qdd[5]-qd[6]*OM35)-S6*(OMp35+qd[5]*qd[6]);
  OMp26 = qdd[6]+qd[4]*qd[5]*C5+qdd[4]*S5;
  OMp36 = C6*(OMp35+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM35);
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BETA26 = BS26-OMp36;
  BETA36 = BS36+OMp26;
  BETA46 = BS26+OMp36;
  BETA66 = BS66-OMp16;
  BETA76 = BS36-OMp26;
  BETA86 = BS66+OMp16;
  ALPHA16 = ALPHA14*C6-ALPHA35*S6;
  ALPHA36 = ALPHA14*S6+ALPHA35*C6;
  OM18 = qd[8]+OM16;
  OM28 = OM26*C8+OM36*S8;
  OM38 = -(OM26*S8-OM36*C8);
  OMp18 = qdd[8]+OMp16;
  OMp38 = C8*(OMp36-qd[8]*OM26)-S8*(OMp26+qd[8]*OM36);
  BETA28 = OM18*OM28-OMp38;
  BETA88 = OMp18+OM28*OM38;
  ALPHA18 = ALPHA16+BETA26*s->dpt[2][1]+BETA36*s->dpt[3][1]+BS16*s->dpt[1][1];
  ALPHA38 = C8*(ALPHA36+BETA76*s->dpt[1][1]+BETA86*s->dpt[2][1]+BS96*s->dpt[3][1])-S8*(ALPHA25+BETA46*s->dpt[1][1]+
 BETA66*s->dpt[3][1]+BS56*s->dpt[2][1]);
  OM29 = qd[9]+OM28;
  OM39 = OM18*S9+OM38*C9;
  OMp29 = qdd[9]+C8*(OMp26+qd[8]*OM36)+S8*(OMp36-qd[8]*OM26);
  OMp39 = C9*(OMp38+qd[9]*OM18)+S9*(OMp18-qd[9]*OM38);
  ALPHA19 = C9*(ALPHA18+BETA28*s->dpt[2][18])-S9*(ALPHA38+BETA88*s->dpt[2][18]);
  ALPHA29 = C8*(ALPHA25+BETA46*s->dpt[1][1]+BETA66*s->dpt[3][1]+BS56*s->dpt[2][1])+S8*(ALPHA36+BETA76*s->dpt[1][1]+
 BETA86*s->dpt[2][1]+BS96*s->dpt[3][1])-s->dpt[2][18]*(OM18*OM18+OM38*OM38);
  ALPHA39 = C9*(ALPHA38+BETA88*s->dpt[2][18])+S9*(ALPHA18+BETA28*s->dpt[2][18]);
  OM110 = qd[10]+OM18*C9-OM38*S9;
  OM210 = OM29*C10+OM39*S10;
  OMp110 = qdd[10]+C9*(OMp18-qd[9]*OM38)-S9*(OMp38+qd[9]*OM18);
  OMp210 = C10*(OMp29+qd[10]*OM39)+S10*(OMp39-qd[10]*OM29);
  ALPHA210 = ALPHA29*C10+ALPHA39*S10;
  ALPHA310 = -(ALPHA29*S10-ALPHA39*C10);
  OM111 = OM110*C11+OM210*S11;
  OM211 = -(OM110*S11-OM210*C11);
  OM311 = qd[11]-OM29*S10+OM39*C10;
  OMp111 = C11*(OMp110+qd[11]*OM210)+S11*(OMp210-qd[11]*OM110);
  OMp211 = C11*(OMp210-qd[11]*OM110)-S11*(OMp110+qd[11]*OM210);
  OMp311 = qdd[11]+C10*(OMp39-qd[10]*OM29)-S10*(OMp29+qd[10]*OM39);
  BS911 = -(OM111*OM111+OM211*OM211);
  BETA311 = OMp211+OM111*OM311;
  ALPHA111 = ALPHA19*C11+ALPHA210*S11;
  ALPHA211 = -(ALPHA19*S11-ALPHA210*C11);
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd[12]+OM211;
  OM312 = OM111*S12+OM311*C12;
  OM113 = qd[13]+OM16;
  OM213 = OM26*C13+OM36*S13;
  OM313 = -(OM26*S13-OM36*C13);
  OMp113 = qdd[13]+OMp16;
  OMp313 = C13*(OMp36-qd[13]*OM26)-S13*(OMp26+qd[13]*OM36);
  BETA213 = OM113*OM213-OMp313;
  BETA813 = OMp113+OM213*OM313;
  ALPHA113 = ALPHA16+BETA26*s->dpt[2][2]+BETA36*s->dpt[3][2]+BS16*s->dpt[1][2];
  ALPHA313 = C13*(ALPHA36+BETA76*s->dpt[1][2]+BETA86*s->dpt[2][2]+BS96*s->dpt[3][2])-S13*(ALPHA25+BETA46*s->dpt[1][2]+
 BETA66*s->dpt[3][2]+BS56*s->dpt[2][2]);
  OM214 = qd[14]+OM213;
  OM314 = OM113*S14+OM313*C14;
  OMp214 = qdd[14]+C13*(OMp26+qd[13]*OM36)+S13*(OMp36-qd[13]*OM26);
  OMp314 = C14*(OMp313+qd[14]*OM113)+S14*(OMp113-qd[14]*OM313);
  ALPHA114 = C14*(ALPHA113+BETA213*s->dpt[2][25])-S14*(ALPHA313+BETA813*s->dpt[2][25]);
  ALPHA214 = C13*(ALPHA25+BETA46*s->dpt[1][2]+BETA66*s->dpt[3][2]+BS56*s->dpt[2][2])+S13*(ALPHA36+BETA76*s->dpt[1][2]+
 BETA86*s->dpt[2][2]+BS96*s->dpt[3][2])-s->dpt[2][25]*(OM113*OM113+OM313*OM313);
  ALPHA314 = C14*(ALPHA313+BETA813*s->dpt[2][25])+S14*(ALPHA113+BETA213*s->dpt[2][25]);
  OM115 = qd[15]+OM113*C14-OM313*S14;
  OM215 = OM214*C15+OM314*S15;
  OMp115 = qdd[15]+C14*(OMp113-qd[14]*OM313)-S14*(OMp313+qd[14]*OM113);
  OMp215 = C15*(OMp214+qd[15]*OM314)+S15*(OMp314-qd[15]*OM214);
  ALPHA215 = ALPHA214*C15+ALPHA314*S15;
  ALPHA315 = -(ALPHA214*S15-ALPHA314*C15);
  OM116 = OM115*C16+OM215*S16;
  OM216 = -(OM115*S16-OM215*C16);
  OM316 = qd[16]-OM214*S15+OM314*C15;
  OMp116 = C16*(OMp115+qd[16]*OM215)+S16*(OMp215-qd[16]*OM115);
  OMp216 = C16*(OMp215-qd[16]*OM115)-S16*(OMp115+qd[16]*OM215);
  OMp316 = qdd[16]+C15*(OMp314-qd[15]*OM214)-S15*(OMp214+qd[15]*OM314);
  BS916 = -(OM116*OM116+OM216*OM216);
  BETA316 = OMp216+OM116*OM316;
  ALPHA116 = ALPHA114*C16+ALPHA215*S16;
  ALPHA216 = -(ALPHA114*S16-ALPHA215*C16);
  OM117 = OM116*C17-OM316*S17;
  OM217 = qd[17]+OM216;
  OM317 = OM116*S17+OM316*C17;
  ALPHA128 = ALPHA16+q[28]*BETA26-(2.0)*qd[28]*OM36+BETA36*s->dpt[3][9]+BS16*s->dpt[1][9];
  ALPHA228 = qdd[28]+ALPHA25+q[28]*BS56+BETA46*s->dpt[1][9]+BETA66*s->dpt[3][9];
  ALPHA328 = ALPHA36+q[28]*BETA86+(2.0)*qd[28]*OM16+BETA76*s->dpt[1][9]+BS96*s->dpt[3][9];
  ALPHA133 = ALPHA128-(2.0)*qd[33]*OM36-Dz332*(OMp36-OM16*OM26);
  ALPHA233 = qdd[33]+ALPHA228-Dz332*(OM16*OM16+OM36*OM36);
  OM134 = OM16*C34+OM26*S34;
  OM234 = -(OM16*S34-OM26*C34);
  OM334 = qd[34]+OM36;
  OM135 = qd[35]+OM16;
  OM235 = OM26*C35+OM36*S35;
  OM335 = -(OM26*S35-OM36*C35);
  OMp135 = qdd[35]+OMp16;
  OMp235 = C35*(OMp26+qd[35]*OM36)+S35*(OMp36-qd[35]*OM26);
  OMp335 = C35*(OMp36-qd[35]*OM26)-S35*(OMp26+qd[35]*OM36);
  BS535 = -(OM135*OM135+OM335*OM335);
  BETA835 = OMp135+OM235*OM335;
  ALPHA235 = C35*(ALPHA25+BETA46*s->dpt[1][12]+BETA66*s->dpt[3][12]+BS56*s->dpt[2][12])+S35*(ALPHA36+BETA76*
 s->dpt[1][12]+BETA86*s->dpt[2][12]+BS96*s->dpt[3][12]);
  ALPHA335 = C35*(ALPHA36+BETA76*s->dpt[1][12]+BETA86*s->dpt[2][12]+BS96*s->dpt[3][12])-S35*(ALPHA25+BETA46*
 s->dpt[1][12]+BETA66*s->dpt[3][12]+BS56*s->dpt[2][12]);
  OM136 = qd[36]+OM135;
  OM236 = OM235*C36+OM335*S36;
  OM336 = -(OM235*S36-OM335*C36);
  OMp136 = qdd[36]+OMp135;
  OMp236 = C36*(OMp235+qd[36]*OM335)+S36*(OMp335-qd[36]*OM235);
  OMp336 = C36*(OMp335-qd[36]*OM235)-S36*(OMp235+qd[36]*OM335);
  BS936 = -(OM136*OM136+OM236*OM236);
  BETA336 = OMp236+OM136*OM336;
  ALPHA136 = ALPHA16+BETA26*s->dpt[2][12]+BETA36*s->dpt[3][12]+BS16*s->dpt[1][12]-s->dpt[2][49]*(OMp335-OM135*OM235);
  ALPHA236 = C36*(ALPHA235+BS535*s->dpt[2][49])+S36*(ALPHA335+BETA835*s->dpt[2][49]);
  ALPHA336 = C36*(ALPHA335+BETA835*s->dpt[2][49])-S36*(ALPHA235+BS535*s->dpt[2][49]);
  OM137 = OM136*C37-OM336*S37;
  OM237 = qd[37]+OM236;
  OM337 = OM136*S37+OM336*C37;
  OM138 = qd[38]+OM16;
  OM238 = OM26*C38+OM36*S38;
  OM338 = -(OM26*S38-OM36*C38);
  OMp138 = qdd[38]+OMp16;
  OMp238 = C38*(OMp26+qd[38]*OM36)+S38*(OMp36-qd[38]*OM26);
  OMp338 = C38*(OMp36-qd[38]*OM26)-S38*(OMp26+qd[38]*OM36);
  BS538 = -(OM138*OM138+OM338*OM338);
  BETA838 = OMp138+OM238*OM338;
  ALPHA238 = C38*(ALPHA25+BETA46*s->dpt[1][13]+BETA66*s->dpt[3][13]+BS56*s->dpt[2][13])+S38*(ALPHA36+BETA76*
 s->dpt[1][13]+BETA86*s->dpt[2][13]+BS96*s->dpt[3][13]);
  ALPHA338 = C38*(ALPHA36+BETA76*s->dpt[1][13]+BETA86*s->dpt[2][13]+BS96*s->dpt[3][13])-S38*(ALPHA25+BETA46*
 s->dpt[1][13]+BETA66*s->dpt[3][13]+BS56*s->dpt[2][13]);
  OM139 = qd[39]+OM138;
  OM239 = OM238*C39+OM338*S39;
  OM339 = -(OM238*S39-OM338*C39);
  OMp139 = qdd[39]+OMp138;
  OMp239 = C39*(OMp238+qd[39]*OM338)+S39*(OMp338-qd[39]*OM238);
  OMp339 = C39*(OMp338-qd[39]*OM238)-S39*(OMp238+qd[39]*OM338);
  BS939 = -(OM139*OM139+OM239*OM239);
  BETA339 = OMp239+OM139*OM339;
  ALPHA139 = ALPHA16+BETA26*s->dpt[2][13]+BETA36*s->dpt[3][13]+BS16*s->dpt[1][13]-s->dpt[2][54]*(OMp338-OM138*OM238);
  ALPHA239 = C39*(ALPHA238+BS538*s->dpt[2][54])+S39*(ALPHA338+BETA838*s->dpt[2][54]);
  ALPHA339 = C39*(ALPHA338+BETA838*s->dpt[2][54])-S39*(ALPHA238+BS538*s->dpt[2][54]);
  OM140 = OM139*C40-OM339*S40;
  OM240 = qd[40]+OM239;
  OM340 = OM139*S40+OM339*C40;
  OM247 = OM26*C47+OM36*S47;
  OM347 = -(OM26*S47-OM36*C47);
  OMp247 = C47*(OMp26+qd[47]*OM36)+S47*(OMp36-qd[47]*OM26);
  OMp347 = C47*(OMp36-qd[47]*OM26)-S47*(OMp26+qd[47]*OM36);
  ALPHA247 = C47*(ALPHA25+BETA46*s->dpt[1][17]+BETA66*s->dpt[3][17]+BS56*s->dpt[2][17])+S47*(ALPHA36+BETA76*
 s->dpt[1][17]+BETA86*s->dpt[2][17]+BS96*s->dpt[3][17]);
  ALPHA347 = C47*(ALPHA36+BETA76*s->dpt[1][17]+BETA86*s->dpt[2][17]+BS96*s->dpt[3][17])-S47*(ALPHA25+BETA46*
 s->dpt[1][17]+BETA66*s->dpt[3][17]+BS56*s->dpt[2][17]);
  OM248 = OM247*C48+OM347*S48;
  OM348 = -(OM247*S48-OM347*C48);
  OMp248 = C48*(OMp247+qd[48]*OM347)+S48*(OMp347-qd[48]*OM247);
  OMp348 = C48*(OMp347-qd[48]*OM247)-S48*(OMp247+qd[48]*OM347);
  ALPHA248 = ALPHA247*C48+ALPHA347*S48;
  ALPHA348 = -(ALPHA247*S48-ALPHA347*C48);
  OM249 = OM248*C49+OM348*S49;
  OM349 = -(OM248*S49-OM348*C49);
  OMp249 = C49*(OMp248+qd[49]*OM348)+S49*(OMp348-qd[49]*OM248);
  OMp349 = C49*(OMp348-qd[49]*OM248)-S49*(OMp248+qd[49]*OM348);
  ALPHA249 = ALPHA248*C49+ALPHA348*S49;
  ALPHA349 = -(ALPHA248*S49-ALPHA348*C49);
  OM150 = qd[47]+qd[48]+qd[49]+qd[50]+OM16;
  OM250 = OM249*C50+OM349*S50;
  OM350 = -(OM249*S50-OM349*C50);
 
// Backward Dynamics 

  Fs150 = -(s->frc[1][50]-s->m[50]*(ALPHA16+BETA26*s->dpt[2][17]+BETA36*s->dpt[3][17]+BS16*s->dpt[1][17]));
  Fs250 = -(s->frc[2][50]-s->m[50]*(ALPHA249*C50+ALPHA349*S50));
  Fs350 = -(s->frc[3][50]+s->m[50]*(ALPHA249*S50-ALPHA349*C50));
  Cq150 = -(s->trq[1][50]-s->In[1][50]*(qdd[47]+qdd[48]+qdd[49]+qdd[50]+OMp16)+OM250*OM350*(s->In[5][50]-s->In[9][50]));
  Cq250 = -(s->trq[2][50]-s->In[5][50]*(C50*(OMp249+qd[50]*OM349)+S50*(OMp349-qd[50]*OM249))-OM150*OM350*(s->In[1][50]-
 s->In[9][50]));
  Cq350 = -(s->trq[3][50]-s->In[9][50]*(C50*(OMp349-qd[50]*OM249)-S50*(OMp249+qd[50]*OM349))+OM150*OM250*(s->In[1][50]-
 s->In[5][50]));
  Fq249 = Fs250*C50-Fs350*S50;
  Fq349 = Fs250*S50+Fs350*C50;
  Cq249 = Cq250*C50-Cq350*S50;
  Cq349 = Cq250*S50+Cq350*C50;
  Fq248 = Fq249*C49-Fq349*S49;
  Fq348 = Fq249*S49+Fq349*C49;
  Cq248 = Cq249*C49-Cq349*S49;
  Cq348 = Cq249*S49+Cq349*C49;
  Fq247 = Fq248*C48-Fq348*S48;
  Fq347 = Fq248*S48+Fq348*C48;
  Cq247 = Cq248*C48-Cq348*S48;
  Cq347 = Cq248*S48+Cq348*C48;
  Fq145 = -(s->frc[1][46]*C46+s->frc[3][46]*S46);
  Fq345 = s->frc[1][46]*S46-s->frc[3][46]*C46;
  Cq145 = -(s->trq[1][46]*C46+s->trq[3][46]*S46);
  Cq345 = s->trq[1][46]*S46-s->trq[3][46]*C46;
  Fq143 = -(s->frc[1][44]*C44+s->frc[3][44]*S44);
  Fq343 = s->frc[1][44]*S44-s->frc[3][44]*C44;
  Cq143 = -(s->trq[1][44]*C44+s->trq[3][44]*S44);
  Cq343 = s->trq[1][44]*S44-s->trq[3][44]*C44;
  Fq142 = -(s->frc[1][42]-Fq143);
  Fq342 = -(s->frc[3][42]+s->frc[2][44]*S43-Fq343*C43);
  Cq142 = -(s->trq[1][42]-Cq143+s->dpt[2][61]*(s->frc[2][44]*S43-Fq343*C43));
  Cq242 = -(s->trq[2][42]+s->trq[2][44]*C43+Cq343*S43-s->dpt[1][61]*(s->frc[2][44]*S43-Fq343*C43));
  Cq342 = -(s->trq[3][42]+s->trq[2][44]*S43-Cq343*C43+Fq143*s->dpt[2][61]+s->dpt[1][61]*(s->frc[2][44]*C43+Fq343*S43));
  Fq141 = -(s->frc[1][41]-Fq145-Fq142*C42-Fq342*S42);
  Fq241 = -(s->frc[2][41]+s->frc[2][42]+s->frc[2][44]*C43+s->frc[2][46]*C45+Fq343*S43+Fq345*S45);
  Fq341 = -(s->frc[3][41]+s->frc[2][46]*S45+Fq142*S42-Fq342*C42-Fq345*C45);
  Cq141 = -(s->trq[1][41]-Cq145-Cq142*C42-Cq342*S42+s->dpt[2][60]*(s->frc[2][46]*S45-Fq345*C45));
  Cq241 = -(s->trq[2][41]-Cq242+s->trq[2][46]*C45+Cq345*S45-s->dpt[1][60]*(s->frc[2][46]*S45-Fq345*C45));
  Cq341 = -(s->trq[3][41]+s->trq[2][46]*S45+Cq142*S42-Cq342*C42-Cq345*C45+Fq145*s->dpt[2][60]+s->dpt[1][60]*(
 s->frc[2][46]*C45+Fq345*S45));
  Fs140 = -(s->frc[1][40]-s->m[40]*(C40*(ALPHA139+BETA339*s->dpt[3][56])-S40*(ALPHA339+BS939*s->dpt[3][56])));
  Fs240 = -(s->frc[2][40]-s->m[40]*(ALPHA239-s->dpt[3][56]*(OMp139-OM239*OM339)));
  Fs340 = -(s->frc[3][40]-s->m[40]*(C40*(ALPHA339+BS939*s->dpt[3][56])+S40*(ALPHA139+BETA339*s->dpt[3][56])));
  Cq140 = -(s->trq[1][40]-s->In[1][40]*(C40*(OMp139-qd[40]*OM339)-S40*(OMp339+qd[40]*OM139))+OM240*OM340*(s->In[5][40]-
 s->In[9][40]));
  Cq240 = -(s->trq[2][40]-s->In[5][40]*(qdd[40]+OMp239)-OM140*OM340*(s->In[1][40]-s->In[9][40]));
  Cq340 = -(s->trq[3][40]-s->In[9][40]*(C40*(OMp339+qd[40]*OM139)+S40*(OMp139-qd[40]*OM339))+OM140*OM240*(s->In[1][40]-
 s->In[5][40]));
  Fq139 = -(s->frc[1][39]-s->m[39]*ALPHA139-Fs140*C40-Fs340*S40);
  Fq239 = -(s->frc[2][39]-Fs240-s->m[39]*ALPHA239);
  Fq339 = -(s->frc[3][39]-s->m[39]*ALPHA339+Fs140*S40-Fs340*C40);
  Cq139 = -(s->trq[1][39]-Cq140*C40-Cq340*S40+Fs240*s->dpt[3][56]);
  Cq239 = -(s->trq[2][39]-Cq240-s->dpt[3][56]*(Fs140*C40+Fs340*S40));
  Cq339 = -(s->trq[3][39]+Cq140*S40-Cq340*C40);
  Fq138 = -(s->frc[1][38]-Fq139);
  Fq238 = -(s->frc[2][38]-Fq239*C39+Fq339*S39);
  Fq338 = -(s->frc[3][38]-Fq239*S39-Fq339*C39);
  Cq138 = -(s->trq[1][38]-Cq139-s->dpt[2][54]*(Fq239*S39+Fq339*C39));
  Cq238 = -(s->trq[2][38]-Cq239*C39+Cq339*S39);
  Cq338 = -(s->trq[3][38]-Cq239*S39-Cq339*C39+Fq139*s->dpt[2][54]);
  Fs137 = -(s->frc[1][37]-s->m[37]*(C37*(ALPHA136+BETA336*s->dpt[3][52])-S37*(ALPHA336+BS936*s->dpt[3][52])));
  Fs237 = -(s->frc[2][37]-s->m[37]*(ALPHA236-s->dpt[3][52]*(OMp136-OM236*OM336)));
  Fs337 = -(s->frc[3][37]-s->m[37]*(C37*(ALPHA336+BS936*s->dpt[3][52])+S37*(ALPHA136+BETA336*s->dpt[3][52])));
  Cq137 = -(s->trq[1][37]-s->In[1][37]*(C37*(OMp136-qd[37]*OM336)-S37*(OMp336+qd[37]*OM136))+OM237*OM337*(s->In[5][37]-
 s->In[9][37]));
  Cq237 = -(s->trq[2][37]-s->In[5][37]*(qdd[37]+OMp236)-OM137*OM337*(s->In[1][37]-s->In[9][37]));
  Cq337 = -(s->trq[3][37]-s->In[9][37]*(C37*(OMp336+qd[37]*OM136)+S37*(OMp136-qd[37]*OM336))+OM137*OM237*(s->In[1][37]-
 s->In[5][37]));
  Fq136 = -(s->frc[1][36]-s->m[36]*ALPHA136-Fs137*C37-Fs337*S37);
  Fq236 = -(s->frc[2][36]-Fs237-s->m[36]*ALPHA236);
  Fq336 = -(s->frc[3][36]-s->m[36]*ALPHA336+Fs137*S37-Fs337*C37);
  Cq136 = -(s->trq[1][36]-Cq137*C37-Cq337*S37+Fs237*s->dpt[3][52]);
  Cq236 = -(s->trq[2][36]-Cq237-s->dpt[3][52]*(Fs137*C37+Fs337*S37));
  Cq336 = -(s->trq[3][36]+Cq137*S37-Cq337*C37);
  Fq135 = -(s->frc[1][35]-Fq136);
  Fq235 = -(s->frc[2][35]-Fq236*C36+Fq336*S36);
  Fq335 = -(s->frc[3][35]-Fq236*S36-Fq336*C36);
  Cq135 = -(s->trq[1][35]-Cq136-s->dpt[2][49]*(Fq236*S36+Fq336*C36));
  Cq235 = -(s->trq[2][35]-Cq236*C36+Cq336*S36);
  Cq335 = -(s->trq[3][35]-Cq236*S36-Cq336*C36+Fq136*s->dpt[2][49]);
  Fs134 = -(s->frc[1][34]-s->m[34]*(ALPHA133*C34+ALPHA233*S34));
  Fs234 = -(s->frc[2][34]+s->m[34]*(ALPHA133*S34-ALPHA233*C34));
  Fs334 = -(s->frc[3][34]-s->m[34]*(ALPHA328+(2.0)*qd[33]*OM16+Dz332*(OMp16+OM26*OM36)));
  Cq134 = -(s->trq[1][34]-s->In[1][34]*(C34*(OMp16+qd[34]*OM26)+S34*(OMp26-qd[34]*OM16))+OM234*OM334*(s->In[5][34]-
 s->In[9][34]));
  Cq234 = -(s->trq[2][34]-s->In[5][34]*(C34*(OMp26-qd[34]*OM16)-S34*(OMp16+qd[34]*OM26))-OM134*OM334*(s->In[1][34]-
 s->In[9][34]));
  Cq334 = -(s->trq[3][34]-s->In[9][34]*(qdd[34]+OMp36)+OM134*OM234*(s->In[1][34]-s->In[5][34]));
  Fq133 = Fs134*C34-Fs234*S34;
  Fq233 = Fs134*S34+Fs234*C34;
  Fq131 = -(s->frc[1][32]*C32-s->frc[2][32]*S32);
  Fq231 = -(s->frc[1][32]*S32+s->frc[2][32]*C32);
  Cq131 = -(s->trq[1][32]*C32-s->trq[2][32]*S32);
  Cq231 = -(s->trq[1][32]*S32+s->trq[2][32]*C32);
  Fq129 = -(s->frc[1][30]*C30-s->frc[2][30]*S30);
  Fq229 = -(s->frc[1][30]*S30+s->frc[2][30]*C30);
  Cq129 = -(s->trq[1][30]*C30-s->trq[2][30]*S30);
  Cq229 = -(s->trq[1][30]*S30+s->trq[2][30]*C30);
  Fq128 = -(s->frc[1][28]-Fq129-Fq131-Fq133-s->m[28]*ALPHA128);
  Fq228 = -(s->frc[2][28]-Fq233-s->frc[3][30]*S29-s->frc[3][32]*S31-s->m[28]*ALPHA228-Fq229*C29-Fq231*C31);
  Fq328 = -(s->frc[3][28]-Fs334+s->frc[3][30]*C29+s->frc[3][32]*C31-s->m[28]*ALPHA328-Fq229*S29-Fq231*S31);
  Fq126 = -(s->frc[1][27]*C27+s->frc[3][27]*S27);
  Fq326 = s->frc[1][27]*S27-s->frc[3][27]*C27;
  Cq126 = -(s->trq[1][27]*C27+s->trq[3][27]*S27);
  Cq326 = s->trq[1][27]*S27-s->trq[3][27]*C27;
  Fq125 = -(s->frc[1][25]-Fq126);
  Fq325 = -(s->frc[3][25]+s->frc[2][27]*S26-Fq326*C26);
  Cq125 = -(s->trq[1][25]-Cq126+s->dpt[2][42]*(s->frc[2][27]*S26-Fq326*C26));
  Cq225 = -(s->trq[2][25]+s->trq[2][27]*C26+Cq326*S26-s->dpt[1][42]*(s->frc[2][27]*S26-Fq326*C26));
  Cq325 = -(s->trq[3][25]+s->trq[2][27]*S26-Cq326*C26+Fq126*s->dpt[2][42]+s->dpt[1][42]*(s->frc[2][27]*C26+Fq326*S26));
  Fq123 = -(s->frc[1][24]*C24+s->frc[3][24]*S24);
  Fq323 = s->frc[1][24]*S24-s->frc[3][24]*C24;
  Cq123 = -(s->trq[1][24]*C24+s->trq[3][24]*S24);
  Cq323 = s->trq[1][24]*S24-s->trq[3][24]*C24;
  Fq122 = -(s->frc[1][22]-Fq123-Fq125*C25-Fq325*S25);
  Fq222 = -(s->frc[2][22]+s->frc[2][25]+s->frc[2][24]*C23+s->frc[2][27]*C26+Fq323*S23+Fq326*S26);
  Fq322 = -(s->frc[3][22]+s->frc[2][24]*S23+Fq125*S25-Fq323*C23-Fq325*C25);
  Cq122 = -(s->trq[1][22]-Cq123-Cq125*C25-Cq325*S25+s->dpt[2][39]*(s->frc[2][24]*S23-Fq323*C23));
  Cq222 = -(s->trq[2][22]-Cq225+s->trq[2][24]*C23+Cq323*S23-s->dpt[1][39]*(s->frc[2][24]*S23-Fq323*C23));
  Cq322 = -(s->trq[3][22]+s->trq[2][24]*S23+Cq125*S25-Cq323*C23-Cq325*C25+Fq123*s->dpt[2][39]+s->dpt[1][39]*(
 s->frc[2][24]*C23+Fq323*S23));
  Fs117 = -(s->frc[1][17]-s->m[17]*(C17*(ALPHA116+BETA316*s->dpt[3][28])-S17*(ALPHA315+BS916*s->dpt[3][28])));
  Fs217 = -(s->frc[2][17]-s->m[17]*(ALPHA216-s->dpt[3][28]*(OMp116-OM216*OM316)));
  Fs317 = -(s->frc[3][17]-s->m[17]*(C17*(ALPHA315+BS916*s->dpt[3][28])+S17*(ALPHA116+BETA316*s->dpt[3][28])));
  Cq117 = -(s->trq[1][17]-s->In[1][17]*(C17*(OMp116-qd[17]*OM316)-S17*(OMp316+qd[17]*OM116))+OM217*OM317*(s->In[5][17]-
 s->In[9][17]));
  Cq217 = -(s->trq[2][17]-s->In[5][17]*(qdd[17]+OMp216)-OM117*OM317*(s->In[1][17]-s->In[9][17]));
  Cq317 = -(s->trq[3][17]-s->In[9][17]*(C17*(OMp316+qd[17]*OM116)+S17*(OMp116-qd[17]*OM316))+OM117*OM217*(s->In[1][17]-
 s->In[5][17]));
  Fq116 = -(s->frc[1][16]-s->m[16]*ALPHA116-Fs117*C17-Fs317*S17);
  Fq216 = -(s->frc[2][16]-Fs217-s->m[16]*ALPHA216);
  Fq316 = -(s->frc[3][16]-s->m[16]*ALPHA315+Fs117*S17-Fs317*C17);
  Cq116 = -(s->trq[1][16]-Cq117*C17-Cq317*S17+Fs217*s->dpt[3][28]);
  Cq216 = -(s->trq[2][16]-Cq217-s->dpt[3][28]*(Fs117*C17+Fs317*S17));
  Cq316 = -(s->trq[3][16]+Cq117*S17-Cq317*C17);
  Fq115 = Fq116*C16-Fq216*S16;
  Fq215 = Fq116*S16+Fq216*C16;
  Cq115 = Cq116*C16-Cq216*S16;
  Cq215 = Cq116*S16+Cq216*C16;
  Fq314 = Fq215*S15+Fq316*C15;
  Cq214 = Cq215*C15-Cq316*S15;
  Cq314 = Cq215*S15+Cq316*C15;
  Fq113 = -(s->frc[1][13]-Fq115*C14-Fq314*S14);
  Fq213 = -(s->frc[2][13]-Fq215*C15+Fq316*S15);
  Fq313 = -(s->frc[3][13]+Fq115*S14-Fq314*C14);
  Cq113 = -(s->trq[1][13]-Cq115*C14-Cq314*S14+s->dpt[2][25]*(Fq115*S14-Fq314*C14));
  Cq213 = -(s->trq[2][13]-Cq214);
  Cq313 = -(s->trq[3][13]+Cq115*S14-Cq314*C14+s->dpt[2][25]*(Fq115*C14+Fq314*S14));
  Fs112 = -(s->frc[1][12]-s->m[12]*(C12*(ALPHA111+BETA311*s->dpt[3][22])-S12*(ALPHA310+BS911*s->dpt[3][22])));
  Fs212 = -(s->frc[2][12]-s->m[12]*(ALPHA211-s->dpt[3][22]*(OMp111-OM211*OM311)));
  Fs312 = -(s->frc[3][12]-s->m[12]*(C12*(ALPHA310+BS911*s->dpt[3][22])+S12*(ALPHA111+BETA311*s->dpt[3][22])));
  Cq112 = -(s->trq[1][12]-s->In[1][12]*(C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111))+OM212*OM312*(s->In[5][12]-
 s->In[9][12]));
  Cq212 = -(s->trq[2][12]-s->In[5][12]*(qdd[12]+OMp211)-OM112*OM312*(s->In[1][12]-s->In[9][12]));
  Cq312 = -(s->trq[3][12]-s->In[9][12]*(C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311))+OM112*OM212*(s->In[1][12]-
 s->In[5][12]));
  Fq111 = -(s->frc[1][11]-s->m[11]*ALPHA111-Fs112*C12-Fs312*S12);
  Fq211 = -(s->frc[2][11]-Fs212-s->m[11]*ALPHA211);
  Fq311 = -(s->frc[3][11]-s->m[11]*ALPHA310+Fs112*S12-Fs312*C12);
  Cq111 = -(s->trq[1][11]-Cq112*C12-Cq312*S12+Fs212*s->dpt[3][22]);
  Cq211 = -(s->trq[2][11]-Cq212-s->dpt[3][22]*(Fs112*C12+Fs312*S12));
  Cq311 = -(s->trq[3][11]+Cq112*S12-Cq312*C12);
  Fq110 = Fq111*C11-Fq211*S11;
  Fq210 = Fq111*S11+Fq211*C11;
  Cq110 = Cq111*C11-Cq211*S11;
  Cq210 = Cq111*S11+Cq211*C11;
  Fq39 = Fq210*S10+Fq311*C10;
  Cq29 = Cq210*C10-Cq311*S10;
  Cq39 = Cq210*S10+Cq311*C10;
  Fq18 = -(s->frc[1][8]-Fq110*C9-Fq39*S9);
  Fq28 = -(s->frc[2][8]-Fq210*C10+Fq311*S10);
  Fq38 = -(s->frc[3][8]+Fq110*S9-Fq39*C9);
  Cq18 = -(s->trq[1][8]-Cq110*C9-Cq39*S9+s->dpt[2][18]*(Fq110*S9-Fq39*C9));
  Cq28 = -(s->trq[2][8]-Cq29);
  Cq38 = -(s->trq[3][8]+Cq110*S9-Cq39*C9+s->dpt[2][18]*(Fq110*C9+Fq39*S9));
  Fs17 = -(s->frc[1][7]-s->m[7]*(qdd[7]+ALPHA16+q[7]*BS16));
  Fs27 = -(s->frc[2][7]-s->m[7]*(ALPHA25+q[7]*BETA46+(2.0)*qd[7]*OM36));
  Fs37 = -(s->frc[3][7]-s->m[7]*(ALPHA36+q[7]*BETA76-(2.0)*qd[7]*OM26));
  Fs16 = -(s->frc[1][6]-s->m[6]*(ALPHA16+BETA36*s->l[3][6]+BS16*s->l[1][6]));
  Fs26 = -(s->frc[2][6]-s->m[6]*(ALPHA25+BETA46*s->l[1][6]+BETA66*s->l[3][6]));
  Fs36 = -(s->frc[3][6]-s->m[6]*(ALPHA36+BETA76*s->l[1][6]+BS96*s->l[3][6]));
  Fq16 = -(s->frc[1][18]+s->frc[1][19]+s->frc[1][20]+s->frc[1][21]-Fq113-Fq128-Fq135-Fq138-Fq18-Fs150-Fs16-Fs17-Fq122*
 C22-Fq141*C41-Fq322*S22-Fq341*S41);
  Fq26 = Fq222+Fq228+Fq241+Fs26+Fs27-s->frc[2][18]*C18-s->frc[2][19]*C19-s->frc[2][20]*C20-s->frc[2][21]*C21+
 s->frc[3][18]*S18+s->frc[3][19]*S19+s->frc[3][20]*S20+s->frc[3][21]*S21+Fq213*C13+Fq235*C35+Fq238*C38+Fq247*C47+Fq28*C8-
 Fq313*S13-Fq335*S35-Fq338*S38-Fq347*S47-Fq38*S8;
  Fq36 = Fq328+Fs36+Fs37-s->frc[2][18]*S18-s->frc[2][19]*S19-s->frc[2][20]*S20-s->frc[2][21]*S21-s->frc[3][18]*C18-
 s->frc[3][19]*C19-s->frc[3][20]*C20-s->frc[3][21]*C21-Fq122*S22-Fq141*S41+Fq213*S13+Fq235*S35+Fq238*S38+Fq247*S47+Fq28*S8+
 Fq313*C13+Fq322*C22+Fq335*C35+Fq338*C38+Fq341*C41+Fq347*C47+Fq38*C8;
  Cq16 = -(s->trq[1][18]+s->trq[1][19]+s->trq[1][20]+s->trq[1][21]+s->trq[1][28]+s->trq[1][6]+s->trq[1][7]-Cq113-Cq129-
 Cq131-Cq135-Cq138-Cq150-Cq18-q[28]*Fq328-s->In[1][6]*OMp16-Cq122*C22-Cq134*C34-Cq141*C41+Cq234*S34-Cq322*S22-Cq341*S41-Dz332
 *Fs334+Fq222*s->dpt[3][8]+Fq228*s->dpt[3][9]+Fq241*s->dpt[3][15]+Fs26*s->l[3][6]+OM26*OM36*(s->In[5][6]-s->In[9][6])-
 s->dpt[2][12]*(Fq235*S35+Fq335*C35)-s->dpt[2][13]*(Fq238*S38+Fq338*C38)-s->dpt[2][17]*(Fq247*S47+Fq347*C47)-s->dpt[2][1]*(
 Fq28*S8+Fq38*C8)-s->dpt[2][2]*(Fq213*S13+Fq313*C13)+s->dpt[2][3]*(s->frc[2][18]*S18+s->frc[3][18]*C18)+s->dpt[2][44]*(
 s->frc[3][30]*C29-Fq229*S29)+s->dpt[2][45]*(s->frc[3][32]*C31-Fq231*S31)+s->dpt[2][4]*(s->frc[2][19]*S19+s->frc[3][19]*C19)+
 s->dpt[2][5]*(s->frc[2][20]*S20+s->frc[3][20]*C20)+s->dpt[2][7]*(s->frc[2][21]*S21+s->frc[3][21]*C21)+s->dpt[3][12]*(Fq235*
 C35-Fq335*S35)+s->dpt[3][13]*(Fq238*C38-Fq338*S38)+s->dpt[3][17]*(Fq247*C47-Fq347*S47)+s->dpt[3][1]*(Fq28*C8-Fq38*S8)+
 s->dpt[3][2]*(Fq213*C13-Fq313*S13));
  Cq26 = -(s->trq[2][28]+s->trq[2][6]+s->trq[2][7]-Cq222-Cq241+q[7]*Fs37-s->In[5][6]*OMp26+s->trq[2][18]*C18+
 s->trq[2][19]*C19+s->trq[2][20]*C20+s->trq[2][21]*C21-s->trq[3][18]*S18-s->trq[3][19]*S19-s->trq[3][20]*S20-s->trq[3][21]*
 S21-s->trq[3][30]*S29-s->trq[3][32]*S31-Cq134*S34-Cq213*C13-Cq229*C29-Cq231*C31-Cq234*C34-Cq235*C35-Cq238*C38-Cq247*C47-Cq28
 *C8+Cq313*S13+Cq335*S35+Cq338*S38+Cq347*S47+Cq38*S8-Fq113*s->dpt[3][2]-Fq128*s->dpt[3][9]-Fq135*s->dpt[3][12]-Fq138*
 s->dpt[3][13]-Fq18*s->dpt[3][1]+Fq328*s->dpt[1][9]-Fs150*s->dpt[3][17]-Fs16*s->l[3][6]+Fs36*s->l[1][6]-OM16*OM36*(
 s->In[1][6]-s->In[9][6])+s->dpt[1][12]*(Fq235*S35+Fq335*C35)+s->dpt[1][13]*(Fq238*S38+Fq338*C38)-s->dpt[1][15]*(Fq141*S41-
 Fq341*C41)+s->dpt[1][17]*(Fq247*S47+Fq347*C47)+s->dpt[1][1]*(Fq28*S8+Fq38*C8)+s->dpt[1][2]*(Fq213*S13+Fq313*C13)-
 s->dpt[1][5]*(s->frc[2][20]*S20+s->frc[3][20]*C20)-s->dpt[1][7]*(s->frc[2][21]*S21+s->frc[3][21]*C21)-s->dpt[1][8]*(Fq122*
 S22-Fq322*C22)-s->dpt[3][15]*(Fq141*C41+Fq341*S41)-s->dpt[3][8]*(Fq122*C22+Fq322*S22));
  Cq36 = -(s->trq[3][28]+s->trq[3][6]+s->trq[3][7]-Cq334+q[28]*Fq128-q[7]*Fs27-s->In[9][6]*OMp36-s->frc[1][18]*
 s->dpt[2][3]-s->frc[1][19]*s->dpt[2][4]-s->frc[1][20]*s->dpt[2][5]-s->frc[1][21]*s->dpt[2][7]+s->trq[2][18]*S18+
 s->trq[2][19]*S19+s->trq[2][20]*S20+s->trq[2][21]*S21+s->trq[3][18]*C18+s->trq[3][19]*C19+s->trq[3][20]*C20+s->trq[3][21]*
 C21+s->trq[3][30]*C29+s->trq[3][32]*C31+Cq122*S22+Cq141*S41-Cq213*S13-Cq229*S29-Cq231*S31-Cq235*S35-Cq238*S38-Cq247*S47-Cq28
 *S8-Cq313*C13-Cq322*C22-Cq335*C35-Cq338*C38-Cq341*C41-Cq347*C47-Cq38*C8+Dz332*Fq133+Fq113*s->dpt[2][2]+Fq129*s->dpt[2][44]+
 Fq131*s->dpt[2][45]+Fq135*s->dpt[2][12]+Fq138*s->dpt[2][13]+Fq18*s->dpt[2][1]-Fq222*s->dpt[1][8]-Fq228*s->dpt[1][9]-Fq241*
 s->dpt[1][15]+Fs150*s->dpt[2][17]-Fs26*s->l[1][6]+OM16*OM26*(s->In[1][6]-s->In[5][6])-s->dpt[1][12]*(Fq235*C35-Fq335*S35)-
 s->dpt[1][13]*(Fq238*C38-Fq338*S38)-s->dpt[1][17]*(Fq247*C47-Fq347*S47)-s->dpt[1][1]*(Fq28*C8-Fq38*S8)-s->dpt[1][2]*(Fq213*
 C13-Fq313*S13)+s->dpt[1][5]*(s->frc[2][20]*C20-s->frc[3][20]*S20)+s->dpt[1][7]*(s->frc[2][21]*C21-s->frc[3][21]*S21));
  Fq15 = Fq16*C6+Fq36*S6;
  Fq35 = -(Fq16*S6-Fq36*C6);
  Cq15 = Cq16*C6+Cq36*S6;
  Fq24 = Fq26*C5-Fq35*S5;
  Fq34 = Fq26*S5+Fq35*C5;
  Cq34 = Cq26*S5-C5*(Cq16*S6-Cq36*C6);
  Fq13 = Fq15*C4-Fq24*S4;
  Fq23 = Fq15*S4+Fq24*C4;

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  Qq[1] = Fq13;
  Qq[2] = Fq23;
  Qq[3] = Fq34;
  Qq[4] = Cq34;
  Qq[5] = Cq15;
  Qq[6] = Cq26;
  Qq[7] = Fs17;
  Qq[8] = Cq18;
  Qq[9] = Cq29;
  Qq[10] = Cq110;
  Qq[11] = Cq311;
  Qq[12] = Cq212;
  Qq[13] = Cq113;
  Qq[14] = Cq214;
  Qq[15] = Cq115;
  Qq[16] = Cq316;
  Qq[17] = Cq217;
  Qq[18] = -s->trq[1][18];
  Qq[19] = -s->trq[1][19];
  Qq[20] = -s->trq[1][20];
  Qq[21] = -s->trq[1][21];
  Qq[22] = Cq222;
  Qq[23] = Cq123;
  Qq[24] = -s->trq[2][24];
  Qq[25] = Cq225;
  Qq[26] = Cq126;
  Qq[27] = -s->trq[2][27];
  Qq[28] = Fq228;
  Qq[29] = Cq129;
  Qq[30] = -s->trq[3][30];
  Qq[31] = Cq131;
  Qq[32] = -s->trq[3][32];
  Qq[33] = Fq233;
  Qq[34] = Cq334;
  Qq[35] = Cq135;
  Qq[36] = Cq136;
  Qq[37] = Cq237;
  Qq[38] = Cq138;
  Qq[39] = Cq139;
  Qq[40] = Cq240;
  Qq[41] = Cq241;
  Qq[42] = Cq242;
  Qq[43] = Cq143;
  Qq[44] = -s->trq[2][44];
  Qq[45] = Cq145;
  Qq[46] = -s->trq[2][46];
  Qq[47] = Cq150;
  Qq[48] = Cq150;
  Qq[49] = Cq150;
  Qq[50] = Cq150;

// ====== END Task 0 ====== 


}
 

