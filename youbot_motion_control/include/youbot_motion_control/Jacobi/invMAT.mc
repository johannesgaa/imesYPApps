#include <math.h>
void invA(double A[5][5],double Ainv[5][5])
{
  double t1;
  double t10;
  double t100;
  double t1000;
  double t1006;
  double t1008;
  double t101;
  double t1010;
  double t1012;
  double t1018;
  double t103;
  double t1032;
  double t1039;
  double t1041;
  double t1043;
  double t1047;
  double t105;
  double t1051;
  double t106;
  double t1065;
  double t107;
  double t109;
  double t111;
  double t112;
  double t114;
  double t115;
  double t117;
  double t119;
  double t12;
  double t121;
  double t122;
  double t124;
  double t126;
  double t128;
  double t13;
  double t130;
  double t132;
  double t134;
  double t135;
  double t136;
  double t14;
  double t143;
  double t145;
  double t146;
  double t147;
  double t148;
  double t15;
  double t150;
  double t151;
  double t153;
  double t154;
  double t155;
  double t157;
  double t158;
  double t160;
  double t161;
  double t163;
  double t165;
  double t166;
  double t167;
  double t169;
  double t17;
  double t171;
  double t172;
  double t174;
  double t176;
  double t178;
  double t18;
  double t180;
  double t181;
  double t183;
  double t185;
  double t187;
  double t188;
  double t190;
  double t192;
  double t194;
  double t195;
  double t2;
  double t20;
  double t202;
  double t209;
  double t21;
  double t212;
  double t213;
  double t214;
  double t216;
  double t218;
  double t219;
  double t22;
  double t221;
  double t223;
  double t225;
  double t227;
  double t228;
  double t230;
  double t232;
  double t234;
  double t236;
  double t238;
  double t24;
  double t240;
  double t241;
  double t245;
  double t249;
  double t256;
  double t26;
  double t263;
  double t265;
  double t266;
  double t268;
  double t27;
  double t270;
  double t272;
  double t274;
  double t276;
  double t278;
  double t279;
  double t28;
  double t286;
  double t290;
  double t294;
  double t3;
  double t301;
  double t308;
  double t31;
  double t312;
  double t313;
  double t316;
  double t319;
  double t32;
  double t322;
  double t325;
  double t328;
  double t33;
  double t331;
  double t332;
  double t335;
  double t338;
  double t341;
  double t344;
  double t347;
  double t35;
  double t350;
  double t36;
  double t365;
  double t378;
  double t38;
  double t380;
  double t382;
  double t384;
  double t386;
  double t388;
  double t39;
  double t390;
  double t394;
  double t396;
  double t398;
  double t4;
  double t400;
  double t402;
  double t407;
  double t409;
  double t41;
  double t417;
  double t43;
  double t432;
  double t44;
  double t445;
  double t447;
  double t45;
  double t450;
  double t453;
  double t456;
  double t459;
  double t461;
  double t463;
  double t465;
  double t467;
  double t468;
  double t473;
  double t475;
  double t477;
  double t48;
  double t484;
  double t487;
  double t490;
  double t493;
  double t496;
  double t5;
  double t503;
  double t504;
  double t51;
  double t511;
  double t518;
  double t52;
  double t532;
  double t54;
  double t545;
  double t556;
  double t558;
  double t56;
  double t560;
  double t562;
  double t564;
  double t569;
  double t57;
  double t571;
  double t579;
  double t593;
  double t6;
  double t60;
  double t606;
  double t609;
  double t616;
  double t623;
  double t628;
  double t63;
  double t630;
  double t638;
  double t640;
  double t647;
  double t654;
  double t66;
  double t667;
  double t68;
  double t682;
  double t69;
  double t695;
  double t70;
  double t709;
  double t71;
  double t714;
  double t716;
  double t724;
  double t73;
  double t739;
  double t74;
  double t752;
  double t76;
  double t766;
  double t77;
  double t779;
  double t79;
  double t794;
  double t8;
  double t80;
  double t807;
  double t82;
  double t821;
  double t83;
  double t834;
  double t839;
  double t841;
  double t843;
  double t845;
  double t847;
  double t849;
  double t85;
  double t851;
  double t853;
  double t857;
  double t86;
  double t864;
  double t867;
  double t870;
  double t872;
  double t874;
  double t876;
  double t88;
  double t880;
  double t882;
  double t884;
  double t886;
  double t888;
  double t89;
  double t890;
  double t892;
  double t894;
  double t896;
  double t899;
  double t9;
  double t90;
  double t900;
  double t902;
  double t912;
  double t915;
  double t917;
  double t92;
  double t93;
  double t932;
  double t945;
  double t95;
  double t959;
  double t96;
  double t972;
  double t98;
  double t987;
  {
    t1 = A[1][1];
    t2 = A[2][2];
    t3 = t1*t2;
    t4 = A[3][3];
    t5 = A[4][4];
    t6 = t4*t5;
    t8 = A[3][4];
    t9 = A[4][3];
    t10 = t8*t9;
    t12 = A[2][3];
    t13 = t1*t12;
    t14 = A[3][2];
    t15 = t14*t5;
    t17 = A[4][2];
    t18 = t8*t17;
    t20 = A[2][4];
    t21 = t1*t20;
    t22 = t14*t9;
    t24 = t4*t17;
    t26 = A[1][2];
    t27 = A[2][1];
    t28 = t26*t27;
    t31 = t26*t12;
    t32 = A[3][1];
    t33 = t32*t5;
    t35 = A[4][1];
    t36 = t8*t35;
    t38 = t26*t20;
    t39 = t32*t9;
    t41 = t4*t35;
    t43 = t3*t6-t3*t10-t13*t15+t13*t18+t21*t22-t21*t24-t28*t6+t28*t10+t31*t33-
t31*t36-t38*t39+t38*t41;
    t44 = A[1][3];
    t45 = t44*t27;
    t48 = t44*t2;
    t51 = t44*t20;
    t52 = t32*t17;
    t54 = t14*t35;
    t56 = A[1][4];
    t57 = t56*t27;
    t60 = t56*t2;
    t63 = t56*t12;
    t66 = t45*t15-t45*t18-t48*t33+t48*t36+t51*t52-t51*t54-t57*t22+t57*t24+t60*
t39-t60*t41-t63*t52+t63*t54;
    t68 = A[0][0];
    t69 = t68*t1;
    t70 = t2*t4;
    t71 = t70*t5;
    t73 = t2*t8;
    t74 = t73*t9;
    t76 = t12*t14;
    t77 = t76*t5;
    t79 = t12*t8;
    t80 = t79*t17;
    t82 = t20*t14;
    t83 = t82*t9;
    t85 = t20*t4;
    t86 = t85*t17;
    t88 = t68*t26;
    t89 = t27*t4;
    t90 = t89*t5;
    t92 = t27*t8;
    t93 = t92*t9;
    t95 = t12*t32;
    t96 = t95*t5;
    t98 = t79*t35;
    t100 = t20*t32;
    t101 = t100*t9;
    t103 = t85*t35;
    t105 = t68*t44;
    t106 = t27*t14;
    t107 = t106*t5;
    t109 = t92*t17;
    t111 = t2*t32;
    t112 = t111*t5;
    t114 = t69*t71-t69*t74-t69*t77+t69*t80+t69*t83-t69*t86-t88*t90+t88*t93+t88*
t96-t88*t98-t88*t101+t88*t103+t105*t107-t105*t109-t105*t112;
    t115 = t73*t35;
    t117 = t100*t17;
    t119 = t82*t35;
    t121 = t68*t56;
    t122 = t106*t9;
    t124 = t89*t17;
    t126 = t111*t9;
    t128 = t70*t35;
    t130 = t95*t17;
    t132 = t76*t35;
    t134 = A[0][1];
    t135 = A[1][0];
    t136 = t134*t135;
    t143 = t105*t115+t105*t117-t105*t119-t121*t122+t121*t124+t121*t126-t121*
t128-t121*t130+t121*t132-t136*t71+t136*t74+t136*t77-t136*t80-t136*t83+t136*t86;
    t145 = t134*t26;
    t146 = A[2][0];
    t147 = t146*t4;
    t148 = t147*t5;
    t150 = t146*t8;
    t151 = t150*t9;
    t153 = A[3][0];
    t154 = t12*t153;
    t155 = t154*t5;
    t157 = A[4][0];
    t158 = t79*t157;
    t160 = t20*t153;
    t161 = t160*t9;
    t163 = t85*t157;
    t165 = t134*t44;
    t166 = t146*t14;
    t167 = t166*t5;
    t169 = t150*t17;
    t171 = t2*t153;
    t172 = t171*t5;
    t174 = t73*t157;
    t176 = t160*t17;
    t178 = t82*t157;
    t180 = t134*t56;
    t181 = t166*t9;
    t183 = t147*t17;
    t185 = t171*t9;
    t187 = t145*t148-t145*t151-t145*t155+t145*t158+t145*t161-t145*t163-t165*
t167+t165*t169+t165*t172-t165*t174-t165*t176+t165*t178+t180*t181-t180*t183-t180
*t185;
    t188 = t70*t157;
    t190 = t154*t17;
    t192 = t76*t157;
    t194 = A[0][2];
    t195 = t194*t135;
    t202 = t194*t1;
    t209 = t180*t188+t180*t190-t180*t192+t195*t90-t195*t93-t195*t96+t195*t98+
t195*t101-t195*t103-t202*t148+t202*t151+t202*t155-t202*t158-t202*t161+t202*t163
;
    t212 = t194*t44;
    t213 = t146*t32;
    t214 = t213*t5;
    t216 = t150*t35;
    t218 = t27*t153;
    t219 = t218*t5;
    t221 = t92*t157;
    t223 = t160*t35;
    t225 = t100*t157;
    t227 = t194*t56;
    t228 = t213*t9;
    t230 = t147*t35;
    t232 = t218*t9;
    t234 = t89*t157;
    t236 = t154*t35;
    t238 = t95*t157;
    t240 = A[0][3];
    t241 = t240*t135;
    t245 = t212*t214-t212*t216-t212*t219+t212*t221+t212*t223-t212*t225-t227*
t228+t227*t230+t227*t232-t227*t234-t227*t236+t227*t238-t241*t107+t241*t109+t241
*t112;
    t249 = t240*t1;
    t256 = t240*t26;
    t263 = -t241*t115-t241*t117+t241*t119+t249*t167-t249*t169-t249*t172+t249*
t174+t249*t176-t249*t178-t256*t214+t256*t216+t256*t219-t256*t221-t256*t223+t256
*t225;
    t265 = t240*t56;
    t266 = t213*t17;
    t268 = t166*t35;
    t270 = t218*t17;
    t272 = t106*t157;
    t274 = t171*t35;
    t276 = t111*t157;
    t278 = A[0][4];
    t279 = t278*t135;
    t286 = t278*t1;
    t290 = t265*t266-t265*t268-t265*t270+t265*t272+t265*t274-t265*t276+t279*
t122-t279*t124-t279*t126+t279*t128+t279*t130-t279*t132-t286*t181+t286*t183+t286
*t185;
    t294 = t278*t26;
    t301 = t278*t44;
    t308 = -t286*t188-t286*t190+t286*t192+t294*t228-t294*t230-t294*t232+t294*
t234+t294*t236-t294*t238-t301*t266+t301*t268+t301*t270-t301*t272-t301*t274+t301
*t276;
    t312 = 1/(t114+t143+t187+t209+t245+t263+t290+t308);
    Ainv[0][0] = (t43+t66)*t312;
    t313 = t134*t2;
    t316 = t134*t12;
    t319 = t134*t20;
    t322 = t194*t27;
    t325 = t194*t12;
    t328 = t194*t20;
    t331 = t313*t6-t313*t10-t316*t15+t316*t18+t319*t22-t319*t24-t322*t6+t322*
t10+t325*t33-t325*t36-t328*t39+t328*t41;
    t332 = t240*t27;
    t335 = t240*t2;
    t338 = t240*t20;
    t341 = t278*t27;
    t344 = t278*t2;
    t347 = t278*t12;
    t350 = t332*t15-t332*t18-t335*t33+t335*t36+t338*t52-t338*t54-t341*t22+t341*
t24+t344*t39-t344*t41-t347*t52+t347*t54;
    Ainv[0][1] = -(t331+t350)*t312;
    t365 = t145*t6-t145*t10-t165*t15+t165*t18+t180*t22-t180*t24-t202*t6+t202*
t10+t212*t33-t212*t36-t227*t39+t227*t41;
    t378 = t249*t15-t249*t18-t256*t33+t256*t36+t265*t52-t265*t54-t286*t22+t286*
t24+t294*t39-t294*t41-t301*t52+t301*t54;
    Ainv[0][2] = (t365+t378)*t312;
    t380 = t12*t5;
    t382 = t20*t9;
    t384 = t2*t5;
    t386 = t20*t17;
    t388 = t2*t9;
    t390 = t12*t17;
    t394 = t27*t5;
    t396 = t20*t35;
    t398 = t27*t9;
    t400 = t12*t35;
    t402 = t145*t380-t145*t382-t165*t384+t165*t386+t180*t388-t180*t390-t202*
t380+t202*t382+t212*t394-t212*t396-t227*t398+t227*t400;
    t407 = t27*t17;
    t409 = t2*t35;
    t417 = t249*t384-t249*t386-t256*t394+t256*t396+t265*t407-t265*t409-t286*
t388+t286*t390+t294*t398-t294*t400-t301*t407+t301*t409;
    Ainv[0][3] = -(t402+t417)*t312;
    t432 = t145*t79-t145*t85-t165*t73+t165*t82+t180*t70-t180*t76-t202*t79+t202*
t85+t212*t92-t212*t100-t227*t89+t227*t95;
    t445 = t249*t73-t249*t82-t256*t92+t256*t100+t265*t106-t265*t111-t286*t70+
t286*t76+t294*t89-t294*t95-t301*t106+t301*t111;
    Ainv[0][4] = (t432+t445)*t312;
    t447 = t135*t2;
    t450 = t135*t12;
    t453 = t135*t20;
    t456 = t26*t146;
    t459 = t153*t5;
    t461 = t8*t157;
    t463 = t153*t9;
    t465 = t4*t157;
    t467 = t447*t6-t447*t10-t450*t15+t450*t18+t453*t22-t453*t24-t456*t6+t456*
t10+t31*t459-t31*t461-t38*t463+t38*t465;
    t468 = t44*t146;
    t473 = t153*t17;
    t475 = t14*t157;
    t477 = t56*t146;
    t484 = t468*t15-t468*t18-t48*t459+t48*t461+t51*t473-t51*t475-t477*t22+t477*
t24+t60*t463-t60*t465-t63*t473+t63*t475;
    Ainv[1][0] = -(t467+t484)*t312;
    t487 = t68*t2;
    t490 = t68*t12;
    t493 = t68*t20;
    t496 = t194*t146;
    t503 = t487*t6-t487*t10-t490*t15+t490*t18+t493*t22-t493*t24-t496*t6+t496*
t10+t325*t459-t325*t461-t328*t463+t328*t465;
    t504 = t240*t146;
    t511 = t278*t146;
    t518 = t504*t15-t504*t18-t335*t459+t335*t461+t338*t473-t338*t475-t511*t22+
t511*t24+t344*t463-t344*t465-t347*t473+t347*t475;
    Ainv[1][1] = (t503+t518)*t312;
    t532 = t88*t6-t88*t10-t105*t15+t105*t18+t121*t22-t121*t24-t195*t6+t195*t10+
t212*t459-t212*t461-t227*t463+t227*t465;
    t545 = t241*t15-t241*t18-t256*t459+t256*t461+t265*t473-t265*t475-t279*t22+
t279*t24+t294*t463-t294*t465-t301*t473+t301*t475;
    Ainv[1][2] = -(t532+t545)*t312;
    t556 = t146*t5;
    t558 = t20*t157;
    t560 = t146*t9;
    t562 = t12*t157;
    t564 = t88*t380-t88*t382-t105*t384+t105*t386+t121*t388-t121*t390-t195*t380+
t195*t382+t212*t556-t212*t558-t227*t560+t227*t562;
    t569 = t146*t17;
    t571 = t2*t157;
    t579 = t241*t384-t241*t386-t256*t556+t256*t558+t265*t569-t265*t571-t279*
t388+t279*t390+t294*t560-t294*t562-t301*t569+t301*t571;
    Ainv[1][3] = (t564+t579)*t312;
    t593 = t88*t79-t88*t85-t105*t73+t105*t82+t121*t70-t121*t76-t195*t79+t195*
t85+t212*t150-t212*t160-t227*t147+t227*t154;
    t606 = t241*t73-t241*t82-t256*t150+t256*t160+t265*t166-t265*t171-t279*t70+
t279*t76+t294*t147-t294*t154-t301*t166+t301*t171;
    Ainv[1][4] = -(t593+t606)*t312;
    t609 = t135*t27;
    t616 = t1*t146;
    t623 = t609*t6-t609*t10-t450*t33+t450*t36+t453*t39-t453*t41-t616*t6+t616*
t10+t13*t459-t13*t461-t21*t463+t21*t465;
    t628 = t153*t35;
    t630 = t32*t157;
    t638 = t468*t33-t468*t36-t45*t459+t45*t461+t51*t628-t51*t630-t477*t39+t477*
t41+t57*t463-t57*t465-t63*t628+t63*t630;
    Ainv[2][0] = (t623+t638)*t312;
    t640 = t68*t27;
    t647 = t134*t146;
    t654 = t640*t6-t640*t10-t490*t33+t490*t36+t493*t39-t493*t41-t647*t6+t647*
t10+t316*t459-t316*t461-t319*t463+t319*t465;
    t667 = t504*t33-t504*t36-t332*t459+t332*t461+t338*t628-t338*t630-t511*t39+
t511*t41+t341*t463-t341*t465-t347*t628+t347*t630;
    Ainv[2][1] = -(t654+t667)*t312;
    t682 = t69*t6-t69*t10-t105*t33+t105*t36+t121*t39-t121*t41-t136*t6+t136*t10+
t165*t459-t165*t461-t180*t463+t180*t465;
    t695 = t241*t33-t241*t36-t249*t459+t249*t461+t265*t628-t265*t630-t279*t39+
t279*t41+t286*t463-t286*t465-t301*t628+t301*t630;
    Ainv[2][2] = (t682+t695)*t312;
    t709 = t69*t380-t69*t382-t105*t394+t105*t396+t121*t398-t121*t400-t136*t380+
t136*t382+t165*t556-t165*t558-t180*t560+t180*t562;
    t714 = t146*t35;
    t716 = t27*t157;
    t724 = t241*t394-t241*t396-t249*t556+t249*t558+t265*t714-t265*t716-t279*
t398+t279*t400+t286*t560-t286*t562-t301*t714+t301*t716;
    Ainv[2][3] = -(t709+t724)*t312;
    t739 = t69*t79-t69*t85-t105*t92+t105*t100+t121*t89-t121*t95-t136*t79+t136*
t85+t165*t150-t165*t160-t180*t147+t180*t154;
    t752 = t241*t92-t241*t100-t249*t150+t249*t160+t265*t213-t265*t218-t279*t89+
t279*t95+t286*t147-t286*t154-t301*t213+t301*t218;
    Ainv[2][4] = (t739+t752)*t312;
    t766 = t609*t15-t609*t18-t447*t33+t447*t36+t453*t52-t453*t54-t616*t15+t616*
t18+t3*t459-t3*t461-t21*t473+t21*t475;
    t779 = t456*t33-t456*t36-t28*t459+t28*t461+t38*t628-t38*t630-t477*t52+t477*
t54+t57*t473-t57*t475-t60*t628+t60*t630;
    Ainv[3][0] = -(t766+t779)*t312;
    t794 = t640*t15-t640*t18-t487*t33+t487*t36+t493*t52-t493*t54-t647*t15+t647*
t18+t313*t459-t313*t461-t319*t473+t319*t475;
    t807 = t496*t33-t496*t36-t322*t459+t322*t461+t328*t628-t328*t630-t511*t52+
t511*t54+t341*t473-t341*t475-t344*t628+t344*t630;
    Ainv[3][1] = (t794+t807)*t312;
    t821 = t69*t15-t69*t18-t88*t33+t88*t36+t121*t52-t121*t54-t136*t15+t136*t18+
t145*t459-t145*t461-t180*t473+t180*t475;
    t834 = t195*t33-t195*t36-t202*t459+t202*t461+t227*t628-t227*t630-t279*t52+
t279*t54+t286*t473-t286*t475-t294*t628+t294*t630;
    Ainv[3][2] = -(t821+t834)*t312;
    t839 = t68*t35;
    t841 = t68*t17;
    t843 = t35*t56;
    t845 = t26*t5;
    t847 = t157*t278;
    t849 = t194*t5;
    t851 = t1*t17;
    t853 = t1*t157;
    t857 = t69*t384-t69*t386+t839*t38+t841*t57-t487*t843-t640*t845-t3*t847-t616
*t849+t851*t511+t853*t328+t28*t847+t409*t279;
    t864 = t35*t26;
    t867 = t17*t135;
    t870 = t35*t135;
    t872 = t157*t194;
    t874 = t157*t134;
    t876 = -t447*t134*t5+t571*t180+t609*t849+t496*t843+t647*t845-t864*t511-t407
*t279+t867*t319-t569*t180-t870*t328-t872*t57-t874*t38;
    Ainv[3][3] = (t857+t876)*t312;
    t880 = t68*t32;
    t882 = t68*t14;
    t884 = t26*t8;
    t886 = t32*t56;
    t888 = t194*t8;
    t890 = t1*t14;
    t892 = t1*t153;
    t894 = t153*t278;
    t896 = t14*t135;
    t899 = t69*t73-t69*t82+t880*t38+t882*t57-t640*t884-t487*t886-t616*t888+t890
*t511+t892*t328-t3*t894+t896*t319-t166*t180;
    t900 = t32*t135;
    t902 = t153*t194;
    t912 = t32*t26;
    t915 = t153*t134;
    t917 = -t900*t328-t902*t57+t28*t894+t111*t279-t447*t134*t8+t171*t180+t609*
t888+t496*t886+t647*t884-t912*t511-t106*t279-t915*t38;
    Ainv[3][4] = -(t899+t917)*t312;
    t932 = t609*t22-t609*t24-t447*t39+t447*t41+t450*t52-t450*t54-t616*t22+t616*
t24+t3*t463-t3*t465-t13*t473+t13*t475;
    t945 = t456*t39-t456*t41-t28*t463+t28*t465+t31*t628-t31*t630-t468*t52+t468*
t54+t45*t473-t45*t475-t48*t628+t48*t630;
    Ainv[4][0] = (t932+t945)*t312;
    t959 = t640*t22-t640*t24-t487*t39+t487*t41+t490*t52-t490*t54-t647*t22+t647*
t24+t313*t463-t313*t465-t316*t473+t316*t475;
    t972 = t496*t39-t496*t41-t322*t463+t322*t465+t325*t628-t325*t630-t504*t52+
t504*t54+t332*t473-t332*t475-t335*t628+t335*t630;
    Ainv[4][1] = -(t959+t972)*t312;
    t987 = t69*t22-t69*t24-t88*t39+t88*t41+t105*t52-t105*t54-t136*t22+t136*t24+
t145*t463-t145*t465-t165*t473+t165*t475;
    t1000 = t195*t39-t195*t41-t202*t463+t202*t465+t212*t628-t212*t630-t241*t52+
t241*t54+t249*t473-t249*t475-t256*t628+t256*t630;
    Ainv[4][2] = (t987+t1000)*t312;
    t1006 = t35*t44;
    t1008 = t26*t9;
    t1010 = t157*t240;
    t1012 = t194*t9;
    t1018 = t69*t388-t69*t390+t839*t31+t841*t45-t487*t1006-t640*t1008-t3*t1010-
t616*t1012+t851*t504+t853*t325+t28*t1010+t409*t241;
    t1032 = -t447*t134*t9+t571*t165+t609*t1012+t496*t1006+t647*t1008-t864*t504-
t407*t241+t867*t316-t569*t165-t870*t325-t872*t45-t874*t31;
    Ainv[4][3] = -(t1018+t1032)*t312;
    t1039 = t26*t4;
    t1041 = t32*t44;
    t1043 = t194*t4;
    t1047 = t153*t240;
    t1051 = t69*t70-t69*t76+t880*t31+t882*t45-t640*t1039-t487*t1041-t616*t1043+
t890*t504+t892*t325-t3*t1047+t896*t316-t166*t165;
    t1065 = -t900*t325-t902*t45+t28*t1047+t111*t241-t447*t134*t4+t171*t165+t609
*t1043+t496*t1041+t647*t1039-t912*t504-t106*t241-t915*t31;
    Ainv[4][4] = (t1051+t1065)*t312;
    return;
  }
}

