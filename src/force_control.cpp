#include "psm_coop/force_control.h"



void PsmForceControl::CalcJaM(const Eigen::VectorXd &q,const Eigen::VectorXd &qd)
{
  
    // Jacobian from matlab dynamic
    /*float t2 = q2-2.908373974667121E-1;
    float t3 = cos(t2);
    float t4 = 3.141592653589793*(1.0/2.0);
    float t5 = -q2+t4+2.908373974667121E-1;
    float t6 = q2-t4;
    float t7 = sin(t5);
    float t8 = cos(t5);
    float t9 = sin(t2);
    float t10 = cos(t6);
    float t11 = t3*t7;
    float t12 = t8*t9;
    float t13 = t11+t12;
    float t14 = t3*t8;
    float t17 = t7*t9;
    float t15 = t14-t17;
    float t16 = sin(t6);
    float t18 = q1-t4;
    float t19 = sin(t18);
    float t20 = t3*t7*t19;
    float t21 = t8*t9*t19;
    float t22 = t20+t21;
    float t23 = t3*t8*t19;
    float t25 = t7*t9*t19;
    float t24 = t23-t25;
    float t26 = q3+4.162E-1;
    float t27 = cos(t18);
    float t28 = t3*t7*t27;
    float t29 = t8*t9*t27;
    float t30 = t28+t29;
    float t31 = t3*t8*t27;
    float t33 = t7*t9*t27;
    float t32 = t31-t33;
    float t34 = t16*t30;
    float t35 = t34-t10*t32;
    float t36 = t10*t24;
    float t37 = t36-t16*t22;

    JaM(0,1) = t3*(3.0/2.0E1)-t10*t13*(4.3E1/1.0E3)+t10*t15*(1.68E2/6.25E2)-t13*t16*(1.68E2/6.25E2)-t15*t16*(4.3E1/1.0E3)-t26*(t10*t15-t13*t16);
    JaM(0,2) = -t10*t13-t15*t16;
    JaM(1,0) = t3*t19*(-3.0/2.0E1)+t10*t22*(4.3E1/1.0E3)-t10*t24*(1.68E2/6.25E2)+t16*t22*(1.68E2/6.25E2)+t16*t24*(4.3E1/1.0E3)+t26*t37-t3*t8*t19*(1.29E2/2.5E2)+t7*t9*t19*(1.29E2/2.5E2);
    JaM(1,1) = t9*t27*(-3.0/2.0E1)-t10*t30*(1.68E2/6.25E2)-t10*t32*(4.3E1/1.0E3)+t16*t30*(4.3E1/1.0E3)-t16*t32*(1.68E2/6.25E2)+t26*(t10*t30+t16*t32);
    JaM(1,2) = t35;
    JaM(2,0) = t3*t27*(-3.0/2.0E1)+t10*t30*(4.3E1/1.0E3)-t10*t32*(1.68E2/6.25E2)+t16*t30*(1.68E2/6.25E2)+t16*t32*(4.3E1/1.0E3)-t26*t35-t3*t8*t27*(1.29E2/2.5E2)+t7*t9*t27*(1.29E2/2.5E2);
    JaM(2,1) = t9*t19*(3.0/2.0E1)+t10*t22*(1.68E2/6.25E2)+t10*t24*(4.3E1/1.0E3)-t16*t22*(4.3E1/1.0E3)+t16*t24*(1.68E2/6.25E2)-t26*(t10*t22+t16*t24);
    JaM(2,2) = t37;
*/
    float t2 = 3.141592653589793*(1.0/2.0);
    float t3 = q2+t2;
    float t4 = sin(t3);
    float t5 = q3+3.7E-3;
    float t6 = q1-t2;
    float t7 = cos(t3);
    float t8 = cos(t6);
    float t9 = sin(t6);

    JaM(0,1) = -t4*t5;
    JaM(0,2) = t7;
    JaM(1,0) = t4*t5*t9;
    JaM(1,1) = -t5*t7*t8;
    JaM(1,2) = -t4*t8;
    JaM(2,0) = t4*t5*t8;
    JaM(2,1) = t5*t7*t9;
    JaM(2,2) = t4*t9;

    JaM = JaM*St;

}

void PsmForceControl::CalcJaInv(const Eigen::VectorXd &q,const Eigen::VectorXd &qd){

  // Kinematic 
  float t4 = 3.141592653589793*(1.0/2.0);
  float t2 = q1-t4;
  float t3 = cos(t2);
  float t5 = sin(t2);
  float t6 = q2+t4;
  float t7 = sin(t6);
  float t8 = t3*t3;
  float t9 = t5*t5;
  float t10 = t7*t8*3.7E1;
  float t11 = t7*t9*3.7E1;
  float t12 = q3*t7*t8*1.0E4;
  float t13 = q3*t7*t9*1.0E4;
  float t14 = t10+t11+t12+t13;
  float t15 = 1.0/t14;
  float t16 = cos(t6);
  float t17 = t16*t16;
  float t18 = t7*t7;
  float t19 = t9*t18*3.7E1;
  float t20 = t8*t17*3.7E1;
  float t21 = t8*t18*3.7E1;
  float t22 = t9*t17*3.7E1;
  float t23 = q3*t8*t17*1.0E4;
  float t24 = q3*t8*t18*1.0E4;
  float t25 = q3*t9*t17*1.0E4;
  float t26 = q3*t9*t18*1.0E4;
  float t27 = t19+t20+t21+t22+t23+t24+t25+t26;
  float t28 = 1.0/t27;
  float t29 = t9*t18;
  float t30 = t8*t17;
  float t31 = t8*t18;
  float t32 = t9*t17;
  float t33 = t29+t30+t31+t32;
  float t34 = 1.0/t33;
  
  JaInv(0,1) = t5*t15*1.0E4;
  JaInv(0,2) = t3*t15*1.0E4;
  JaInv(1,0) = (t7*-1.0E4)/(t17*3.7E1+t18*3.7E1+q3*t17*1.0E4+q3*t18*1.0E4);
  JaInv(1,1) = t3*t16*t28*-1.0E4;
  JaInv(1,2) = t5*t16*t28*1.0E4;
  JaInv(2,0) = t16/(t17+t18);
  JaInv(2,1) = -t3*t7*t34;
  JaInv(2,2) = t5*t7*t34;
}

void PsmForceControl::CalcN(const Eigen::VectorXd &q,const Eigen::VectorXd &qd) {
    if (name == "PSM1") {
 
        // Try without KE fourier test 2
       /*   float t2 = q1-q2;
          float t3 = q1+q2-2.908373974667121E-1;
          float t4 = q1-q2+2.908373974667121E-1;
          float t5 = q1+q2;
          float t6 = qd2*qd2;
          float t7 = sin(t4);
          float t8 = sin(t5);
          float t9 = sin(t2);
          float t10 = sin(t3);
          float t11 = sin(q2);
          float t12 = q2-2.908373974667121E-1;
          float t13 = cos(t12);
          float t14 = q2*2.0;
          float t15 = t14-5.816747949334241E-1;
          float t16 = cos(t15);
          float t17 = t14-2.908373974667121E-1;
          float t18 = cos(t17);
          float t19 = sin(t15);
          float t20 = cos(q2);
          float t21 = cos(t14);
          float t22 = sin(t14);
          float t23 = q3*q3;
          float t24 = sin(t17);
          float t25 = cos(t2);
          float t26 = cos(t3);
          float t27 = t10*3.017526851309462;
          float t28 = cos(t4);
          float t29 = t28*7.320145176142429E-1;
          float t30 = t7*3.017526851309462;
          float t31 = cos(t5);
          float t32 = t31*1.499393709809464;
          float t33 = qd1*qd1;
          float t34 = q3*t8*2.448643206032768;
          float t35 = q3*t9*2.448643206032768;
          float t36 = q2-5.816747949334241E-1;
          float t37 = sin(t36);
          float t38 = cos(2.908373974667121E-1);
          
          N(0) =  q1*6.119063107247842E-1-t8*3.344751175535797-t9*3.344751175535797-t25*1.499393709809464-t26*7.320145176142429E-1+t27+t29+t30+t32+t34+t35-sin(q1-2.908373974667121E-1)*3.512986425818344E-1-sin(q1+2.908373974667121E-1)*3.512986425818344E-1-cos(q1)*4.704437037389851E-2+sin(q1)*1.185584490767175-qd1*qd3*2.69507168036979E-1-q3*t7*2.34295976006465-q3*t10*2.34295976006465-t6*t11*1.366589184398208E-4+t6*t13*1.261218951123317E-1-t6*t20*1.308102487121939E-1+qd2*qd3*sin(t12)*2.064848982586517E-1+q3*qd1*qd3*1.575623045498661-qd1*qd2*t11*2.150808015807149E-2-qd2*qd3*t11*6.356228764642827E-3+qd1*qd2*t16*1.13991403679422-qd1*qd3*t16*8.323902276159846E-1+qd1*qd2*t18*1.835992297725874E-1+qd1*qd2*t19*1.197266709321526-qd1*qd3*t18*1.49916930981598E-1-qd1*qd3*t19*1.800158150459516E-1-qd1*qd2*t21*3.934770614322237E-1-qd1*qd2*t22*2.03284026406765+qd1*qd3*t21*5.628830595790056E-1+qd1*qd3*t22*8.098713700747587E-1+qd1*qd2*t24*1.688649178737017E-1-qd1*qd2*t37*2.150808015807149E-2-qd1*qd3*t38*1.49916930981598E-1+q3*t6*t13*1.032424491293259E-1-q3*t6*t20*3.178114382321413E-3-q3*qd1*qd2*t16*3.600316300919032E-1+q3*qd1*qd3*t16*5.76176838954674E-1+q3*qd1*qd2*t19*1.664780455231969+q3*qd1*qd2*t21*1.619742740149517-q3*qd1*qd2*t22*1.125766119158011+q3*qd1*qd3*t21*9.994462065439869E-1+q3*qd1*qd2*t24*2.998338619631961E-1-qd1*qd2*t19*t23*5.76176838954674E-1-qd1*qd2*t22*t23*9.994462065439869E-1;
          N(1) = q2*1.178371077512102-t8*3.344751175535797+t9*3.344751175535797+t25*1.499393709809464-t26*7.320145176142429E-1+t27-t29-t30+t32+t34-t35-qd2*qd3*5.390143360739581E-1+q3*t7*2.34295976006465-q3*t10*2.34295976006465+t11*t33*1.075404007903575E-2-t16*t33*5.699570183971099E-1-t18*t33*9.179961488629371E-2-t19*t33*5.986333546607628E-1+t21*t33*1.967385307161119E-1+t22*t33*1.016420132033825-t24*t33*8.443245893685084E-2+t33*t37*1.075404007903575E-2+q3*qd2*qd3*3.151246090997322-qd2*qd3*t38*2.998338619631961E-1+q3*t16*t33*1.800158150459516E-1-q3*t19*t33*8.323902276159846E-1-q3*t21*t33*8.098713700747587E-1+q3*t22*t33*5.628830595790056E-1-q3*t24*t33*1.49916930981598E-1+t19*t23*t33*2.88088419477337E-1+t22*t23*t33*4.997231032719934E-1;
          N(2) = t6*2.69507168036979E-1-t25*2.448643206032768+t26*2.34295976006465+t28*2.34295976006465-t31*2.448643206032768+t33*1.347535840184895E-1-q3*t6*1.575623045498661-q3*t33*7.878115227493304E-1+t6*t38*1.49916930981598E-1+t16*t33*4.161951138079923E-1+t18*t33*7.495846549079901E-2+t19*t33*9.000790752297579E-2-t21*t33*2.814415297895028E-1-t22*t33*4.049356850373793E-1+t33*t38*7.495846549079901E-2-q3*t16*t33*2.88088419477337E-1-q3*t21*t33*4.997231032719934E-1;
*/

          // New MidJuly_3dof with fourier test 3
          float t2 = q1-q2;
          float t3 = q1+q2;
          float t4 = qd2*qd2;
          float t5 = sin(t3);
          float t6 = sin(t2);
          float t7 = sin(q2);
          float t8 = q2*2.0;
          float t9 = t8-2.908373974667121E-1;
          float t10 = cos(t9);
          float t11 = cos(q2);
          float t12 = cos(t8);
          float t13 = sin(t8);
          float t14 = sin(t9);
          float t15 = cos(t2);
          float t16 = q1+q2-2.908373974667121E-1;
          float t17 = sin(t16);
          float t18 = t17*6.120266493943314E-1;
          float t19 = q1-q2+2.908373974667121E-1;
          float t20 = sin(t19);
          float t21 = t20*6.120266493943314E-1;
          float t22 = cos(t3);
          float t23 = t22*2.300749300157408E-2;
          float t24 = qd1*qd1;
          float t25 = t8-5.816747949334241E-1;
          float t26 = sin(t25);
          float t27 = q3*t5*7.174176609412888E-2;
          float t28 = q3*t6*7.174176609412888E-2;
          float t29 = q2-5.816747949334241E-1;
          float t30 = sin(t29);
          float t31 = cos(2.908373974667121E-1);
          float t32 = q3*q3;
          float t33 = cos(q1);
          float t34 = t11*t11;
          N(0,0) = t5*(-4.928652039114835E-1)-t6*4.928652039114835E-1-t15*2.300749300157408E-2+t18+t21+t23+t27+t28-t33*3.690362487729971E-1-sin(q1-2.908373974667121E-1)*7.71908746137314E-1-sin(q1+2.908373974667121E-1)*7.71908746137314E-1+sin(q1)*1.371930099108582-qd1*qd3*2.740042504345347E-1-t4*t7*6.467430257386333E-5-t4*t11*1.26446665596096E-2+t4*cos(q2-2.908373974667121E-1)*1.129702802366318E-2+q3*qd1*qd3*1.354134045331792-qd1*qd2*t7*4.725971915126412E-2-qd2*qd3*t7*1.453340279419851E-1+qd1*qd2*t10*6.035064563941906E-2-qd1*qd3*t10*4.392353026171156E-3+qd1*qd2*t12*3.652485645509734E-2-qd1*qd2*t13*2.235801481782498E-1-qd1*qd3*t12*2.740042504345347E-1-qd1*qd2*t14*6.035084129528369E-2+qd1*qd3*t13*1.962302195253671E-2+qd1*qd2*t26*3.747101935067335E-2-qd1*qd2*t30*4.725971915126412E-2-qd1*qd3*t31*4.392353026171156E-3-q3*t4*t11*7.266701397099254E-2+q3*qd1*qd2*t12*3.924604390507343E-2+q3*qd1*qd2*t13*5.480085008690695E-1+q3*qd1*qd3*t12*1.354134045331792+q3*qd1*qd2*t14*8.784706052342312E-3-qd1*qd2*t13*t32*1.354134045331792;
          N(1,0) = t5*(-4.928652039114835E-1)+t6*4.928652039114835E-1+t15*2.300749300157408E-2+t18-t21+t23+t27-t28-qd2*qd3*5.480085008690695E-1+t7*t24*2.362985957563206E-2-t10*t24*3.017532281970953E-2-t12*t24*1.826242822754867E-2+t13*t24*1.117900740891249E-1+t14*t24*3.017542064764185E-2-t24*t26*1.873550967533668E-2+t24*t30*2.362985957563206E-2+q3*qd2*qd3*2.708268090663583-qd2*qd3*t31*8.784706052342312E-3-q3*t12*t24*1.962302195253671E-2-q3*t13*t24*2.740042504345347E-1-q3*t14*t24*4.392353026171156E-3+t13*t24*t32*6.770670226658958E-1;
          N(2,0) = t4*2.740042504345347E-1-q3*t4*1.354134045331792+t4*t31*4.392353026171156E-3-t13*t24*9.811510976268357E-3-t11*t33*1.434835321882578E-1+t24*t34*2.740042504345347E-1-q3*t24*t34*1.354134045331792+t24*t31*t34*4.392353026171156E-3+t7*t11*t24*sin(2.908373974667121E-1)*4.392353026171156E-3;

    }

    else if (name == "PSM2")
    {

        float t2 = q1-q2;
        float t3 = q1+q2-2.908373974667121E-1;
        float t4 = q1-q2+2.908373974667121E-1;
        float t5 = q1+q2;
        float t6 = qd2*qd2;
        float t7 = sin(t4);
        float t8 = sin(t5);
        float t9 = sin(t2);
        float t10 = sin(t3);
        float t11 = sin(q2);
        float t12 = q2-2.908373974667121E-1;
        float t13 = cos(t12);
        float t14 = q2*2.0;
        float t15 = t14-5.816747949334241E-1;
        float t16 = cos(t15);
        float t17 = t14-2.908373974667121E-1;
        float t18 = cos(t17);
        float t19 = sin(t15);
        float t20 = cos(q2);
        float t21 = cos(t14);
        float t22 = sin(t14);
        float t23 = q3*q3;
        float t24 = sin(t17);
        float t25 = cos(t2);
        float t26 = t25*3.230561622574884E-1;
        float t27 = t9*6.087527876985603E-1;
        float t28 = cos(t3);
        float t29 = t28*1.650701241431906E-1;
        float t30 = cos(t4);
        float t31 = cos(t5);
        float t32 = t8*6.087527876985603E-1;
        float t33 = qd1*qd1;
        float t34 = q3*t8*9.264907881420566E-1;
        float t35 = q3*t9*9.264907881420566E-1;
        float t36 = q2-5.816747949334241E-1;
        float t37 = sin(t36);
        float t38 = cos(2.908373974667121E-1);
        N(0) = t7*(-6.152471329736313E-1)-t10*6.152471329736313E-1+t26+t27+t29-t30*1.650701241431906E-1-t31*3.230561622574884E-1+t32+t34+t35+sin(q1-2.908373974667121E-1)*4.233989932885032E-1+sin(q1+2.908373974667121E-1)*4.233989932885032E-1+cos(q1)*1.365447828277939E-1-sin(q1)*5.771205349044905E-1-qd1*qd3*1.430207495877635E-1-q3*t7*1.00252195621172-q3*t10*1.00252195621172+t6*t11*6.598650859624342E-3+t6*t13*2.742250281407224E-2-t6*t20*1.164727835653528E-1-qd2*qd3*sin(t12)*1.356082488254422E-1+q3*qd1*qd3*6.246980680651502E-1+qd1*qd2*t11*2.592238734419407E-2+qd2*qd3*t11*3.069139934708996E-1+qd1*qd2*t16*7.643475282474036E-1+qd1*qd3*t16*1.171244051584794E-1-qd1*qd2*t18*3.95578974192843E-2+qd1*qd2*t19*7.027172933594252E-1-qd1*qd3*t18*5.672392580461571E-2+qd1*qd3*t19*4.059377542704992E-2-qd1*qd2*t21*1.065983807484365E-1-qd1*qd2*t22*1.238295974705483-qd1*qd3*t21*2.601451547462429E-1+qd1*qd3*t22*1.624356199945572E-1-qd1*qd2*t24*7.804354642387286E-2+qd1*qd2*t37*2.592238734419407E-2-qd1*qd3*t38*5.672392580461571E-2-q3*t6*t13*6.78041244127211E-2+q3*t6*t20*1.534569967354498E-1+q3*qd1*qd2*t16*8.118755085409985E-2+q3*qd1*qd3*t16*2.465385627010454E-1-q3*qd1*qd2*t19*2.342488103169588E-1+q3*qd1*qd2*t21*3.248712399891145E-1+q3*qd1*qd2*t22*5.202903094924857E-1+q3*qd1*qd3*t21*3.781595053641048E-1+q3*qd1*qd2*t24*1.134478516092314E-1-qd1*qd2*t19*t23*2.465385627010454E-1-qd1*qd2*t22*t23*3.781595053641048E-1;
        N(1) = t7*6.152471329736313E-1-t10*6.152471329736313E-1-t26-t27+t29+t30*1.650701241431906E-1-t31*3.230561622574884E-1+t32+t34-t35-qd2*qd3*2.860414991755269E-1+q3*t7*1.00252195621172-q3*t10*1.00252195621172-t11*t33*1.296119367209704E-2-t16*t33*3.821737641237018E-1+t18*t33*1.977894870964215E-2-t19*t33*3.513586466797126E-1+t21*t33*5.329919037421824E-2+t22*t33*6.191479873527417E-1+t24*t33*3.902177321193643E-2-t33*t37*1.296119367209704E-2+q3*qd2*qd3*1.2493961361303-qd2*qd3*t38*1.134478516092314E-1-q3*t16*t33*4.059377542704992E-2+q3*t19*t33*1.171244051584794E-1-q3*t21*t33*1.624356199945572E-1-q3*t22*t33*2.601451547462429E-1-q3*t24*t33*5.672392580461571E-2+t19*t23*t33*1.232692813505227E-1+t22*t23*t33*1.890797526820524E-1;
        N(2) = t6*1.430207495877635E-1-t25*9.264907881420566E-1+t28*1.00252195621172+t30*1.00252195621172-t31*9.264907881420566E-1+t33*7.151037479388174E-2-q3*t6*6.246980680651502E-1-q3*t33*3.123490340325751E-1+t6*t38*5.672392580461571E-2-t16*t33*5.856220257923969E-2+t18*t33*2.836196290230786E-2-t19*t33*2.029688771352496E-2+t21*t33*1.300725773731214E-1-t22*t33*8.121780999727861E-2+t33*t38*2.836196290230786E-2-q3*t16*t33*1.232692813505227E-1-q3*t21*t33*1.890797526820524E-1;

    }

        for (int i = 0; i < 3; i++) {
            if (isnan(N(i)) == 1) {
                N(i) = 0;
            }
        }


}



void PsmForceControl::CalcJd(const Eigen::VectorXd &q, const Eigen::VectorXd &qd)
{
  
    // Dynamics
  /*  float t2 = 3.141592653589793*(1.0/2.0);
    float t3 = -q2+t2+2.908373974667121E-1;
    float t4 = q2-2.908373974667121E-1;
    float t5 = cos(t4);
    float t6 = cos(t3);
    float t7 = sin(t4);
    float t8 = sin(t3);
    float t9 = q2-t2;
    float t10 = t5*t6;
    float t17 = t7*t8;
    float  t11 = t10-t17;
    float  t12 = sin(t9);
    float  t13 = t5*t8;
    float  t14 = t6*t7;
    float  t15 = t13+t14;
    float  t16 = cos(t9);
    float  t18 = q1-t2;
    float  t19 = sin(t18);
    float  t20 = q3+4.162E-1;
    float  t21 = cos(t18);
    float  t22 = t5*t8*t19;
    float t23 = t6*t7*t19;
    float  t24 = t22+t23;
    float  t25 = t5*t6*t19;
    float  t33 = t7*t8*t19;
    float  t26 = t25-t33;
    float  t27 = qd1*t5*t8*t21;
    float t28 = qd1*t6*t7*t21;
    float  t29 = t27+t28;
    float  t30 = qd1*t5*t6*t21;
    float  t32 = qd1*t7*t8*t21;
    float  t31 = t30-t32;
    float  t34 = qd1*t5*t8*t19;
    float  t35 = qd1*t6*t7*t19;
    float  t36 = t34+t35;
    float  t37 = qd1*t5*t6*t19;
    float  t39 = qd1*t7*t8*t19;
    float  t38 = t37-t39;
    float  t40 = t5*t6*t21;
    float  t45 = t7*t8*t21;
    float  t41 = t40-t45;
    float  t42 = t5*t8*t21;
    float  t43 = t6*t7*t21;
    float  t44 = t42+t43;
    float  t46 = t16*t38;
    float  t47 = qd2*t16*t44;
    float  t48 = qd2*t12*t41;
    float  t49 = t46+t47+t48-t12*t36;
    float  t50 = t12*t29;
    float  t51 = qd2*t16*t24;
    float  t52 = qd2*t12*t26;

    Jd(0,1) = t20*(qd2*t15*cos(q2-3.141592653589793*(1.0/2.0))+qd2*t11*t12)-qd2*t7*(3.0/2.0E1)-qd3*(t11*t16-t12*t15)-qd2*t11*t12*(1.68E2/6.25E2)-qd2*t11*t16*(4.3E1/1.0E3)+qd2*t12*t15*(4.3E1/1.0E3)-qd2*t15*t16*(1.68E2/6.25E2);
    Jd(0,2) = -qd2*t11*t16+qd2*t12*t15;
    Jd(1,0) = t12*t29*(1.68E2/6.25E2)+t12*t31*(4.3E1/1.0E3)+t16*t29*(4.3E1/1.0E3)-t16*t31*(1.68E2/6.25E2)-qd3*(t12*t24-t16*t26)-t20*(t50+t51+t52-t16*t31)-qd1*t5*t21*(3.0/2.0E1)+qd2*t7*t19*(3.0/2.0E1)-qd2*t12*t24*(4.3E1/1.0E3)+qd2*t12*t26*(1.68E2/6.25E2)+qd2*t16*t24*(1.68E2/6.25E2)+qd2*t16*t26*(4.3E1/1.0E3)-qd1*t5*t6*t21*(1.29E2/2.5E2)+qd1*t7*t8*t21*(1.29E2/2.5E2);
    Jd(1,1) = -t20*(t12*t38+t16*t36+qd2*t12*t44-qd2*t16*t41)-t12*t36*(4.3E1/1.0E3)+t12*t38*(1.68E2/6.25E2)+t16*t36*(1.68E2/6.25E2)+t16*t38*(4.3E1/1.0E3)+qd3*(t12*t41+t16*t44)+qd1*t7*t19*(3.0/2.0E1)-qd2*t5*t21*(3.0/2.0E1)+qd2*t12*t41*(4.3E1/1.0E3)+qd2*t12*t44*(1.68E2/6.25E2)-qd2*t16*t41*(1.68E2/6.25E2)+qd2*t16*t44*(4.3E1/1.0E3);
    Jd(1,2) = t49;
    Jd(2,0) = t12*t36*(-1.68E2/6.25E2)-t12*t38*(4.3E1/1.0E3)-t16*t36*(4.3E1/1.0E3)+t16*t38*(1.68E2/6.25E2)-t20*t49-qd3*(t12*t44-t16*t41)+qd1*t5*t19*(3.0/2.0E1)+qd2*t7*t21*(3.0/2.0E1)+qd2*t12*t41*(1.68E2/6.25E2)-qd2*t12*t44*(4.3E1/1.0E3)+qd2*t16*t41*(4.3E1/1.0E3)+qd2*t16*t44*(1.68E2/6.25E2)+qd1*t5*t6*t19*(1.29E2/2.5E2)-qd1*t7*t8*t19*(1.29E2/2.5E2);
    Jd(2,1) = -t20*(t12*t31+t16*t29-qd2*t12*t24+qd2*t16*t26)-t12*t29*(4.3E1/1.0E3)+t12*t31*(1.68E2/6.25E2)+t16*t29*(1.68E2/6.25E2)+t16*t31*(4.3E1/1.0E3)-qd3*(t12*t26+t16*t24)+qd2*t5*t19*(3.0/2.0E1)+qd1*t7*t21*(3.0/2.0E1)-qd2*t12*t24*(1.68E2/6.25E2)-qd2*t12*t26*(4.3E1/1.0E3)-qd2*t16*t24*(4.3E1/1.0E3)+qd2*t16*t26*(1.68E2/6.25E2);
    Jd(2,2) = -t50-t51-t52+t16*t31;*/

    // ROS_INFO_STREAM("Jd: "<<Jd);

    // Kinematic Jd
    f2 = 3.141592653589793*(1.0/2.0);
    f3 = q2+f2;
    f4 = sin(f3);
    f5 = q1-f2;
    f6 = q3+3.7E-3;
    f7 = cos(f3);
    f8 = sin(f5);
    f9 = cos(f5);
    Jd(0,1) = -qd3*f4-qd2*f6*f7;
    Jd(0,2) = -qd2*f4;
    Jd(1,0) = qd3*f4*f8+qd1*f4*f6*f9+qd2*f6*f7*f8;
    Jd(1,1) = -qd3*f7*f9+qd2*f4*f6*f9+qd1*f6*f7*f8;
    Jd(1,2) = qd1*f4*f8-qd2*f7*f9;
    Jd(2,0) = qd3*f4*f9-qd1*f4*f6*f8+qd2*f6*f7*f9;
    Jd(2,1) = qd3*f7*f8-qd2*f4*f6*f8+qd1*f6*f7*f9;
    Jd(2,2) = qd1*f4*f9+qd2*f7*f8;

}
void PsmForceControl::CalcM(const Eigen::VectorXd &q) //Eigen::VectorXd qd)
{   

    if (name == "PSM1")
    {
        //Without KE Fourier Test 2
   /*     float t2 = q2*2.0;
        float t3 = t2-5.816747949334241E-1;
        float t4 = t2-2.908373974667121E-1;
        float t5 = cos(t2);
        float t6 = sin(t2);
        float t7 = cos(t3);
        float t8 = q3*q3;
        float t9 = cos(2.908373974667121E-1);
        float t10 = cos(t4);
        float t11 = sin(t3);
        float t12 = cos(q2);
        float t13 = q2-2.908373974667121E-1;
        float t14 = sin(t13);
        float t15 = sin(q2);
        float t16 = t14*6.306094755616586E-2;
        float t17 = t12*6.832945921991039E-5;
        float t18 = q3*t14*5.162122456466293E-2;
        float t19 = t15*(-6.540512435609696E-2)+t16+t17+t18-q3*t15*1.589057191160707E-3+2.139869112385656E-2;
        float t20 = sin(2.908373974667121E-1);
        float t21 = cos(t13);
        float t22 = t12*1.589057191160707E-3;
        float t23 = t21*(-5.162122456466293E-2)+t22;
        float t24 = t20*(-7.495846549079901E-2)+3.149277775144036E-1;

        M(0,0) = q3*(-1.347535840184895E-1)+t5*5.082100660169126E-1-t6*9.836926535805593E-2-t7*2.993166773303814E-1+t8*3.939057613746652E-1-t9*4.221622946842542E-2-t10*4.221622946842542E-2+t11*2.849785091985549E-1+t12*1.075404007903575E-2+t20*4.589980744314686E-2+cos(q2-5.816747949334241E-1)*1.075404007903575E-2+sin(t4)*4.589980744314686E-2+q3*t5*2.814415297895028E-1+q3*t6*4.049356850373793E-1-q3*t7*4.161951138079923E-1-q3*t9*7.495846549079901E-2-q3*t10*7.495846549079901E-2-q3*t11*9.000790752297579E-2+t5*t8*2.498615516359967E-1+t7*t8*1.440442097386685E-1+3.415399107369573E-1;
        M(0,1) = t19;
        M(0,2) = t23;
        M(1,0) = t19;
        M(1,1) = q3*(-2.69507168036979E-1)+t8*7.878115227493304E-1-t9*8.443245893685084E-2+t20*9.179961488629371E-2-q3*t9*1.49916930981598E-1+4.51376943612784E-1;
        M(1,2) = t24;
        M(2,0) = t23;
        M(2,1) = t24;
        M(2,2) = 7.878115227493304E-1;*/

      // MidJuly_3dof fourier test 3
      float t2 = q2*2.0;
      float t3 = t2-2.908373974667121E-1;
      float t4 = cos(t2);
      float t5 = sin(t2);
      float t6 = cos(2.908373974667121E-1);
      float t7 = q3*q3;
      float t8 = cos(t3);
      float t9 = cos(q2);
      float t10 = sin(q2);
      float t11 = q2-2.908373974667121E-1;
      float t12 = sin(t11);
      float t13 = t12*5.648514011831588E-3;
      float t14 = t9*3.233715128693166E-5;
      float t15 = t10*(-6.322333279804798E-3)+t13+t14-q3*t10*3.633350698549627E-2+6.036324572650495E-3;
      float t16 = sin(2.908373974667121E-1);
      float t17 = t9*3.633350698549627E-2;
      float t18 = t16*(-2.196176513085578E-3)+9.811510976268357E-3;
      M(0,0) = q3*(-1.370021252172674E-1)+t4*5.589503704456244E-2+t5*9.131214113774334E-3+t6*1.508771032382092E-2+t7*3.385335113329479E-1+t8*1.508771032382092E-2+t9*2.362985957563206E-2+t16*1.508766140985477E-2+cos(q2-5.816747949334241E-1)*2.362985957563206E-2-cos(t2-5.816747949334241E-1)*9.367754837668338E-3+sin(t3)*1.508766140985477E-2-q3*t4*1.370021252172674E-1+q3*t5*9.811510976268357E-3-q3*t6*2.196176513085578E-3-q3*t8*2.196176513085578E-3+t4*t7*3.385335113329479E-1+2.722195153879596E-1;
      M(0,1) = t15;
      M(0,2) = t17;
      M(1,0) = t15;
      M(1,1) = q3*(-2.740042504345347E-1)+t6*3.017542064764185E-2+t7*6.770670226658958E-1+t16*3.017532281970953E-2-q3*t6*4.392353026171156E-3+3.922469263646697E-1;
      M(1,2) = t18;
      M(2,0) = t17;
      M(2,1) = t18;
      M(2,2) = 6.770670226658958E-1;

    }

    else if (name == "PSM2")
    {

        float t2 = q2*2.0;
        float t3 = t2-5.816747949334241E-1;
        float t4 = t2-2.908373974667121E-1;
        float t5 = cos(t2);
        float t6 = sin(t2);
        float t7 = cos(t3);
        float t8 = q3*q3;
        float t9 = cos(2.908373974667121E-1);
        float t10 = cos(t4);
        float t11 = sin(t3);
        float t12 = cos(q2);
        float t13 = q2-2.908373974667121E-1;
        float t14 = sin(t13);
        float t15 = sin(q2);
        float t16 = t14*1.371125140703612E-2;
        float t17 = q3*t15*7.672849836772491E-2;
        float t18 = t12*(-3.299325429812171E-3)-t15*5.823639178267639E-2+t16+t17-q3*t14*3.390206220636055E-2-2.26924003813518E-2;
        float t19 = sin(2.908373974667121E-1);
        float t20 = cos(t13);
        float t21 = t20*3.390206220636055E-2;
        float t22 = t12*(-7.672849836772491E-2)+t21;
        float t23 = t19*(-2.836196290230786E-2)+1.015146977108036E-1;

        M(0,0) =  q3*(-7.151037479388174E-2)+t5*3.095739936763709E-1-t6*2.664959518710912E-2-t7*1.756793233398563E-1+t8*1.561745170162875E-1+t9*1.951088660596821E-2+t10*1.951088660596821E-2+t11*1.910868820618509E-1-t12*1.296119367209704E-2-t19*9.889474354821074E-3-cos(q2-5.816747949334241E-1)*1.296119367209704E-2-sin(t4)*9.889474354821074E-3-q3*t5*1.300725773731214E-1+q3*t6*8.121780999727861E-2+q3*t7*5.856220257923969E-2-q3*t9*2.836196290230786E-2-q3*t10*2.836196290230786E-2+q3*t11*2.029688771352496E-2+t5*t8*9.453987634102619E-2+t7*t8*6.163464067526135E-2+3.77311613898454E-1;
        M(0,1) =  t18;
        M(0,2) =  t22;
        M(1,0) =  t18;
        M(1,1) =  q3*(-1.430207495877635E-1)+t8*3.123490340325751E-1+t9*3.902177321193643E-2-t19*1.977894870964215E-2-q3*t9*5.672392580461571E-2+4.257562539388422E-1;
        M(1,2) =  t23;
        M(2,0) =  t22;
        M(2,1) =  t23;
        M(2,2) =  3.123490340325751E-1;
    }

}



PsmForceControl::PsmForceControl(std::shared_ptr<ros::NodeHandle> n, const string nam, const string ctrl_type) {

    nhandle = n;

    name = nam;
    ctrl = ctrl_type;

    ROS_INFO_STREAM(name << " Control START");

    desplot_x = nhandle->advertise<std_msgs::Float64>("/" + name + "/d0", 10);
    desplot_y = nhandle->advertise<std_msgs::Float64>("/" + name + "/d1", 10);
    desplot_z = nhandle->advertise<std_msgs::Float64>("/" + name + "/d2", 10);

    plot_x = nhandle->advertise<std_msgs::Float64>("/" + name + "/0", 10);
    plot_y = nhandle->advertise<std_msgs::Float64>("/" + name + "/1", 10);
    plot_z = nhandle->advertise<std_msgs::Float64>("/" + name + "/2", 10);

    joint_pub = nhandle->advertise<sensor_msgs::JointState>("/dvrk/" + name + "/set_effort_joint", 3);
    pose_pub  = nhandle->advertise<geometry_msgs::Pose>("/dvrk/" + name + "/set_position_cartesian", 3);

//jacobian_sub=n.subscribe("/dvrk/"+ name + "/jacobian_body", 200, &PsmForceControl::CallbackJacobian,this);

    //Data Subscribers
    joint_sub = nhandle->subscribe("/dvrk/" + name + "/state_joint_current", 10, &PsmForceControl::CallbackJoint, this);
    cartesian_sub = nhandle->subscribe("/dvrk/" + name + "/position_cartesian_current", 10, &PsmForceControl::CallbackCartesian, this);
    force_sub = nhandle->subscribe("/force_data/load_cell", 10, &PsmForceControl::CallbackForce,this);

    //
    setforce_sub = nhandle->subscribe("/psm_sense/setforce", 10, &PsmForceControl::CallbackSetForce, this);
    setpos_sub = nhandle->subscribe("/psm/cmd_vel2", 10, &PsmForceControl::CallbackSetPosition, this);

    //Command Teleop Subscribers Turn Off when using Cooperative
    force_sub2 = nhandle->subscribe("/psm/cmd_force", 10, &PsmForceControl::CallbackSetForceIncrement, this);
    setpos_sub2 = nhandle->subscribe("/psm/cmd_vel", 10, &PsmForceControl::CallbackSetPositionIncrement, this);

//Joint States and Pub data
    dof = 3;
    cart_dof = 3;
    q.resize(dof);
    qd.resize(dof);
    eff.resize(dof);
    u.resize(cart_dof);
    q0.resize(3);
    joint_act.resize(3), joint_des.resize(3);
    data_trans.resize(3,3);
    orient_cart.resize(4);

//Cartesian States and data
    ve.resize(3);
    fd.resize(3);
    he.resize(3);
    ha.resize(3);
    xf.resize(3);
    vd.resize(3);
    ad.resize(3);
    y.resize(3);
    x0.resize(3);
    x_int.resize(3);
    v_int.resize(3);
    a_int.resize(3), deadband.resize(3),

// Impedance Controller Data
    Ja.resize(3, 3);
    JaM.resize(3, 3);
    JaInv.resize(3,3);
    Jd.resize(3, 3);
    Jmin.resize(3, 3);
    N.resize(3);
    G.resize(3);
    C.resize(3, 3);
    Fr.resize(3);
    M.resize(3, 3);
    Mt.resize(3, 3);
    Kp.resize(3, 3);
    Kd.resize(3, 3);
    Cp.resize(3, 3);
    Ci.resize(3, 3);
    St.resize(3,3);

// Wrist PID Controller Data
    wrist_u.resize(3), wrist_eq.resize(3), wrist_eqd.resize(3), wrist_kp.resize(3), wrist_kd.resize(3);

//JointMsgs
    joint_msg.name.push_back("Joint Publisher");
    for (int j = 0; j < 6; j++) {
        joint_msg.effort.push_back(0.0);
        msg2.velocity.push_back(0.0);
    }


    myq[0] = que1;
    myq[1] = que2;
    myq[2] = que3;
    myq[3] = que4;
    myq[4] = que5;
    myq[5] = que6;

    rate = 2000;
    tf = 1; // moving 0.001 m in 0.2 s is pretty good for u values.
    filter_n = 20;
    index = 0;
    f_index = 0;

    q1_traj.ts = 1 / rate;
    q2_traj.ts = 1 / rate;
    q3_traj.ts = 1 / rate;

    q1_traj.tf = tf;
    q2_traj.tf = tf;
    q3_traj.tf = tf;

    q1_traj.check = false;
    q2_traj.check = false;
    q3_traj.check = false;

    q1_traj.qd.resize(6);
    q2_traj.qd.resize(6);
    q3_traj.qd.resize(6);

    q_traj[0] = q1_traj;
    q_traj[1] = q2_traj;
    q_traj[2] = q3_traj;

// Initializers
    fd << 0.0, 0.0, 0.0;
    he << 0.0, 0.0, 0.0;
    ha << 0.0, 0.0, 0.0;

    xd << 0.0, 0.0, 0.0;
    xe << 0.0, 0.0, 0.0;

    x0 << 0.0, 0.0, 0.0;
    q0 << 0.0, 0.0, 0.0;

    q << 0.0, 0.0, 0.0;
    qd << 0.0, 0.0, 0.0;
    u << 0.0, 0.0, 0.0;



    Fr << 0, 0, 0;
    N << 0, 0, 0;

    M << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Mt << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Ja << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    JaM << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Jd << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    St << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Kd << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Kp << 0, 0, 0, 0, 0, 0, 0, 0, 0;



    //Data Transform from cisst-saw, to matlab model
    data_trans << 0, 1, 0,
                -1, 0, 0,
                0, 0, 1;

    orient_cart << 0 , 0, 0, 0;

    interp = false;

/*    for (int i=0;i<dof;i++) {
        for (int j = 0; j < filter_n; j++) {
            myq[i].push_back(0.0);
        }
    }

    for(int j = 0;j<filter_n;j++) {
        f_myq.push_back(0.0);
    }*/
}

void PsmForceControl::CalcFr(const Eigen::VectorXd &q, const Eigen::VectorXd &qd)
{
 
  float x[3];

  for (int i=0;i<3;i++)
    {
        if (abs(qd(i)) < deadband(i))
        {
            x[i] = 0;
        }
        else
        {
            //if (i==2 & q_traj[2].check==true)
            if (i==2 & v_int(i) > qd(i))
            {
                // This makes it so that friction is only actuating while given a desired command
                x[i] = v_int(i);
                //x[i] = qd(i);
            }
            else
            {
                x[i] = qd(i);
            }

        }
    }

    // From test_3dof_svd fourier_test2


    float a1 =4.0E2;
    float a2 =4.0E2;
    float scale = 1;



    //float Fs_pos = 0.6;
    //float Fs_neg = -0.4;

    float Fs_pos = 1;
    float Fs_neg = -1;


    //Computed Torque
    //float x_e = xd(2)-q(2);

    //Impedance Controller
        float x_e = joint_des(2) - joint_act(2);


    if(name == "PSM1")
    {
       
        //Without KE
/*        Fr(0,0) = x[0]*5.121004410809419E-2+1.561585962952595E-1/(exp(x[0]*-a1)+1.0)-7.807929814762977E-2;
        Fr(1,0) = x[1]*1.277466080900284E-1+2.91620349820475E-1/(exp(x[1]*-a2)+1.0)-1.458101749102375E-1;
*/
        //MidJuly_3dof fourier test 3
        Fr(0) = x[0]*9.542485682180182E-2+1.0897508499757E-1/(exp(x[0]*-4.0E2)+1.0)+2.751172483473256E-1;
        Fr(1) = x[1]*1.63259829973678E-1+1.631162536205055E-1/(exp(x[1]*-4.0E2)+1.0)+1.937201881157299E-1;
   


        //Fr(0) =  qd1*8.857790114534859E-2+1.330230787728563E-1/(exp(qd1*-a1)+1.0)-6.651153938642816E-2;
        //Fr(1) =  qd2*1.585859192149214E-1+1.935467306471518E-1/(exp(qd2*-a2)+1.0)-9.67733653235759E-2;

         //Fr(0) =  0;
        // Fr(1) = 0;

        //Friction Compensation Stick Based on Position
        if(abs(qd(2)) < deadband(2))
        {
            if (x_e > pos_deadband)
            {
                Fr(2) = Fs_pos;
            } else if (x_e < -pos_deadband)
            {
                Fr(2) = Fs_neg;
            } else
            {
                Fr(2) = 0;
            }

        }
        else {
            //Fr(2) = qd3*5.3115714926966E-1+9.158363928863775E-1/(exp(qd3*-4.0E2)+1.0)-4.579181964431888E-1;
              //Without KE
               //Fr(2,0) = x[2]*8.397843687710638E-1+1.117115567283898/(exp(x[2]*-4.0E2)+1.0)-5.585577836419491E-1;

               // MidJuly_3dof
               Fr(2) = x[2]*7.662943767346468E-1+1.069206734315578/(exp(x[2]*-4.0E2)+1.0)-3.845210662560725E-1;
        }

    }
    else if(name == "PSM2")
    {

        Fs_pos = 1;
        Fs_neg = -1;

        Fr(0) = x[0]*6.424159338795044E-2+1.064251634172642E-1/(exp(x[0]*-4.0E2)+1.0)-5.321258170863212E-2;
        Fr(1) = x[1]*1.827594952276174E-1+3.450439652017518E-1/(exp(x[1]*-4.0E2)+1.0)-1.725219826008759E-1;

        //Friction Compensation Stick Based on Position
        if(abs(qd(2)) < deadband(2))
        {
            if (x_e > pos_deadband)
            {
                Fr(2) = Fs_pos;
            } else if (x_e < -pos_deadband)
            {
                Fr(2) = Fs_neg;
            } else
            {
                Fr(2) = 0;
            }

        }
        else {

            Fr(2) = x[2]*1.577287853588895+1.444058135392009/(exp(x[2]*-4.0E2)+1.0)-7.220290676960047E-1;

        }
    }


    Fr(2) = scale* Fr(2);
}

void PsmForceControl::SetGainsInit()
{
    // Original MT.diagonal of real robot
    //Mt.diagonal()<<0.3, 0.4, 0.5;

    if(name == "PSM1")
    {
        Mt.diagonal()<<0.35, 0.36, 0.3;
        Kp.diagonal() << 15, 15, 15;
        Kd.diagonal() << 3, 3, 3;
    }
    else if (name == "PSM2")
    {
        Mt.diagonal()<<0.42, 0.44, 0.3123;
        Kp.diagonal() << 100, 80, 120;
        Kd.diagonal() << 7, 4, 6;
    }

    //Real Coefficients


    //Wrist Coefficients;
    wrist_kp << 0.5, 0.5, 0.5;
    wrist_kd <<  0.5,  0.5,  0.5;

    //Test Damping
   // Kp.diagonal()<<1, 1, 3;
    //Kd.diagonal()<<5, 0.5, 10;

    //Friction Compensation
    pos_deadband = 0.006;
    // PSM2
    //deadband << 0.0045, 0.005, 0.005;

    // PSM1
    deadband << 0.01, 0.01, 0.005;

    //Force Stuff
    force_deadband = 0.5;
    force_increment = 0.00002; //meters a t( 2000 / 4 )hz?

    fl << 10, 10, 20; // Nm, Nm, N

    //jacobian scaling factor
    St.diagonal()<< 1, 1, 0.16;
    //St<< 1, 0, 0, 0 , 1, 0, 0 , 0, 1/6;
}

void PsmForceControl::SetDesiredInit()
{
 x0 = xe;
 q0 = q;

 double incre [3] = {0, 0, 0};
 fd << 0, 0, 0;

 //Impedance Controller
 xd << incre[0] + x0(0), incre[1] + x0(1), incre[2] + x0(2) ;

 // Computed Torque Controller
 //xd << incre[0] + q0(0),incre[1] + q0(1) ,incre[2] + q0(2) ;

 vd << 0, 0, 0;
 ad << 0, 0, 0;

 force_set = 5;
}


void PsmForceControl::CallbackJacobian(const std_msgs::Float64MultiArray &msg)
{
for (int i=0;i<3;i++)
   {
     for (int j=0;j<3;j++)
 {
  Ja(i,j)=msg.data[i*6+j];
     }
   }
}


void PsmForceControl::CallbackJoint(const sensor_msgs::JointState &msg)
{
    // Filter
    for (int i=0;i<dof;i++)
    {
        q(i)=msg.position[i];
        qd(i)=msg.velocity[i];
        eff(i)=msg.effort[i];

        myq[i].push_back(msg.velocity[i]);

        if (index > filter_n-1)
        {
            myq[i].pop_front();
            sum[i] = 0;
            for(int j = 0;j<filter_n;j++)
            {
                sum[i] = sum[i]+myq[i][j];
            }
            qd(i) = sum[i]/filter_n;

        }
    }
    index = index + 1;

    q1 = q(0);
    q2 = q(1);
    q3 = q(2);
    qd1 = qd(0);
    qd2 = qd(1);
    qd3 = qd(2);

    //ROS_INFO_STREAM("myq : "<< myq[0].size() <<endl); its good
    //drop = msg.header.seq-drop_p;
    //drop_p = msg.header.seq;
}

void PsmForceControl::CallbackCartesian(const geometry_msgs::PoseStamped &msg)
{

    temp_x << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    xe = data_trans*temp_x;

    orient_cart << msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w;
/*
    xe(0) = msg.pose.position.y;  // y becomes x in my calculation
    xe(1) =-msg.pose.position.x; // x becomes -y in my calculation
    xe(2) = msg.pose.position.z;
*/

}

void PsmForceControl::CallbackForce(const std_msgs::Float32 &msg)
{
    // Filter
    //ROS_INFO_STREAM("potato");
        f_myq.push_back(msg.data);
        if (f_index > filter_n-1)
        {
            f_myq.pop_front();
            f_sum = 0;
            for(int j = 0;j<filter_n;j++)
            {
                f_sum = f_sum + f_myq[j];
            }
            force_magnitude = f_sum/filter_n;
            force_magnitude = -force_magnitude;
        }
        f_index = f_index + 1;

    // Force Direction
    this -> ForceSet();

}

void PsmForceControl::ForceSet()
{
    he(0) = force_magnitude;
}

 void PsmForceControl::CallbackSetForceIncrement(const geometry_msgs::Twist &msg){
  /*   he(0) = msg.linear.x + he(0);
     he(1) = msg.linear.y + he(1);
     he(2) = msg.linear.z + he(2);
*/
     xd(1)= msg.linear.x + xd(1);
     xd(0)= msg.linear.y + xd(0);
     xd(2)= msg.linear.z + xd(2);
 }

 void PsmForceControl::CallbackSetForce(const geometry_msgs::Pose &msg)
 {
    fd(0)=msg.position.x;
    fd(1)=msg.position.y;
    fd(2)=msg.position.z;
 }

 void PsmForceControl::CallbackSetPosition(const geometry_msgs::Twist &msg)
 {
    xd(0)=msg.linear.x;
    xd(1)=msg.linear.y;
    xd(2)=msg.linear.z;
 }

void PsmForceControl::CallbackSetPositionIncrement(const geometry_msgs::Twist &msg)
 {


     arr(0) = msg.linear.x;
     arr(1) = msg.linear.y;
     arr(2) = msg.linear.z;

     //Just for testing
     xd(0)= msg.linear.x + xd(0);
     xd(1)= msg.linear.y + xd(1);
     xd(2)= msg.linear.z + xd(2);


     for (int i=0;i<3;i++)
     {
         // Impedance Controller
         q_traj[i].qd << xe(i), 0, 0, xd(i),0,0;

         // Computed Torque controller
         //q_traj[i].qd << q(i), 0, 0, xd(i),0,0;


         if (arr(i) != 0)
         {
             q_traj[i]=interpolate(q_traj[i]);
         }
     }

     //ROS_INFO_STREAM("v_int:" << q_traj[0].check);

     t0 = ros::Time::now().toSec();
     t = 0;
     interp = true;
 }


void PsmForceControl::CalcU()
 {    // This is parallel/position/force

    //int fl = 5; //force limit
    ve = JaM*qd;

    if (interp==true)
    {
        for (int i=0;i<3;i++)
        {

            if(q_traj[i].check==false)
            {
                x_int(i)=xd(i);
                v_int(i)=0;
                a_int(i)=0;
            }
            else
            {
                x_int(i)=q_traj[i].x(t);
                v_int(i)=q_traj[i].v(t);
                a_int(i)=q_traj[i].a(t);

            }
        }



        // Impedance Controller
        //y = JaM.inverse()*Mt.inverse()*(Mt*a_int+Kd*(v_int-ve)+Kp*(x_int-xe)-Mt*Jd*qd-he);

                y = JaM.inverse()*Mt.inverse()*(Mt*a_int+Kd*(v_int-ve)+Kp*(x_int-xe)-Mt*Jd*qd);

        //Computed Torque Controller
        //y =  Kd*(v_int-qd) + Kp*(x_int-q);
        t = t + 1;

   /*   ROS_INFO_STREAM("u_int 1 at time" << t << " : " << u(0));
        ROS_INFO_STREAM("u_int 2 at time" << t << " : " << u(1));
        ROS_INFO_STREAM("u_int 3 at time" << t << " : " << u(2));*/

        if (t == rate*tf)
        {
            t = 0;
            q_traj[0].check = false;
            q_traj[1].check = false;
            q_traj[2].check = false;
            interp = false;


        }
    }
    else
    {
        // Impedance Controller
       // y = JaM.inverse()*Mt.inverse()*(Mt*ad+Kd*(vd-ve)+Kp*(xd-xe)-Mt*Jd*qd-he);
        //y = JaInv*Mt.inverse()*(Mt*ad+Kd*(vd-ve)+Kp*(xd-xe)-Mt*Jd*qd);
        y = JaM.inverse()*Mt.inverse()*(Mt*ad+Kd*(vd-ve)+Kp*(xd-xe)-Mt*Jd*qd);

        // Computed Torque controller
        //y =  Kd*(vd-qd) + Kp*(xd-q);
    }


    //u = M*y + N +Fr +JaM.transpose()*he;
      u = M*y + N +Fr;

     test = u;

     if(std::fabs(test(0))>fl(0)|std::fabs(test(1))>fl(1)|std::fabs(test(2))>fl(2)){
         ROS_INFO_STREAM("Jaminv: "<< JaM.inverse()<<endl << "Jd: "<< Jd<< endl<< "qd:" << qd<< endl<<"test:" << test<< endl);
     }

    // Conclusion: the JaINv* Jd interaction is a problem!!

     //ROS_INFO_STREAM("Jinv: "<< Mt.inverse() <<endl);
    // ROS_INFO_STREAM("Kd*(vd-ve): "<< Kd*(vd-ve) <<endl);
    // ROS_INFO_STREAM("Jacobian singularity : "<< JaM.inverse()*Mt.inverse() <<endl);
     // ROS_INFO_STREAM("he : "<< u <<endl);

 }
Eigen::VectorXd PsmForceControl::InverseKinematic(const Eigen::VectorXd &fed)
{
    /*joint_angle(0) = atan(xd(1)/xd(2));
    joint_angle(1) = -atan(xd(0)/sqrt(pow(xd(1),2)+pow(xd(2),2)));
    joint_angle(2) = sqrt(pow(xd(1),2)+pow(xd(0),2)+pow(xd(2),2))+0.006;*/

    Eigen::VectorXd joint_a;
    joint_a.resize(3);

    joint_a(0) = atan(fed(1)/fed(2));
    joint_a(1) = -atan(fed(0)/sqrt(pow(fed(1),2)+pow(fed(2),2)));
    joint_a(2) = sqrt(pow(fed(1),2)+pow(fed(0),2)+pow(fed(2),2))+0.006;

    return joint_a;
}

void PsmForceControl::WristPID()
{
   /* for (int i=0;i<3;i++)
    {
        wrist_eq(i)= 0.5 - q(i+3);
        wrist_eqd(i) = 0.0 - qd(i+3);
    }

    for (int i=0;i<3;i++)
    {
        wrist_u(i) = wrist_eq(i) * wrist_kp(i) + wrist_eqd(i) * wrist_kd(i);
        //ROS_INFO_STREAM("  wrist_u: "<< wrist_u<< endl<< "wrist_eq: "<< wrist_eq << endl<< "wrist_eqd: "<< wrist_eqd<<endl);

        //wrist_u(i) = 0;
    }*/
}
void PsmForceControl::output()
{
     if(ctrl == "Impedance")
     {
         for (int i=0;i<3;i++)
         {

             if(i>=3)
             {
                // joint_msg.effort[i] = -wrist_u(i-3);
                 //ROS_INFO_STREAM(endl<<joint_msg.effort[i]);
             }
             else if(std::abs(u(i))>fl(i))
             {
                joint_msg.effort[i] = 0;
             }
             else
             {
                 joint_msg.effort[i] = u(i);
             }
         }

         // ----------------------- IMPORTANT---This runs Robot-----------------
         joint_pub.publish(joint_msg);
         // ------------------------------------------------------
     }

     else if(ctrl == "Cartesian")
     {   Eigen::Vector3d x;

        if (interp==true)
        {
            x = data_trans.inverse()*x_int;
        }
        else
        {
            x = data_trans.inverse()*xd;
        }

         pose_msg.position.x = x(0);
         pose_msg.position.y = x(1);
         pose_msg.position.z = x(2);

         pose_msg.orientation.x = orient_cart(0);
         pose_msg.orientation.y = orient_cart(1);
         pose_msg.orientation.z = orient_cart(2);
         pose_msg.orientation.w = orient_cart(3);

         // --------------PUBLISHING -----------
         //pose_pub.publish(pose_msg);
         // ------------- publsih ----------

         //ROS_INFO_STREAM(name<<" POSE : "<< x <<endl);
     }

/*     dq0.data = qd(0);
     dq1.data = qd(1);
     dq2.data = qd(2);

     desplot_x.publish(dq0);
     desplot_y.publish(dq1);
     desplot_z.publish(dq2);*/

  /* msg2.velocity[0] = qd(0);
   msg2.velocity[1] = qd(1);
   msg2.velocity[2] = qd(2);

   plot_x.publish(msg2);*/


  // Check Joint torque computed stuff
/*     if(interp==true)
     {
         dq0.data = x_int(0);
         dq1.data = x_int(1);
         dq2.data = x_int(2);
     }
     else
     {
         dq0.data = xd(0);
         dq1.data = xd(1);
         dq2.data = xd(2);
     }*/

// Check Joint Angles
/*   dq0.data = joint_des(0);
    dq1.data = joint_des(1);
    dq2.data = joint_des(2);

    desplot_x.publish(dq0);
    desplot_y.publish(dq1);
    desplot_z.publish(dq2);

    mq0.data = joint_act(0);
    mq1.data = joint_act(1);
    mq2.data = joint_act(2);

    plot_x.publish(mq0);
    plot_y.publish(mq1);
    plot_z.publish(mq2);*/


    // Check Cartesian Positions
  /*  dq0.data = xd(0);
    dq1.data = xd(1);
    dq2.data = xd(2);

    desplot_x.publish(dq0);
    desplot_y.publish(dq1);
    desplot_z.publish(dq2);

    mq0.data = xe(0);
    mq1.data = xe(1);
    mq2.data = xe(2);

    plot_x.publish(mq0);
    plot_y.publish(mq1);
    plot_z.publish(mq2);*/


    // Check Joint velocities Positions
    dq0.data = test(0);
    dq1.data = test(1);
    dq2.data = test(2);

    desplot_x.publish(dq0);
    desplot_y.publish(dq1);
    desplot_z.publish(dq2);

    //mq0.data = xe(0);
    //mq1.data = xe(1);
    //mq2.data = xe(2);

    //plot_x.publish(mq0);
    //plot_y.publish(mq1);
    //plot_z.publish(mq2);

// Fore Measure
 /*   dq0.data = force_magnitude;
    desplot_x.publish(dq0);*/
 }

void  PsmForceControl::Loop()
 {
     //ROS_INFO_STREAM("xd"<< name << " " << xd);
     //ROS_INFO_STREAM("orient"<< name << " " << orient_cart);

     this->joint_act = this->InverseKinematic(xe);
     this->joint_des = this->InverseKinematic(xd);
     this->CalcJaM(q, qd);
     //this->CalcJaInv(q, qd);
     this->CalcJd(q, qd);
     this->CalcFr(q, qd);
     this->CalcN(q, qd);
     this->CalcM(q);
     this->CalcU();
     //this->WristPID();
     this->output();
     ros::spinOnce();
 }

/*int main(int argc, char **argv)
/*int main(int argc, char **argv)
{
  // Options
    string name = "PSM1";

  ros::init(argc, argv, "PsmForceControl_node");
  ros::NodeHandle n;
  ROS_INFO("It started");

  PsmForceControl obj(n,name);
  ros::Rate r(obj.rate);

  // int i, j;
  int count = 0;
  ros::spinOnce();
  ros::Duration(1).sleep();
  ros::spinOnce();

  obj.SetGainsInit();
  obj.SetDesiredInit();

  while(ros::ok())
  {
    obj.Loop();
    ros::spinOnce();

  //drop=0;

  r.sleep();
  }
//ros::spin();

}*/
