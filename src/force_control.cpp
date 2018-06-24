#include "psm_coop/force_control.h"



void PsmForceControl::CalcJaM(const Eigen::VectorXd &q,const Eigen::VectorXd &qd)
{
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    float qd1 = qd(0);
    float qd2 = qd(1);
    float qd3 = qd(2);

 /*   // Jacobian from matlab
    float t2 = q2-2.908373974667121E-1;
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

}

void PsmForceControl::CalcN(const Eigen::VectorXd &q,const Eigen::VectorXd &qd) {
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    float qd1 = qd(0);
    float qd2 = qd(1);
    float qd3 = qd(2);

    // N from TEST: test_3dof_svd -> fourier_test2
    if (name == "PSM1") {
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

        N(0) = t8*(-3.344751175535797)-t9*3.344751175535797-t25*1.499393709809464-t26*7.320145176142429E-1+t27+t29+t30+t32+t34+t35-sin(q1-2.908373974667121E-1)*3.512986425818344E-1-sin(q1+2.908373974667121E-1)*3.512986425818344E-1-cos(q1)*4.704437037389851E-2+sin(q1)*1.185584490767175-qd1*qd3*2.69507168036979E-1-q3*t7*2.34295976006465-q3*t10*2.34295976006465-t6*t11*1.366589184398208E-4+t6*t13*1.261218951123317E-1-t6*t20*1.308102487121939E-1+qd2*qd3*sin(t12)*2.064848982586517E-1+q3*qd1*qd3*1.575623045498661-qd1*qd2*t11*2.150808015807149E-2-qd2*qd3*t11*6.356228764642827E-3+qd1*qd2*t16*1.13991403679422-qd1*qd3*t16*8.323902276159846E-1+qd1*qd2*t18*1.835992297725874E-1+qd1*qd2*t19*1.197266709321526-qd1*qd3*t18*1.49916930981598E-1-qd1*qd3*t19*1.800158150459516E-1-qd1*qd2*t21*3.934770614322237E-1-qd1*qd2*t22*2.03284026406765+qd1*qd3*t21*5.628830595790056E-1+qd1*qd3*t22*8.098713700747587E-1+qd1*qd2*t24*1.688649178737017E-1-qd1*qd2*t37*2.150808015807149E-2-qd1*qd3*t38*1.49916930981598E-1+q3*t6*t13*1.032424491293259E-1-q3*t6*t20*3.178114382321413E-3-q3*qd1*qd2*t16*3.600316300919032E-1+q3*qd1*qd3*t16*5.76176838954674E-1+q3*qd1*qd2*t19*1.664780455231969+q3*qd1*qd2*t21*1.619742740149517-q3*qd1*qd2*t22*1.125766119158011+q3*qd1*qd3*t21*9.994462065439869E-1+q3*qd1*qd2*t24*2.998338619631961E-1-qd1*qd2*t19*t23*5.76176838954674E-1-qd1*qd2*t22*t23*9.994462065439869E-1;
        N(1) = t8*(-3.344751175535797)+t9*3.344751175535797+t25*1.499393709809464-t26*7.320145176142429E-1+t27-t29-t30+t32+t34-t35-qd2*qd3*5.390143360739581E-1+q3*t7*2.34295976006465-q3*t10*2.34295976006465+t11*t33*1.075404007903575E-2-t16*t33*5.699570183971099E-1-t18*t33*9.179961488629371E-2-t19*t33*5.986333546607628E-1+t21*t33*1.967385307161119E-1+t22*t33*1.016420132033825-t24*t33*8.443245893685084E-2+t33*t37*1.075404007903575E-2+q3*qd2*qd3*3.151246090997322-qd2*qd3*t38*2.998338619631961E-1+q3*t16*t33*1.800158150459516E-1-q3*t19*t33*8.323902276159846E-1-q3*t21*t33*8.098713700747587E-1+q3*t22*t33*5.628830595790056E-1-q3*t24*t33*1.49916930981598E-1+t19*t23*t33*2.88088419477337E-1+t22*t23*t33*4.997231032719934E-1;
        N(2) = t6*2.69507168036979E-1-t25*2.448643206032768+t26*2.34295976006465+t28*2.34295976006465-t31*2.448643206032768+t33*1.347535840184895E-1-q3*t6*1.575623045498661-q3*t33*7.878115227493304E-1+t6*t38*1.49916930981598E-1+t16*t33*4.161951138079923E-1+t18*t33*7.495846549079901E-2+t19*t33*9.000790752297579E-2-t21*t33*2.814415297895028E-1-t22*t33*4.049356850373793E-1+t33*t38*7.495846549079901E-2-q3*t16*t33*2.88088419477337E-1-q3*t21*t33*4.997231032719934E-1;

        for (int i = 0; i < 3; i++) {
            if (isnan(N(i)) == 1) {
                N(i) = 0;
            }
        }

    }
}



void PsmForceControl::CalcC(const Eigen::VectorXd &q,const Eigen::VectorXd &qd)
{
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    float qd1 = qd(0);
    float qd2 = qd(1);
    float qd3 = qd(2);

    float t2 = q2*2.0;
    float t3 = q3+t2;
    float t4 = t2-5.816747949334241E-1;
    float t5 = t2-2.908373974667121E-1;
    float t6 = q3*2.0;
    float t7 = -q3+t2;
    float t8 = t2-t6;
    float t9 = t2+t6;
    float t10 = q3-t2+2.908373974667121E-1;
    float t11 = q3+t2-2.908373974667121E-1;
    float t12 = q3-2.908373974667121E-1;
    float t13 = q3+2.908373974667121E-1;
    float t14 = cos(t2);
    float t15 = cos(t6);
    float t16 = sin(t2);
    float t17 = sin(t6);
    float t18 = cos(t7);
    float t19 = cos(t8);
    float t20 = cos(t9);
    float t21 = cos(t4);
    float t22 = sin(t7);
    float t23 = sin(t8);
    float t24 = sin(t9);
    float t25 = cos(t10);
    float t26 = cos(t11);
    float t27 = sin(t10);
    float t28 = sin(t11);
    float t29 = cos(t12);
    float  t30 = cos(t13);
    float t31 = q2-5.816747949334241E-1;
    float t32 = sin(t12);
    float  t33 = sin(t13);
    float  t34 = cos(q3);
    float  t35 = sin(q3);
    float  t36 = q3*q3;
    float  t37 = cos(t3);
    float  t38 = cos(t5);
    float  t39 = sin(t3);
    float  t40 = sin(t4);
    float  t41 = sin(t5);
    float  t42 = q2-q3;
    float  t43 = q2-t6;
    float  t44 = q2+t6;
    float  t45 = q2+q3-2.908373974667121E-1;
    float  t46 = -q2+q3+2.908373974667121E-1;
    float  t47 = q2+q3;
    float  t48 = q2-2.908373974667121E-1;
    float  t49 = cos(t46);
    float  t50 = sin(t46);
    float  t51 = cos(t47);
    float  t52 = cos(t48);
    float  t53 = sin(t47);
    float  t54 = sin(t48);
    float  t55 = sin(t31);
    float  t56 = cos(q2);
    float  t57 = sin(q2);
    float  t58 = cos(t42);
    float  t59 = cos(t43);
    float  t60 = cos(t44);
    float  t61 = sin(t42);
    float  t62 = sin(t43);
    float  t63 = sin(t44);
    float  t64 = cos(t45);
    float  t65 = sin(t45);
    float  t66 = t49*1.882831077710977E-4;
    float  t67 = t50*8.929284668366714E-4;
    float  t68 = t51*2.187023801023081E-3;
    float  t69 = qd1*t20*9.641693446251764E-3;
    float  t70 = qd2*t60*2.382719144631059E-1;
    float  t71 = qd2*t62*1.928338689250353E-2;
    float  t72 = qd2*t63*1.928338689250353E-2;
    float  t73 = cos(2.908373974667121E-1);
    float  t74 = qd1*t16*4.659634886359594E-1;
    float  t75 = sin(2.908373974667121E-1);
    float  t76 = qd3*t56*5.811329365550027E-1;
    float t77 = qd1*t57*2.825070207040501E-1;
    float t78 = sin(5.816747949334241E-1);
    float t79 = cos(5.816747949334241E-1);
    float t80 = t56*t56;
    float t81 = t73*t73;
    float t82 = t34*t34;
    float t83 = qd2*t17*4.765438289262118E-1;
    float t84 = t35*t73*3.765662155421955E-4;
    float t85 = q3*qd2*3.547726451423457E-2;
    float t86 = qd2*t34*2.047782617278766E-3;
    float t87 = qd1*t34*t57*1.954397302546342E-2;
    float t88 = qd2*t35*t75*1.506264862168782E-3;
    float t89 = qd1*t34*t56*t75*1.506264862168782E-3;
    float t90 = qd1*t35*t56*t75*7.143427734693371E-3;
    C(0,0) = q3*5.641684399813122E-2-qd3*1.128336879962624E-1+t14*1.164908721589898E-1-t15*5.956797861577647E-2+t16*3.78435941017915E-3-t17*4.820846723125882E-3+t18*4.465001762795682E-2-t19*2.978398930788824E-2-t20*2.978398930788824E-2-t21*2.014865689785166E-1-t22*2.698969455342773E-3+t23*2.410423361562941E-3-t24*2.410423361562941E-3+t25*8.929284668366714E-4-t26*8.929284668366714E-4-t27*1.882831077710977E-4+t28*1.882831077710977E-4+t29*8.929284668366714E-4-t30*8.929284668366714E-4-t32*1.882831077710977E-4+t33*1.882831077710977E-4-t34*1.079489817887627E-4-t35*5.119456543196916E-4-t36*4.434658064279322E-3-t37*4.454206864616806E-2-t38*1.32012537564389E-2-t39*2.187023801023081E-3+t40*8.799769460551694E-2+t41*1.862739453565847E-3+t56*1.412535103520251E-1-t73*1.32012537564389E-2+t75*1.862739453565847E-3-t78*7.206792799878022E-2+cos(t31)*1.412535103520251E-1+q3*qd3*1.773863225711729E-2+q3*t21*5.641684399813122E-2-q3*t40*1.979388287369336E-2-qd2*t14*1.51374376407166E-2+qd2*t16*4.659634886359594E-1+qd3*t15*1.928338689250353E-2+qd2*t18*1.079587782137109E-2-qd3*t17*2.382719144631059E-1-qd2*t19*9.641693446251764E-3-qd3*t18*5.397938910685546E-3+qd2*t20*9.641693446251764E-3+qd3*t19*9.641693446251764E-3-qd2*t21*3.519907784220678E-1+qd3*t20*9.641693446251764E-3+qd2*t22*1.786000705118273E-1-qd3*t21*1.128336879962624E-1-qd2*t23*1.191359572315529E-1-qd3*t22*8.930003525591364E-2-qd2*t24*1.191359572315529E-1+qd3*t23*1.191359572315529E-1-qd2*t25*7.53132431084391E-4-qd3*t24*1.191359572315529E-1-qd2*t26*7.53132431084391E-4+qd3*t25*3.765662155421955E-4-qd2*t27*3.571713867346686E-3-qd3*t26*3.765662155421955E-4-qd2*t28*3.571713867346686E-3+qd3*t27*1.785856933673343E-3-qd3*t28*1.785856933673343E-3+qd3*t29*3.765662155421955E-4-qd3*t30*3.765662155421955E-4+qd3*t32*1.785856933673343E-3-qd3*t33*1.785856933673343E-3+qd3*t34*1.023891308639383E-3-qd3*t35*2.158979635775254E-4+qd2*t37*8.748095204092325E-3-qd2*t38*7.450957814263388E-3+qd3*t37*4.374047602046162E-3-qd2*t39*1.781682745846722E-1-qd2*t40*8.059462759140663E-1-qd3*t39*8.908413729233612E-2-qd2*t41*5.280501502575559E-2+qd3*t40*3.958776574738673E-2+qd2*t55*2.825070207040501E-1+qd2*t57*2.825070207040501E-1-t21*t36*4.434658064279322E-3+q3*qd2*t21*7.917553149477345E-2+q3*qd3*t21*1.773863225711729E-2+q3*qd2*t40*2.256673759925249E-1-qd2*t36*t40*1.773863225711729E-2-3.323437414950833E-1;
    C(0,1) = t52*4.915806880307994E-2-t53*4.454206864616806E-2+t54*1.320771962680341E-1-t58*2.698969455342773E-3+t59*4.820846723125882E-3+t60*4.820846723125882E-3-t61*4.465001762795682E-2+t62*5.956797861577647E-2-t63*5.956797861577647E-2-t64*1.882831077710977E-4-t65*8.929284668366714E-4+t66+t67+t68+t69+t70+t71+t72+t74+t76+t77+q3*t54*1.266821036021037E-1-qd1*t14*1.51374376407166E-2+qd1*t18*1.079587782137109E-2-qd1*t19*9.641693446251764E-3-qd1*t21*3.519907784220678E-1+qd1*t22*1.786000705118273E-1-qd1*t23*1.191359572315529E-1-qd1*t24*1.191359572315529E-1-qd1*t25*7.53132431084391E-4-qd1*t26*7.53132431084391E-4-qd1*t27*3.571713867346686E-3-qd1*t28*3.571713867346686E-3+qd1*t37*8.748095204092325E-3-qd1*t38*7.450957814263388E-3-qd1*t39*1.781682745846722E-1-qd1*t40*8.059462759140663E-1-qd1*t41*5.280501502575559E-2+qd2*t49*3.571713867346686E-3-qd2*t50*7.53132431084391E-4+qd2*t51*1.781682745846722E-1-qd2*t52*5.283087850721363E-1+qd2*t53*8.748095204092325E-3+qd1*t55*2.825070207040501E-1+qd2*t54*1.966322752123198E-1-qd3*t54*5.06728414408415E-1+qd2*t58*1.786000705118273E-1-qd2*t59*2.382719144631059E-1+qd3*t59*2.382719144631059E-1-qd2*t61*1.079587782137109E-2+qd3*t60*2.382719144631059E-1-qd3*t62*1.928338689250353E-2+qd2*t64*3.571713867346686E-3+qd3*t63*1.928338689250353E-2-qd2*t65*7.53132431084391E-4+q3*qd1*t21*7.917553149477345E-2+q3*qd1*t40*2.256673759925249E-1-q3*qd2*t52*5.06728414408415E-1-qd1*t36*t40*1.773863225711729E-2-1.374393583700811E-2;
    C(0,2) = qd1*(-1.128336879962624E-1)-t52*1.266821036021037E-1+t53*4.454206864616806E-2-t57*2.905664682775014E-1-t58*2.698969455342773E-3-t61*4.465001762795682E-2+t64*1.882831077710977E-4+t65*8.929284668366714E-4+t66+t67-t68+t69+t70-t71+t72+q3*qd1*1.773863225711729E-2+qd1*t15*1.928338689250353E-2-qd1*t17*2.382719144631059E-1-qd1*t18*5.397938910685546E-3+qd1*t19*9.641693446251764E-3-qd1*t21*1.128336879962624E-1-qd1*t22*8.930003525591364E-2+qd1*t23*1.191359572315529E-1-qd1*t24*1.191359572315529E-1+qd1*t25*3.765662155421955E-4-qd1*t26*3.765662155421955E-4+qd1*t27*1.785856933673343E-3-qd1*t28*1.785856933673343E-3+qd1*t29*3.765662155421955E-4-qd1*t30*3.765662155421955E-4+qd1*t32*1.785856933673343E-3-qd1*t33*1.785856933673343E-3+qd1*t34*1.023891308639383E-3-qd1*t35*2.158979635775254E-4+qd1*t37*4.374047602046162E-3-qd1*t39*8.908413729233612E-2+qd1*t40*3.958776574738673E-2-qd3*t49*3.571713867346686E-3+qd3*t50*7.53132431084391E-4-qd3*t51*1.781682745846722E-1-qd2*t54*5.06728414408415E-1-qd3*t53*8.748095204092325E-3+qd2*t56*5.811329365550027E-1+qd2*t59*2.382719144631059E-1-qd3*t58*1.786000705118273E-1+qd3*t61*1.079587782137109E-2-qd3*t64*3.571713867346686E-3+qd3*t65*7.53132431084391E-4+q3*qd1*t21*1.773863225711729E-2;
    C(1,0) = -t74-t76-t77+qd1*t14*1.51374376407166E-2+t15*t56*9.641693446251764E-3-t17*t56*1.191359572315529E-1-t34*t56*5.119456543196916E-4-t34*t57*8.919208627412488E-2+t35*t56*1.079489817887627E-4-t35*t57*4.885993256365854E-3+t56*t73*4.915806880307994E-2+t57*t73*1.320771962680341E-1-t56*t75*1.320771962680341E-1+t57*t75*4.915806880307994E-2+q3*t57*t73*1.266821036021037E-1-q3*t56*t75*1.266821036021037E-1+qd1*t15*t16*2.382719144631059E-1+qd1*t16*t17*1.928338689250353E-2-qd1*t14*t34*1.954397302546342E-2+qd1*t14*t35*3.567683450964995E-1-qd1*t16*t34*4.317959271550508E-4-qd1*t16*t35*2.047782617278766E-3+qd3*t15*t56*4.765438289262118E-1+qd3*t17*t56*3.856677378500706E-2+qd1*t14*t73*7.450957814263388E-3-qd1*t14*t75*5.280501502575559E-2+qd1*t16*t73*5.280501502575559E-2+qd1*t16*t75*7.450957814263388E-3-qd1*t14*t78*8.059462759140663E-1-qd3*t34*t56*4.317959271550508E-4+qd1*t14*t79*3.519907784220678E-1+qd3*t34*t57*1.954397302546342E-2-qd3*t35*t56*2.047782617278766E-3+qd1*t16*t78*3.519907784220678E-1-qd3*t35*t57*3.567683450964995E-1+qd1*t16*t79*8.059462759140663E-1+qd1*t56*t78*2.825070207040501E-1-qd1*t57*t79*2.825070207040501E-1-t34*t57*t73*1.785856933673343E-3+t34*t56*t75*1.785856933673343E-3+t35*t57*t73*3.765662155421955E-4-t35*t56*t75*3.765662155421955E-4+q3*qd1*t14*t78*2.256673759925249E-1-q3*qd1*t14*t79*7.917553149477345E-2-q3*qd1*t16*t78*7.917553149477345E-2-q3*qd1*t16*t79*2.256673759925249E-1+qd1*t14*t34*t73*1.506264862168782E-3+qd1*t14*t35*t73*7.143427734693371E-3+qd1*t16*t34*t75*1.506264862168782E-3+qd1*t16*t35*t75*7.143427734693371E-3-qd1*t14*t36*t78*1.773863225711729E-2+qd1*t16*t36*t79*1.773863225711729E-2-qd3*t34*t57*t73*1.506264862168782E-3+qd3*t34*t56*t75*1.506264862168782E-3-qd3*t35*t57*t73*7.143427734693371E-3+qd3*t35*t56*t75*7.143427734693371E-3-1.374393583700811E-2;
    C(1,1) = q3*1.128336879962624E-1-qd3*2.256673759925249E-1+t15*1.191359572315529E-1+t17*9.641693446251764E-3-t34*2.158979635775254E-4-t35*1.023891308639383E-3-t36*8.869316128558643E-3-t73*2.640250751287779E-2+t75*3.725478907131694E-3+q3*qd3*3.547726451423457E-2-qd3*t15*3.856677378500706E-2+qd3*t17*4.765438289262118E-1+qd3*t34*2.047782617278766E-3-qd3*t35*4.317959271550508E-4+t34*t75*7.53132431084391E-4+t35*t75*3.571713867346686E-3-qd3*t34*t75*7.143427734693371E-3+qd3*t35*t75*1.506264862168782E-3-4.711246955561407E-1;
    C(1,2) = qd2*(-2.256673759925249E-1)-t34*8.919208627412488E-2-t35*4.885993256365854E-3+t83+t84+t85+t86+t87+t88+t89+t90-qd2*t15*3.856677378500706E-2-qd2*t35*4.317959271550508E-4+qd3*t34*1.954397302546342E-2-qd3*t35*3.567683450964995E-1-qd1*t56*5.811329365550027E-1-t34*t73*1.785856933673343E-3+qd1*t15*t56*4.765438289262118E-1+qd1*t17*t56*3.856677378500706E-2-qd1*t34*t56*4.317959271550508E-4-qd1*t35*t56*2.047782617278766E-3-qd1*t35*t57*3.567683450964995E-1-qd3*t34*t73*1.506264862168782E-3-qd2*t34*t75*7.143427734693371E-3-qd3*t35*t73*7.143427734693371E-3-qd1*t34*t57*t73*1.506264862168782E-3-qd1*t35*t57*t73*7.143427734693371E-3-1.979388287369336E-2;
    C(2,0) = qd1*2.256673759925249E-1-t57*2.905664682775014E-1-q3*qd1*3.547726451423457E-2+qd1*t16*3.958776574738673E-2-qd1*t34*2.047782617278766E-3+qd1*t35*4.317959271550508E-4+qd2*t56*1.057676765481214-qd1*t78*3.958776574738673E-2-qd1*t80*1.871006022075178E-1-qd1*t81*2.256673759925249E-1-t34*t56*4.885993256365854E-3-t34*t57*1.079489817887627E-4+t35*t56*8.919208627412488E-2-t35*t57*5.119456543196916E-4-t56*t73*1.266821036021037E-1-t57*t75*1.266821036021037E-1+q3*qd1*t80*3.547726451423457E-2+q3*qd1*t81*3.547726451423457E-2+qd2*t34*t56*4.317959271550508E-4-qd2*t34*t57*1.954397302546342E-2+qd2*t35*t56*2.047782617278766E-3+qd2*t35*t57*3.567683450964995E-1+qd1*t34*t75*7.143427734693371E-3-qd1*t35*t75*1.506264862168782E-3+qd1*t34*t80*2.047782617278766E-3-qd1*t35*t80*4.317959271550508E-4-qd2*t56*t82*9.530876578524235E-1+qd1*t80*t81*4.513347519850498E-1-qd1*t80*t82*7.713354757001412E-2+t34*t56*t73*3.765662155421955E-4+t35*t56*t73*1.785856933673343E-3+t34*t57*t75*3.765662155421955E-4+t35*t57*t75*1.785856933673343E-3-q3*qd1*t80*t81*7.095452902846915E-2-qd2*t34*t35*t56*7.713354757001412E-2+qd1*t34*t56*t57*3.567683450964995E-1+qd1*t35*t56*t57*1.954397302546342E-2+qd1*t34*t35*t80*9.530876578524235E-1+qd2*t34*t57*t73*1.506264862168782E-3-qd2*t34*t56*t75*1.506264862168782E-3+qd2*t35*t57*t73*7.143427734693371E-3-qd2*t35*t56*t75*7.143427734693371E-3-qd1*t34*t75*t80*7.143427734693371E-3+qd1*t35*t75*t80*1.506264862168782E-3-qd1*t56*t57*t81*1.583510629895469E-1+qd1*t73*t75*t80*1.583510629895469E-1+qd1*t34*t56*t57*t73*7.143427734693371E-3-qd1*t35*t56*t57*t73*1.506264862168782E-3+qd1*t56*t57*t73*t75*4.513347519850498E-1-q3*qd1*t56*t57*t73*t75*7.095452902846915E-2;
    C(2,1) = qd2*1.871006022075178E-1-t34*8.919208627412488E-2-t35*4.885993256365854E-3-t83+t84-t85-t86-t87-t88-t89-t90+qd2*t35*4.317959271550508E-4+qd1*t56*1.057676765481214+qd2*t82*7.713354757001412E-2-t34*t73*1.785856933673343E-3+qd1*t34*t56*4.317959271550508E-4+qd1*t35*t56*2.047782617278766E-3+qd1*t35*t57*3.567683450964995E-1+qd2*t34*t75*7.143427734693371E-3-qd1*t56*t82*9.530876578524235E-1-qd1*t34*t35*t56*7.713354757001412E-2+qd1*t34*t57*t73*1.506264862168782E-3+qd1*t35*t57*t73*7.143427734693371E-3-1.979388287369336E-2;
    C(2,2) = 1.342936099164326;

}
void PsmForceControl::CalcG(const Eigen::VectorXd &q)
{
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);

    float t2 = cos(q1);
    float t3 = sin(q1);
    float t4 = cos(q2);
    float t5 = cos(2.908373974667121E-1);
    float t6 = sin(q2);
    float t7 = sin(q3);
    float t8 = sin(2.908373974667121E-1);
    float t9 = cos(q3);

    G(0) = q1*(-6.789509964297665)+t2*1.430783703993206E-1-t3*1.028155744757573-t3*t4*8.624819120873413E-1+t3*t5*9.228562676332305+t2*t7*2.460232608209011E-2+t3*t6*1.216989776329687E-1-t2*t9*1.166759863333251E-1+t3*t4*t5*1.038316632917131E-1-t3*t5*t6*1.93980052162195E-1+t3*t4*t8*1.93980052162195E-1+t3*t6*t7*1.166759863333251E-1+t3*t6*t8*1.038316632917131E-1+t3*t6*t9*2.460232608209011E-2-q3*t3*t4*t5*8.69192980598747E-2-q3*t3*t6*t8*8.69192980598747E-2;
    G(1) = q2*8.435824140033144E-1-t2*t4*1.216989776329687E-1-t2*t6*8.624819120873413E-1+t2*t4*t5*1.93980052162195E-1-t2*t4*t7*1.166759863333251E-1+t2*t5*t6*1.038316632917131E-1-t2*t4*t8*1.038316632917131E-1-t2*t4*t9*2.460232608209011E-2+t2*t6*t8*1.93980052162195E-1-q3*t2*t5*t6*8.69192980598747E-2+q3*t2*t4*t8*8.69192980598747E-2;
    G(2) = t3*t7*1.166759863333251E-1+t3*t9*2.460232608209011E-2+t2*t4*t5*8.69192980598747E-2+t2*t6*t7*2.460232608209011E-2+t2*t6*t8*8.69192980598747E-2-t2*t6*t9*1.166759863333251E-1;

}

void PsmForceControl::CalcDiffJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &qd)
{
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    float qd1 = qd(0);
    float qd2 = qd(1);
    float qd3 = qd(2);

    float t2 = 3.141592653589793*(1.0/2.0);
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
    Jd(2,2) = -t50-t51-t52+t16*t31;

    // ROS_INFO_STREAM("Jd: "<<Jd);

}
void PsmForceControl::CalcM(const Eigen::VectorXd &q) //Eigen::VectorXd qd)
{   float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);

    if (name == "PSM1")
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
        M(2,2) = 7.878115227493304E-1;

    }

}



PsmForceControl::PsmForceControl(ros::NodeHandle n, const string nam) {

    name = nam;

    desplot_x = n.advertise<std_msgs::Float64>("/d0", 10);
    desplot_y = n.advertise<std_msgs::Float64>("/d1", 10);
    desplot_z = n.advertise<std_msgs::Float64>("/d2", 10);

    plot_x = n.advertise<std_msgs::Float64>("/0", 10);
    plot_y = n.advertise<std_msgs::Float64>("/1", 10);
    plot_z = n.advertise<std_msgs::Float64>("/2", 10);

    joint_pub = n.advertise<sensor_msgs::JointState>("/dvrk/" + name + "/set_effort_joint", 1);

//jacobian_sub=n.subscribe("/dvrk/"+ name + "/jacobian_body", 200, &PsmForceControl::CallbackJacobian,this);
    joint_sub = n.subscribe("/dvrk/" + name + "/state_joint_current", 1, &PsmForceControl::CallbackJoint, this);
    cartesian_sub = n.subscribe("/dvrk/" + name + "/position_cartesian_current", 1, &PsmForceControl::CallbackCartesian,
                                this);

    force_sub = n.subscribe("/psm_sense/" + name + "/tool_forces", 10, &PsmForceControl::CallbackForce, this);
    setforce_sub = n.subscribe("/psm_sense/setforce", 10, &PsmForceControl::CallbackSetForce, this);
    setpos_sub = n.subscribe("/psm/cmd_vel2", 10, &PsmForceControl::CallbackSetPosition, this);
    setpos_sub2 = n.subscribe("/psm/cmd_vel", 10, &PsmForceControl::CallbackSetPositionIncrement, this);

//Joint States and Pub data
    dof = 6;
    cart_dof = 3;
    q.resize(dof);
    qd.resize(dof);
    eff.resize(dof);
    u.resize(cart_dof);
    q0.resize(3);
    joint_act.resize(3), joint_des.resize(3);

//Cartesian States and data
    xe.resize(3);
    xd.resize(3);
    ve.resize(3);
    fd.resize(3);
    he.resize(3);
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

// Initialize to zero
    fd << 0.0, 0.0, 0.0;
    he << 0.0, 0.0, 0.0;

    xd << 0.0, 0.0, 0.0;
    xe << 0.0, 0.0, 0.0;

    x0 << 0.0, 0.0, 0.0;
    q0 << 0.0, 0.0, 0.0;

    q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    u << 0.0, 0.0, 0.0;

    C << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    G << 0, 0, 0;
    Fr << 0, 0, 0;
    N << 0, 0, 0;

    M << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Mt << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Ja << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    JaM << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Jd << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Kd << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Kp << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    deadband << 0.005, 0.01, 0.005;

    interp = false;

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
    float q1 = qd(0);
    float q2 = qd(1);

    float qd1 = x[0];
    float qd2 = x[1];
    float qd3 = x[2];

    float a = 6.0E2;
    float scale = 1;


    float pos_deadband = 0.003; // rad
    float Fs_pos = 0.6;
    float Fs_neg = -0.4;

    //Computed Torque
    //float x_e = xd(2)-q(2);

    //Impedance Controller
        float x_e = joint_des(2) - joint_act(2);
        
    /*


    Fr(0) = q1*6.119063107247842E-1+qd1*5.121004410809419E-2+1.561585962952595E-1/(exp(qd1*-a)+1.0)-7.807929814762977E-2;
    Fr(1) = q2*1.178371077512102+qd2*1.277466080900284E-1+2.91620349820475E-1/(exp(qd2*-a)+1.0)-1.458101749102375E-1;
    Fr(2) = qd3*8.397843687710638E-1+1.117115567283898/(exp(qd3*-a)+1.0)-5.585577836419491E-1;
*/

    Fr(0) =  q1*(-3.374542425099348E-1)+qd1*8.857790114534859E-2+1.330230787728563E-1/(exp(qd1*-4.0E2)+1.0)-6.651153938642816E-2;
    Fr(1) = q2*2.544692215878968+qd2*1.585859192149214E-1+1.935467306471518E-1/(exp(qd2*-6.0E2)+1.0)-9.67733653235759E-2;

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
        Fr(2) = qd3*5.3115714926966E-1+9.158363928863775E-1/(exp(qd3*-4.0E2)+1.0)-4.579181964431888E-1;
    }


    Fr(2) = scale* Fr(2);
}

void PsmForceControl::SetGainsInit()
{
    // Original MT.diagonal of real robot
    //Mt.diagonal()<<0.3, 0.4, 0.5;

    Mt.diagonal()<<0.35, 0.36, 0.3;

    //Real Coefficients
    Kp.diagonal() << 70, 100, 300;
    Kd.diagonal() << 10, 7, 15;

    //Wrist Coefficients;
    wrist_kp << 0.5, 0.5, 0.5;
    wrist_kd <<  0.5,  0.5,  0.5;

    //Test Damping
    //Kp.diagonal()<<1, 1, 3;
    //Kd.diagonal()<<5, 0.5, 10;
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

    drop = msg.header.seq-drop_p;
    drop_p = msg.header.seq;
}

void PsmForceControl::CallbackCartesian(const geometry_msgs::PoseStamped &msg)
{
    xe(0) = msg.pose.position.y;  // y becomes x in my calculation
    xe(1) =-msg.pose.position.x; // x becomes -y in my calculation
    xe(2) = msg.pose.position.z;

}

 void PsmForceControl::CallbackForce(const geometry_msgs::Wrench &msg)
 {
     he(0) = msg.force.x;
     he(1) = msg.force.y;
     he(2) = msg.force.z;
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
     double arr[3];

     arr[0] = msg.linear.x;
     arr[1] = msg.linear.y;
     arr[2] = msg.linear.z;

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


         if (arr[i] != 0)
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

    int fl = 2; //force limit
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

        //ROS_INFO_STREAM("v_int:" << v_int);

        // Impedance Controller
        y = JaM.inverse()*Mt.inverse()*(Mt*a_int+Kd*(v_int-ve)+Kp*(x_int-xe)-Mt*Jd*qd-he);

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

        fl = fl*3;

    }
    else
    {
        // Impedance Controller
        y = JaM.inverse()*Mt.inverse()*(Mt*ad+Kd*(vd-ve)+Kp*(xd-xe)-Mt*Jd*qd-he);

        // Computed Torque controller
        //y =  Kd*(vd-qd) + Kp*(xd-q);
    }

    u = M*y + N +Fr +JaM.transpose()*he;
     
/*
     ROS_INFO_STREAM("u_steady 1: "<< u(0));
     ROS_INFO_STREAM("u_steady 2: "<< u(1));
     ROS_INFO_STREAM("u_steady 3: "<< u(2));*/

// ///// SAFETY ///////
    if (std::abs(u(0))>fl|std::abs(u(1))>fl|std::abs(u(2))>fl*2.5)
    {
        u<< 0, 0, 0;
    }
    //ROS_INFO_STREAM("  xd: "<< xd << endl <<" xe[]: " << q << endl);

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
    for (int i=0;i<3;i++)
    {
        wrist_eq(i)= 0.1 - q(i+3);
        wrist_eqd(i) = 0.0 - qd(i+3);
    }

    for (int i=0;i<3;i++)
    {
        wrist_u(i) = wrist_eq(i) * wrist_kp(i) + wrist_eqd(i) * wrist_kd(i);
        //ROS_INFO_STREAM("  wrist_u: "<< wrist_u<< endl<< "wrist_eq: "<< wrist_eq << endl<< "wrist_eqd: "<< wrist_eqd<<endl);

        //wrist_u(i) = 0;
    }
}
void PsmForceControl::output()
 {
     for (int i=0;i<3;i++)
     {
         joint_msg.effort[i] = u(i);
         if(i>=3)
         {
             joint_msg.effort[i] = wrist_u(i-3);
             //ROS_INFO_STREAM(endl<<joint_msg.effort[i]);
         }

     }


  // ----------------------- IMPORTANT---This runs Robot-----------------
 joint_pub.publish(joint_msg);

  // ------------------------------------------------------
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
/*    dq0.data = joint_des(0);
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
    dq0.data = xd(0);
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
    plot_z.publish(mq2);


 }

void  PsmForceControl::Loop()
 {
     this->joint_act = this->InverseKinematic(xe);
     this->joint_des = this->InverseKinematic(xd);
     this->CalcJaM(q, qd);
     this->CalcDiffJacobian(q, qd);
     this->CalcFr(q, qd);
     this->CalcN(q, qd);
     this->CalcM(q);
     this->CalcU();
     this->WristPID();
     this->output();
 }

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
