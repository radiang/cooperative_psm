#include "force_control.h"

void PsmForceControl::SetGainsInit()
{
 M.diagonal()<<100, 100,100;
 Kp.diagonal()<<300,400, 100;
 Kd.diagonal()<<100,50,30;
 Cp.diagonal()<<40,40,40;
 Ci.diagonal()<<1,1,1;

}

void PsmForceControl::SetDesiredInit()
{
 x0 = xe;
 q0 = q;

 double incre [3] = {0.002,0, 0};
 fd << 0, 0, 0;
 xd<< incre[0]+x0(0),incre[1]+x0(1) ,incre[2]+x0(2) ;

 vd << 0, 0, 0;
 ad << 0, 0, 0;

 ROS_INFO_STREAM("x0: "<< x0<< "   xd: "<< xd);
}


void PsmForceControl::callback_jacobian(std_msgs::Float64MultiArray msg)
{
  
    //Jmin = Ja;

for (int i=0;i<3;i++)
   {
     for (int j=0;j<3;j++)
 {
  Ja(i,j)=msg.data[i*6+j];
     }
  }
  //ROS_INFO_STREAM("Jacobian"<<Ja);

}


void PsmForceControl::callback_joint(sensor_msgs::JointState msg)
{
    for (int i=0;i<3;i++)
    {
    q(i)=msg.position[i];
    qd(i)=msg.velocity[i];
    eff(i)=msg.effort[i];
    }

  //ROS_INFO_STREAM("joint_states"<<q);
}

void PsmForceControl::callback_cartesian(geometry_msgs::PoseStamped msg)
{
    xe(0)=msg.pose.position.x;
    xe(1)=msg.pose.position.y;
    xe(2)=msg.pose.position.z;
    //ROS_INFO_STREAM("cartesian"<<xe);

}

 void PsmForceControl::callback_force(geometry_msgs::Wrench msg)
 {
     he(0) = msg.force.x;
     he(1) = msg.force.y;
     he(2) = msg.force.z;
 }
 void PsmForceControl::CallbackSetForce(geometry_msgs::Pose msg)
 {
    fd(0)=msg.position.x;
    fd(1)=msg.position.y;
    fd(2)=msg.position.z;

 }

 void PsmForceControl::CallbackSetPosition(geometry_msgs::Pose msg)
 {
    xd(0)=msg.position.x;
    xd(1)=msg.position.y;
    xd(2)=msg.position.z;
 }

 void PsmForceControl::CallbackSetPositionIncrement(geometry_msgs::Pose msg)
 {
     xd(0)=msg.position.x+xe(0);
     xd(1)=msg.position.y+xe(1);
     xd(2)=msg.position.z+xe(2);
 }
 void PsmForceControl::CalcN(Eigen::VectorXd q, Eigen::VectorXd qd)
 {

   float q1 = q(0);
   float q2 = q(1);
   float q3 = q(2);
   float qd1 = qd(0);
   float qd2 = qd(1);
   float qd3 = qd(2);

   float  t2 = q1-q2;
   float  t3 = q1+q2;
   float  t4 = q1-q2+2.908373974667121E-1;
   float  t5 = sin(t4);
   float  t6 = sin(t3);
   float  t7 = sin(t2);
   float  t8 = qd2*qd2;
   float  t9 = q1+q2-2.908373974667121E-1;
   float  t10 = sin(t9);
   float  t11 = q2*2.0;
   float  t12 = t11-2.908373974667121E-1;
   float  t13 = cos(t12);
   float  t14 = t11-5.816747949334241E-1;
   float  t15 = cos(t11);
   float  t16 = sin(t11);
   float  t17 = sin(t14);
   float  t18 = q3*q3;
   float  t19 = cos(t14);
   float  t20 = sin(t12);
   float  t21 = cos(t2);
   float  t22 = t7*1.666242411930973E-1;
   float  t23 = cos(t3);
   float  t24 = t23*1.597671686020965E-2;
   float  t25 = t6*1.666242411930973E-1;
   float  t26 = qd1*qd1;
   float  t27 = q3*t5*2.752628805758386E-1;
   float  t28 = q3*t10*2.752628805758386E-1;
   float  t29 = cos(2.908373974667121E-1);

   N(0) = q1*(-1.34790240657964E-1)-qd1*9.696753713450551E-3-t5*1.375111307286038E-3-t10*1.375111307286038E-3-t21*1.597671686020965E-2+t22+t24+t25+t27+t28-sin(q1)*2.204309194240241E-1-((qd1/fabs(qd1)))*1.230394239122956E-1+qd1*qd3*2.156904135054586E-1-q3*t6*3.715515548885964E-1-q3*t7*3.715515548885964E-1-t8*sin(q2-2.908373974667121E-1)*7.123681747559926E-2+t8*cos(q2)*3.885409334554282E-4-q3*qd1*qd3*2.640058920263E-1+qd1*qd2*t13*1.956332676760365E-3+qd1*qd3*t13*2.274805438093448E-2+qd1*qd2*t15*1.657341756215823E-2+qd1*qd2*t16*1.681334101616743E-1+qd1*qd3*t15*1.927141576995144E-1+qd1*qd2*t17*9.720258457989834E-2-qd1*qd3*t16*6.52110892253455E-3+qd1*qd2*t20*2.040296830935885E-2+qd1*qd3*t19*2.297625580594423E-2+qd1*qd3*t29*2.274805438093448E-2-q3*qd1*qd2*t15*1.30422178450691E-2-q3*qd1*qd2*t16*3.854283153990287E-1-q3*qd1*qd3*t15*1.516536958728965E-1-q3*qd1*qd2*t17*4.595251161188846E-2-q3*qd1*qd2*t20*4.549610876186895E-2-q3*qd1*qd3*t19*1.123521961534035E-1+qd1*qd2*t16*t18*1.516536958728965E-1+qd1*qd2*t17*t18*1.123521961534035E-1;
   N(1) = q2*(-5.296329561725754E-1)-qd2*6.305689528551164E-2+t5*1.375111307286038E-3-t10*1.375111307286038E-3+t21*1.597671686020965E-2-t22+t24+t25-t27+t28-((qd2/fabs(qd2)))*1.848345018648948E-1+qd2*qd3*4.313808270109172E-1-q3*t6*3.715515548885964E-1+q3*t7*3.715515548885964E-1-t13*t26*9.781663383801825E-4-t15*t26*8.286708781079117E-3-t16*t26*8.406670508083713E-2-t17*t26*4.860129228994917E-2-t20*t26*1.020148415467943E-2-q3*qd2*qd3*5.280117840526001E-1+qd2*qd3*t29*4.549610876186895E-2+q3*t15*t26*6.52110892253455E-3+q3*t16*t26*1.927141576995144E-1+q3*t17*t26*2.297625580594423E-2+q3*t20*t26*2.274805438093448E-2-t16*t18*t26*7.582684793644826E-2-t17*t18*t26*5.617609807670176E-2;
   N(2) = qd3*(-1.251106427885117)-t8*2.156904135054586E-1+t21*3.715515548885964E-1+t23*3.715515548885964E-1-t26*1.078452067527293E-1-cos(t4)*2.752628805758386E-1-cos(t9)*2.752628805758386E-1-((qd3/fabs(qd3)))*3.717912911946995E-1+q3*t8*2.640058920263E-1+q3*t26*1.3200294601315E-1-t8*t29*2.274805438093448E-2-t13*t26*1.137402719046724E-2-t15*t26*9.635707884975718E-2+t16*t26*3.260554461267275E-3-t19*t26*1.148812790297212E-2-t26*t29*1.137402719046724E-2+q3*t15*t26*7.582684793644826E-2+q3*t19*t26*5.617609807670176E-2;

   for (int i=0;i<3;i++)
   {
       if (isnan(N(i))==1)
       {
           N(i)=0;
       }
   }
  // ROS_INFO_STREAM("N: "<<N); //only works if robot is moving
 }


 void PsmForceControl::CalcDiffJacobian(Eigen::VectorXd q, Eigen::VectorXd qd)
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
void PsmForceControl::CalcM(Eigen::VectorXd q) //Eigen::VectorXd qd)
{

    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    //float qd1 = qd(1);
    //float qd2 = qd(2);
    //float qd3 = qd(3);

    float t2 = q2*2.0;
    float t3 = t2-2.908373974667121E-1;
    float t4 = cos(t2);
    float t5 = sin(t2);
    float t6 = t2-5.816747949334241E-1;
    float t7 = cos(t6);
    float t8 = q3*q3;
    float t9 = cos(2.908373974667121E-1);
    float t10 = cos(t3);
    float t11 = q2-2.908373974667121E-1;
    float t12 = cos(t11);
    float t13 = sin(q2);
    float t14 = t12*(-3.561840873779963E-2)-t13*1.942704667277141E-4+7.268090711841164E-2;
    float t15 = sin(2.908373974667121E-1);
    float t16 = t15*(-1.137402719046724E-2)+3.260554461267275E-3;

    M(0,0) = q3*(-1.078452067527293E-1)+t4*4.203335254041857E-2-t5*4.143354390539559E-3+t7*2.430064614497458E-2+t8*6.600147300657501E-2+t9*5.100742077339714E-3+t10*5.100742077339714E-3-t15*4.890831691900912E-4-sin(t3)*4.890831691900912E-4-q3*t4*9.635707884975718E-2+q3*t5*3.260554461267275E-3-q3*t7*1.148812790297212E-2-q3*t9*1.137402719046724E-2-q3*t10*1.137402719046724E-2+t4*t8*3.791342396822413E-2+t7*t8*2.808804903835088E-2+2.575886940496592;
    M(0,1) = t14;
    M(1,0) = t14;
    M(1,1) = q3*(-2.156904135054586E-1)+t8*1.3200294601315E-1+t9*1.020148415467943E-2-t15*9.781663383801825E-4-q3*t9*2.274805438093448E-2+3.473111134750065;
    M(1,2) = t16;
    M(2,1) = t16;
    M(2,2) = 1.3200294601315E-1;

    //ROS_INFO_STREAM("M: "<< M);
}

void PsmForceControl::CalcU()
 {    // This is parallel/position/force

//     xf = Cp*(fd-he);
//     ve = Ja*qd;
//     y = Ja.inverse()*M.inverse()*(-Kd*xe_v+Kp*((xd-xe)+xf)-M*Jd*qd);
//     u = M*y+N+Ja.transpose()*he;
      //this->interpolate();

       ve = Ja*qd;
       y = Ja.inverse()*M.inverse()*(M*ad+Kd*(vd-ve)+Kp*(xd-xe)-M*Jd*qd-he);
       u = M*y + Ja.transpose()*he;
       ROS_INFO_STREAM("\nu: "<< u);
       ROS_INFO_STREAM("x increment: "<< xd - xe);
       //ROS_INFO_STREAM("M inverse: "<< M.inverse());
       //ROS_INFO_STREAM("he: "<<(M*ad+Kd*(vd-ve)+Kp*(xd-xe)-M*Jd*qd-he));
       //ROS_INFO_STREAM("KP: "<<Kp*(xd-xe));
       //ROS_INFO_STREAM("Kd: "<<Kd*(vd-ve));
       //ROS_INFO_STREAM("ve: "<< ve<< "vd"<< vd << "Kd: "<<Kd );
 }

void PsmForceControl::Interpolate(Eigen::VectorXd x0, Eigen::VectorXd xf)
{
 int a = 0 ;
}

void PsmForceControl::output()
 {
   std::cout<<u;

  joint_msg.effort[0]=u(0);
  joint_msg.effort[1]=u(1);
  joint_msg.effort[2]=u(2);
  joint_pub.publish(joint_msg);


   msg2.data = Jd(0,0);
   plot_x.publish(msg2);

 }

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "PsmForceControl_node");
  ros::NodeHandle n;
 
  std_msgs::String mes;
  std::stringstream ss;
  ss<<"it starteed";
  mes.data = ss.str();
  ROS_INFO("%s", mes.data.c_str());

  PsmForceControl obj(n);
  // int i, j;
  int count = 0;
  ros::Duration(0.1).sleep();
  ros::spinOnce();
  obj.SetGainsInit();
  obj.SetDesiredInit();

  std_msgs::Float64 msg;
  // char link[100];

  ROS_INFO("%s", mes.data.c_str());

  ros::Rate r(3);

  while(ros::ok())
  {
  obj.CalcN(obj.q,obj.qd);
  obj.CalcDiffJacobian(obj.q,obj.qd);
  obj.CalcM(obj.q);
  obj.CalcU();
  //obj.output();
  //count=count+1;
  r.sleep();
  ros::spinOnce();
  }
ros::spin();

}
