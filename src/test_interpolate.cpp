#include "Interpolate.h"


int main(int argc, char **argv)
{
 Eigen::VectorXd x_arr;
 Eigen::VectorXd q0f;

 traject des;

 des.qd.resize(6);
 des.qd<< 0, 0, 0, 0.4, 0, 0; //q0, qd0, qdd0, qf, qdf, qddf
 des.ts = 0.1;
 des.tf = 2;
 des.check = false;

 des= interpolate(des);


 //std:: cout<< t;
 if (des.check==true) {
  std::cout <<des.a;
  std::cout<<"\n size is: "<<des.x.size();
 }
  else{
  //std::cout << x_arr(1);
 }
}