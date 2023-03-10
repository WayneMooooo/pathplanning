#include"Bezier.h"

int fact(int n);

void Bezier::SolveTrajectory(){//计算bezier曲线 pts为控制点序列
  n=n-1;
  for (double t = 0;t < 1;t += 0.002) {
  double Bt = 0;
  double Xt = 0;
  double Yt = 0;
  for (int i = 0;i <= n; i++) {
    Bt = fact(n)/(fact(n-i)*fact(i))*pow((1-t),(n-i))*pow(t,i);//计算伯恩斯坦基底
    Xt += Bt * (pts[i].x);
    Yt += Bt * (pts[i].y);
  }
  Point q({Xt, Yt});
  path_sequence.push_back(q);
  //fillcircle(Xt, Yt, 1);
  }
}
void Bezier::draw_path(){
  for(auto point : path_sequence)
    fillcircle(point.x,point.y,1);
}

int fact(int n) {//阶乘函数
  if (n == 0) {
    return 1;
  }
  else {
    int res = 1;
    for (int i = n;i > 0;i--) {
      res *= i;
    }
    return res;
  }
}