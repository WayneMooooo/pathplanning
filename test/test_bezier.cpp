#include "Bezier.h"
#include"Bezier.cpp"
void multiline(const Point* a, int n){
    for (int i = 0;i < n;i++) {
        fillcircle(a[i].x, a[i].y, 3);//画出控制点的位置
    }
    for(int i = 0;i<n-1;i++)
        line(a[i].x, a[i].y,a[i+1].x, a[i+1].y);
}
int main()
{
    initgraph(1000,800);  // 创建画布
    setfillcolor(WHITE);//设置画笔颜色、填充颜色
    setlinecolor(WHITE);

    Point a[] = { {100,200},{350,600},{600,200},{850,600} };
    int len = sizeof(a) / sizeof(a[0]);

    multiline(a,4);//画出控制点及其连线
    
    setfillcolor(RED);//设置画笔颜色、填充颜色
    setlinecolor(RED);
    
    Bezier B(a, 4); 
    B.SolveTrajectory();//计算bezier曲线
    B.draw_path();

    _getch();        // Press any key to continue
    closegraph();      // Close the graphics window
    return 0;
}