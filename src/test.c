#include<stdio.h>

int main()
{
    int wyq;
    int mwm;
    printf("请输入wyq和mwm的值：\n");
    scanf("%d %d", &wyq, &mwm);
    printf("(wyq+mwm)/2 = %f", (double)(wyq+mwm)/2);
    getchar();
    getchar();

    return 0;
}