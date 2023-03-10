#include <iostream>
#include <random>
#include <ctime>

using namespace std;
int main()
{
    uniform_real_distribution<double> u(10, 15);
    default_random_engine e(time(NULL));
    cout << u(e) << endl;
    system("pause");
}