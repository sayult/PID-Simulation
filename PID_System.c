#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define K   1.853    //开环增益系数
/*#define KP  1.00    //比例系数
#define KI  0.2    //积分系数
#define KD  0.8    //微分系数*/
double KP,KI,KD;

#define T_STEP 0.01
#define T_TOTAL_TIME 40
#define EPS 0.02   //调节时间判定阈值(2%误差带)，意思是进入稳态值%2误差带中不再超出的时间定义为调节时间。
#define REF 0.25    /*阶跃输入参考值。可以用来测试PID系统实际的合理性。后续实际应用的时候可以将这一个数值改为对应的实际值。例如：
想让小球在距离一维板子边缘两个单位停下，将REF改为2即可。*/
#define INTEGRAL_MAX 0.1
#define INTEGRAL_MIN -1.0  //定义积分上下限，模拟调节时控制量u转换后的PWM占空比在0-1之间


/*
y-系统当前的实际输出值
dy-实际输出值的变化率
integral_e-偏差的积分累计值
*/
double differential_equation(double y, double dy, double integral_e){
    double e = REF - y;   //e-误差值
    double de = -dy;      //de-偏差变化率
    double u = KP * e + KI * integral_e + KD*de;
    return K*u;           //返回最终输出的控制量
}


int main(int argc, char *argv[]){
    if (argc != 4) { //检查命令行参数数量
        fprintf(stderr,"用法：程序名<KP><KI><KD>\n");
        fprintf(stderr,"示例：./pid_simulation.exe 1.0 0.2 0.8\n");
        return 1;
    }
    //步骤2：解析命令行参数为浮点型PID系数
    KP = atof(argv[1]);
    KI = atof(argv[2]);
    KD = atof(argv[3]);

    if (isnan(KP) || isnan(KI) || isnan(KD)) {
        fprintf(stderr,"error:PID系数必须是数字\n");
        return 1;
    }

    double t = 0.0;  
    double y = 0.0, dy = 0.0;
    double integral_e = 0.0;
    double max_y = 0.0;      //输出最大值
    double y_ss;             //稳态值
    double overshoot = 0.0;  //超调量
    double time_settle = 0.0;//调节时间
    int settle_flag = 0;     //调节时间标记
    int is_stady = 0;        //判断系统是否稳态的标记(有时参数设置不对可能导致仿真时间内一直在振荡)
    int steady_count = 0;    //连续满足稳态条件的步数计数
    double steady_threshold = 0.02 * REF;  //定义%2误差带
    double dy_threshold = 0.01;            //速度阈值，接近0表示无明显运动
    double min_y_late = 1e9;               //仿真后期输出的最小值   仿真后两秒验证系统输出是否为稳态
    double max_y_late = -1e9;              //仿真后期输出的最大值
    double late_start = T_TOTAL_TIME - 2.0;   //后期数据起始时间
    //保存仿真数据到文件
    /*FILE *fp = fopen("C:\\Users\\SAYULT\\Desktop\\PID.txt","w");
    if(fp == NULL){
        printf("Fail to open the file!\n");
        return -1;
    }*/

    //使用四阶龙格-库塔法求解微分方程(RK4)/*这是在仿真时间内基于当前PID控制器输出的控制量
    //数值求解系统微分方程，连续求解系统状态，进而精确反映系统从当前状态向目标状态演变的核心方法。 */
    /*通过4个中间斜率的加权平均逼近真实解*/
    while(t <= T_TOTAL_TIME){
        //计算误差积分
        double e = REF - y;
        integral_e += e * T_STEP;
        //积分饱和限制
        if (integral_e > INTEGRAL_MAX) {
            integral_e = INTEGRAL_MAX;
        }
        if (integral_e < INTEGRAL_MIN) {
            integral_e = INTEGRAL_MIN;
        }
        //龙格-库塔系数
        //计算k1（两个状态量的斜率）
        double k1_v = dy;
        double k1_a = differential_equation(y,dy,integral_e);
        //计算k2（基于k1的半程状态更新）
        double y2 = y + k1_v *T_STEP/2;
        double dy2 = dy + k1_a *T_STEP/2;
        double k2_v = dy2;
        double k2_a = differential_equation(y2,dy2,integral_e);
        //计算k3（基于k2的半程状态更新）
        double y3 = y + k2_v *T_STEP/2;
        double dy3 = dy + k2_a *T_STEP/2;
        double k3_v = dy3;
        double k3_a = differential_equation(y3,dy3,integral_e);
        //计算k4（基于k3的全过程状态更新）
        double y4 = y + k3_v * T_STEP;
        double dy4 = dy + k3_a * T_STEP;
        double k4_v = dy4;
        double k4_a = differential_equation(y4,dy4,integral_e);

        dy += (k1_a + 2 * k2_a + 2 * k3_a + k4_a) * T_STEP/6;
        y += (k1_v + 2 * k2_v + 2 * k3_v + k4_v) * T_STEP/6;



        //记录输出最大值
        if(y > max_y){
            max_y = y;
        }
        if(t >= late_start){
            if(y <= min_y_late){
                min_y_late = y;
            }
            if(y >= max_y_late){
                max_y_late = y;
            }
        }
        //稳态判定：连续满足：1.误差带内2.速度接近于0
        double y_ss_temp = (min_y_late + max_y_late) / 2.0; //暂存当前输出作为参考稳态值
        if(fabs(y - y_ss_temp) <= steady_threshold && fabs(dy) <= dy_threshold){
            steady_count ++;
        }
        else{
            steady_count = 0;
        }
        //连续100步满足条件，判定为稳态
        if(steady_count >= 100){
            is_stady = 1;
        }

        printf("%.6f %.6f\n",t,y);
        t += T_STEP;
    }
    //fclose(fp);


    //计算性能指标：
    y_ss = (min_y_late + max_y_late) / 2.0; //取仿真解除时的系统输出平均值作为稳态值，且要符合稳态条件
    //计算超调量
    if(y_ss != 0){
        overshoot = ((max_y - y_ss)/y_ss * 100.0);
        overshoot = fmax(overshoot,0.0);
    }

    double steady_error = fabs(REF - y_ss); //稳态误差

    //重新遍历并计算调节时间
    t = 0.0;
    y = 0.0;
    dy = 0.0;
    integral_e = 0.0;
    settle_flag = 0;
    while(t <= T_TOTAL_TIME){
        double e = REF-y;
        integral_e += e * T_STEP;
        if (integral_e > INTEGRAL_MAX) {
            integral_e = INTEGRAL_MAX;
        }
        if (integral_e < INTEGRAL_MIN) {
            integral_e = INTEGRAL_MIN;
        }
        double k1_v = dy;
        double k1_a = differential_equation(y,dy,integral_e);
        //计算k2（基于k1的半程状态更新）
        double y2 = y + k1_v *T_STEP/2;
        double dy2 = dy + k1_a *T_STEP/2;
        double k2_v = dy2;
        double k2_a = differential_equation(y2,dy2,integral_e);
        //计算k3（基于k2的半程状态更新）
        double y3 = y + k2_v *T_STEP/2;
        double dy3 = dy + k2_a *T_STEP/2;
        double k3_v = dy3;
        double k3_a = differential_equation(y3,dy3,integral_e);
        //计算k4（基于k3的全过程状态更新）
        double y4 = y + k3_v * T_STEP;
        double dy4 = dy + k3_a * T_STEP;
        double k4_v = dy4;
        double k4_a = differential_equation(y4,dy4,integral_e);

        dy += (k1_a + 2 * k2_a + 2 * k3_a + k4_a) * T_STEP/6;
        y += (k1_v + 2 * k2_v + 2 * k3_v + k4_v) * T_STEP/6;

        if(fabs(y - y_ss) <= EPS*REF){
            if(settle_flag == 0){
                time_settle = t;
                settle_flag = 1;
            }
        }
        else{
            settle_flag = 0;
        }
        t += T_STEP;
    }
    //输出相关的性能指标
    printf("overshoot: %.2f%%\n", overshoot);
    printf("time_settle: %.2f s\n", time_settle);
    fflush(stdout);
    /*if(is_stady == 1){
        printf("The system has reached a steady state.(%%2 error band)\n");
        printf("Steady-state error : %.6lf\n",steady_error);
        printf("Overshoot : %.2lf%%\n",overshoot);
        printf("Operation time(2%% error band) : %.2lf s\n",time_settle);
        printf("PREF:%.2f %.2f\n", overshoot, time_settle);
        fflush(stdout);
    }
    else{
        printf("After the system simulation ends, it fails to reach a steady state and there is continuous oscillation.\n");
    }*/
    
    return 0;
}




