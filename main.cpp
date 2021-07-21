// 台車の倒立制御（P制御）
#include "mbed.h"
#include "adrobo.h"
#include "Motor.h"
#include "BMX055.h"

//#define ZERO_ADV    351        //棒の角度が0になる時のAD値（機体により異なります）
//#define ADV_TO_RAD       0.003753  //とりあえず秘密（値も適切かどうかは知りません）
//#define MAX_V   8                     //駆動系の最大電圧
#define KP  0.11                 //Pゲイン
#define KD  0.002
#define KI  0.7
#define T  0.001

#define left_max 0.7    //左モータ最大duty
#define right_max 0.7   //右モータ最大duty


//モータ制御用オブジェクト


//***************　台車の制御　ここから　***************//

double theta = 0;            //取得した角速度を格納
double theta_dev;           //目標角と現在の角度の偏差
double theta_dev_integral;                //目標角と現在角の偏差の積分
double theta_dev_differential;      //目標角の偏差の微分
double theta_dev_before;                                        //前回の偏差
double v_ref;                       //電圧指令値　，　デューティー比
double duty_ratio = 0;
double monitor;



void initial_set();
void enable();
void desable();
double calc_theta();
void intr();

Ticker tic;    //台車の制御用タイマー割り込み
Motor motor_left(MOTOR11, MOTOR12);       //左モータ
Motor motor_right(MOTOR21, MOTOR22);      //右モータ
BMX055 imu(PB_7, PB_6);
Serial pc(USBTX, USBRX);                        //デバッグ用シリアル通信
DigitalIn sw(SW1);  //スイッチのピン設定



//起動時セットアップ処理
void initial_set(){
    sw.mode( PullUp ); 
    pc.baud(115200);
    motor_left.setMaxRatio(left_max);   //左モータ最大duty
    motor_right.setMaxRatio(right_max); //右モータ最大duty
    motor_left = 0;
    motor_right = 0;
    pc.printf("initialaize complete.\r\n");
}

void enable(){
    pc.printf("enabled!\r\n");
    tic.attach(intr, T);        //台車の制御用のタイマー関数を設定
}

void desable(){
    tic.detach();  //タイマー割り込みを止める

}


//角度を計算する関数
double calc_theta(){
    //pc.printf("calc_theta\r\n");
    float data = 0;
    float th = 0;
    data = (imu.gyroscope[0]*125)/2048;
    th = data * T;
    return double(th);
}


//割り込みコールバック関数
void intr(){

    theta += calc_theta();
    monitor = theta;
    theta_dev = theta;
    theta_dev_integral = (theta_dev - theta_dev_before)/T;
    theta_dev_differential += theta_dev*T;
    theta_dev_before = theta_dev;
    
    if(theta_dev_differential > 10000) theta_dev_differential = 10000;
    if(theta_dev_differential < -10000) theta_dev_differential = -10000;

    v_ref = (theta_dev * KP + theta_dev_differential * KI + theta_dev_integral * KD);
    
    duty_ratio = v_ref;

/*
motor_left,motor_rightに-1~1までの値を代入すると、モータが回る
*/
//    if(duty_ratio > 1.00) duty_ratio = 1.00;    //  -1~1の範囲からオーバーする場合に訂正
//    if(duty_ratio < -1.00) duty_ratio = -1.00;

    //計算結果をモータの速度指令に反映
    motor_left =  -duty_ratio;
    motor_right = -duty_ratio;

    //motor_left = 0.5f;
    //motor_right = 0.5f;
}
//***************　台車の制御　ここまで　***************//
//***************　main関数　ここから　***************//
int main() {
    initial_set();

    while(sw){  //sw1が押されるまで待機
        wait(0.1);
        pc.printf("stand by...\r\n");
    }
    
    enable();

    while(1) {
//        printf("theta_adv:%d duty_ratio:%2.2f theta:%2.2f \r\n", theta_adv ,duty_ratio,theta);
        //pc.printf("0.1sec!\r\n");

        imu.getGyro();
        //pc.printf("GyroX:%2.3lf", (imu.gyroscope[0]*125)/2048);
        //printf("theta:%2.3lf\r\n", monitor);
        //pc.printf("getGyro...\r\n");
        /*
        if(theta > 70 || theta < -70){
            desable();
        }*/ 
        wait(0.001);
    }
}