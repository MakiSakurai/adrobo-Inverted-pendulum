// 台車の倒立制御（P制御）
#include "mbed.h"
#include "adrobo.h"
#include "Motor.h"
#include "QEI.h"
#define THETA_REF    0                    //振子の目標値(rad表記)
#define ZERO_ADV    361          //棒の角度が0になる時のAD値（機体により異なります）
#define ADV_TO_RAD      0.003753  //とりあえず秘密（値も適切かどうかは知りません）
#define MAX_V   8                  //駆動の最大電圧
#define T 0.0005
#define KP  200
#define KI  0.3
#define KD  2.8           //Pゲイン（値も適切ではありません）
BusOut led(D2,D4,D5,D7,D8);     //基板LED用IO設定
AnalogIn pen(A0);                               //ポテンショメータ用IO設定
Ticker pen_control;                             //台車の制御用タイマー割り込み
Serial pc(USBTX, USBRX);                        //デバッグ用シリアル通信
//モータ制御用オブジェクト
Motor motor_left(MOTOR11, MOTOR12);       //左モータ
Motor motor_right(MOTOR21, MOTOR22);      //右モータ
//***************　台車の制御　ここから　***************//
int theta_adv;                                                //振子についているポテンショメータのAD値格納用
double  theta, e,ei,ed,e0;                                          //振子の角度
double v_ref, duty_ratio, er=0;                       //電圧指令値　，　デューティー比
void pen_control_handler(){
    theta_adv = pen.read_u16()>>6;                    //ADCを通してポテンショメータのAD値を取得
    //搭載されているLPC1114のADCは10bitのため6bit右にシフト
    theta = (double)(theta_adv - ZERO_ADV) *ADV_TO_RAD;
    e = THETA_REF - theta;
    ed = (e - e0);
    ei += e;
    e0 = e;
    if(ei > 10000) ei = 10000;
    if(ei < -10000) ei = -10000;
    v_ref = (e * KP + ei * KI + ed * KD);
                                     //電圧指令値をP制御で決める
    duty_ratio = v_ref / MAX_V;
     //指令値の頭打ち処理
    if(duty_ratio > 1.00) duty_ratio = 1.00;
    if(duty_ratio < -1.00) duty_ratio = -1.00;
    //**** 指令値によって発光するLEDを変える ****//
        if(duty_ratio > 0.8 ){
            led = 8;
        }else if(duty_ratio <= 0.8 && duty_ratio >= 0){
            led = 4;
        }else if(duty_ratio < 0 && duty_ratio >= -0.8){
            led = 2;
        }else if(duty_ratio < -0.8){
            led = 1;
        }
    //**** 指令値によって発光するLEDを変える　ここまで ****//
    //計算結果をモータの速度指令に反映
    motor_left =  duty_ratio;
    motor_right = duty_ratio;
    if(theta_adv > 450|| theta_adv < 260){
            while(1)
    {
    motor_left =  0;
    motor_right = 0;
    }
    }
}
//***************　台車の制御　ここまで　***************//
//***************　main関数　ここから　***************//
int main() {
    //モータの最大電圧範囲を設定
    motor_left.setMaxRatio(0.6);
    motor_right.setMaxRatio(0.6);
    pen_control.attach(&pen_control_handler, T);        //台車の制御用のタイマー関数を設定
    led = 1;        //LEDの値を設定　動作確認用
    //wait(1.0);      //なんとなく1秒待つ
    while(1) {
       //無限ループ
        printf("theta_adv:%d duty_ratio:%2.2f theta:%2.2f \r\n", theta_adv ,duty_ratio,theta);
        wait(0.08);
    }
}
