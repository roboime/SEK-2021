#include <stdio.h>
#include <time.h> 
#include <stdlib.h>
#include <string.h>
#include <math.h> 
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/utils/system.h>
 
#define TIME_STEP 32
 
int width, height, tamanhodorobo = 27, d180 = 1, color_detected, viuchao = 0;
WbDeviceTag right_camera, left_camera, left_motor, right_motor, empilhadeira, frds, flds, fmds, ltds, lbds, pos, iu, left_pos, right_pos;
double epsilon = 3;
 
typedef struct _rgb {
    int r, g, b;
} rgb;

rgb verde_min = {1000, 12000, 1000}, verde_max = {10000, 14000, 10000};

rgb getrgbs(WbDeviceTag camera) { //função para pegar a soma de todos os rgbs que a camera vê, 8 x 8 pixels
    const unsigned char *image = wb_camera_get_image(camera);
    rgb ans = {0, 0, 0};
    for(int i = 0; i < width; i++) {
        for(int j = 0; j < height; j++) { //esse loop pega os valores de rgb de cada pixel e adiciona à estrutura
            ans.r += wb_camera_image_get_red(image, width, i, j);
            ans.g += wb_camera_image_get_green(image, width, i, j);
            ans.b += wb_camera_image_get_blue(image, width, i, j);
        }
    }
    return ans;
}

void pegar_tubo() { //função para pegar cano

    while(wb_robot_step(TIME_STEP) != -1) {
        
        double frdsvalue = wb_distance_sensor_get_value(frds);
        double fldsvalue = wb_distance_sensor_get_value(flds);
        wb_motor_set_velocity(left_motor, 1);
        wb_motor_set_velocity(right_motor, 1);
        
        if(frdsvalue < 75 && fldsvalue < 75)  //fazer o robô andar reto até chegar próximo ao cano, quando irá abaixar a empilhadeira
            //printf("oi");
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            while(wb_robot_step(TIME_STEP) != -1) {
                double k = wb_position_sensor_get_value(pos);
                wb_motor_set_velocity(empilhadeira, -0.3);
                if(k < -0.12) {
                    wb_motor_set_velocity(empilhadeira, 0);
                    break;
                }
            }
            break;
        }    
    }
    while(wb_robot_step(TIME_STEP) != -1) { //quando chegar perto o suficiente, irá subir a garra
        wb_motor_set_velocity(left_motor, 0.5);
        wb_motor_set_velocity(right_motor, 0.5);
        double frdsvalue = wb_distance_sensor_get_value(frds);
        double fldsvalue = wb_distance_sensor_get_value(flds);
        if(frdsvalue < 20 && fldsvalue < 20){
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);

            while(wb_robot_step(TIME_STEP) != -1) {
                
                double k = wb_position_sensor_get_value(pos);
                //printf("%lf\n", k); 
                wb_motor_set_velocity(empilhadeira, 0.3);
                if(k > 0.179) {
                    break;
                }
            }
            
            wb_motor_set_velocity(empilhadeira, 0);
            return;
        }
    }
}

void entregar_tubo() { //função para colocar o cano no tubo
    
    while(wb_robot_step(TIME_STEP) != -1) { //andar reto até chegar em uma distância que a garra possa colocar o cano
        double fldsvalue = wb_distance_sensor_get_value(flds);
        double frdsvalue = wb_distance_sensor_get_value(frds);
        wb_motor_set_velocity(right_motor, 1);
        wb_motor_set_velocity(left_motor, 1);
        if(fldsvalue < 40 && frdsvalue < 40) {
            wb_motor_set_velocity(right_motor, 0);
            wb_motor_set_velocity(left_motor, 0);
            break;
        }
    }
    while(wb_robot_step(TIME_STEP) != -1) { //descer
        double k = wb_position_sensor_get_value(pos);
        wb_motor_set_velocity(empilhadeira, -0.3);
        if(k < 0.014) {
            wb_motor_set_velocity(empilhadeira, 0);
            break;
        }
    }
    
    int j = 0;
    while(wb_robot_step(TIME_STEP) != -1) { //espera, para o cano estabilizar
        j++;
        if(j == 60) {
            break;
        }
    }

    while(wb_robot_step(TIME_STEP) != -1) { //recuar

        double frdsvalue = wb_distance_sensor_get_value(frds);
        double fldsvalue = wb_distance_sensor_get_value(flds);
        
        wb_motor_set_velocity(left_motor, -1);
        wb_motor_set_velocity(right_motor, -1);
 
        if(frdsvalue > 200 && fldsvalue > 200){
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
 
            while(wb_robot_step(TIME_STEP) != -1) {
                double k = wb_position_sensor_get_value(pos);
                wb_motor_set_velocity(empilhadeira, 0.3);
                if(k > 0.1) {
                    wb_motor_set_velocity(empilhadeira, 0);
                    break;
                }   
            }
            return;
        }
    }
}
 
double odometria() { //o próprio nome já diz, ele fará a conta sabendo quantos radianos a roda girou enquanto via espaço vazio no sensor lateral
    double tube; 
    double pos_inicial = wb_position_sensor_get_value(left_pos);

    printf("start %lf\n", pos_inicial);
 
    while(wb_robot_step(TIME_STEP) != -1) {
        double cur = wb_position_sensor_get_value(left_pos);
        double ltdsvalue = wb_distance_sensor_get_value(ltds);
        double lbdsvalue = wb_distance_sensor_get_value(lbds);
        printf("%lf\n", cur);
        wb_motor_set_velocity(left_motor, 2);
        wb_motor_set_velocity(right_motor, 2);
 
        if(ltdsvalue < 200) { //se a tubulação recomeça
            wb_motor_set_velocity(left_motor, 0);   
            wb_motor_set_velocity(right_motor, 0);

            break;
        }

        if(lbdsvalue > 150) { //se a tubulação dá um zig-zag
            wb_motor_set_velocity(left_motor, 0);   
            wb_motor_set_velocity(right_motor, 0);
            break;
        }
 
        rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
        rgb avg = {(lc.r + rc.r) / 2, (lc.g + rc.g) / 2, (lc.b + rc.b) / 2};
 
        if(avg.b < 7000 && avg.r < 9000 && avg.g < 8000) { //caso veja chão, não foi feito do novo jeito nessa função
            wb_motor_set_velocity(left_motor, 0);   
            wb_motor_set_velocity(right_motor, 0);
 
            tube = (cur - pos_inicial) * 0.0374 * 100;
            if(tube < 10) {
                if(!via2()) {
                    exit(0);
                }
                return odometria();
            }
            return tube;
        }

    }
 
    double end = wb_position_sensor_get_value(left_pos);

    tube = ((end - pos_inicial) * 0.0374) * 100;
    if(tube < 10) {
        if(!via2()) {
            exit(0);
        }
        return odometria();
    }
 
    return tube; 
}
 
void alinhar_chao() { //função para teóricamente alinhar o robô com o lado, mas acabou em desuso por causa de problemas no simulador
    while(wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(right_motor, 2);
        wb_motor_set_velocity(left_motor, 2);
        rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
        int left_sensor = 0, right_sensor = 0;
        if(rc.b < 7000 && rc.r < 9000 && rc.g < 8000) {
            right_sensor = 1;
        }
        if(lc.b < 7000 && lc.r < 9000 && lc.g < 8000) {
            left_sensor = 1;
        }
        if(right_sensor && !left_sensor) {
            wb_motor_set_velocity(right_motor, 0);
            wb_motor_set_velocity(left_motor, 0.4);
            while(wb_robot_step(TIME_STEP) != -1) {
                rc = getrgbs(right_camera), lc = getrgbs(left_camera);
                if(abs(rc.b - lc.b) < 200 && abs(rc.g - lc.g) < 200 && abs(rc.r - lc.r) < 200) {
                    wb_motor_set_velocity(right_motor, -3);
                    wb_motor_set_velocity(left_motor, -3);
                    int j = 0;
                    while(wb_robot_step(TIME_STEP) != -1) {
                        j++;
                        if(j == 50) {
                            break;
                        }
                    }
                    return;
                }
            }
        }
        if(!right_sensor && left_sensor) {
            wb_motor_set_velocity(right_motor, 0.4);
            wb_motor_set_velocity(left_motor, 0);
            while(wb_robot_step(TIME_STEP) != -1) {
                rc = getrgbs(right_camera), lc = getrgbs(left_camera);
                if(abs(rc.b - lc.b) < 200 && abs(rc.g - lc.g) < 200 && abs(rc.r - lc.r) < 200) {
                    wb_motor_set_velocity(right_motor, -3);
                    wb_motor_set_velocity(left_motor, -3);
                    int j = 0;
                    while(wb_robot_step(TIME_STEP) != -1) {
                        j++;
                        if(j == 50) {
                            break;
                        }
                    }
                    return;
                }
            }
        }
        if(right_sensor && left_sensor) {
            wb_motor_set_velocity(right_motor, -3);
            wb_motor_set_velocity(left_motor, -3);
            int j = 0;
            while(wb_robot_step(TIME_STEP) != -1) {
                j++;
                if(j == 50) {
                    break;
                }
            }
            return;
        }
    }
}
 
void alinhar(rgb min, rgb max) { //a função teoricamente alinha o robô com um range de cor que você especificar, mas acabamos não aperfeiçoando tão bem assim
    while(wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(right_motor, 4);
        wb_motor_set_velocity(left_motor, 4);
        rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
        rgb avg = {(lc.r + rc.r) / 2, (lc.g + rc.g) / 2, (lc.b + rc.b) / 2};
        //printf("avg %d %d %d\n", avg.r, avg.g, avg.b);
        if(avg.r >= min.r && avg.r <= max.r && avg.b >= min.b && avg.b <= max.b && avg.g >= min.g && avg.g <= max.g) {
            int j = 0;
            while(wb_robot_step(TIME_STEP) != -1) {
                wb_motor_set_velocity(right_motor, 2);
                wb_motor_set_velocity(left_motor, 2);
                j++;
                if(j == 29) {
                    wb_motor_set_velocity(right_motor, 0);
                    wb_motor_set_velocity(left_motor, 0);
                    break;
                }
            }
            return;
        }
        if(rc.r >= min.r && rc.r <= max.r && rc.b >= min.b && rc.b <= max.b && rc.g >= min.g && rc.g <= max.g) {
            while(wb_robot_step(TIME_STEP) != -1) {
                wb_motor_set_velocity(left_motor, 0.5);
                wb_motor_set_velocity(right_motor, 0);
                rc = getrgbs(right_camera), lc = getrgbs(left_camera);
                if(lc.r >= min.r && lc.r <= max.r && lc.b >= min.b && lc.b <= max.b && lc.g >= min.g && lc.g <= max.g) {
                    int j = 0;
                    while(wb_robot_step(TIME_STEP) != -1) {
                        wb_motor_set_velocity(right_motor, 2);
                        wb_motor_set_velocity(left_motor, 2);
                        j++;
                        if(j == 29) {
                            wb_motor_set_velocity(right_motor, 0);
                            wb_motor_set_velocity(left_motor, 0);
                            break;
                        }
                    }
                    return;
                }
            }
        }
        if(lc.r >= min.r && lc.r <= max.r && lc.b >= min.b && lc.b <= max.b && lc.g >= min.g && lc.g <= max.g) {
            while(wb_robot_step(TIME_STEP) != -1) {
                wb_motor_set_velocity(left_motor, 0);
                wb_motor_set_velocity(right_motor, 0.5);
                rc = getrgbs(right_camera), lc = getrgbs(left_camera);
                if(rc.r >= min.r && rc.r <= max.r && rc.b >= min.b && rc.b <= max.b && rc.g >= min.g && rc.g <= max.g) {
                    int j = 0;
                    while(wb_robot_step(TIME_STEP) != -1) {
                        wb_motor_set_velocity(right_motor, 2);
                        wb_motor_set_velocity(left_motor, 2);
                        j++;
                        if(j == 29) {
                            wb_motor_set_velocity(right_motor, 0);
                            wb_motor_set_velocity(left_motor, 0);
                            break;
                        }
                    }
                    return;
                }
            }
        }
    }
}
 
void desviar() { //acabou não sendo usado
    rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
    if(rc.g < 5800 && rc.b < 5800 && rc.r < 5800) {
        while(wb_robot_step(TIME_STEP) != -1) {
            wb_motor_set_velocity(right_motor, -2);
            wb_motor_set_velocity(left_motor, 0);
            if(rc.g > 5800 && rc.b > 5800 && rc.r > 5800) {
                return;
            }
        }
    }
    if(lc.g < 5800 && lc.b < 5800 && lc.r < 5800) {
        while(wb_robot_step(TIME_STEP) != -1) {
            wb_motor_set_velocity(right_motor, 0);
            wb_motor_set_velocity(left_motor, -2);
            if(lc.g > 5800 && lc.b > 5800 && lc.r > 5800) {
                return;
            }
        }
    }
    return;
}
 
void giro_() { //um giro a direita, as funções de giro acabaram ficando bem complicadas por causa de, novamente, problemas com o simulador
    wb_motor_set_velocity(right_motor, -0.5);
    wb_motor_set_velocity(left_motor, 0.5);
    wb_inertial_unit_enable(iu, TIME_STEP);
    const double *init = wb_inertial_unit_get_roll_pitch_yaw(iu);
    double k = init[2], prev = k;
    int cnt = 0;
    while(wb_robot_step(TIME_STEP) != -1) {
        const double *val = wb_inertial_unit_get_roll_pitch_yaw(iu);
        double cur = val[2];
        //printf("%lf %lf %lf\n", val[2], k, fabs(val[2] - k));
        if(cur > 0) {
            if(prev < 0 && fabs(k - prev) > 1.3 && cnt > 20) {
                break;
            } else if(prev > 0) {
                k *= -1.0;
            }
        } else {
            if(prev > 0 && fabs(k - prev) > 1.3 && cnt > 20) {
                break;
            } else if(prev < 0) {
                k *= -1.0;
            }
        }
        if(fabs(val[2] - k) >= 1.56 && fabs(val[2] - k) <= 1.58 && cnt > 20) {
            break;
        }
        prev = cur;
        cnt++;
    }
    wb_inertial_unit_disable(iu);
    wb_motor_set_velocity(right_motor, 0);
    wb_motor_set_velocity(left_motor, 0);
    return;
}

void _giro() { //giro à esquerda
    wb_motor_set_velocity(right_motor, 0.5);
    wb_motor_set_velocity(left_motor, -0.5);
    wb_inertial_unit_enable(iu, TIME_STEP);
    const double *init = wb_inertial_unit_get_roll_pitch_yaw(iu);
    double k = init[2], prev = k;
    int cnt = 0;
    while(wb_robot_step(TIME_STEP) != -1) {
        const double *val = wb_inertial_unit_get_roll_pitch_yaw(iu);
        double cur = val[2];
        //printf("%lf %lf %lf\n", val[2], k, fabs(val[2] - k));
        if(cur > 0) {
            if(prev < 0 && fabs(k - prev) > 1.3) {
                break;
            } else if(prev > 0) {
                k *= -1.0;
            }
        } else {
            if(prev > 0 && fabs(k - prev) > 1.3) {
                break;
            } else if(prev > 0) {
                k *= -1.0;
            }
        }
        if(fabs(val[2] - k) >= 1.56 && fabs(val[2] - k) <= 1.58 && cnt > 20) {
            break;
        }
        prev = cur;
        cnt++;
    }
    wb_inertial_unit_disable(iu);
    wb_motor_set_velocity(right_motor, 0);
    wb_motor_set_velocity(left_motor, 0);
    return;
}
 
void _giro_() { //giro 180
    giro_();
    giro_();
    return;
}
 
void vi0() { //primeira VI, o objetivo dela é ir para o canto da pista e depois virar a direita, como intenção de chegar até a tubulação
    wb_motor_set_position(empilhadeira, INFINITY);
    while(wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(right_motor, 8);
        wb_motor_set_velocity(left_motor, 8);
        double dis = wb_distance_sensor_get_value(fmds);
        rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
        printf("%lf\n", dis);
        if(dis > 100) {
            int j = 0;
            while(wb_robot_step(TIME_STEP) != -1) {
                wb_motor_set_velocity(right_motor, -2);
                wb_motor_set_velocity(left_motor, -2);
                j++;
                if(j == 75) {
                    wb_motor_set_velocity(right_motor, 0);
                    wb_motor_set_velocity(left_motor, 0);
                    break;
                }
            }
            viuchao = 1;
            giro_();
        }
        if((rc.r < 5000 && rc.g < 5000 && rc.b < 5000) || (lc.r < 5000 && lc.g < 5000 && lc.b < 5000)) {
            int j = 0;
            while(wb_robot_step(TIME_STEP) != -1) {
                wb_motor_set_velocity(right_motor, -2);
                wb_motor_set_velocity(left_motor, -2);
                j++;
                if(j == 30) {
                    wb_motor_set_velocity(right_motor, 0);
                    wb_motor_set_velocity(left_motor, 0);
                    break;
                }
            }
            giro_();
        }
        if((rc.r >= verde_min.r && rc.r <= verde_max.r && rc.g >= verde_min.g && rc.g <= verde_max.g && rc.b >= verde_min.b && rc.b <= verde_max.b) || (lc.r >= verde_min.r && lc.r <= verde_max.r && lc.g >= verde_min.g && lc.g <= verde_max.g && lc.b >= verde_min.b && lc.b <= verde_max.b)) {
            if(viuchao) {
                return;
            } else {
                int j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, -2);
                    wb_motor_set_velocity(left_motor, -2);
                    j++;
                    if(j == 30) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                giro_();
            }
        }
    }
}

void vi0_tubo() { //a mesma coisa de antes porém com a restrição de andar devagar para não tombar
    viuchao = 0;
    while(wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(right_motor, 3);
        wb_motor_set_velocity(left_motor, 3);
        double dis = wb_distance_sensor_get_value(fmds);
        rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
        printf("right %d %d %d\n", rc.r, rc.g, rc.b);
        printf("left %d %d %d\n", lc.r, lc.g, lc.b);
        if(dis > 100) {
            int j = 0;
            while(wb_robot_step(TIME_STEP) != -1) {
                wb_motor_set_velocity(right_motor, -2);
                wb_motor_set_velocity(left_motor, -2);
                j++;
                if(j == 75) {
                    wb_motor_set_velocity(right_motor, 0);
                    wb_motor_set_velocity(left_motor, 0);
                    break;
                }
            }
            viuchao = 1;
            giro_();
        }
        if((rc.r < 5000 && rc.g < 5000 && rc.b < 5000) || (lc.r < 5000 && lc.g < 5000 && lc.b < 5000)) {
            int j = 0;
            while(wb_robot_step(TIME_STEP) != -1) {
                wb_motor_set_velocity(right_motor, -2);
                wb_motor_set_velocity(left_motor, -2);
                j++;
                if(j == 30) {
                    wb_motor_set_velocity(right_motor, 0);
                    wb_motor_set_velocity(left_motor, 0);
                    break;
                }
            }
            giro_();
        }
        if((rc.r >= verde_min.r && rc.r <= verde_max.r && rc.g >= verde_min.g && rc.g <= verde_max.g && rc.b >= verde_min.b && rc.b <= verde_max.b) || (lc.r >= verde_min.r && lc.r <= verde_max.r && lc.g >= verde_min.g && lc.g <= verde_max.g && lc.b >= verde_min.b && lc.b <= verde_max.b)) {
            if(viuchao) {
                return;
            } else {
                int j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, -2);
                    wb_motor_set_velocity(left_motor, -2);
                    j++;
                    if(j == 30) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                giro_();
            }
        }
    }
}

void via1() { //função para chegar até a tubulação
    //wb_motor_set_velocity(right_motor, -2);
    //wb_motor_set_velocity(left_motor, -2);
    //giro_();
    wb_inertial_unit_enable(iu, TIME_STEP);
    const double *val = wb_inertial_unit_get_roll_pitch_yaw(iu);
    //printf("yaw = %lf\n", val[2]);
    if(val[2] <= -1.5 && val[2] >= -1.6) { //será que isso daqui tava causando problema?? nunca deu merda aqui...
        vi0();
    }
    wb_inertial_unit_disable(iu);
    while(wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(right_motor, 6);
        wb_motor_set_velocity(left_motor, 6);
        double fr = wb_distance_sensor_get_value(frds), fl = wb_distance_sensor_get_value(flds);
        double avg = (fr + fl) / 2;
        if(avg < 200) {
            break;
        }
        rgb lc = getrgbs(left_camera);
        if(lc.r < 3200 && lc.g < 3200 && lc.b < 3200) {
            wb_motor_set_velocity(right_motor, 1);
            int j = 0;
            for(int i = 0; i < 20000000; i++) {
                j++;
            }
            wb_motor_set_velocity(right_motor, 2);
        }
    }
    giro_();
}

void via1_tube() { //a mesma coisa de antes porém com a restrição de andar devagar ao longo da rampa, pois se andasse um pouco mais rápido o nosso robô iria tombar
    //wb_motor_set_velocity(right_motor, -2);
    //wb_motor_set_velocity(left_motor, -2);
    //giro_();
    wb_inertial_unit_enable(iu, TIME_STEP);
    const double *val = wb_inertial_unit_get_roll_pitch_yaw(iu);
    //printf("yaw = %lf\n", val[2]);
    if(val[2] <= -1.5 && val[2] >= -1.6) {
        vi0();
    }
    wb_inertial_unit_disable(iu);
    while(wb_robot_step(TIME_STEP) != -1) {
        rgb lc = getrgbs(left_camera), rc = getrgbs(right_camera);
        rgb avg = {(lc.r + rc.r) / 2, (lc.g + rc.g) / 2, (lc.b + rc.b) / 2};
        if(avg.r <= 4000 && avg.g <= 15000 && avg.b <= 4000) {
            wb_motor_set_velocity(right_motor, 0.5);
            wb_motor_set_velocity(left_motor, 0.5);
        } else {
            wb_motor_set_velocity(right_motor, 3);
            wb_motor_set_velocity(left_motor, 3);
        }
        double fr = wb_distance_sensor_get_value(frds), fl = wb_distance_sensor_get_value(flds);
        double avgg = (fr + fl) / 2;
        if(avgg < 250) {
            break;
        }
        if(lc.r < 3200 && lc.g < 3200 && lc.b < 3200) {
            wb_motor_set_velocity(right_motor, 1);
            int j = 0;
            for(int i = 0; i < 20000000; i++) {
                j++;
            }
            wb_motor_set_velocity(right_motor, 2);
        }
    }
    giro_();
}

int via2() { //andar ao longo da tubulação até encontrar um espaço vazio ou descobrir que está completo
    while(wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(right_motor, 5);
        wb_motor_set_velocity(left_motor, 5);
        double dsl = wb_distance_sensor_get_value(ltds), dsr = wb_distance_sensor_get_value(flds);
        if(dsl > 200 || dsr < 200) {
            wb_motor_set_velocity(right_motor, 0);
            wb_motor_set_velocity(left_motor, 0);
            break;
        }
        rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
        rgb md = {(rc.r + lc.r) / 2, (rc.g + lc.g) / 2, (rc.b + lc.b) / 2};
        if(md.r < 9000 && md.g < 8000 && md.b < 7000) {
            wb_motor_set_velocity(right_motor, 0);
            wb_motor_set_velocity(left_motor, 0);
            return 0;
        }
    }
    double dsl = wb_distance_sensor_get_value(ltds), dsr = wb_distance_sensor_get_value(flds), dsb = wb_distance_sensor_get_value(lbds);
    if(dsl > 200) {
        if(dsb < 200 && dsr > 200) {
            return 1;
        } else if(dsb < 200 && dsr < 200) {
            giro_();
            via2(); 
        } else {
            int j = 0;
            while(wb_robot_step(TIME_STEP) != -1) {
                wb_motor_set_velocity(right_motor, 2);
                wb_motor_set_velocity(left_motor, 2);
                j++;
                if(j == 25) {
                    wb_motor_set_velocity(right_motor, 0);
                    wb_motor_set_velocity(left_motor, 0);
                    break;
                }
            }
            _giro();
            via2();
        }
    } 
    if(dsr < 200) {
        giro_();
        via2();
    }
    wb_motor_set_velocity(right_motor, 0);
    wb_motor_set_velocity(left_motor, 0);
    return 1;
}

void vit1() { //voltar para a área de coleta, isso daqui poderia teoricamente ser feito por um outro robô
    double lbdsvalue = wb_distance_sensor_get_value(lbds);
    if(lbdsvalue < 200) {
        giro_();
    }
    while(wb_robot_step(TIME_STEP) != -1) { //andar até chegar naquela faixa preta, teoricamente
        wb_motor_set_velocity(right_motor, 8);
        wb_motor_set_velocity(left_motor, 8);
        rgb lc = getrgbs(left_camera), rc = getrgbs(right_camera);
        rgb avg = {(lc.r + rc.r) / 2, (lc.g + rc.g) / 2, (lc.b + rc.b) / 2};
        //printf("avg %d %d %d\n", avg.r, avg.g, avg.b);
        if(avg.r > 10000 && avg.g > 10000 && avg.b > 10000) {
            wb_motor_set_velocity(right_motor, 0);
            wb_motor_set_velocity(left_motor, 0);
            break;
        }
    }
    rgb mn = {0, 0, 0}, mx = {4000, 4000, 4000};
    alinhar(mn, mx);
}
 
void vit3(rgb cor_min, rgb cor_max) { //andar pela área dos canos até achar o que precisa
    _giro();
    d180 = 1; //d180 = 1 -> quer dizer que os sensores de distância estão virados para o outro lado, sem ser para os canos
    while(wb_robot_step(TIME_STEP) != -1) { //chegar até o final
        wb_motor_set_velocity(right_motor, 4);
        wb_motor_set_velocity(left_motor, 4);
        rgb rrc = getrgbs(right_camera), llc = getrgbs(left_camera);
        rgb avg = {(llc.r + rrc.r) / 2, (llc.g + rrc.g) / 2, (llc.b + rrc.b) / 2};
        if(avg.r <= 9000 && avg.g <= 8000 && avg.b <= 7000) {
            wb_motor_set_velocity(right_motor, 0);
            wb_motor_set_velocity(left_motor, 0);
            break;
        }
    }
    while(wb_robot_step(TIME_STEP) != -1) { //procurando a cor
        wb_motor_set_velocity(right_motor, 4);
        wb_motor_set_velocity(left_motor, 4);
        rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
        //printf("right %d %d %d\n", rc.r, rc.g, rc.b);
        //printf("left %d %d %d\n", lc.r, lc.g, lc.b);
        //printf("deu 180? %d\n", d180);
        if(!d180) {
            if(lc.r >= cor_min.r && lc.r <= cor_max.r && lc.b >= cor_min.b && lc.b <= cor_max.b && lc.g >= cor_min.g && lc.g <= cor_max.g) {
                int j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, 2);
                    wb_motor_set_velocity(left_motor, 2);
                    j++;
                    if(j == 15) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                wb_motor_set_velocity(right_motor, 0);
                wb_motor_set_velocity(left_motor, 0);
                return;
            }
            if(lc.r <= 8500 && lc.g <= 7500 && lc.b <= 6500) {
                int j = 0;
                wb_motor_set_velocity(right_motor, -2);
                wb_motor_set_velocity(left_motor, -2);
                while(wb_robot_step(TIME_STEP) != -1) {
                    j++;
                    if(j == 100) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                _giro_();
                j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, -2);
                    wb_motor_set_velocity(left_motor, -2);
                    j++;
                    if(j == 62) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                d180 = 1;
            }
        } else {
            if(rc.r >= cor_min.r && rc.r <= cor_max.r && rc.b >= cor_min.b && rc.b <= cor_max.b && rc.g >= cor_min.g && rc.g <= cor_max.g) {
                int j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, 2);
                    wb_motor_set_velocity(left_motor, 2);
                    j++;
                    if(j == 15) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                wb_motor_set_velocity(right_motor, 0);
                wb_motor_set_velocity(left_motor, 0);
                return;
            }
            if(rc.r <= 8500 && rc.g <= 7500 && rc.b <= 6500) {
                int j = 0;
                wb_motor_set_velocity(right_motor, -2);
                wb_motor_set_velocity(left_motor, -2);
                while(wb_robot_step(TIME_STEP) != -1) {
                    j++;
                    if(j == 100) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                _giro_();
                j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, -2);
                    wb_motor_set_velocity(left_motor, -2);
                    j++;
                    if(j == 62) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                d180 = 0;
            }
        }
    }
}

void vit3_1() { //função para achar o tubo dentro da cor especificada
    if(d180) {
        rgb rinit = getrgbs(right_camera);
        while(wb_robot_step(TIME_STEP) != -1) {
            wb_motor_set_velocity(right_motor, 2);
            wb_motor_set_velocity(left_motor, 2);
            rgb rc = getrgbs(right_camera);
            if(abs(rc.r - rinit.r) >= 200 && abs(rc.g - rinit.g) >= 200 && abs(rc.b - rinit.b) >= 200) {
                wb_motor_set_velocity(right_motor, 0);
                wb_motor_set_velocity(left_motor, 0);
                int j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, -2);
                    wb_motor_set_velocity(left_motor, -2);
                    j++;
                    if(j == 100) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                _giro_();
                j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, -2);
                    wb_motor_set_velocity(left_motor, -2);
                    j++;
                    if(j == 70) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                break;
            }
        }
    }
    while(wb_robot_step(TIME_STEP) != -1) {
        double lbdsvalue = wb_distance_sensor_get_value(lbds);
        wb_motor_set_velocity(right_motor, 2);
        wb_motor_set_velocity(left_motor, 2);
        if (lbdsvalue < 300){
            break;
        }
    }
    int j = 0;
    while(wb_robot_step(TIME_STEP) != -1) {
        wb_motor_set_velocity(right_motor, 2);
        wb_motor_set_velocity(left_motor, 2);
        j++;
        if((color_detected == 1 && j == 14) || (color_detected == 2 && j == 8) || (color_detected == 3 && j == 1)) {
            wb_motor_set_velocity(right_motor, 0);
            wb_motor_set_velocity(left_motor, 0);
            break;
        }
    }
    _giro();
}

int main(int argc, char **argv) {
    wb_robot_init();
    
    //initializing our tags
    frds = wb_robot_get_device("FrontRightDistanceSensor");
    flds = wb_robot_get_device("FrontLeftDistanceSensor");
    ltds = wb_robot_get_device("LeftTopDistanceSensor");
    lbds = wb_robot_get_device("LeftBottomDistanceSensor");
    fmds = wb_robot_get_device("FrontMiddleDistanceSensor");
    pos = wb_robot_get_device("EMPPositionSensor");
    right_camera = wb_robot_get_device("RightCamera");
    left_camera = wb_robot_get_device("LeftCamera");
    width = wb_camera_get_width(right_camera);
    height = wb_camera_get_height(right_camera);
    left_motor = wb_robot_get_device("LeftWheel");
    right_motor = wb_robot_get_device("RightWheel");
    empilhadeira = wb_robot_get_device("EMPLinearMotor");
    iu = wb_robot_get_device("inertial_unit");
    left_pos = wb_robot_get_device("LWpossensor");
    right_pos = wb_robot_get_device("RWpossensor");
    wb_distance_sensor_enable(frds, TIME_STEP);
    wb_distance_sensor_enable(flds, TIME_STEP);
    wb_distance_sensor_enable(fmds, TIME_STEP);
    wb_distance_sensor_enable(ltds, TIME_STEP);
    wb_distance_sensor_enable(lbds, TIME_STEP);
    wb_position_sensor_enable(pos, TIME_STEP);
    wb_position_sensor_enable(left_pos, TIME_STEP);
    wb_position_sensor_enable(right_pos, TIME_STEP);
    wb_camera_enable(right_camera, TIME_STEP);
    wb_camera_enable(left_camera, TIME_STEP);
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_position(empilhadeira, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    wb_motor_set_velocity(empilhadeira, 0.0);   
    while(true) {
        printf("stt vi0\n");
        vi0();
        printf("ff vi0\n");
        via1(); 
        if(!via2()) {
            break;
        }
        double length = odometria();
        printf("achado %lf\n", length);
        //length = 18;
        rgb mn, mx;
        if(length > 20) { //BLUE
            rgb tmp1 = {0, 0, 10000}, tmp2 = {4000, 4000, 15000};
            mn = tmp1;
            mx = tmp2;
            color_detected = 1;
            printf("BLUE\n");
        } else if(length > 15) { //RED
            rgb tmp1 = {10000, 0, 0}, tmp2 = {15000, 4000, 4000};
            mn = tmp1;
            mx = tmp2;
            color_detected = 2;
            printf("RED\n");
        } else { //YELLOW
            rgb tmp1 = {10000, 10000, 0}, tmp2 = {15000, 15000, 4000};
            mn = tmp1;
            mx = tmp2;
            color_detected = 3;
            printf("YELLOW\n");
        }
        vit1();
        vit3(mn, mx);
        vit3_1();
        int _ = 0;
        while(wb_robot_step(TIME_STEP) != -1) {
            wb_motor_set_velocity(right_motor, -2);
            wb_motor_set_velocity(left_motor, -2);
            _++;
            if(_ == 25) {
                wb_motor_set_velocity(right_motor, 0);
                wb_motor_set_velocity(left_motor, 0);
                break;
            }
        }
        pegar_tubo();
        int j = 0;

        while(wb_robot_step(TIME_STEP) != -1) {

            rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
            wb_motor_set_velocity(right_motor, -2);
            wb_motor_set_velocity(left_motor, -2);

            if((rc.r > 10000 && rc.r < 15000 && rc.g > 10000 && rc.g < 15000 && rc.b > 10000 && rc.b < 15000) || (lc.r > 10000 && lc.r < 15000 && lc.g > 10000 && lc.g < 15000 && lc.b > 10000 && lc.b < 15000)) {
                wb_motor_set_velocity(right_motor, 0);
                wb_motor_set_velocity(left_motor, 0);
                break;
            }
        }

        while(wb_robot_step(TIME_STEP) != -1) {
            wb_motor_set_velocity(right_motor, -2);
            wb_motor_set_velocity(left_motor, -2);
            j++; 
            if(j == 15){
                wb_motor_set_velocity(right_motor, 0);
                wb_motor_set_velocity(left_motor, 0);
                break;
            }
        }
        vi0_tubo();
        printf("aodasij\n");
        via1_tube();

        while(wb_robot_step(TIME_STEP) != -1) {
            rgb rc = getrgbs(right_camera), lc = getrgbs(left_camera);
            rgb md = {(rc.r + lc.r) / 2, (rc.g + lc.g) / 2, (rc.b + lc.b) / 2};
            if(md.r < 9000 && md.g < 8000 && md.b < 7000) {
                wb_motor_set_velocity(right_motor, 0);
                wb_motor_set_velocity(left_motor, 0);
                break;
            }
            via2();
            double length2 = odometria();
            printf("%lf %lf\n", length, length2);
            if(fabs(length - length2) < 3) {
                printf("aousdaoisjd\n");
                int j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, -2);
                    wb_motor_set_velocity(left_motor, -2);
                    j++;
                    double ltdsvalue = wb_distance_sensor_get_value(ltds);
                    if(ltdsvalue < 200 && j > 3) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                j = 0;
                while(wb_robot_step(TIME_STEP) != -1) {
                    wb_motor_set_velocity(right_motor, 2);
                    wb_motor_set_velocity(left_motor, 2);
                    j++;
                    if((color_detected == 1 && j == 19) || (color_detected == 2 && j == 9) || (color_detected == 3 && j == 3)) {
                        wb_motor_set_velocity(right_motor, 0);
                        wb_motor_set_velocity(left_motor, 0);
                        break;
                    }
                }
                _giro();
                entregar_tubo();
                break;
            }
        }
        _giro_();
        j = 0;
        while(wb_robot_step(TIME_STEP) != -1) {
            wb_motor_set_velocity(right_motor, 6);
            wb_motor_set_velocity(left_motor, 6);
            j++;
            if(j == 200) {
                wb_motor_set_velocity(right_motor, 0);
                wb_motor_set_velocity(left_motor, 0);
                break;
            }
        }
    }
    wb_robot_cleanup();
    return 0;
}
