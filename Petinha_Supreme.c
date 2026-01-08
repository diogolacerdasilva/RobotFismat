#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "pid_ctrl.h"
#include "freertos/semphr.h"

SemaphoreHandle_t adc_mutex;



#define TAG "PETINHA_DEBUG"



// PWM config
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_A    LEDC_CHANNEL_0
#define LEDC_CHANNEL_B    LEDC_CHANNEL_1
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY  10000 // 10 kHz

// QTR config
#define QTR_ADC_UNIT      ADC_UNIT_1
#define QTR_ADC_ATTEN     ADC_ATTEN_DB_12
#define QTR_ADC_BITWIDTH  ADC_BITWIDTH_DEFAULT
#define QTR_COUNT         8
#define TOTAL_READINGS 100   // Número de leituras por sensor
#define QTR_SIDE_COUNT 2

//Misc
#define TARGET_POSITION   3500
#define PID_LOOP_PERIOD_MS 10
#define MARGIN_ERR 500
#define MAX_SPEED 1023
#define THRESHOLD 100


//GPIO's
#define AIN2_GPIO  GPIO_NUM_41
#define AIN1_GPIO  GPIO_NUM_42
#define BIN1_GPIO  GPIO_NUM_39
#define BIN2_GPIO  GPIO_NUM_40
#define PWMB_GPIO  GPIO_NUM_1
#define PWMA_GPIO  GPIO_NUM_2
#define BUTTON_IN  GPIO_NUM_12
#define BUTTON_OUT GPIO_NUM_11

int readings[QTR_COUNT][TOTAL_READINGS],
avg_MIN[QTR_COUNT],
avg_MAX[QTR_COUNT];


TaskHandle_t pid_task_handle = NULL;
static const struct {
    int gpio_num;
    adc_channel_t channel;
} qtr_sensors[QTR_COUNT] = {
    { 3,  ADC_CHANNEL_2 },
    { 4,  ADC_CHANNEL_3 },
    { 5,  ADC_CHANNEL_4 },
    { 6,  ADC_CHANNEL_5 },
    { 7,  ADC_CHANNEL_6 },
    { 8,  ADC_CHANNEL_7 },
    { 9,  ADC_CHANNEL_8 },
    { 10, ADC_CHANNEL_9 },
};

static const struct{
	int gpio_num;
	adc_channel_t channel;
} qtr_right_sensor[QTR_SIDE_COUNT] = {
	{15, ADC_CHANNEL_4},
	{16, ADC_CHANNEL_5}
};

//Definindo estrutura context
typedef struct{
	pid_ctrl_block_handle_t     pid_ctrl;
	adc_oneshot_unit_handle_t   adc_handle;
	adc_oneshot_unit_handle_t	adc_handle_side;
	adc_cali_handle_t           cali_handle;
	adc_cali_handle_t			cali_handle_side;
	int                         last_position;
	int                         report_position;
} control_context_t;

typedef struct {
    control_context_t *ctx;
    int *MAX;
    int *MIN;
    int tar_max;
    int tar_min;
    int *last_i;
} pid_task_args_t;


//────────────────Inicializações e setups────────────────────────────────────────
void setup_motor_gpio(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AIN1_GPIO) | (1ULL << AIN2_GPIO) | (1ULL << BIN1_GPIO) | (1ULL << BIN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(AIN1_GPIO, 0);
    gpio_set_level(AIN2_GPIO, 0);
	gpio_set_level(BIN1_GPIO, 0);
	gpio_set_level(BIN2_GPIO, 0);
}
void setup_pwm(void) {
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch_a = {
        .gpio_num       = PWMA_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_A,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // inicialmente parado
        .hpoint         = 0,
    };
    ledc_channel_config(&ch_a);
	
	ledc_channel_config_t ch_b = {
	    .gpio_num       = PWMB_GPIO,
	    .speed_mode     = LEDC_MODE,
	    .channel        = LEDC_CHANNEL_B,
	    .intr_type      = LEDC_INTR_DISABLE,
	    .timer_sel      = LEDC_TIMER,
	    .duty           = 0, // inicialmente parado
	    .hpoint         = 0,
	};
	ledc_channel_config(&ch_b);
}

void qtr_adc_init(control_context_t *ctx) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = QTR_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &ctx->adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = QTR_ADC_ATTEN,
        .bitwidth = QTR_ADC_BITWIDTH,
    };

    for (int i = 0; i < QTR_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(ctx->adc_handle, qtr_sensors[i].channel, &chan_cfg));
    }

	adc_cali_curve_fitting_config_t cali_cfg = {
	        .unit_id  = QTR_ADC_UNIT,
	        .atten    = QTR_ADC_ATTEN,
	        .bitwidth = QTR_ADC_BITWIDTH,
	    };
	    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &ctx -> cali_handle));
		// ADC2 - Sensores laterais
		   adc_oneshot_unit_init_cfg_t init_cfg_side = {
		       .unit_id  = ADC_UNIT_2,
		       .ulp_mode = ADC_ULP_MODE_DISABLE,
		   };
		   ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg_side, &ctx->adc_handle_side));

		   for (int i = 0; i < QTR_SIDE_COUNT; i++) {
		       ESP_ERROR_CHECK(adc_oneshot_config_channel(ctx->adc_handle_side, qtr_right_sensor[i].channel, &chan_cfg));
		   }

		   adc_cali_curve_fitting_config_t cali_cfg_side = {
		       .unit_id  = ADC_UNIT_2,
		       .atten    = QTR_ADC_ATTEN,
		       .bitwidth = QTR_ADC_BITWIDTH,
		   };
		   ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg_side, &ctx->cali_handle_side));

}

void setup_gpio(void) {
    // Configurar o pino de saída
    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << BUTTON_OUT),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&output_conf);

    // Configurar o pino de entrada
    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << BUTTON_IN),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&input_conf);
}


//────────────────Função de Calibração────────────────────────────────────────────────
/*void get_MIN_MAX(int *MAX, int *MIN, control_context_t *ctx) {
    int value = 0;
    int input_value = 1;
    while (input_value == 1) {
        if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_ERROR_CHECK(adc_oneshot_read(ctx->adc_handle, qtr_right_sensor->channel, &value));
            xSemaphoreGive(adc_mutex);
        }
        *MAX = (*MAX < value) ? value : *MAX;
        *MIN = (*MIN > value) ? value : *MIN;
        ESP_LOGI(TAG, "Sensor  Max: %d   Min: %d", *MAX, *MIN);
        vTaskDelay(pdMS_TO_TICKS(700));
        input_value = gpio_get_level(BUTTON_IN);
    }
    return;
}*/


void get_min_max_avg_per_sensor(int readings[QTR_COUNT][TOTAL_READINGS], control_context_t *ctx) {
    int num;
    int value = 0;
    int input_value = 1;
    int index[QTR_COUNT] = {0};

    while (input_value == 1) {
        for (int i = 0; i < QTR_COUNT; i++) {
            if (index[i] < TOTAL_READINGS) {
                if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    ESP_ERROR_CHECK(adc_oneshot_read(ctx->adc_handle, qtr_sensors[i].channel, &value));
                    xSemaphoreGive(adc_mutex);
                }
                num = index[i];
                index[i]++;
                readings[i][num] = value;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        input_value = gpio_get_level(BUTTON_IN);

        int full = 1;
        for (int i = 0; i < QTR_COUNT; i++) {
            if (index[i] < TOTAL_READINGS) {
                full = 0;
                break;
            }
        }
        if (full) break;
    }
}


void avg_MIN_MAX(int readings[QTR_COUNT][TOTAL_READINGS],int *avg_MIN,int *avg_MAX){
	float total_MIN[QTR_COUNT] = {0};
	float total_MAX[QTR_COUNT] = {0};
	int min_count[QTR_COUNT] = {0};
	int max_count[QTR_COUNT] = {0};
	
	
	
	for(int i = 0;i<QTR_COUNT;i++){
		for(int j = 0;j<TOTAL_READINGS;j++){
			//ESP_LOGI(TAG,"Estou em avg_MIN_MAX");
			if(readings[i][j]<2048){
				min_count[i]++;
				total_MIN[i] += readings[i][j];
			} else{
				max_count[i]++;
				total_MAX[i] += readings[i][j];
			}
		}
		
		avg_MIN[i] = (int)(total_MIN[i] / min_count[i]);
		avg_MAX[i] = (int)(total_MAX[i] / max_count[i]);
		//ESP_LOGI(TAG,"sensor %d -> MAX: %d ; MIN: %d", i , avg_MAX[i], avg_MIN[i]);
		
	}
	return;
}

//────────────────Função de mapeamento───────────────────────────────────────────────
int map(int raw, int MIN, int MAX, int tar_min, int tar_max){
	if(MAX - MIN == 0) return tar_min;//Evitar divisão por 0
	return (raw - MIN) * (tar_max - tar_min) / (MAX - MIN) + tar_min;
}

//────────────────Função de Leitura de Sensor────────────────────────────────────────
int qtr_read_pos(control_context_t *ctx, int *MAX, int *MIN, int tar_max, int tar_min, int *last_pos) {
    int64_t den = 0, num = 0;
    int pos;
    for (int i = 0; i < 8; i++) {
        int raw = 0;
        if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_ERROR_CHECK(adc_oneshot_read(ctx->adc_handle, qtr_sensors[i].channel, &raw));
            xSemaphoreGive(adc_mutex);
        }

        int val = tar_max - map(raw, MIN[i], MAX[i], tar_min, tar_max);
        val = (val < tar_min) ? tar_min : (val > tar_max) ? tar_max : val;
        //ESP_LOGI(TAG, "Sensor %d: %d", i, val);
        last_pos[i] = val;

        num += (int64_t)val * (i * 1000);
        den += (int64_t)val;
    }

    if (den == 0) return -1;
    pos = (int)(num / den);
    return pos;
}


int qtr_read_right(control_context_t *ctx, int *MAX, int *MIN, int tar_max, int tar_min) {
    int raw = 0;
    if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_ERROR_CHECK(adc_oneshot_read(ctx->adc_handle_side, qtr_right_sensor[1].channel, &raw));
	
        xSemaphoreGive(adc_mutex);
    }
    int val = tar_max - map(raw, *MIN, *MAX, tar_min, tar_max);
    val = (val < tar_min) ? tar_min : (val > tar_max) ? tar_max : val;
	ESP_LOGI(TAG,"VALOR DO SENSOR LATERAL: %d", val);

    return val;
}


//────────────────Função de controle dos motores──────────────────────────────────────
void controle_motor(int position, uint16_t duty){
	//MOTOR A = Direito, MOTOR B = Esquerdo
	if(abs(position) < MARGIN_ERR){
		gpio_set_level(AIN1_GPIO, 1);
		gpio_set_level(AIN2_GPIO, 0);
		ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, duty);
		ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
		gpio_set_level(BIN1_GPIO, 1);
		gpio_set_level(BIN2_GPIO, 0);
		ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, duty);
		ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
		}
	else if(position > 0){
		gpio_set_level(AIN1_GPIO, 0);
		gpio_set_level(AIN2_GPIO, 1);
		gpio_set_level(BIN1_GPIO, 1);
		gpio_set_level(BIN2_GPIO, 0);
	}
	else if(position < 0){
		gpio_set_level(AIN1_GPIO, 1);
		gpio_set_level(AIN2_GPIO, 0);
		gpio_set_level(BIN1_GPIO, 0);
		gpio_set_level(BIN2_GPIO, 1);
	}
	
	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, duty);
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, duty);
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

//────────────────Função do PID────────────────────────────────────────────────────────
void pid_task(void *args) {
    pid_task_args_t *task_args = (pid_task_args_t *)args;
    control_context_t *ctx = task_args->ctx;
    int *MAX = task_args->MAX;
    int *MIN = task_args->MIN;
    int tar_max = task_args->tar_max;
    int tar_min = task_args->tar_min;
    int *last_i = task_args->last_i;

    while (1) {
        int last_pos[QTR_COUNT];
        int flag = 0;
        float max_speed = 1023.0f;
        float base_speed = 511.0f;

        int pos = qtr_read_pos(ctx, MAX, MIN, tar_max, tar_min, last_pos);

        for (int i = 0; i < QTR_COUNT; i++) {
            if (last_pos[i] < 1000) {
                flag++;
            } else {
                *last_i = i;
            }
        }
        //ESP_LOGI(TAG, "last_i = %d", *last_i);

        ctx->last_position = pos;
        ctx->report_position = pos;

        float error = (float)(TARGET_POSITION - pos);
        float correction = 0;
        pid_compute(ctx->pid_ctrl, error, &correction);

        float lSpeed = base_speed + correction;
        float rSpeed = base_speed - correction;

        lSpeed = (lSpeed > max_speed) ? max_speed : (lSpeed < -max_speed) ? -max_speed : lSpeed;
        rSpeed = (rSpeed > max_speed) ? max_speed : (rSpeed < -max_speed) ? -max_speed : rSpeed;

        if (flag == QTR_COUNT) {
            if (*last_i >= 4) {
                //ESP_LOGI(TAG, "Linha perdida → Virando Direita");

                gpio_set_level(AIN1_GPIO, 1);
                gpio_set_level(AIN2_GPIO, 0);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, (uint32_t)fabs(max_speed));
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);

                gpio_set_level(BIN1_GPIO, 0);
                gpio_set_level(BIN2_GPIO, 1);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, (uint32_t)fabs(base_speed));
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
            } else {
               // ESP_LOGI(TAG, "Linha perdida → Virando Esquerda");

                gpio_set_level(AIN1_GPIO, 0);
                gpio_set_level(AIN2_GPIO, 1);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, (uint32_t)fabs(base_speed));
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);

                gpio_set_level(BIN1_GPIO, 1);
                gpio_set_level(BIN2_GPIO, 0);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, (uint32_t)fabs(max_speed));
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
            }
        } else if (fabs(error) <= MARGIN_ERR) {
            //ESP_LOGI(TAG, "Linha centralizada");

            gpio_set_level(AIN1_GPIO, 1);
            gpio_set_level(AIN2_GPIO, 0);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, max_speed);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);

            gpio_set_level(BIN1_GPIO, 1);
            gpio_set_level(BIN2_GPIO, 0);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, max_speed);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
        } else {
            //ESP_LOGI(TAG, "Correção PID → Erro: %.2f | Correção: %.2f", error, correction);

            if (rSpeed >= 0) {
                gpio_set_level(AIN1_GPIO, 1);
                gpio_set_level(AIN2_GPIO, 0);
            } else {
                gpio_set_level(AIN1_GPIO, 0);
                gpio_set_level(AIN2_GPIO, 1);
            }
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, (uint32_t)fabs(rSpeed));
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);

            if (lSpeed >= 0) {
                gpio_set_level(BIN1_GPIO, 1);
                gpio_set_level(BIN2_GPIO, 0);
            } else {
                gpio_set_level(BIN1_GPIO, 0);
                gpio_set_level(BIN2_GPIO, 1);
            }
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, (uint32_t)fabs(lSpeed));
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Frequência de atualização da Task (100Hz)
    }
}


void lateral_line_task(void *args) {
    control_context_t *ctx = (control_context_t *)args;
    static int line_count = 0;
    int last_pos[QTR_COUNT], MAX = 3300, MIN = 200;
    int flag, last_i;
    float max_speed = 1023.0f * 0.5f;
    float base_speed = 511.0f * 0.5f;
	

    // Defina seus limites de calibração lateral
    //get_MIN_MAX(&MAX, &MIN, ctx);
    while (1) {
        if (qtr_read_right(ctx, &MAX, &MIN, 2000, 0) > THRESHOLD) {
			ESP_LOGI(TAG,"Linhas: %d", line_count);
            line_count++;
            ESP_LOGI(TAG, "Linha lateral detectada! Total: %d", line_count);

            if (line_count >= 26) {
                int timer = 0;
                while (timer < 6) {
					ESP_LOGI(TAG,"TEMPO desde a linha: %d", timer);
                    int pos = qtr_read_pos(ctx, avg_MAX, avg_MIN, 2000, 0, last_pos);  // Passe NULLs ou ajuste com suas variáveis reais
                    flag = 0;

                    for (int i = 0; i < QTR_COUNT; i++) {
                        if (last_pos[i] < 1000) {
                            flag++;
                        } else {
                            last_i = i;
                        }
                    }

                    ctx->last_position = pos;
                    ctx->report_position = pos;

                    float error = (float)(TARGET_POSITION - pos);
                    float correction = 0;
                    pid_compute(ctx->pid_ctrl, error, &correction);

                    float lSpeed = base_speed + correction;
                    float rSpeed = base_speed - correction;

                    lSpeed = (lSpeed > max_speed) ? max_speed : (lSpeed < -max_speed) ? -max_speed : lSpeed;
                    rSpeed = (rSpeed > max_speed) ? max_speed : (rSpeed < -max_speed) ? -max_speed : rSpeed;

                    if (flag == QTR_COUNT) {
						if(last_i >= 4){
										//ESP_LOGI(TAG, "Posição: %d | Erro: %.2f | Correção PID: %.2f", pos, error, correction);
										
										gpio_set_level(AIN1_GPIO, 1);
										gpio_set_level(AIN2_GPIO, 0);
										//ESP_LOGI(TAG, "Motor A (Direito) → Ré    | PWM: %.0f", (max_speed));
										ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, (uint32_t)fabs(max_speed));
										ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
										
										
										gpio_set_level(BIN1_GPIO, 0);
										gpio_set_level(BIN2_GPIO, 1);
										//ESP_LOGI(TAG, "Motor B (Esquerdo) → Frente | PWM: %.0f", (-base_speed));
										ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, (uint32_t)fabs(base_speed));
										ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
										}
										
										if(last_i < 4){
										//ESP_LOGI(TAG, "Posição: %d | Erro: %.2f | Correção PID: %.2f", pos, error, correction);
													
										gpio_set_level(AIN1_GPIO, 0);
										gpio_set_level(AIN2_GPIO, 1);
										//ESP_LOGI(TAG, "Motor A (Direito) → Frente    | PWM: %.0f", (-base_speed));
										ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, (uint32_t)fabs(base_speed));
										ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
													
													
										gpio_set_level(BIN1_GPIO, 1);
										gpio_set_level(BIN2_GPIO, 0);
										//ESP_LOGI(TAG, "Motor B (Esquerdo) → Ré | PWM: %.0f", (max_speed));
										ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, (uint32_t)fabs(max_speed));
										ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
										}

										
									
                    } else if (fabs(error) <= MARGIN_ERR) {
                        gpio_set_level(AIN1_GPIO, 1);
                        gpio_set_level(AIN2_GPIO, 0);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, max_speed);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);

                        gpio_set_level(BIN1_GPIO, 1);
                        gpio_set_level(BIN2_GPIO, 0);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, max_speed);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
						pid_compute(ctx->pid_ctrl, error, &correction);
                    } else {
                        if (rSpeed >= 0) {
                            gpio_set_level(AIN1_GPIO, 1);
                            gpio_set_level(AIN2_GPIO, 0);
                        } else {
                            gpio_set_level(AIN1_GPIO, 0);
                            gpio_set_level(AIN2_GPIO, 1);
                        }
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, (uint32_t)fabs(rSpeed));
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);

                        if (lSpeed >= 0) {
                            gpio_set_level(BIN1_GPIO, 1);
                            gpio_set_level(BIN2_GPIO, 0);
                        } else {
                            gpio_set_level(BIN1_GPIO, 0);
                            gpio_set_level(BIN2_GPIO, 1);
                        }
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, (uint32_t)fabs(lSpeed));
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
                    }

                    timer++;
                    vTaskDelay(pdMS_TO_TICKS(50));
                }

                // Parar os motores
                gpio_set_level(AIN1_GPIO, 0);
                gpio_set_level(AIN2_GPIO, 0);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);

                gpio_set_level(BIN1_GPIO, 0);
                gpio_set_level(BIN2_GPIO, 0);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);

                ESP_LOGI(TAG, "Fim da pista, robô parado");
				vTaskDelete(pid_task_handle);
				while(1){}
            }

            // Delay para evitar múltiplas contagens da mesma linha
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        // Pequeno delay para evitar sobrecarga de CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
	typedef struct {
	    control_context_t *ctx;
	    int *MAX;
	    int *MIN;
	    int tar_max;
	    int tar_min;
	    int *last_i;
	} pid_task_args_t;
	adc_mutex = xSemaphoreCreateMutex();
	setup_gpio();
	gpio_set_level(BUTTON_OUT, 1);
	static control_context_t ctrl = {
		.last_position = TARGET_POSITION,
		.report_position = TARGET_POSITION,
	};
	static const int tar_min = 0, tar_max = 2000;
	
	int /*MAX[8]={0,0,0,0,0,0,0,0,}, 
	MIN[8] = {4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096},*/

	last_i = 0;
	
	qtr_adc_init(&ctrl);
	setup_motor_gpio();
	setup_pwm();
	
	pid_ctrl_parameter_t pid_params = {
	    .kp = 0.075f,
	    .ki = 0.00f,
	    .kd = 0.0002f,
	    .cal_type = PID_CAL_TYPE_POSITIONAL,
	    .max_output = 512.0f,
	    .min_output = -512.0f,
	    .max_integral = 0.6f,
	    .min_integral = -0.6f,
	};
	pid_ctrl_config_t pid_cfg = {.init_param = pid_params};
	ESP_ERROR_CHECK(pid_new_control_block(&pid_cfg, &ctrl.pid_ctrl));
	
	//get_MIN_MAX(MAX, MIN, &ctrl);
	
	get_min_max_avg_per_sensor(readings, &ctrl);
	avg_MIN_MAX(readings, avg_MIN, avg_MAX);
	vTaskDelay(pdMS_TO_TICKS(2000));
	pid_task_args_t pid_args = {
	    .ctx = &ctrl,
	    .MAX = avg_MAX,
	    .MIN = avg_MIN,
	    .tar_max = tar_max,
	    .tar_min = tar_min,
	    .last_i = &last_i,
	};
	
	xTaskCreatePinnedToCore(pid_task, "PID Task", 8192, &pid_args, 5, &pid_task_handle, 1);
	xTaskCreatePinnedToCore(lateral_line_task, "Lateral Line Task", 8192, &ctrl, 5, NULL, 0);

		

		vTaskDelay(pdMS_TO_TICKS(100));
		
	}


