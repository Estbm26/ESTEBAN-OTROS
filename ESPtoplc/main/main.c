#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"

#include "mqtt_client.h"
#include "driver/gpio.h"


/* =========================================================
 * TAG
 * ========================================================= */

static const char *TAG = "ESP_PLC";


/* =========================================================
 * WIFI
 * ========================================================= */

#define WIFI_SSID "CLAROR84KU"
#define WIFI_PASS "48575443C1B857B5"

#define MAX_WIFI_RETRY 10


/* =========================================================
 * MQTT
 * ========================================================= */

#define MQTT_BROKER "mqtt://broker.emqx.io:1883"


/* =========================================================
 * SALIDAS
 * ========================================================= */

#define PIN_INICIO      GPIO_NUM_16
#define PIN_STOP        GPIO_NUM_17
#define PIN_EMERGENCIA  GPIO_NUM_18


/* =========================================================
 * SALIDAS POR COLOR
 *
 * AZUL  -> GPIO10
 * VERDE -> GPIO15
 * ROJO  -> GPIO23
 * ========================================================= */

#define PIN_SALIDA_AZUL   GPIO_NUM_10
#define PIN_SALIDA_VERDE  GPIO_NUM_15
#define PIN_SALIDA_ROJO   GPIO_NUM_23


/* =========================================================
 * ENTRADAS
 * ========================================================= */

#define PIN_TAMANO1  GPIO_NUM_13
#define PIN_TAMANO2  GPIO_NUM_14
#define PIN_TAMANO3  GPIO_NUM_35
#define PIN_METALES  GPIO_NUM_21


/* =========================================================
 * BOTON RESET DEL CONTADOR
 *
 * GPIO19
 *
 * SIN PRESIONAR -> 1
 * PRESIONADO    -> 0
 * ========================================================= */

#define PIN_RESET_CONTADOR GPIO_NUM_19


/* =========================================================
 * SENSOR ULTRASONICO
 *
 * TRIG -> GPIO32
 * ECHO -> GPIO33
 * ========================================================= */

#define PIN_TRIG GPIO_NUM_32
#define PIN_ECHO GPIO_NUM_33


/* =========================================================
 * CONFIGURACION ULTRASONICO
 * ========================================================= */

/*
 * Distancia a partir de la cual consideramos
 * que existe un objeto.
 */

#define DISTANCIA_DETECCION_CM 15.0


/*
 * Distancia necesaria para volver a armar
 * el detector y permitir contar otro objeto.
 */

#define DISTANCIA_REARME_CM 20.0


/*
 * Timeout para esperar el ECHO.
 * 30000 us = 30 ms
 */

#define ULTRASONICO_TIMEOUT_US 30000


/* =========================================================
 * SENSOR DE COLOR TCS3200
 *
 * S2  -> GPIO26
 * S3  -> GPIO27
 * OUT -> GPIO34
 * ========================================================= */

#define PIN_S2      GPIO_NUM_26
#define PIN_S3      GPIO_NUM_27
#define PIN_OUTTCS  GPIO_NUM_34


/* =========================================================
 * TCS3200
 * ========================================================= */

#define TCS_MEASURE_TIME_MS 100


/* =========================================================
 * EVENTOS
 * ========================================================= */

#define WIFI_CONNECTED_BIT  BIT0
#define MQTT_CONNECTED_BIT  BIT1


/* =========================================================
 * VARIABLES GLOBALES
 * ========================================================= */

static EventGroupHandle_t s_event_group = NULL;

static esp_mqtt_client_handle_t s_mqtt_client = NULL;

static int s_retry_num = 0;


/* =========================================================
 * CONTADOR TCS3200
 * ========================================================= */

static volatile uint32_t tcs_pulse_count = 0;


/* =========================================================
 * CONTADOR DE OBJETOS ULTRASONICO
 * ========================================================= */

static volatile uint32_t contador_objetos = 0;


/*
 * Indica si actualmente existe un objeto
 * dentro de la zona de deteccion.
 *
 * false = no hay objeto
 * true  = hay objeto
 */

static volatile bool objeto_detectado = false;


/* =========================================================
 * ISR TCS3200
 * ========================================================= */

static void IRAM_ATTR tcs3200_isr_handler(void *arg)
{
    tcs_pulse_count++;
}


/* =========================================================
 * SELECCIONAR FILTRO TCS3200
 *
 * S2 S3
 *
 * 0  0 = ROJO
 * 0  1 = AZUL
 * 1  1 = VERDE
 * 1  0 = CLEAR
 * ========================================================= */

static void tcs3200_select_filter(
    int s2,
    int s3)
{
    gpio_set_level(
        PIN_S2,
        s2
    );

    gpio_set_level(
        PIN_S3,
        s3
    );
}


/* =========================================================
 * MEDIR FRECUENCIA TCS3200
 * ========================================================= */

static uint32_t tcs3200_measure(
    int s2,
    int s3)
{
    tcs3200_select_filter(
        s2,
        s3
    );

    vTaskDelay(
        pdMS_TO_TICKS(20)
    );

    tcs_pulse_count = 0;

    vTaskDelay(
        pdMS_TO_TICKS(
            TCS_MEASURE_TIME_MS
        )
    );

    uint32_t pulses =
        tcs_pulse_count;

    uint32_t frequency =
        pulses * 10;

    return frequency;
}


/* =========================================================
 * DETERMINAR COLOR
 * ========================================================= */

static const char *tcs3200_detect_color(
    uint32_t red,
    uint32_t green,
    uint32_t blue)
{
    if (
        red < 10 &&
        green < 10 &&
        blue < 10
    )
    {
        return "SIN_COLOR";
    }

    if (
        red >= green &&
        red >= blue
    )
    {
        return "ROJO";
    }

    if (
        green >= red &&
        green >= blue
    )
    {
        return "VERDE";
    }

    return "AZUL";
}


/* =========================================================
 * ACTIVAR SALIDA SEGUN COLOR
 *
 * AZUL  -> GPIO10
 * VERDE -> GPIO15
 * ROJO  -> GPIO23
 *
 * Solo una salida permanece activa.
 * ========================================================= */

static void activar_salida_color(
    const char *color)
{
    gpio_set_level(
        PIN_SALIDA_AZUL,
        0
    );

    gpio_set_level(
        PIN_SALIDA_VERDE,
        0
    );

    gpio_set_level(
        PIN_SALIDA_ROJO,
        0
    );


    if (
        strcmp(
            color,
            "AZUL"
        ) == 0
    )
    {
        gpio_set_level(
            PIN_SALIDA_AZUL,
            1
        );

        ESP_LOGI(
            TAG,
            "COLOR AZUL -> GPIO10 ACTIVADO"
        );
    }

    else if (
        strcmp(
            color,
            "VERDE"
        ) == 0
    )
    {
        gpio_set_level(
            PIN_SALIDA_VERDE,
            1
        );

        ESP_LOGI(
            TAG,
            "COLOR VERDE -> GPIO15 ACTIVADO"
        );
    }

    else if (
        strcmp(
            color,
            "ROJO"
        ) == 0
    )
    {
        gpio_set_level(
            PIN_SALIDA_ROJO,
            1
        );

        ESP_LOGI(
            TAG,
            "COLOR ROJO -> GPIO23 ACTIVADO"
        );
    }

    else
    {
        ESP_LOGI(
            TAG,
            "SIN COLOR -> TODAS LAS SALIDAS APAGADAS"
        );
    }
}


/* =========================================================
 * MEDIR DISTANCIA ULTRASONICA
 *
 * Retorna:
 *
 * distancia en cm
 *
 * Si existe un error retorna -1.0
 * ========================================================= */

static float ultrasonico_medir_distancia(void)
{
    /*
     * Asegurar TRIG en LOW
     */

    gpio_set_level(
        PIN_TRIG,
        0
    );

    esp_rom_delay_us(2);


    /*
     * Pulso de 10 us
     */

    gpio_set_level(
        PIN_TRIG,
        1
    );

    esp_rom_delay_us(10);

    gpio_set_level(
        PIN_TRIG,
        0
    );


    /*
     * Esperar a que ECHO pase a HIGH
     */

    int64_t timeout_inicio =
        esp_timer_get_time();

    while (
        gpio_get_level(PIN_ECHO) == 0
    )
    {
        if (
            esp_timer_get_time() -
            timeout_inicio >
            ULTRASONICO_TIMEOUT_US
        )
        {
            return -1.0;
        }
    }


    /*
     * Inicio del pulso ECHO
     */

    int64_t inicio_echo =
        esp_timer_get_time();


    /*
     * Esperar a que ECHO vuelva a LOW
     */

    while (
        gpio_get_level(PIN_ECHO) == 1
    )
    {
        if (
            esp_timer_get_time() -
            inicio_echo >
            ULTRASONICO_TIMEOUT_US
        )
        {
            return -1.0;
        }
    }


    /*
     * Duracion del pulso
     */

    int64_t fin_echo =
        esp_timer_get_time();

    int64_t duracion =
        fin_echo - inicio_echo;


    /*
     * Distancia en cm
     */

    float distancia =
        (float)duracion / 58.0f;


    return distancia;
}


/* =========================================================
 * PUBLICAR CONTADOR ULTRASONICO
 * ========================================================= */

static void publicar_contador_ultrasonico(void)
{
    if (
        s_mqtt_client == NULL
    )
    {
        return;
    }

    /*
     * Verificar que MQTT este conectado
     */

    EventBits_t bits =
        xEventGroupGetBits(
            s_event_group
        );

    if (
        !(bits & MQTT_CONNECTED_BIT)
    )
    {
        return;
    }


    char payload[32];

    snprintf(
        payload,
        sizeof(payload),
        "%lu",
        (unsigned long)contador_objetos
    );


    esp_mqtt_client_publish(
        s_mqtt_client,
        "ultrasonico/conteo",
        payload,
        0,
        1,
        1
    );


    /*
     * Mostrar tambien el valor publicado
     * en la terminal.
     */

    ESP_LOGI(
        TAG,
        "MQTT -> ultrasonico/conteo = %s",
        payload
    );
}


/* =========================================================
 * RESET CONTADOR
 *
 * IMPORTANTE:
 *
 * El RESET solamente modifica:
 *
 *     contador_objetos
 *
 * NO modifica:
 *
 *     objeto_detectado
 *
 * Por lo tanto, el reset NO reinicia el estado
 * fisico del sensor.
 * ========================================================= */

static void reset_contador(void)
{
    /*
     * Reset solamente del contador.
     */

    contador_objetos = 0;


    /*
     * Mostrar reset en terminal.
     */

    ESP_LOGI(
        TAG,
        "========================================"
    );

    ESP_LOGI(
        TAG,
        "RESET CONTADOR ULTRASONICO"
    );

    ESP_LOGI(
        TAG,
        "CONTEO ULTRASONICO: %lu",
        (unsigned long)contador_objetos
    );

    ESP_LOGI(
        TAG,
        "========================================"
    );


    /*
     * Publicar inmediatamente el 0
     * en MQTT.
     */

    publicar_contador_ultrasonico();
}


/* =========================================================
 * TAREA SENSOR ULTRASONICO
 * ========================================================= */

static void ultrasonico_task(
    void *pvParameters)
{
    int ultimo_estado_reset = 1;


    while (1)
    {
        /*
         * ================================================
         * BOTON FISICO DE RESET
         * ================================================
         */

        int estado_reset =
            gpio_get_level(
                PIN_RESET_CONTADOR
            );


        /*
         * Detectar flanco de bajada.
         *
         * 1 -> 0 = boton presionado
         */

        if (
            ultimo_estado_reset == 1 &&
            estado_reset == 0
        )
        {
            reset_contador();


            /*
             * Anti-rebote
             */

            vTaskDelay(
                pdMS_TO_TICKS(300)
            );
        }


        ultimo_estado_reset =
            estado_reset;


        /*
         * ================================================
         * SOLO MEDIR SI MQTT ESTA CONECTADO
         * ================================================
         */

        EventBits_t bits =
            xEventGroupGetBits(
                s_event_group
            );


        if (
            bits & MQTT_CONNECTED_BIT
        )
        {
            /*
             * ============================================
             * MEDIR DISTANCIA
             * ============================================
             */

            float distancia =
                ultrasonico_medir_distancia();


            /*
             * ============================================
             * DISTANCIA VALIDA
             * ============================================
             */

            if (
                distancia > 0
            )
            {
                char payload[32];


                /*
                 * ========================================
                 * PUBLICAR DISTANCIA
                 * ========================================
                 */

                snprintf(
                    payload,
                    sizeof(payload),
                    "%.2f",
                    distancia
                );

                esp_mqtt_client_publish(
                    s_mqtt_client,
                    "ultrasonico/distancia",
                    payload,
                    0,
                    1,
                    1
                );


                /*
                 * Mostrar distancia en terminal
                 */

                ESP_LOGI(
                    TAG,
                    "ULTRASONICO -> DISTANCIA: %.2f cm",
                    distancia
                );


                /*
                 * ========================================
                 * DETECCION DE OBJETO
                 * ========================================
                 */

                if (
                    distancia <=
                    DISTANCIA_DETECCION_CM
                )
                {
                    /*
                     * Solo contar si anteriormente
                     * NO habia objeto.
                     */

                    if (
                        objeto_detectado == false
                    )
                    {
                        objeto_detectado = true;


                        /*
                         * Incrementar contador
                         */

                        contador_objetos++;


                        /*
                         * =================================
                         * MOSTRAR CONTEO EN TERMINAL
                         * =================================
                         */

                        ESP_LOGI(
                            TAG,
                            "========================================"
                        );

                        ESP_LOGI(
                            TAG,
                            "OBJETO DETECTADO"
                        );

                        ESP_LOGI(
                            TAG,
                            "DISTANCIA: %.2f cm",
                            distancia
                        );

                        ESP_LOGI(
                            TAG,
                            "CONTEO ULTRASONICO: %lu",
                            (unsigned long)contador_objetos
                        );

                        ESP_LOGI(
                            TAG,
                            "========================================"
                        );


                        /*
                         * =================================
                         * PUBLICAR NUEVO CONTEO EN MQTT
                         * =================================
                         */

                        publicar_contador_ultrasonico();
                    }
                }


                /*
                 * ========================================
                 * REARMAR SENSOR
                 * ========================================
                 *
                 * Cuando el objeto se aleja a 20 cm
                 * o mas, el sensor queda listo para
                 * contar otro objeto.
                 */

                if (
                    objeto_detectado == true &&
                    distancia >=
                    DISTANCIA_REARME_CM
                )
                {
                    objeto_detectado = false;


                    ESP_LOGI(
                        TAG,
                        "OBJETO SALIO DE LA ZONA"
                    );

                    ESP_LOGI(
                        TAG,
                        "SENSOR ULTRASONICO REARMADO"
                    );
                }
            }
            else
            {
                /*
                 * No se recibio ECHO correctamente.
                 */

                ESP_LOGW(
                    TAG,
                    "ULTRASONICO -> SIN ECHO / DISTANCIA NO VALIDA"
                );
            }
        }


        /*
         * Medir aproximadamente cada 200 ms.
         */

        vTaskDelay(
            pdMS_TO_TICKS(200)
        );
    }
}


/* =========================================================
 * TAREA TCS3200
 * ========================================================= */

static void tcs3200_task(
    void *pvParameters)
{
    while (1)
    {
        EventBits_t bits =
            xEventGroupGetBits(
                s_event_group
            );


        if (
            bits & MQTT_CONNECTED_BIT
        )
        {
            /*
             * ROJO
             */

            uint32_t red =
                tcs3200_measure(
                    0,
                    0
                );


            /*
             * AZUL
             */

            uint32_t blue =
                tcs3200_measure(
                    0,
                    1
                );


            /*
             * VERDE
             */

            uint32_t green =
                tcs3200_measure(
                    1,
                    1
                );


            /*
             * DETERMINAR COLOR
             */

            const char *color =
                tcs3200_detect_color(
                    red,
                    green,
                    blue
                );


            /*
             * ACTIVAR SALIDA
             */

            activar_salida_color(
                color
            );


            /*
             * TERMINAL
             */

            ESP_LOGI(
                TAG,
                "TCS3200 -> ROJO: %lu Hz | VERDE: %lu Hz | AZUL: %lu Hz | COLOR: %s",
                (unsigned long)red,
                (unsigned long)green,
                (unsigned long)blue,
                color
            );


            char payload[32];


            /*
             * MQTT ROJO
             */

            snprintf(
                payload,
                sizeof(payload),
                "%lu",
                (unsigned long)red
            );

            esp_mqtt_client_publish(
                s_mqtt_client,
                "tcs3200/rojo",
                payload,
                0,
                1,
                1
            );


            /*
             * MQTT VERDE
             */

            snprintf(
                payload,
                sizeof(payload),
                "%lu",
                (unsigned long)green
            );

            esp_mqtt_client_publish(
                s_mqtt_client,
                "tcs3200/verde",
                payload,
                0,
                1,
                1
            );


            /*
             * MQTT AZUL
             */

            snprintf(
                payload,
                sizeof(payload),
                "%lu",
                (unsigned long)blue
            );

            esp_mqtt_client_publish(
                s_mqtt_client,
                "tcs3200/azul",
                payload,
                0,
                1,
                1
            );


            /*
             * MQTT COLOR
             */

            esp_mqtt_client_publish(
                s_mqtt_client,
                "tcs3200/color",
                color,
                0,
                1,
                1
            );
        }


        vTaskDelay(
            pdMS_TO_TICKS(500)
        );
    }
}


/* =========================================================
 * MQTT EVENT HANDLER
 * ========================================================= */

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    esp_mqtt_event_handle_t event =
        (esp_mqtt_event_handle_t)event_data;


    switch (event_id)
    {
        /* =================================================
         * MQTT CONECTADO
         * ================================================= */

        case MQTT_EVENT_CONNECTED:

            ESP_LOGI(
                TAG,
                "MQTT CONECTADO"
            );

            xEventGroupSetBits(
                s_event_group,
                MQTT_CONNECTED_BIT
            );


            /*
             * Suscripcion INICIO
             */

            esp_mqtt_client_subscribe(
                s_mqtt_client,
                "inicio",
                1
            );


            /*
             * Suscripcion STOP
             */

            esp_mqtt_client_subscribe(
                s_mqtt_client,
                "stop",
                1
            );


            /*
             * Suscripcion EMERGENCIA
             */

            esp_mqtt_client_subscribe(
                s_mqtt_client,
                "emergencia",
                1
            );


            /*
             * Suscripcion RESET ULTRASONICO
             */

            esp_mqtt_client_subscribe(
                s_mqtt_client,
                "ultrasonico/reset",
                1
            );


            /*
             * Publicar contador actual
             */

            publicar_contador_ultrasonico();


            ESP_LOGI(
                TAG,
                "MQTT: suscripciones realizadas"
            );

            break;


        /* =================================================
         * MQTT DESCONECTADO
         * ================================================= */

        case MQTT_EVENT_DISCONNECTED:

            xEventGroupClearBits(
                s_event_group,
                MQTT_CONNECTED_BIT
            );

            ESP_LOGW(
                TAG,
                "MQTT desconectado"
            );

            break;


        /* =================================================
         * MENSAJE MQTT RECIBIDO
         * ================================================= */

        case MQTT_EVENT_DATA:

            /*
             * =============================================
             * INICIO
             * =============================================
             */

            if (
                event->topic_len == 6 &&
                strncmp(
                    event->topic,
                    "inicio",
                    6
                ) == 0
            )
            {
                if (
                    event->data_len == 2 &&
                    strncmp(
                        event->data,
                        "ON",
                        2
                    ) == 0
                )
                {
                    gpio_set_level(
                        PIN_INICIO,
                        1
                    );

                    ESP_LOGI(
                        TAG,
                        "MQTT -> INICIO: ON"
                    );
                }

                else if (
                    event->data_len == 3 &&
                    strncmp(
                        event->data,
                        "OFF",
                        3
                    ) == 0
                )
                {
                    gpio_set_level(
                        PIN_INICIO,
                        0
                    );

                    ESP_LOGI(
                        TAG,
                        "MQTT -> INICIO: OFF"
                    );
                }
            }


            /*
             * =============================================
             * STOP
             * =============================================
             */

            else if (
                event->topic_len == 4 &&
                strncmp(
                    event->topic,
                    "stop",
                    4
                ) == 0
            )
            {
                if (
                    event->data_len == 2 &&
                    strncmp(
                        event->data,
                        "ON",
                        2
                    ) == 0
                )
                {
                    gpio_set_level(
                        PIN_STOP,
                        1
                    );

                    ESP_LOGI(
                        TAG,
                        "MQTT -> STOP: ON"
                    );
                }

                else if (
                    event->data_len == 3 &&
                    strncmp(
                        event->data,
                        "OFF",
                        3
                    ) == 0
                )
                {
                    gpio_set_level(
                        PIN_STOP,
                        0
                    );

                    ESP_LOGI(
                        TAG,
                        "MQTT -> STOP: OFF"
                    );
                }
            }


            /*
             * =============================================
             * EMERGENCIA
             * =============================================
             */

            else if (
                event->topic_len == 10 &&
                strncmp(
                    event->topic,
                    "emergencia",
                    10
                ) == 0
            )
            {
                if (
                    event->data_len == 2 &&
                    strncmp(
                        event->data,
                        "ON",
                        2
                    ) == 0
                )
                {
                    gpio_set_level(
                        PIN_EMERGENCIA,
                        1
                    );

                    ESP_LOGI(
                        TAG,
                        "MQTT -> EMERGENCIA: ON"
                    );
                }

                else if (
                    event->data_len == 3 &&
                    strncmp(
                        event->data,
                        "OFF",
                        3
                    ) == 0
                )
                {
                    gpio_set_level(
                        PIN_EMERGENCIA,
                        0
                    );

                    ESP_LOGI(
                        TAG,
                        "MQTT -> EMERGENCIA: OFF"
                    );
                }
            }


            /*
             * =============================================
             * RESET CONTADOR ULTRASONICO
             *
             * "ultrasonico/reset" = 17 caracteres
             * =============================================
             */

            else if (
                event->topic_len == 17 &&
                strncmp(
                    event->topic,
                    "ultrasonico/reset",
                    17
                ) == 0
            )
            {
                if (
                    event->data_len == 2 &&
                    strncmp(
                        event->data,
                        "ON",
                        2
                    ) == 0
                )
                {
                    reset_contador();

                    ESP_LOGI(
                        TAG,
                        "MQTT -> ULTRASONICO RESET"
                    );
                }
            }

            break;


        /* =================================================
         * ERROR MQTT
         * ================================================= */

        case MQTT_EVENT_ERROR:

            ESP_LOGE(
                TAG,
                "ERROR MQTT"
            );

            break;


        default:

            break;
    }
}


/* =========================================================
 * WIFI EVENT HANDLER
 * ========================================================= */

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    /*
     * WIFI INICIADO
     */

    if (
        event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START
    )
    {
        ESP_LOGI(
            TAG,
            "Conectando a WiFi: %s",
            WIFI_SSID
        );

        esp_wifi_connect();
    }


    /*
     * WIFI DESCONECTADO
     */

    else if (
        event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_DISCONNECTED
    )
    {
        xEventGroupClearBits(
            s_event_group,
            WIFI_CONNECTED_BIT
        );

        if (
            s_retry_num < MAX_WIFI_RETRY
        )
        {
            s_retry_num++;

            esp_wifi_connect();
        }

        else
        {
            ESP_LOGE(
                TAG,
                "No se pudo conectar al WiFi"
            );
        }
    }


    /*
     * WIFI CONECTADO / IP OBTENIDA
     */

    else if (
        event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP
    )
    {
        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;

        s_retry_num = 0;

        xEventGroupSetBits(
            s_event_group,
            WIFI_CONNECTED_BIT
        );

        ESP_LOGI(
            TAG,
            "WIFI CONECTADO - IP: " IPSTR,
            IP2STR(&event->ip_info.ip)
        );
    }
}


/* =========================================================
 * WIFI INIT
 * ========================================================= */

static void wifi_init(void)
{
    ESP_ERROR_CHECK(
        esp_netif_init()
    );


    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );


    esp_netif_create_default_wifi_sta();


    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();


    ESP_ERROR_CHECK(
        esp_wifi_init(
            &cfg
        )
    );


    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL
        )
    );


    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL
        )
    );


    wifi_config_t wifi_config = {};


    strcpy(
        (char *)wifi_config.sta.ssid,
        WIFI_SSID
    );


    strcpy(
        (char *)wifi_config.sta.password,
        WIFI_PASS
    );


    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_STA
        )
    );


    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );


    ESP_ERROR_CHECK(
        esp_wifi_start()
    );


    ESP_LOGI(
        TAG,
        "WiFi inicializado"
    );
}


/* =========================================================
 * MQTT INIT
 * ========================================================= */

static void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {

        .broker = {
            .address = {
                .uri = MQTT_BROKER
            }
        }
    };


    s_mqtt_client =
        esp_mqtt_client_init(
            &mqtt_cfg
        );


    if (
        s_mqtt_client == NULL
    )
    {
        ESP_LOGE(
            TAG,
            "No se pudo crear cliente MQTT"
        );

        return;
    }


    ESP_ERROR_CHECK(
        esp_mqtt_client_register_event(
            s_mqtt_client,
            ESP_EVENT_ANY_ID,
            mqtt_event_handler,
            NULL
        )
    );


    ESP_ERROR_CHECK(
        esp_mqtt_client_start(
            s_mqtt_client
        )
    );


    ESP_LOGI(
        TAG,
        "Cliente MQTT iniciado"
    );
}


/* =========================================================
 * GPIO INIT
 * ========================================================= */

static void gpio_init(void)
{
    /*
     * =====================================================
     * SALIDAS
     * =====================================================
     */

    gpio_config_t output_config = {

        .pin_bit_mask =
            (1ULL << PIN_INICIO) |
            (1ULL << PIN_STOP) |
            (1ULL << PIN_EMERGENCIA) |
            (1ULL << PIN_SALIDA_AZUL) |
            (1ULL << PIN_SALIDA_VERDE) |
            (1ULL << PIN_SALIDA_ROJO),

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &output_config
        )
    );


    /*
     * =====================================================
     * ESTADO INICIAL DE SALIDAS
     * =====================================================
     */

    gpio_set_level(
        PIN_INICIO,
        0
    );

    gpio_set_level(
        PIN_STOP,
        0
    );

    gpio_set_level(
        PIN_EMERGENCIA,
        0
    );

    gpio_set_level(
        PIN_SALIDA_AZUL,
        0
    );

    gpio_set_level(
        PIN_SALIDA_VERDE,
        0
    );

    gpio_set_level(
        PIN_SALIDA_ROJO,
        0
    );


    /*
     * =====================================================
     * ENTRADAS NORMALES
     * =====================================================
     */

    gpio_config_t input_config = {

        .pin_bit_mask =
            (1ULL << PIN_TAMANO1) |
            (1ULL << PIN_TAMANO2) |
            (1ULL << PIN_METALES),

        .mode =
            GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_ENABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &input_config
        )
    );


    /*
     * =====================================================
     * GPIO35
     * =====================================================
     */

    gpio_config_t input_gpio35 = {

        .pin_bit_mask =
            (1ULL << PIN_TAMANO3),

        .mode =
            GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &input_gpio35
        )
    );


    /*
     * =====================================================
     * BOTON RESET
     * =====================================================
     */

    gpio_config_t reset_button_config = {

        .pin_bit_mask =
            (1ULL << PIN_RESET_CONTADOR),

        .mode =
            GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_ENABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &reset_button_config
        )
    );


    /*
     * =====================================================
     * SENSOR ULTRASONICO - TRIG
     * =====================================================
     */

    gpio_config_t ultrasonic_output_config = {

        .pin_bit_mask =
            (1ULL << PIN_TRIG),

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &ultrasonic_output_config
        )
    );


    /*
     * =====================================================
     * SENSOR ULTRASONICO - ECHO
     * =====================================================
     */

    gpio_config_t ultrasonic_input_config = {

        .pin_bit_mask =
            (1ULL << PIN_ECHO),

        .mode =
            GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &ultrasonic_input_config
        )
    );


    /*
     * =====================================================
     * TCS3200 - SALIDAS
     * ===================================================== */

    gpio_config_t sensor_out_config = {

        .pin_bit_mask =
            (1ULL << PIN_S2) |
            (1ULL << PIN_S3),

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &sensor_out_config
        )
    );


    /*
     * =====================================================
     * TCS3200 - ENTRADA
     * ===================================================== */

    gpio_config_t sensor_in_config = {

        .pin_bit_mask =
            (1ULL << PIN_OUTTCS),

        .mode =
            GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &sensor_in_config
        )
    );


    /*
     * =====================================================
     * INTERRUPCION TCS3200
     * ===================================================== */

    ESP_ERROR_CHECK(
        gpio_set_intr_type(
            PIN_OUTTCS,
            GPIO_INTR_POSEDGE
        )
    );


    ESP_ERROR_CHECK(
        gpio_install_isr_service(0)
    );


    ESP_ERROR_CHECK(
        gpio_isr_handler_add(
            PIN_OUTTCS,
            tcs3200_isr_handler,
            NULL
        )
    );


    /*
     * =====================================================
     * ESTADOS INICIALES
     * ===================================================== */

    gpio_set_level(
        PIN_S2,
        0
    );

    gpio_set_level(
        PIN_S3,
        0
    );

    gpio_set_level(
        PIN_TRIG,
        0
    );


    ESP_LOGI(
        TAG,
        "TCS3200 configurado"
    );


    ESP_LOGI(
        TAG,
        "TCS3200 -> S2: GPIO%d | S3: GPIO%d | OUT: GPIO%d",
        PIN_S2,
        PIN_S3,
        PIN_OUTTCS
    );


    ESP_LOGI(
        TAG,
        "SALIDAS COLOR -> AZUL: GPIO10 | VERDE: GPIO15 | ROJO: GPIO23"
    );


    ESP_LOGI(
        TAG,
        "ULTRASONICO -> TRIG: GPIO32 | ECHO: GPIO33"
    );


    ESP_LOGI(
        TAG,
        "RESET CONTADOR -> GPIO19"
    );


    ESP_LOGI(
        TAG,
        "DISTANCIA DETECCION: %.1f cm",
        DISTANCIA_DETECCION_CM
    );


    ESP_LOGI(
        TAG,
        "DISTANCIA REARME: %.1f cm",
        DISTANCIA_REARME_CM
    );


    ESP_LOGI(
        TAG,
        "GPIO inicializados"
    );
}


/* =========================================================
 * PUBLICAR ENTRADAS MQTT
 * ========================================================= */

static void publish_inputs_task(
    void *pvParameters)
{
    char payload[8];

    int last_tamano1 = -1;
    int last_tamano2 = -1;
    int last_tamano3 = -1;
    int last_metales = -1;


    while (1)
    {
        EventBits_t bits =
            xEventGroupGetBits(
                s_event_group
            );


        if (
            bits & MQTT_CONNECTED_BIT
        )
        {
            /*
             * LEER ENTRADAS
             */

            int tamano1 =
                gpio_get_level(
                    PIN_TAMANO1
                );

            int tamano2 =
                gpio_get_level(
                    PIN_TAMANO2
                );

            int tamano3 =
                gpio_get_level(
                    PIN_TAMANO3
                );

            int metales =
                gpio_get_level(
                    PIN_METALES
                );


            /*
             * TAMAÑO 1
             */

            snprintf(
                payload,
                sizeof(payload),
                "%s",
                tamano1 ? "ON" : "OFF"
            );

            esp_mqtt_client_publish(
                s_mqtt_client,
                "tamano1",
                payload,
                0,
                1,
                1
            );


            if (
                tamano1 != last_tamano1
            )
            {
                ESP_LOGI(
                    TAG,
                    "ENTRADA -> TAMANO1: %s",
                    tamano1 ? "ON" : "OFF"
                );

                last_tamano1 =
                    tamano1;
            }


            /*
             * TAMAÑO 2
             */

            snprintf(
                payload,
                sizeof(payload),
                "%s",
                tamano2 ? "ON" : "OFF"
            );

            esp_mqtt_client_publish(
                s_mqtt_client,
                "tamano2",
                payload,
                0,
                1,
                1
            );


            if (
                tamano2 != last_tamano2
            )
            {
                ESP_LOGI(
                    TAG,
                    "ENTRADA -> TAMANO2: %s",
                    tamano2 ? "ON" : "OFF"
                );

                last_tamano2 =
                    tamano2;
            }


            /*
             * TAMAÑO 3
             */

            snprintf(
                payload,
                sizeof(payload),
                "%s",
                tamano3 ? "ON" : "OFF"
            );

            esp_mqtt_client_publish(
                s_mqtt_client,
                "tamano3",
                payload,
                0,
                1,
                1
            );


            if (
                tamano3 != last_tamano3
            )
            {
                ESP_LOGI(
                    TAG,
                    "ENTRADA -> TAMANO3: %s",
                    tamano3 ? "ON" : "OFF"
                );

                last_tamano3 =
                    tamano3;
            }


            /*
             * METALES
             */

            snprintf(
                payload,
                sizeof(payload),
                "%s",
                metales ? "ON" : "OFF"
            );

            esp_mqtt_client_publish(
                s_mqtt_client,
                "metales",
                payload,
                0,
                1,
                1
            );


            if (
                metales != last_metales
            )
            {
                ESP_LOGI(
                    TAG,
                    "ENTRADA -> METALES: %s",
                    metales ? "ON" : "OFF"
                );

                last_metales =
                    metales;
            }
        }


        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}


/* =========================================================
 * MAIN
 * ========================================================= */

void app_main(void)
{
    /*
     * =====================================================
     * EVENT GROUP
     * ===================================================== */

    s_event_group =
        xEventGroupCreate();


    if (
        s_event_group == NULL
    )
    {
        ESP_LOGE(
            TAG,
            "No se pudo crear Event Group"
        );

        return;
    }


    /*
     * =====================================================
     * INICIO
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "      ESP32 PLC INICIANDO"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );


    /*
     * =====================================================
     * GPIO
     * ===================================================== */

    gpio_init();


    /*
     * =====================================================
     * NVS
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "Inicializando NVS..."
    );


    esp_err_t ret =
        nvs_flash_init();


    if (
        ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND
    )
    {
        ESP_LOGW(
            TAG,
            "NVS necesita ser borrada"
        );


        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );


        ret =
            nvs_flash_init();
    }


    ESP_ERROR_CHECK(
        ret
    );


    ESP_LOGI(
        TAG,
        "NVS OK"
    );


    /*
     * =====================================================
     * WIFI
     * ===================================================== */

    wifi_init();


    /*
     * =====================================================
     * ESPERAR WIFI
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "Esperando conexion WiFi..."
    );


    EventBits_t bits =
        xEventGroupWaitBits(
            s_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(30000)
        );


    if (
        !(bits & WIFI_CONNECTED_BIT)
    )
    {
        ESP_LOGE(
            TAG,
            "NO SE PUDO CONECTAR AL WIFI"
        );

        ESP_LOGE(
            TAG,
            "Verifica SSID, contraseña y red 2.4 GHz"
        );

        return;
    }


    /*
     * =====================================================
     * WIFI OK
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "WiFi conectado correctamente"
    );


    /*
     * =====================================================
     * MQTT
     * ===================================================== */

    mqtt_init();


    /*
     * =====================================================
     * TAREA DE ENTRADAS
     * ===================================================== */

    BaseType_t task_result =
        xTaskCreate(
            publish_inputs_task,
            "publish_inputs",
            4096,
            NULL,
            5,
            NULL
        );


    if (
        task_result != pdPASS
    )
    {
        ESP_LOGE(
            TAG,
            "No se pudo crear tarea MQTT"
        );

        return;
    }


    /*
     * =====================================================
     * TAREA TCS3200
     * ===================================================== */

    task_result =
        xTaskCreate(
            tcs3200_task,
            "tcs3200_task",
            4096,
            NULL,
            5,
            NULL
        );


    if (
        task_result != pdPASS
    )
    {
        ESP_LOGE(
            TAG,
            "No se pudo crear tarea TCS3200"
        );

        return;
    }


    /*
     * =====================================================
     * TAREA ULTRASONICO
     * ===================================================== */

    task_result =
        xTaskCreate(
            ultrasonico_task,
            "ultrasonico_task",
            4096,
            NULL,
            5,
            NULL
        );


    if (
        task_result != pdPASS
    )
    {
        ESP_LOGE(
            TAG,
            "No se pudo crear tarea ULTRASONICO"
        );

        return;
    }


    /*
     * =====================================================
     * SISTEMA LISTO
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "         SISTEMA LISTO"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "MQTT: broker.emqx.io:1883"
    );

    ESP_LOGI(
        TAG,
        "TCS3200: GPIO26=S2 | GPIO27=S3 | GPIO34=OUT"
    );

    ESP_LOGI(
        TAG,
        "COLORES:"
    );

    ESP_LOGI(
        TAG,
        "AZUL  -> GPIO10"
    );

    ESP_LOGI(
        TAG,
        "VERDE -> GPIO15"
    );

    ESP_LOGI(
        TAG,
        "ROJO  -> GPIO23"
    );

    ESP_LOGI(
        TAG,
        "ULTRASONICO: GPIO32=TRIG | GPIO33=ECHO"
    );

    ESP_LOGI(
        TAG,
        "RESET FISICO: GPIO19"
    );

    ESP_LOGI(
        TAG,
        "CONTEO MQTT: ultrasonico/conteo"
    );

    ESP_LOGI(
        TAG,
        "DISTANCIA MQTT: ultrasonico/distancia"
    );

    ESP_LOGI(
        TAG,
        "RESET MQTT: ultrasonico/reset -> ON"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );
}