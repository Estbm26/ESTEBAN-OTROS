#include <stdio.h>
#include <string.h>
#include <stdint.h>

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

#define WIFI_SSID "MALLAS CONTROL"
#define WIFI_PASS "21778PAAE"

#define MAX_WIFI_RETRY 10


/* =========================================================
 * MQTT
 * ========================================================= */

#define MQTT_BROKER "mqtt://broker.emqx.io:1883"


/* =========================================================
 * SALIDAS
 * ========================================================= */

#define PIN_INICIO     GPIO_NUM_16
#define PIN_STOP       GPIO_NUM_17
#define PIN_EMERGENCIA GPIO_NUM_18


/* =========================================================
 * ENTRADAS
 * ========================================================= */

#define PIN_TAMANO1 GPIO_NUM_13
#define PIN_TAMANO2 GPIO_NUM_14
#define PIN_TAMANO3 GPIO_NUM_35
#define PIN_METALES GPIO_NUM_21


/* =========================================================
 * SENSOR ULTRASONICO
 * ========================================================= */

#define PIN_TRIG GPIO_NUM_32
#define PIN_ECHO GPIO_NUM_33


/* =========================================================
 * SENSOR DE COLOR TCS3200
 *
 * SOLO UTILIZAMOS 3 PINES:
 *
 * S2  -> GPIO26
 * S3  -> GPIO27
 * OUT -> GPIO34
 *
 * OE no se utiliza.
 * S0 y S1 no se utilizan.
 * ========================================================= */

#define PIN_S2     GPIO_NUM_26
#define PIN_S3     GPIO_NUM_27
#define PIN_OUTTCS GPIO_NUM_34


/* =========================================================
 * TCS3200
 *
 * Tiempo de medición de cada color.
 * ========================================================= */

#define TCS_MEASURE_TIME_MS 100


/* =========================================================
 * EVENTOS
 * ========================================================= */

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT1


/* =========================================================
 * VARIABLES GLOBALES
 * ========================================================= */

static EventGroupHandle_t s_event_group = NULL;

static esp_mqtt_client_handle_t s_mqtt_client = NULL;

static int s_retry_num = 0;


/* =========================================================
 * CONTADOR TCS3200
 *
 * Cada pulso que sale por OUT del TCS3200
 * incrementa este contador.
 * ========================================================= */

static volatile uint32_t tcs_pulse_count = 0;


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
    /*
     * Seleccionar filtro
     */

    tcs3200_select_filter(
        s2,
        s3
    );


    /*
     * Esperar un pequeño tiempo para
     * que el sensor se estabilice.
     */

    vTaskDelay(
        pdMS_TO_TICKS(20)
    );


    /*
     * Reiniciar contador
     */

    tcs_pulse_count = 0;


    /*
     * Medir durante 100 ms
     */

    vTaskDelay(
        pdMS_TO_TICKS(
            TCS_MEASURE_TIME_MS
        )
    );


    /*
     * Guardar cantidad de pulsos.
     */

    uint32_t pulses =
        tcs_pulse_count;


    /*
     * Convertir a aproximadamente
     * pulsos por segundo.
     *
     * 100 ms = 0.1 segundos
     *
     * frecuencia = pulsos / 0.1
     *             = pulsos * 10
     */

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
    /*
     * Si prácticamente no hay lectura,
     * consideramos que no hay color válido.
     */

    if (
        red < 10 &&
        green < 10 &&
        blue < 10
    )
    {
        return "SIN_COLOR";
    }


    /*
     * ROJO predominante
     */

    if (
        red >= green &&
        red >= blue
    )
    {
        return "ROJO";
    }


    /*
     * VERDE predominante
     */

    if (
        green >= red &&
        green >= blue
    )
    {
        return "VERDE";
    }


    /*
     * AZUL predominante
     */

    return "AZUL";
}


/* =========================================================
 * TAREA TCS3200
 * ========================================================= */

static void tcs3200_task(
    void *pvParameters)
{
    while (1)
    {

        /*
         * Solamente leer si MQTT está conectado.
         */

        EventBits_t bits =
            xEventGroupGetBits(
                s_event_group
            );


        if (
            bits & MQTT_CONNECTED_BIT
        )
        {

            /* =============================================
             * ROJO
             *
             * S2 = 0
             * S3 = 0
             * ============================================= */

            uint32_t red =
                tcs3200_measure(
                    0,
                    0
                );


            /* =============================================
             * AZUL
             *
             * S2 = 0
             * S3 = 1
             * ============================================= */

            uint32_t blue =
                tcs3200_measure(
                    0,
                    1
                );


            /* =============================================
             * VERDE
             *
             * S2 = 1
             * S3 = 1
             * ============================================= */

            uint32_t green =
                tcs3200_measure(
                    1,
                    1
                );


            /* =============================================
             * DETERMINAR COLOR
             * ============================================= */

            const char *color =
                tcs3200_detect_color(
                    red,
                    green,
                    blue
                );


            /* =============================================
             * TERMINAL
             * ============================================= */

            ESP_LOGI(
                TAG,
                "TCS3200 -> ROJO: %lu Hz | VERDE: %lu Hz | AZUL: %lu Hz | COLOR: %s",
                (unsigned long)red,
                (unsigned long)green,
                (unsigned long)blue,
                color
            );


            /* =============================================
             * MQTT ROJO
             * ============================================= */

            char payload[32];


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


            /* =============================================
             * MQTT VERDE
             * ============================================= */

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


            /* =============================================
             * MQTT AZUL
             * ============================================= */

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


            /* =============================================
             * MQTT COLOR
             * ============================================= */

            esp_mqtt_client_publish(
                s_mqtt_client,
                "tcs3200/color",
                color,
                0,
                1,
                1
            );
        }


        /*
         * Esperar antes de volver a medir.
         */

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


            /* Suscripciones */

            esp_mqtt_client_subscribe(
                s_mqtt_client,
                "inicio",
                1
            );


            esp_mqtt_client_subscribe(
                s_mqtt_client,
                "stop",
                1
            );


            esp_mqtt_client_subscribe(
                s_mqtt_client,
                "emergencia",
                1
            );


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


            /* =================================================
             * INICIO
             * ================================================= */

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


            /* =================================================
             * STOP
             * ================================================= */

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


            /* =================================================
             * EMERGENCIA
             * ================================================= */

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

    /* =====================================================
     * WIFI INICIADO
     * ===================================================== */

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


    /* =====================================================
     * WIFI DESCONECTADO
     * ===================================================== */

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


    /* =====================================================
     * WIFI CONECTADO / IP OBTENIDA
     * ===================================================== */

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

    /* =====================================================
     * SALIDAS
     * ===================================================== */

    gpio_config_t output_config = {

        .pin_bit_mask =
            (1ULL << PIN_INICIO) |
            (1ULL << PIN_STOP) |
            (1ULL << PIN_EMERGENCIA),

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


    /* =====================================================
     * ENTRADAS NORMALES
     * ===================================================== */

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


    /* =====================================================
     * GPIO35
     * ===================================================== */

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


    /* =====================================================
     * SENSORES - SALIDAS
     * ===================================================== */

    gpio_config_t sensor_out_config = {

        .pin_bit_mask =
            (1ULL << PIN_TRIG) |
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


    /* =====================================================
     * SENSORES - ENTRADAS
     * ===================================================== */

    gpio_config_t sensor_in_config = {

        .pin_bit_mask =
            (1ULL << PIN_ECHO) |
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


    /* =====================================================
     * INTERRUPCIÓN TCS3200
     *
     * Cada flanco de subida del OUT cuenta como
     * un pulso.
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


    /* =====================================================
     * ESTADO INICIAL TCS3200
     *
     * Comenzamos con rojo:
     *
     * S2 = 0
     * S3 = 0
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


    /*
     * Estados anteriores.
     */

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

            /* =============================================
             * LEER ENTRADAS
             * ============================================= */

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


            /* =============================================
             * TAMAÑO 1
             * ============================================= */

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


            /* =============================================
             * TAMAÑO 2
             * ============================================= */

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


            /* =============================================
             * TAMAÑO 3
             * ============================================= */

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


            /* =============================================
             * METALES
             * ============================================= */

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

    /* =====================================================
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


    /* =====================================================
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


    /* =====================================================
     * GPIO
     * ===================================================== */

    gpio_init();


    /* =====================================================
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


    /* =====================================================
     * WIFI
     * ===================================================== */

    wifi_init();


    /* =====================================================
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


    /* =====================================================
     * WIFI OK
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "WiFi conectado correctamente"
    );


    /* =====================================================
     * MQTT
     * ===================================================== */

    mqtt_init();


    /* =====================================================
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


    /* =====================================================
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


    /* =====================================================
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
        "MQTT: broker.emqx.io:1883"
    );


    ESP_LOGI(
        TAG,
        "TCS3200: GPIO26=S2 | GPIO27=S3 | GPIO34=OUT"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );
}