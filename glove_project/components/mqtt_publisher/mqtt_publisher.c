#include"mqtt_publisher.h"
#include "driver/adc.h"
// #include "accel.h"
// #include "potentiometre.h"
#define CONFIG_BROKER_URL "mqtt://10.42.0.1:1883"
const char *TAG = "MQTT_EXAMPLE";

esp_mqtt_client_handle_t client;
void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "wala", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%ld\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void send_value(int motor, int angle)
{
    char motor_id[10];
    sprintf(motor_id, "%u", motor);
    char topic[15] = "/moteur/";
    strcat(topic, motor_id);
    char angle_str[10];
    sprintf(angle_str, "%u", angle);
    printf("topic : %s\nvalue : %s\n",topic,angle_str);
    
    esp_mqtt_client_publish(client, topic, angle_str, 0, 0, 0);
}

void mqtt_init(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);
    // loop();
    // vTaskDelay(50/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
    // xTaskCreate(&my_task, "my_task", 4096, NULL, 1, NULL);
}


   void my_task()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);


    while (1)
    {
        int potentiometer_value3 = adc1_get_raw(ADC1_CHANNEL_3);
        int potentiometer_value4 = adc1_get_raw(ADC1_CHANNEL_4);
        int potentiometer_value5 = adc1_get_raw(ADC1_CHANNEL_5);
        int potentiometer_value6 = adc1_get_raw(ADC1_CHANNEL_6);
        int potentiometer_value7 = adc1_get_raw(ADC1_CHANNEL_7);
        send_value(3,potentiometer_value3);
        send_value(4,potentiometer_value4);
        send_value(5,potentiometer_value5);
        send_value(6,potentiometer_value6);
        send_value(7,potentiometer_value7);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// void my_task(void *pvParameter) {
//     // Code exécuté dans le thread
//     // int pot = pot_init(8);
//     // int val = get_pot_value(pot);
//     // while(1)
//     // {
//     //     val = get_pot_value(pot);
//     //     printf("my value : %d\n",val);
//     //     vTaskDelay(500/portTICK_PERIOD_MS);
//     // }
//     esp_rom_gpio_pad_select_gpio(POTENTIOMETER_GPIO);
//     gpio_set_direction(POTENTIOMETER_GPIO, GPIO_MODE_INPUT);

//     // Configurer l'ADC pour lire la valeur du potentiomètre
//     adc1_config_width(ADC_WIDTH_BIT_12);
//     adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);

//     // Boucle infinie pour lire la valeur du potentiomètre
//     while(1) {
//         // Lire la valeur de l'ADC pour le potentiomètre
//         int value = adc1_get_raw(ADC1_CHANNEL_6);

//         // Faire quelque chose avec la valeur lue, par exemple l'afficher sur la console
//         printf("Valeur lue : %d\n", value);

//         send_value(0,value);

//         // Attendre un certain temps avant de lire à nouveau la valeur du potentiomètre
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }


// }