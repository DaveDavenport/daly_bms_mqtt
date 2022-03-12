#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_adc_cal.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_wifi.h"

#include "nvs_flash.h"

#include "cJSON.h"
#include "mqtt_client.h"

#include "daly_bms_monitor.h"
#include "user_config.h"

static xQueueHandle s_example_espnow_queue;
static const char *TAG = "daly_bms_monitor";

#define BUF_SIZE (256)

static uint8_t daly_bms_calc_checksum(uint8_t *msg) {
  uint8_t chk = 0;
  chk += msg[0];
  chk += msg[1];
  chk += msg[2];
  chk += msg[3];
  for (uint8_t iter = 0; iter < msg[3]; iter++) {
    chk += msg[4 + iter];
  }
  return chk;
}

static void daly_bms_send_command(DalyCommandID cmdid) {
  // With BMS we send a command, and wait for reply.
  uint8_t command_msg[13] = {
      0xA5,  // Start byte
      0x80,  // 'Upper' module
      cmdid, // Command byte.
      0x08,  // Lenght
      0x00,  // data 0
      0x00,  // data 1
      0x00,  // data 2
      0x00,  // data 3
      0x00,  // data 4
      0x00,  // data 5
      0x00,  // data 6
      0x00,  // data 7
      0x00   // Checksum
  };
  // Calculate the checksum
  command_msg[12] = daly_bms_calc_checksum(command_msg);

  // Send it to the bms.
  uart_write_bytes(UART_NUM_1, (const char *)command_msg, 13);
}

static void daly_process_msg(const uint8_t *msg) {
  DalyMsg ret_msg = {
      0,
  };
  ret_msg.id = (DalyCommandID)(msg[CMD_INDEX_DATA_ID]);
  switch ((DalyCommandID)(msg[CMD_INDEX_DATA_ID])) {
  case CMD_ID_SOC_TOTAL_VOLTAGE: {
    ret_msg.soc.pressure = ((float)((msg[4] << 8) | msg[5]) / 10.0f);
    ret_msg.soc.aquisition = ((float)((msg[6] << 8) | msg[7]) / 10.0f);
    ret_msg.soc.current = ((float)(((msg[8] << 8) | msg[9]) - 30000) / 10.0f);
    ret_msg.soc.soc = ((float)((msg[10] << 8) | msg[11]) / 10.0f);
    break;
  }
  case CMD_ID_MIN_MAX_CELL_VOLTAGE: {
    ret_msg.mmcv.max_mv = (msg[4] << 8 | msg[5]);
    ret_msg.mmcv.max_id = (msg[6]);
    ret_msg.mmcv.min_mv = (msg[7] << 8 | msg[9]);
    ret_msg.mmcv.min_id = (msg[9]);
    break;
  }
  case CMD_ID_MIN_MAX_TEMPERATURE: {
    ret_msg.mmt.max_temp = (msg[4] << 8 | msg[5]) - 40;
    ret_msg.mmt.max_id = (msg[6]);
    ret_msg.mmt.min_temp = (msg[7] << 8 | msg[9]) - 40;
    ret_msg.mmt.min_id = (msg[9]);
    break;
  }
  case CMD_ID_CELL_VOLTAGES: {
    ret_msg.cvs.frame_num = msg[4];
    ret_msg.cvs.mvoltage[0] = (msg[5] << 8 | msg[6]);
    if (ret_msg.cvs.mvoltage[0] != 0) {
      ret_msg.cvs.mvoltage[1] = (msg[7] << 8 | msg[8]);
      if (ret_msg.cvs.mvoltage[1] != 0) {
        ret_msg.cvs.mvoltage[2] = (msg[9] << 8 | msg[10]);
      }
    }
    break;
  }
  case CMD_ID_CHARGE_STATE_REM_CAP: {
    ret_msg.cs.status = msg[4];
    ret_msg.cs.charge = msg[5];
    ret_msg.cs.discharge = msg[6];
    ret_msg.cs.life = msg[7];
    ret_msg.cs.residual_charge =
        msg[8] << 24 | msg[9] << 16 | msg[10] << 8 | msg[11];
    break;
  }
  case CMD_ID_TEMPERATURES: {
    ret_msg.tmps.frame_num = msg[4];
    ret_msg.tmps.temp[0] = (msg[5] << 8 | msg[6]) - 40;
    ret_msg.tmps.temp[1] = (msg[7] << 8 | msg[8]) - 40;
    ret_msg.tmps.temp[2] = (msg[9] << 8 | msg[10]) - 40;
    break;
  }
  case CMD_ID_STATUS_INFO_1: {
    ret_msg.status.battery = msg[4];
    ret_msg.status.temperature = msg[5];
    ret_msg.status.charger = msg[6];
    ret_msg.status.load = msg[7];
    // msg[8] skip, no idea what it is.
    ret_msg.status.cycles = msg[9] << 8 | msg[10];
    break;
  }
  default:
    return;
  }
  xQueueSend(s_example_espnow_queue, &ret_msg, pdMS_TO_TICKS(500));
}

static void daly_bms_read_response(void) {
  size_t remainder = 0;
  do {

    uint8_t max_msg[14] = {
        0x00,
    };
    uint8_t c = 0x0;
    int l = 0;
    // Search for header.
    while (c != 0xA5) {
      l = uart_read_bytes(UART_NUM_1, &c, 1, pdMS_TO_TICKS(250));
      if (l == 0) {
        // Timeout
        ESP_LOGE(TAG, "Error, uart timeout");
        return;
      }
    }
    max_msg[CMD_INDEX_START] = c;
    l = uart_read_bytes(UART_NUM_1, &c, 1, pdMS_TO_TICKS(250));
    if (l == 0) {
      // Timeout
      ESP_LOGE(TAG, "Error, uart timeout");
      return;
    }
    max_msg[CMD_INDEX_ADDRESS] = c;
    l = uart_read_bytes(UART_NUM_1, &c, 1, pdMS_TO_TICKS(250));
    if (l == 0) {
      // Timeout
      ESP_LOGE(TAG, "Error, uart timeout");
      return;
    }
    max_msg[CMD_INDEX_DATA_ID] = c;
    l = uart_read_bytes(UART_NUM_1, &c, 1, pdMS_TO_TICKS(250));
    if (l == 0) {
      // Timeout
      ESP_LOGE(TAG, "Error, uart timeout");
      return;
    }
    max_msg[CMD_INDEX_DATA_LEN] = c;
    for (uint8_t iter = 0; iter < max_msg[CMD_INDEX_DATA_LEN]; iter++) {
      l = uart_read_bytes(UART_NUM_1, &c, 1, pdMS_TO_TICKS(250));
      if (l == 0) {
        // Timeout
        ESP_LOGE(TAG, "Error, uart timeout");
        return;
      }
      max_msg[4 + iter] = c;
    }
    l = uart_read_bytes(UART_NUM_1, &c, 1, pdMS_TO_TICKS(250));
    if (l == 0) {
      // Timeout
      ESP_LOGE(TAG, "Error, uart timeout");
      return;
    }
    max_msg[12] = c;
    uint8_t checksum = daly_bms_calc_checksum(max_msg);
    if (c != checksum) {
      ESP_LOGE(TAG, "Error, checksum failed: %02X - %02X (%d)", c, checksum,
               max_msg[CMD_INDEX_DATA_LEN]);
      for (int i = 0; i < 13; i++) {
        ESP_LOGE(TAG, "%d: %08X", i, max_msg[i]);
      }
      return;
    }
    ESP_LOGI(TAG, "Got msg: %02X id of length: %02X",
             max_msg[CMD_INDEX_DATA_ID], max_msg[CMD_INDEX_DATA_LEN]);

    // Process msg
    daly_process_msg(max_msg);
    uart_get_buffered_data_len(UART_NUM_1, &remainder);
  } while (remainder != 0);
}

static void uart_read_task(void *arg) {
  /* Configure parameters of an UART driver,
   * communication pins and install the driver */
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, DALY_UART_TXD, DALY_UART_RXD, DALY_UART_RTS,
               DALY_UART_CTS);

  ESP_LOGW(TAG, "testing");

  uint32_t count = 0;
  while (1) {
    daly_bms_send_command(CMD_ID_SOC_TOTAL_VOLTAGE);
    daly_bms_read_response();
    if ((count % 5) == 0) {
      daly_bms_send_command(CMD_ID_CELL_VOLTAGES);
      daly_bms_read_response();
      daly_bms_send_command(CMD_ID_TEMPERATURES);
      daly_bms_read_response();
      daly_bms_send_command(CMD_ID_CHARGE_STATE_REM_CAP);
      daly_bms_read_response();
      daly_bms_send_command(CMD_ID_MIN_MAX_CELL_VOLTAGE);
      daly_bms_send_command(CMD_ID_STATUS_INFO_1);
      daly_bms_read_response();
    }
    count++;
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGE(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
  }
}
/*init wifi as sta and set power save mode*/
static void wifi_power_save(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASSWORD,
              .listen_interval = 3,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "esp_wifi_set_ps().");
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
  switch (event->event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
  return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base,
           event_id);
  mqtt_event_handler_cb(event_data);
}
static esp_mqtt_client_handle_t mqtt_app_start(void) {
  esp_mqtt_client_config_t mqtt_cfg = {
      .uri = MQTT_URI,
  };

  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 NULL);
  esp_mqtt_client_start(client);
  return client;
}

void mqtt_msg_publish_task(void *parameter) {
  esp_mqtt_client_handle_t client = mqtt_app_start();
  for (;;) {
    DalyMsg status = {
        0,
    };
    xQueueReceive(s_example_espnow_queue, &status, pdMS_TO_TICKS(5 * 300000));
    ESP_LOGI(TAG, "Got daly msg: %d", status.id);
    switch (status.id) {
    case CMD_ID_SOC_TOTAL_VOLTAGE: {
      gpio_set_level(MQTT_LED, 1);
      char buf[128] = {
          0,
      };
      snprintf(buf, 128,
               "{ \"pressure\": %.1f, \"aquisition\": %.1f, \"current\": %.1f, "
               "\"soc\": %.f}",
               status.soc.pressure, status.soc.aquisition, status.soc.current,
               status.soc.soc);
      int msg_id =
          esp_mqtt_client_publish(client, mqtt_topic_soc, buf, 0, 0, 0);
      ESP_LOGD(TAG, "sent publish successful, msg_id=%d", msg_id);
      ESP_LOGI(TAG, "buf: %s", buf);
      gpio_set_level(MQTT_LED, 0);
      break;
    }
    case CMD_ID_MIN_MAX_CELL_VOLTAGE: {
      char buf[128] = {
          0,
      };
      snprintf(buf, 128,
               "{ \"max_mv\": %hu, \"max_cell\": %u, \"min_mv\": %hu, "
               "\"min_cell\": %u}",
               status.mmcv.max_mv, status.mmcv.max_id, status.mmcv.min_mv,
               status.mmcv.min_id);
      int msg_id =
          esp_mqtt_client_publish(client, mqtt_topic_cvs, buf, 0, 0, 0);
      ESP_LOGD(TAG, "sent publish successful, msg_id=%d", msg_id);
      ESP_LOGI(TAG, "buf: %s", buf);
      break;
    }
    case CMD_ID_MIN_MAX_TEMPERATURE: {
      break;
    }
    case CMD_ID_CHARGE_STATE_REM_CAP: {
      char buf[256] = {
          0,
      };
      snprintf(buf, 256,
               "{ \"status\": %u, \"charge\": %u, \"discharge\": %u, "
               "\"life\": %u, \"residual_charge\": %u }",
               status.cs.status, status.cs.charge, status.cs.discharge,
               status.cs.life, status.cs.residual_charge);
      int msg_id = esp_mqtt_client_publish(client, mqtt_topic_cs, buf, 0, 0, 0);
      ESP_LOGD(TAG, "sent publish successful, msg_id=%d", msg_id);
      ESP_LOGI(TAG, "buf: %s", buf);
      break;
    }
    case CMD_ID_STATUS_INFO_1: {
      char buf[128] = {
          0,
      };
      snprintf(buf, 128,
               "{ \"num_bats\": %u, \"num_temps\": %u, \"charger\": %u, "
               "\"load\": %u, \"cycles\": %u }",
               status.status.battery, status.status.temperature,
               status.status.charger, status.status.load, status.status.cycles);
      int msg_id =
          esp_mqtt_client_publish(client, mqtt_topic_status, buf, 0, 0, 0);
      ESP_LOGD(TAG, "sent publish successful, msg_id=%d", msg_id);
      ESP_LOGI(TAG, "buf: %s", buf);
      break;
    }
    case CMD_ID_TEMPERATURES: {
      char buf[128] = {
          0,
      };
      int start = status.tmps.frame_num - 1;
      snprintf(buf, 128, "{ \"temp%d\": %d, \"temp%d\": %d, \"temp%d\": %d}",
               3 * start + 1, status.tmps.temp[0], 3 * start + 2,
               status.tmps.temp[1], 3 * start + 3, status.tmps.temp[2]);

      int msg_id =
          esp_mqtt_client_publish(client, mqtt_topic_tmps, buf, 0, 0, 0);
      ESP_LOGD(TAG, "sent publish successful, msg_id=%d", msg_id);
      ESP_LOGI(TAG, "buf: %s", buf);
      break;
    }
    case CMD_ID_CELL_VOLTAGES: {
      char buf[128] = {
          0,
      };
      int start = status.cvs.frame_num - 1;
      snprintf(buf, 128,
               "{ \"voltage%d\": %u, \"voltage%d\": %u, \"voltage%d\": %u}",
               3 * start + 1, status.cvs.mvoltage[0], 3 * start + 2,
               status.cvs.mvoltage[1], 3 * start + 3, status.cvs.mvoltage[2]);

      int msg_id =
          esp_mqtt_client_publish(client, mqtt_topic_cvs, buf, 0, 0, 0);
      ESP_LOGD(TAG, "sent publish successful, msg_id=%d", msg_id);
      ESP_LOGI(TAG, "buf: %s", buf);
      break;
    }
    default:
      break;
    }
  }
}

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGW(TAG, "GPIO Setup");
  gpio_config_t io_conf;
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = (1ULL << MQTT_LED);
  // disable pull-down mode
  io_conf.pull_down_en = 0;
  // disable pull-up mode
  io_conf.pull_up_en = 0;
  // configure GPIO with the given settings
  gpio_config(&io_conf);

  gpio_set_level(MQTT_LED, 0);

  wifi_power_save();
  s_example_espnow_queue = xQueueCreate(8, sizeof(DalyMsg));
  if (s_example_espnow_queue == NULL) {
    ESP_LOGE(TAG, "Create queue fail");
  }

  xTaskCreate(mqtt_msg_publish_task, "mqtt_msg_publish_task", 4096, NULL, 10,
              NULL);

  xTaskCreate(uart_read_task, "uart_read_task", 2048, NULL, 0, NULL);
}
