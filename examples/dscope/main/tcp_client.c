/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "i2s_adc_poll.h"
#include <tcpip_adapter.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "wifi_connect.h"

#include "ad77681evb.h"

#define GPIO_OUTPUT_IO_0    21
// #define GPIO_OUTPUT_IO_1    19
// #define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT
#define USE_UDP

//#define ECHO_TEST_TXD  (GPIO_NUM_4)
//#define ECHO_TEST_RXD  (GPIO_NUM_5)
#define ECHO_TXD0  (GPIO_NUM_1)
#define ECHO_RXD0  (GPIO_NUM_3)
 
#define ECHO_TXD1  (GPIO_NUM_23)
#define ECHO_RXD1  (GPIO_NUM_22)
 
#define ECHO_TXD2  (GPIO_NUM_21)
#define ECHO_RXD2  (GPIO_NUM_19)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
 
#define BUF_SIZE (256)

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

static int sock = -1;
struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6

static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr;
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", HOST_IP_ADDR, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (1) {
            int err = send(sock, payload, strlen(payload), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void tcp_client_sock_init()
{
    char addr_str[128];
    char source_addr_str[128];
    char dest_addr_str[128];
    char rx_buffer[128];
    int addr_family;
    int ip_protocol;

#ifdef CONFIG_EXAMPLE_IPV4
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
    struct sockaddr_in6 dest_addr;
    inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(PORT);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
    inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif
#ifdef USE_UDP
    int udp_sock = -1;
    while (udp_sock < 0) {
        udp_sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (udp_sock < 0) {
            ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
        }
    }

    struct sockaddr_in broadcast_addr;
    broadcast_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(PORT);

    // int serr = sendto(udp_sock, payload, strlen(payload), 0, (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
    // if (serr < 0) {
    //     ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    // }
    // ESP_LOGI(TAG, "Message sent");
    dest_addr.sin_addr.s_addr = 0;
    unsigned char optval = 1;
    setsockopt(udp_sock,SOL_SOCKET,SO_BROADCAST,( void *)&optval,sizeof(optval));
    int err = bind(udp_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);
#endif
    while (1) {
#ifdef USE_UDP
        ESP_LOGI(TAG, "Receiving from UDP socket...");
        // struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(udp_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
        // Data received
        else {
            inet_ntoa_r(source_addr.sin_addr, source_addr_str, sizeof(source_addr_str) - 1);
            rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            ESP_LOGI(TAG, "Received %d bytes from %s on port:%d", len, source_addr_str,ntohs(source_addr.sin_port));
            ESP_LOGI(TAG, "%s", rx_buffer);
        }
#endif
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            continue;
        }
#ifdef USE_UDP
        source_addr.sin_port = htons(PORT);
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", source_addr_str, PORT);
        int err = connect(sock, (struct sockaddr *)&source_addr, sizeof(source_addr));
#else
        tcpip_adapter_ip_info_t ipinfo; 
        char str[256];
        // IP address.
        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipinfo);
        printf("My IP: " IPSTR "\n", IP2STR(&ipinfo.ip));

        // Create addr
        static char ip_cnt = 0;
        ip_cnt++;
        dest_addr.sin_addr.s_addr = ipinfo.ip.addr + (ip_cnt<<24);
        // dest_addr.sin_addr.s_addr = inet_addr("192.168.43.255");
        inet_ntoa_r(dest_addr.sin_addr, dest_addr_str, sizeof(dest_addr_str) - 1);
        // struct timeval tv_out;
        // tv_out.tv_sec = 0;
        // tv_out.tv_usec = 1000;
        // setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv_out, sizeof(tv_out));
        // setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv_out, sizeof(tv_out));
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", dest_addr_str, PORT);
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
#endif
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");
        return;
    }
}

static void adc_i2s_read_task(void *arg)
{
    const size_t i2s_read_len = 16*1024;
    int chCnt[16];
    uint16_t *i2s_read_buff = (uint16_t*) calloc(i2s_read_len, sizeof(char));
    while(1){
        size_t bytes_read = 0;
        //read data from I2S bus, in this case, from ADC.
        adc_i2s_read(i2s_read_buff, i2s_read_len, &bytes_read);
        // memset(chCnt, 0, 16*sizeof(int));
        // for (int i = 0; i < bytes_read/2; i+=1) {
        //     // printf("num:%3d ch:%2d raw:%4d\n", i, (i2s_read_buff[i] >> 12), (i2s_read_buff[i]) & 0xfff); //i/4
        //     chCnt[i2s_read_buff[i] >> 12]++;
        // }
        // for (int i = 0; i < 16; i+=1) {
        //     if (chCnt[i] > 0) {
        //         printf("ch%2d:%5d ",i,chCnt[i]);
        //     }
        // }
        // printf("\n");
        int err = send(sock, i2s_read_buff, bytes_read, 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
#ifdef ENABLE_SOCKET_RECONNECT
            i2s_driver_uninstall(0); // Must be same as defined in i2s_adc_poll.c
            if (sock != -1) {
                ESP_LOGE(TAG, "Shutting down socket and restarting...");
                shutdown(sock, 0);
                close(sock);
            }
            int addr_family = AF_INET;
            int ip_protocol = IPPROTO_IP;
            sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                continue;
            }
            char source_addr_str[128];
            while(1)
            {
                inet_ntoa_r(source_addr.sin_addr, source_addr_str, sizeof(source_addr_str) - 1);
                source_addr.sin_port = htons(PORT);
                ESP_LOGI(TAG, "Socket created, connecting to %s:%d", source_addr_str, PORT);
                int err = connect(sock, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err != 0) {
                    ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
                    continue;
                }else{
                    break;
                }
            }
            ESP_LOGE(TAG, "Socket reconnected");
            adc_i2s_init();
            i2s_adc_check_clk();
#else
            break;
#endif
        }
    }
}

static void uart_task()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, ECHO_TXD0, ECHO_RXD0, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    char *data = (char *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)data, BUF_SIZE, 2000 / portTICK_RATE_MS);
        printf("%d\r\n",len);
        if(len > 8)
        {
            wifi_ssid = data;
            for(int i=0;i<len;i++)
            {
                printf("curChar:%d",data[i]);
                if(data[i] == 0)
                {
                    wifi_password = &(data[i+1]);
                    break;
                }
            }
        }else{
            wifi_ssid = "HNSDFZ";
            wifi_password = "12345678";
        }
        wifi_connect();
        break;
        // Write data back to the UART
        // uart_write_bytes(UART_NUM_0, (const char *) data, len);
    }
}

void led_init_task()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

static void led_control_task(void *arg)
{
    int cnt = 0;
    while(1) {
        cnt++;
        // printf("cnt: %d\n", cnt);
        ESP_LOGI("led_control_task", "%d", cnt);
        vTaskDelay(1000 / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        // gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // uart_task(); // Update new Wifi name and password from UART. It will try to connect WiFi!
    led_init_task();
    esp_log_level_set("led_control_task", ESP_LOG_INFO);
    xTaskCreate(led_control_task, "led_control_task", 4*1024, NULL, 5, NULL);

    esp_log_level_set("ad77681_evb_task", ESP_LOG_INFO);
    esp_log_level_set("AD7768-1 Driver", ESP_LOG_INFO);
    esp_log_level_set("SPI Engine", ESP_LOG_INFO);
    xTaskCreate(ad77681_evb_task, "ad77681_evb_task", 4*1024, NULL, 5, NULL);

    // /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    //  * Read "Establishing Wi-Fi or Ethernet Connection" section in
    //  * examples/protocols/README.md for more information about this function.
    //  */
    // // ESP_ERROR_CHECK(example_connect());
    //     esp_wifi_set_ps(WIFI_PS_NONE);

    // // xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    // tcp_client_sock_init();
    // adc_i2s_init();
    // i2s_adc_check_clk();
    // esp_log_level_set("I2S", ESP_LOG_INFO);
    // xTaskCreate(adc_i2s_read_task, "ADC I2S read task", 4*1024, NULL, 5, NULL);
    // // xTaskCreate(uart_task, "uart_echo_task", 1024, NULL, 10, NULL);
}
