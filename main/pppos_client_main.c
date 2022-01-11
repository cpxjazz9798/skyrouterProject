/* PPPoS Client Example with GSM
 *  (tested with SIM800)
 *  Author: LoBo (loboris@gmail.com, loboris.github)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */


#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/semphr.h"

#include "driver/uart.h"

#include "netif/ppp/pppos.h"
#include "netif/ppp/ppp.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
//#include "lwip/pppapi.h"
#include "netif/ppp/pppapi.h"

#include "mbedtls/platform.h"
#include "mbedtls/net.h"
#include "mbedtls/esp_debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"

#include <esp_event.h>
#include <esp_wifi.h>
//*************************************************
#include "protocol_examples_common.h"
#include "esp_netif.h"
#include <sys/param.h>
//***************************************************
//#include "apps/sntp/sntp.h"
#include "lwip/apps/sntp.h"
//#include "cJSON.h"

#include "libGSM.h"
#include "Modem4G.h"

#ifdef CONFIG_GSM_USE_WIFI_AP
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#endif

#include "lwip/lwip_napt.h"


#define EXAMPLE_TASK_PAUSE	300		// pause between task runs in seconds
#define TASK_SEMAPHORE_WAIT 140000	// time to wait for mutex in miliseconds


static const char *TIME_TAG = "[SNTP]";
static const char *TCP_CLIENT_TAG = "[TCP_CLIENT]";
static const char *WIFI_TAG = "[WIFI]";

#define TCP_SERVER "120.76.100.197"
#define TCP_SERVER_PORT 10002
static const char *payload = "Message from ESP32 ";

#define MY_DNS_IP_ADDR 0x08080808

#ifdef CONFIG_GSM_USE_WIFI_AP

// ==== WiFi handling & simple WebServer ====================================================

// FreeRTOS event group to signal when we are connected & ready to make a request
static EventGroupHandle_t wifi_event_group;

// The event group allows multiple bits for each event,
const int CONNECTED_BIT = BIT0;
const int APSTARTED_BIT = BIT1;

//-------------------------------------------------------------------
esp_err_t esp32_wifi_eventHandler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_START:
        xEventGroupSetBits(wifi_event_group, APSTARTED_BIT);
        ESP_LOGD(WIFI_TAG, "AP Started");
        break;
    case SYSTEM_EVENT_AP_STOP:
        xEventGroupClearBits(wifi_event_group, APSTARTED_BIT);
        ESP_LOGD(WIFI_TAG, "AP Stopped");
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGD(WIFI_TAG, "station connected to access point. Connected stations:");
        wifi_sta_list_t sta_list;
        ESP_ERROR_CHECK( esp_wifi_ap_get_sta_list(&sta_list));
        for(int i = 0; i < sta_list.num; i++) {
            //Print the mac address of the connected station
            wifi_sta_info_t sta = sta_list.sta[i];
            ESP_LOGD(WIFI_TAG, "Station %d MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", i, sta.mac[0], sta.mac[1], sta.mac[2], sta.mac[3], sta.mac[4], sta.mac[5]);
        }
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGD(WIFI_TAG, "station disconnected from access point");
        break;
    default:
        break;
    }
	return ESP_OK;
}
#endif



//=============
//*********************************************************************************************************************
static void tcp_client_task(void *pvParameters)
{
    if(ppposInit(0)==0)
        goto disconnect;
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

//#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(TCP_SERVER);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(TCP_SERVER_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
/*
#else // IPV6
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        // Setting scope_id to the connecting interface for correct routing if IPv6 Local Link supplied
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif
*/
        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TCP_CLIENT_TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TCP_CLIENT_TAG, "Socket created, connecting to %s:%d",TCP_SERVER, TCP_SERVER_PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TCP_CLIENT_TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TCP_CLIENT_TAG, "Successfully connected");

        while (1) {
            int err = send(sock, payload, strlen(payload), 0);
            if (err < 0) {
                ESP_LOGE(TCP_CLIENT_TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TCP_CLIENT_TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TCP_CLIENT_TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TCP_CLIENT_TAG, "%s", rx_buffer);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TCP_CLIENT_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
disconnect:
    ESP_LOGE(TCP_CLIENT_TAG, "GSM module not be initializatied,tcp client cannot run");
    vTaskDelete(NULL);
}
//*********************************************************************************************************************

void app_main()
{
    //http_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
   // ESP_ERROR_CHECK(example_connect());
	#ifdef CONFIG_GSM_USE_WIFI_AP
	// ----- Set AP(STA)---------------------------------------------------
	nvs_flash_init();
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK( esp_event_loop_init(esp32_wifi_eventHandler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
	wifi_config_t apConfig = {
	   .ap = {
		  .ssid="ESP32_TESTAP",
		  .ssid_len=0,
		  .password="",
		  .channel=0,
		  .authmode=WIFI_AUTH_OPEN,
		  .ssid_hidden=0,
		  .max_connection=8,
		  .beacon_interval=100
	   },
	};
	ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &apConfig) );

    //wifi_config_t staConfig = {
	//   .sta = {
	//	   .ssid = "MySSID",
	//	   .password = "MyPassword",
	//   }
	//};
	//ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &staConfig) );

    ESP_ERROR_CHECK( esp_wifi_start() );
	// ---------------------------------------------------------------------

	xEventGroupWaitBits(wifi_event_group, APSTARTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
		// start the HTTP Server task
	//xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);********************************************************被注释

	vTaskDelay(1000 / portTICK_RATE_MS);
	#endif
///***********************************************原始任务入口***************************************************************
	if (ppposInit() == 0) {
		ESP_LOGE("PPPoS EXAMPLE", "ERROR: GSM not initialized, HALTED");
		//while (1) {
			//vTaskDelay(1000 / portTICK_RATE_MS);
		//}
	}
//********************************************************************************************************************************

ESP_LOGI("PPPOS", "run now");
    //******************************************************************************************************************************/
//ppposInit();
//G4_init();
	// ==== 对连接到wifi的站点设置DNS =====
    
    ip_addr_t dnsserver,dhcps_dns_server;
    dhcps_offer_t dhcps_dns_value = OFFER_DNS;
    dhcps_set_option_info(6, &dhcps_dns_value, sizeof(dhcps_dns_value));
    dhcps_dns_server.u_addr.ip4.addr = htonl(MY_DNS_IP_ADDR);
    dhcps_dns_server.type = IPADDR_TYPE_V4;
    dhcps_dns_setserver(&dhcps_dns_server);
    ESP_LOGI(TIME_TAG, "SYSTEM_EVENT_STA_GOT_IP");
    inet_pton(AF_INET, "8.8.8.8", &dnsserver);
    dns_setserver(0, &dnsserver);
    inet_pton(AF_INET, "114.114.114.114", &dnsserver);
    dns_setserver(1, &dnsserver);
    u32_t napt_netif_ip = 0xC0A80401; // Set to ip address of softAP netif (Default is 192.168.4.1)
    ip_napt_enable(htonl(napt_netif_ip), 1);
    ESP_LOGI(TIME_TAG, "NAT is enabled");

	// ==== Get time from NTP server =====
	time_t now = 0;
	struct tm timeinfo = { 0 };
	int retry = 0;
	const int retry_count = 10;

	time(&now);
	localtime_r(&now, &timeinfo);

	while (1) {
		printf("\r\n");
		ESP_LOGI(TIME_TAG,"OBTAINING TIME");
	    ESP_LOGI(TIME_TAG, "Initializing SNTP");
	    sntp_setoperatingmode(SNTP_OPMODE_POLL);
	    //sntp_setservername(0, "pool.ntp.org");
	    sntp_setservername(0, "cn.pool.ntp.org");
	    sntp_init();
		ESP_LOGI(TIME_TAG,"SNTP INITIALIZED");

       /* u32_t napt_netif_ip = 0xC0A80401; // Set to ip address of softAP netif (Default is 192.168.4.1)
        ip_napt_enable(htonl(napt_netif_ip), 1);
        ESP_LOGI(TIME_TAG, "NAT is enabled");
*/
		// wait for time to be set
		now = 0;
		while ((timeinfo.tm_year < (2016 - 1900)) && (++retry < retry_count)) {
			ESP_LOGI(TIME_TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			time(&now);
			localtime_r(&now, &timeinfo);
			if (ppposStatus() != GSM_STATE_CONNECTED) break;
		}
		if (ppposStatus() != GSM_STATE_CONNECTED) {
			sntp_stop();
			ESP_LOGE(TIME_TAG, "Disconnected, waiting for reconnect");
			retry = 0;
			while (ppposStatus() != GSM_STATE_CONNECTED) {
				vTaskDelay(100 / portTICK_RATE_MS);
			}
			continue;
		}

		if (retry < retry_count) {
			ESP_LOGI(TIME_TAG, "TIME SET TO %s", asctime(&timeinfo));
			break;
		}
		else {
			ESP_LOGI(TIME_TAG, "ERROR OBTAINING TIME\n");
		}
		sntp_stop();
		break;
	}
ESP_LOGI("PPPOS", "START INIT G4");
if (G4_init()!=1) {
		ESP_LOGE("TCP EXAMPLE", "ERROR: G4 not initialized, HALTED");
	//	while (1) {
	//		vTaskDelay(1000 / portTICK_RATE_MS);
	//	}
	}
	// ==== Create PPPoS tasks ====
    //xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    
    while(1)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

