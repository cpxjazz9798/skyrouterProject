#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"
//#include "tcpip_adapter.h"
//#include "netif/ppp/pppos.h"
//#include "netif/ppp/ppp.h"
//#include "lwip/pppapi.h"
//#include "netif/ppp/pppapi.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include <sys/fcntl.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/select.h>


#include "Modem4G.h"


#define G4_TX CONFIG_4G_TX
#define G4_RX CONFIG_4G_RX
#define G4_Modem_BDRATE CONFIG_4G_BDRATE

#ifdef CONFIG_4G_DEBUG
#define G4_DEBUG 1
#else
#define G4_DEBUG 1
#endif
#define BUF_SIZE (1024)
#define G4_OK_Str "OK"
#define TCPMUTEX_TIMEOUT 1000 / portTICK_RATE_MS

#define G4_TASK_STACK_SIZE 1024*2

// shared variables, use mutex to access them
static uint8_t g4_status =G4_Modem_STATE_FIRSTINIT ;
//static int socket_connect = 1;
//static uint32_t tcp_rx_count;
//static uint32_t tcp_tx_count;
static int do_tcp_connect = 1;
static uint8_t tcp_task_started = 0;
static uint8_t G4_rfOff = 0;
//static int tcp_status=G4_TCP_STATE_DISCONNECT;
//static uint8_t g4_rfOff = 0;

// local variables
static QueueHandle_t G4_mutex = NULL;
static QueueHandle_t G4_uart_queue;
static int G4_uart_num = UART_NUM_2;

//function declare
void G4_tcp_task();
static void uart_select_task();
 static void infoCommand(char *cmd, int cmdSize, char *info);
 static void enableAllInitCmd();
 static int atCmd_waitResponse(char * cmd, char *resp, char * resp1, int cmdSize, int timeout, char **response, int size);

static const char *G4TAG = "[4G_Modem]";

typedef struct
{
	char		*cmd;
	uint16_t	cmdSize;
	char		*cmdResponseOnOk;
	uint16_t	timeoutMs;
	uint16_t	delayMs;
	uint8_t		skip;
}G4_Cmd;

static G4_Cmd cmd_AT =
{
	.cmd = "AT\r\n",
	.cmdSize = sizeof("AT\r\n")-1,
	.cmdResponseOnOk = G4_OK_Str,
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_EchoOff =
{
	.cmd = "ATE0\r\n",
	.cmdSize = sizeof("ATE0\r\n")-1,
	.cmdResponseOnOk = G4_OK_Str,
	.timeoutMs = 3000,
	.delayMs = 2000,
	.skip = 0,
};

static G4_Cmd cmd_Reg =
{
	.cmd = "AT+CREG?\r\n",
	.cmdSize = sizeof("AT+CREG?\r\n")-1,
	//.cmdResponseOnOk = "CREG: 1,1",
	.cmdResponseOnOk = "CREG: 0,1",
	.timeoutMs = 3000,
	.delayMs = 2000,
	.skip = 0,
};

static G4_Cmd cmd_RFOn =
{
	.cmd = "AT+CFUN=1\r\n",
	.cmdSize = sizeof("AT+CFUN=1,0\r\n")-1,
	.cmdResponseOnOk = G4_OK_Str,
	.timeoutMs = 10000,
	.delayMs = 1000,
	.skip = 0,
};

static G4_Cmd cmd_RFOff =
{
	.cmd = "AT+CFUN=4\r\n",
	.cmdSize = sizeof("AT+CFUN=4\r\n")-1,
	.cmdResponseOnOk = G4_OK_Str,
	.timeoutMs = 10000,
	.delayMs = 1000,
	.skip = 0,
};

static G4_Cmd cmd_Tcp_Client =//打开socket
{
	.cmd = "AT+QIOPEN=1,0,\"TCP\",\"120.76.100.197\",10002,5555,2\r\n",
	.cmdSize = sizeof("AT+QIOPEN=1,0,\"TCP\",\"120.76.100.197\",10002,5555,2\r\n")-1,
	.cmdResponseOnOk = "CONNECT",
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_Pdp_Config =//配置TCP场景参数
{
	.cmd = "AT+QICSGP=1,1,\"cmnet\",\"\",\"\",1\r\n",
	.cmdSize = sizeof("AT+QICSGP=1,1,\"cmnet\",\"\",\"\",1\r\n")-1,
	.cmdResponseOnOk = G4_OK_Str,
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_Pdp_Active =//场景激活
{
	.cmd = "AT+QIACT=1\r\n",
	.cmdSize = sizeof("AT+QIACT=1\r\n")-1,
	.cmdResponseOnOk = G4_OK_Str,
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_Pdp_Deactive =//场景去激活
{
	.cmd = "AT+QIDEACT=1\r\n",
	.cmdSize = sizeof("AT+QIDEACT=1\r\n")-1,
	.cmdResponseOnOk = G4_OK_Str,
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_Cmdline_Mode =//从数据状态切换为命令状态
{
	.cmd = "+++\r\n",
	.cmdSize = sizeof("+++\r\n")-1,
	.cmdResponseOnOk = G4_OK_Str,
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_Data_Mode =//切换到数据模式
{
	.cmd = "AT+QISWTMD=1,2\r\n",
	.cmdSize = sizeof("AT+QISWTMD=1,2\r\n")-1,
	.cmdResponseOnOk ="CONNECT",
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_close_tcp =//关闭socket
{
	.cmd = "AT+QICLOSE=1\r\n",
	.cmdSize = sizeof("AT+QICLOSE=1\r\n")-1,
	.cmdResponseOnOk =G4_OK_Str,
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_send_data =//发送数据，仅直接吞吐，和缓存模式可用
{
	.cmd = "AT+QISEND\r\n",
	.cmdSize = sizeof("AT+QISEND\r\n")-1,
	.cmdResponseOnOk ="CONNECT",
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_read_data =//接受tcp的数据，仅缓存模式可用
{
	.cmd = "AT+QIRD\r\n",
	.cmdSize = sizeof("AT+QIRD\r\n")-1,
	.cmdResponseOnOk ="CONNECT",
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_ping =//PING服务器
{
	.cmd = "AT+QPING=1,\"192.168.1.1\",4,4\r\n",
	.cmdSize = sizeof("AT+QPING=1,\"192.168.1.1\",4,4")-1,
	.cmdResponseOnOk =G4_OK_Str,
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_ntp=//ntp同步时间
{
	.cmd = "AT+QNTP\r\n",
	.cmdSize = sizeof("AT+QNTP\r\n")-1,
	.cmdResponseOnOk ="CONNECT",
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_DNS_cfg =//配置DNS服务器
{
	.cmd = "AT+QIDNSCFG=1,\"8.8.8.8\",\"114.114.114.114\"\r\n",
	.cmdSize = sizeof("AT+QIDNSCFG=1,\"8.8.8.8\",\"114.114.114.114\"\r\n")-1,
	.cmdResponseOnOk ="CONNECT",
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd cmd_SDE =//控制是否回显AT+QISEND要发送的数据
{
	.cmd = "AT+QISDE\r\n",
	.cmdSize = sizeof("AT+QISDE\r\n")-1,
	.cmdResponseOnOk ="CONNECT",
	.timeoutMs = 3000,
	.delayMs = 0,
	.skip = 0,
};

static G4_Cmd* G4_Init[] =
{
		&cmd_AT,//AT
		//&cmd_Reset,//ATZ
		&cmd_EchoOff,//ATE0
		//&cmd_RFOn,//AT+CFUN=1
		//&cmd_NoSMSInd,//AT+CNMI=0,0,0,0,0
		//&cmd_Pin,//AT+CPIN?
		&cmd_Reg,//AT+CREG?
		//&cmd_APN,//
		//&cmd_CGACT,
		//&cmd_CGDADTA,//AT+CGDATA=\"PPP\",1
		//&cmd_ATD,
		&cmd_Pdp_Config,
		&cmd_Pdp_Active,
		&cmd_Tcp_Client,

};

#define G4_InitCmdsSize  (sizeof(G4_Init)/sizeof(G4_Cmd *))

static int atCmd_waitResponse(char * cmd, char *resp, char * resp1, int cmdSize, int timeout, char **response, int size)
{
	char sresp[256] = {'\0'};
	char data[256] = {'\0'};
    int len, res = 1, idx = 0, tot = 0, timeoutCnt = 0;

	// ** Send command to GSM
	vTaskDelay(100 / portTICK_PERIOD_MS);
	uart_flush(G4_uart_num);

	if (cmd != NULL) {
		if (cmdSize == -1) cmdSize = strlen(cmd);
		#if G4_DEBUG
		infoCommand(cmd, cmdSize, "AT COMMAND:");
		#endif
		uart_write_bytes(G4_uart_num, (const char*)cmd, cmdSize);
		uart_wait_tx_done(G4_uart_num, 100 / portTICK_RATE_MS);
	}

	if (response != NULL) {
		// Read GSM response into buffer
		char *pbuf = *response;
		len = uart_read_bytes(G4_uart_num, (uint8_t*)data, 256, timeout / portTICK_RATE_MS);
		while (len > 0) {
			if ((tot+len) >= size) {
				char *ptemp = realloc(pbuf, size+512);
				if (ptemp == NULL) return 0;
				size += 512;
				pbuf = ptemp;
			}
			memcpy(pbuf+tot, data, len);
			tot += len;
			response[tot] = '\0';
			len = uart_read_bytes(G4_uart_num, (uint8_t*)data, 256, 100 / portTICK_RATE_MS);
		}
		*response = pbuf;
		return tot;
	}

    // ** Wait for and check the response
	idx = 0;
	while(1)
	{
		memset(data, 0, 256);
		len = 0;
		len = uart_read_bytes(G4_uart_num, (uint8_t*)data, 256, 10 / portTICK_RATE_MS);
		if (len > 0) {
			for (int i=0; i<len;i++) {
				if (idx < 256) {
					if ((data[i] >= 0x20) && (data[i] < 0x80)) sresp[idx++] = data[i];
					else sresp[idx++] = 0x2e;
				}
			}
			tot += len;
		}
		else {
			if (tot > 0) {
				// Check the response
				if (strstr(sresp, resp) != NULL) {
					#if G4_DEBUG
					ESP_LOGI(G4TAG,"AT RESPONSE: [%s]", sresp);
					#endif
					break;
				}
				else {
					if (resp1 != NULL) {
						if (strstr(sresp, resp1) != NULL) {
							#if G4_DEBUG
							ESP_LOGI(G4TAG,"AT RESPONSE (1): [%s]", sresp);
							#endif
							res = 2;
							break;
						}
					}
					// no match
					#if G4_DEBUG
					ESP_LOGI(G4TAG,"AT BAD RESPONSE: [%s]", sresp);
					#endif
					res = 0;
					break;
				}
			}
		}

		timeoutCnt += 10;
		if (timeoutCnt > timeout) {
			// timeout
			#if G4_DEBUG
			ESP_LOGE(G4TAG,"AT: TIMEOUT");
			#endif
			res = 0;
			break;
		}
	}

	return res;
}

static void enableAllInitCmd()
{
	for (int idx = 0; idx < G4_InitCmdsSize; idx++) {
		G4_Init[idx]->skip = 0;
	}
}


static void infoCommand(char *cmd, int cmdSize, char *info)
{
	char buf[cmdSize+2];
	memset(buf, 0, cmdSize+2);

	for (int i=0; i<cmdSize;i++) {
		if ((cmd[i] != 0x00) && ((cmd[i] < 0x20) || (cmd[i] > 0x7F))) buf[i] = '.';
		else buf[i] = cmd[i];
		if (buf[i] == '\0') break;
	}
	ESP_LOGI(G4TAG,"%s [%s]", info, buf);
}

static void _disconnect(uint8_t rfOff)
{
	int res = atCmd_waitResponse("AT\r\n", G4_OK_Str, NULL, 4, 1000, NULL, 0);
	if (res == 1) {
		if (rfOff) {
			cmd_Reg.timeoutMs = 10000;
			res = atCmd_waitResponse("AT+CFUN=4\r\n", G4_OK_Str, NULL, 11, 10000, NULL, 0); // disable RF function
		}
		return;
	}

	#if G4_DEBUG
	ESP_LOGI(G4TAG,"ONLINE, DISCONNECTING...");
	#endif
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	uart_flush(G4_uart_num);
	uart_write_bytes(G4_uart_num, "+++", 3);
    uart_wait_tx_done(G4_uart_num, 10 / portTICK_RATE_MS);
	vTaskDelay(1100 / portTICK_PERIOD_MS);

	int n = 0;
	res = atCmd_waitResponse("ATH\r\n", G4_OK_Str, "NO CARRIER", 5, 3000, NULL, 0);
	while (res == 0) {
		n++;
		if (n > 10) {
			#if GSM_DEBUG
			ESP_LOGI(TAG,"STILL CONNECTED.");
			#endif
			n = 0;
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			uart_flush(G4_uart_num);
			uart_write_bytes(G4_uart_num, "+++", 3);
		    uart_wait_tx_done(G4_uart_num, 10 / portTICK_RATE_MS);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
		res = atCmd_waitResponse("ATH\r\n", G4_OK_Str, "NO CARRIER", 5, 3000, NULL, 0);
	}
	vTaskDelay(100 / portTICK_PERIOD_MS);
	if (rfOff) {
		cmd_Reg.timeoutMs = 10000;
		res = atCmd_waitResponse("AT+CFUN=4\r\n", G4_OK_Str, NULL, 11, 3000, NULL, 0);
	}
	#if GSM_DEBUG
	ESP_LOGI(TAG,"DISCONNECTED.");
	#endif
}

static void uart_select_task()
{
    
    while (1) {
        int fd;

        if ((fd = open("/dev/uart/2", O_RDWR)) == -1) {
            ESP_LOGE(G4TAG, "Cannot open UART");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        // We have a driver now installed so set up the read/write functions to use driver also.
        esp_vfs_dev_uart_use_driver(0);

        while (1) 
		{
            int s;
            fd_set rfds;
            struct timeval tv = {
                .tv_sec = 0,
                .tv_usec = 1000,
            };

            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);

            s = select(fd + 1, &rfds, NULL, NULL, &tv);

            if (s < 0) {
				xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
				g4_status=G4_TCP_STATE_DISCONNECT;
				xSemaphoreGive(G4_mutex);
                ESP_LOGE(G4TAG, "Select failed: errno %d", errno);
                break;
            } 
			else if (s == 0) {
                ESP_LOGI(G4TAG, "Timeout has been reached and nothing has been received");
            } 
			else 
			{
                if (FD_ISSET(fd, &rfds)) {
                    char buf[1024]={0};
                    if (read(fd, &buf, sizeof(buf)) > 0) {
                        ESP_LOGI(G4TAG, "Received: %s", buf);
                        // Note: Only one character was read even the buffer contains more. The other characters will
                        // be read one-by-one by subsequent calls to select() which will then return immediately
                        // without timeout.
						memset(buf,0,sizeof(buf));
                    } 
					else {
						xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
						g4_status=G4_TCP_STATE_DISCONNECT;
						xSemaphoreGive(G4_mutex);
                        ESP_LOGE(G4TAG, "UART read error");
                        break;
                    }
                } 
				else
				{
                    ESP_LOGE(G4TAG, "No FD has been set in select()");
                    break;
                }
            }
        }

        close(fd);
    }

    //vTaskDelete(NULL);
}


void G4_tcp_task()
{
	xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
	tcp_task_started = 1;
	xSemaphoreGive(G4_mutex);
	
	char* data = (char*) malloc(BUF_SIZE);
    if (data == NULL) {
		#if G4_DEBUG
		ESP_LOGE(G4TAG,"Failed to allocate data buffer.");
		#endif
    	goto exit;
    }

	if (gpio_set_direction(G4_TX, GPIO_MODE_OUTPUT)) goto exit;
	if (gpio_set_direction(G4_RX, GPIO_MODE_INPUT)) goto exit;
	if (gpio_set_pull_mode(G4_RX, GPIO_PULLUP_ONLY)) goto exit;

		uart_config_t uart_config = {
			.baud_rate = G4_Modem_BDRATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

	if (uart_driver_install(G4_uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0)) goto exit;
	if (uart_param_config(G4_uart_num, &uart_config)) goto exit;
	//Set UART1 pins(TX, RX, RTS, CTS)
	if (uart_set_pin(G4_uart_num, G4_TX, G4_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) goto exit;

	xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
    //tcp_tx_count = 0;
   // tcp_rx_count = 0;
	g4_status = G4_Modem_STATE_FIRSTINIT;
	xSemaphoreGive(G4_mutex);

	enableAllInitCmd();

	
	while(1)
	{
		#if G4_DEBUG
		ESP_LOGI(G4TAG,"G4 initialization start");
		#endif
		vTaskDelay(500 / portTICK_PERIOD_MS);

		int g4CmdIter = 0;
		int nfail = 0;
		// * GSM Initialization loop
		while(g4CmdIter < G4_InitCmdsSize)
		{
			if (G4_Init[g4CmdIter]->skip) {
				#if G4_DEBUG
				infoCommand(G4_Init[g4CmdIter]->cmd, G4_Init[g4CmdIter]->cmdSize, "Skip command:");
				#endif
				g4CmdIter++;
				continue;
			}
			if (atCmd_waitResponse(G4_Init[g4CmdIter]->cmd,
					G4_Init[g4CmdIter]->cmdResponseOnOk, NULL,
					G4_Init[g4CmdIter]->cmdSize,
					G4_Init[g4CmdIter]->timeoutMs, NULL, 0) == 0)
			{
				// * No response or not as expected, start from first initialization command
				#if G4_DEBUG
				ESP_LOGW(G4TAG,"Wrong response, restarting...");
				#endif

				nfail++;
				if (nfail > 20) goto exit;

				vTaskDelay(3000 / portTICK_PERIOD_MS);
				g4CmdIter = 0;//下次又从第一条命令开始
				continue;
			}

			if (G4_Init[g4CmdIter]->delayMs > 0) vTaskDelay(G4_Init[g4CmdIter]->delayMs / portTICK_PERIOD_MS);
			G4_Init[g4CmdIter]->skip = 1;
			if (G4_Init[g4CmdIter] == &cmd_Reg) G4_Init[g4CmdIter]->delayMs = 0;
			// Next command
			g4CmdIter++;
		}


		#if G4_DEBUG
		ESP_LOGI(G4TAG,"G4 initialized，TCP client has connected");
		#endif

		// *** LOOP: Handle GSM modem responses & disconnects ***
		while(1) {
			// === Check if disconnect requested ===
			xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
			if (do_tcp_connect <= 0) {
				int end_task = do_tcp_connect;
				do_tcp_connect = 1;
				xSemaphoreGive(G4_mutex);
				#if G4_DEBUG
				printf("\r\n");
				ESP_LOGI(G4TAG, "Disconnect requested.");
				#endif

				//pppapi_close(ppp, 0);
				int gstat = 1;
				while (g4_status != G4_TCP_STATE_DISCONNECT) {
					// Handle data received from GSM
					memset(data, 0, BUF_SIZE);
					int len = uart_read_bytes(G4_uart_num, (uint8_t*)data, BUF_SIZE, 30 / portTICK_RATE_MS);
					if (len > 0)	{
						/*
						pppos_input_tcpip(ppp, (u8_t*)data, len);
						xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
					    pppos_tx_count += len;
						xSemaphoreGive(pppos_mutex);
						*/
					#if G4_DEBUG
					//printf("\r\n");
					ESP_LOGI(G4TAG, "Received error data: %s", data);
					#endif
					
					}
					xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
					gstat = g4_status;
					xSemaphoreGive(G4_mutex);
				}
				vTaskDelay(1000 / portTICK_PERIOD_MS);

				xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
				uint8_t rfoff = G4_rfOff;
				xSemaphoreGive(G4_mutex);
				_disconnect(rfoff); // Disconnect GSM if still connected

				#if G4_DEBUG
				ESP_LOGI(G4TAG, "Disconnected.");
				#endif

				g4CmdIter = 0;
				enableAllInitCmd();
				xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
				g4_status = G4_TCP_STATE_IDLE;
				do_tcp_connect = 0;
				xSemaphoreGive(G4_mutex);

				if (end_task < 0) goto exit;

				// === Wait for reconnect request ===
				gstat = 0;
				while (gstat == 0) {
					vTaskDelay(100 / portTICK_PERIOD_MS);
					xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
					gstat = do_tcp_connect;
					xSemaphoreGive(G4_mutex);
				}
				#if G4_DEBUG
				printf("\r\n");
				ESP_LOGI(G4TAG, "Reconnect requested.");
				#endif
				break;
			}

			// === Check if disconnected ===
			if (g4_status == G4_TCP_STATE_DISCONNECT) {
				xSemaphoreGive(G4_mutex);
				#if G4_DEBUG
				printf("\r\n");
				ESP_LOGE(G4TAG, "Disconnected, trying again...");
				#endif
				//pppapi_close(ppp, 0);

				enableAllInitCmd();
				g4CmdIter = 0;
				g4_status = G4_TCP_STATE_IDLE;
				vTaskDelay(10000 / portTICK_PERIOD_MS);
				break;
			}
			else xSemaphoreGive(G4_mutex);

			// === Handle data received from GSM ===
			memset(data, 0, BUF_SIZE);
			int len = uart_read_bytes(G4_uart_num, (uint8_t*)data, BUF_SIZE, 30 / portTICK_RATE_MS);
			if (len > 0)	{
				/*
				pppos_input_tcpip(ppp, (u8_t*)data, len);
				xSemaphoreTake(pppos_mutex, PPPOSMUTEX_TIMEOUT);
			    pppos_tx_count += len;
				xSemaphoreGive(pppos_mutex);
				*/
				#if G4_DEBUG
					//printf("\r\n");
					ESP_LOGI(G4TAG, "Received: %s", data);
				#endif
				vTaskDelay(10000 / portTICK_PERIOD_MS);
			}

		}  // Handle GSM modem responses & disconnects loop
	}
	exit:
		if (data) free(data);  // free data buffer
		//if (ppp) ppp_free(ppp);
		xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
		tcp_task_started = 0;
		//gsm_status = GSM_STATE_FIRSTINIT;
		xSemaphoreGive(G4_mutex);
		#if G4_DEBUG
		ESP_LOGE(G4TAG, "TCP TASK TERMINATED");
		#endif
		vTaskDelete(NULL);
}

int G4_init()
{
	if(G4_mutex != NULL) xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
	do_tcp_connect = 1;
	int g4stat=0;
	int task_s=tcp_task_started;
	if(G4_mutex!=NULL) xSemaphoreGive(G4_mutex);
	if(task_s==0)
	{
			if(G4_mutex==NULL) G4_mutex=xSemaphoreCreateMutex();
			if(G4_mutex==NULL) return 0;
			
			ESP_LOGI(G4TAG, "TCP TASK will STARTING");
			
			xTaskCreate(&G4_tcp_task, "tcp_client", 4096, NULL, 10, NULL);
			while(task_s==0)
			{
					vTaskDelay(10 / portTICK_RATE_MS);
					xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
					task_s = tcp_task_started;
					xSemaphoreGive(G4_mutex);
			}
	}
	while (g4stat != 1)
	{
		vTaskDelay(10 / portTICK_RATE_MS);
		xSemaphoreTake(G4_mutex, TCPMUTEX_TIMEOUT);
		g4stat = g4_status;
		task_s = tcp_task_started;
		xSemaphoreGive(G4_mutex);
		if (task_s == 0) return 0;
	}
	
	return 1;
}



