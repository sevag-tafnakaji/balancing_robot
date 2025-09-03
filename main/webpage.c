/* openSSL server example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "protocol_examples_common.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <sys/socket.h>

#include "openssl/ssl.h"

static const char *websocket_tag = "WEBSOCKET";

extern const uint8_t ca_pem_start[] asm("_binary_ca_pem_start");
extern const uint8_t ca_pem_end[] asm("_binary_ca_pem_end");
extern const uint8_t server_pem_start[] asm("_binary_server_pem_start");
extern const uint8_t server_pem_end[] asm("_binary_server_pem_end");
extern const uint8_t server_key_start[] asm("_binary_server_key_start");
extern const uint8_t server_key_end[] asm("_binary_server_key_end");

/*
Fragment size range 2048~8192
| Private key len | Fragment size recommend |
| RSA2048         | 2048                    |
| RSA3072         | 3072                    |
| RSA4096         | 4096                    |
*/
#define OPENSSL_SERVER_FRAGMENT_SIZE 2048

/* Local server tcp port */
#define OPENSSL_SERVER_LOCAL_TCP_PORT 443

#define OPENSSL_SERVER_REQUEST "{\"path\": \"/v1/ping/\", \"method\": \"GET\"}\r\n"

/* receive length */
#define OPENSSL_SERVER_RECV_BUF_LEN 1024

static char send_data[] = OPENSSL_SERVER_REQUEST;
static int send_bytes = sizeof(send_data);

static char recv_buf[OPENSSL_SERVER_RECV_BUF_LEN];

static void openssl_server_task(void *p)
{
    int ret;

    SSL_CTX *ctx;
    SSL *ssl;

    struct sockaddr_in sock_addr;
    int sockfd, new_sockfd;
    int recv_bytes = 0;
    socklen_t addr_len;

    ESP_LOGI(websocket_tag, "OpenSSL server thread start...\n");

    ESP_LOGI(websocket_tag, "create SSL context ......");
    ctx = SSL_CTX_new(TLSv1_2_server_method());

    if (!ctx)
    {
        ESP_LOGI(websocket_tag, "failed\n");
        goto failed1;
    }

    ESP_LOGI(websocket_tag, "OK\n");

    ESP_LOGI(websocket_tag, "load server crt ......");
    ret = SSL_CTX_use_certificate_ASN1(ctx, server_pem_end - server_pem_start, server_pem_start);

    if (ret)
    {
        ESP_LOGI(websocket_tag, "OK\n");
    }
    else
    {
        ESP_LOGI(websocket_tag, "failed\n");
        goto failed2;
    }

    ESP_LOGI(websocket_tag, "load server private key ......");
    ret = SSL_CTX_use_PrivateKey_ASN1(0, ctx, server_key_start, server_key_end - server_key_start);

    if (ret)
    {
        ESP_LOGI(websocket_tag, "OK\n");
    }
    else
    {
        ESP_LOGI(websocket_tag, "failed\n");
        goto failed2;
    }

    ESP_LOGI(websocket_tag, "set verify mode verify peer\n");
    SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, NULL);

    ESP_LOGI(websocket_tag, "create socket ......");
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0)
    {
        ESP_LOGI(websocket_tag, "failed\n");
        goto failed2;
    }

    ESP_LOGI(websocket_tag, "OK\n");

    ESP_LOGI(websocket_tag, "socket bind ......");
    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.sin_family = AF_INET;
    sock_addr.sin_addr.s_addr = 0;
    sock_addr.sin_port = htons(OPENSSL_SERVER_LOCAL_TCP_PORT);

    ret = bind(sockfd, (struct sockaddr *)&sock_addr, sizeof(sock_addr));

    if (ret)
    {
        ESP_LOGI(websocket_tag, "bind failed\n");
        goto failed3;
    }

    ESP_LOGI(websocket_tag, "bind OK\n");

    ESP_LOGI(websocket_tag, "server socket listen ......");
    ret = listen(sockfd, 32);

    if (ret)
    {
        ESP_LOGI(websocket_tag, "failed\n");
        goto failed3;
    }

    ESP_LOGI(websocket_tag, "OK\n");

reconnect:
    ESP_LOGI(websocket_tag, "SSL server create ......");
    ssl = SSL_new(ctx);

    if (!ssl)
    {
        ESP_LOGI(websocket_tag, "failed\n");
        goto failed3;
    }

    ESP_LOGI(websocket_tag, "OK\n");

    ESP_LOGI(websocket_tag, "SSL server socket accept client ......");
    new_sockfd = accept(sockfd, (struct sockaddr *)&sock_addr, &addr_len);

    if (new_sockfd < 0)
    {
        ESP_LOGI(websocket_tag, "failed");
        goto failed4;
    }

    ESP_LOGI(websocket_tag, "OK\n");

    SSL_set_fd(ssl, new_sockfd);

    ESP_LOGI(websocket_tag, "SSL server accept client ......");
    ret = SSL_accept(ssl);

    if (!ret)
    {
        ESP_LOGI(websocket_tag, "failed\n");
        goto failed5;
    }

    ESP_LOGI(websocket_tag, "OK\n");

    ESP_LOGI(websocket_tag, "send data to client ......");
    ret = SSL_write(ssl, send_data, send_bytes);

    if (ret <= 0)
    {
        ESP_LOGI(websocket_tag, "failed, return [-0x%x]\n", -ret);
        goto failed5;
    }

    ESP_LOGI(websocket_tag, "OK\n\n");

    do
    {
        ret = SSL_read(ssl, recv_buf, OPENSSL_SERVER_RECV_BUF_LEN - 1);

        if (ret <= 0)
        {
            break;
        }

        recv_bytes += ret;
        recv_buf[ret] = '\0';
        ESP_LOGI(websocket_tag, "%s", recv_buf);
    } while (1);

    SSL_shutdown(ssl);
failed5:
    close(new_sockfd);
    new_sockfd = -1;
failed4:
    SSL_free(ssl);
    ssl = NULL;
    goto reconnect;
failed3:
    close(sockfd);
    sockfd = -1;
failed2:
    SSL_CTX_free(ctx);
    ctx = NULL;
failed1:
    vTaskDelete(NULL);
    ESP_LOGI(websocket_tag, "task exit\n");

    return;
}
