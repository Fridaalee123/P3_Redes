
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "mqtt_freertos.h"

#include "board.h"
#include "fsl_silicon_id.h"

#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/apps/mqtt.h"
#include "lwip/tcpip.h"
#include <stdio.h>
#include "fsl_gpio.h"
#include "app.h"
#include "GPIO.h"
#include "RGB.h"


// FIXME cleanup

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief MQTT server host name or IP address. */
#ifndef EXAMPLE_MQTT_SERVER_HOST
#define EXAMPLE_MQTT_SERVER_HOST "broker.hivemq.com"
#endif

/*! @brief MQTT server port number. */
#ifndef EXAMPLE_MQTT_SERVER_PORT
#define EXAMPLE_MQTT_SERVER_PORT 1883
#endif

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*! @brief Stack size of the temporary initialization thread. */
#define APP_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary initialization thread. */
#define APP_THREAD_PRIO DEFAULT_THREAD_PRIO
#define PAYLOAD_BUFFER_SIZE 64

uint16_t pulso = 1;
uint16_t temperatura = 1;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void connect_to_mqtt(void *ctx);

static mqtt_client_t *mqtt_client;

/*! @brief MQTT client ID string. */
static char client_id[(SILICONID_MAX_LENGTH * 2) + 5];

/*! @brief MQTT client information. */
static const struct mqtt_connect_client_info_t mqtt_client_info = {
    .client_id   = (const char *)&client_id[0],
    .client_user = NULL,
    .client_pass = NULL,
    .keep_alive  = 100,
    .will_topic  = NULL,
    .will_msg    = NULL,
    .will_qos    = 0,
    .will_retain = 0,
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    .tls_config = NULL,
#endif
};

enum TopicID
{
    TOPIC_UNKNOWN,
    TOPIC_SW,
    TOPIC_MED,
	TOPIC_HR,
	TOPIC_TEMP,
	TOPIC_ALARM,
	TOPIC_MSG
};

enum MsgState
{
    OFF,
	ON,
	SUPPLY,
	DANGER,
	SAFE,
	CAUTION
};

static enum TopicID current_topic_id = TOPIC_UNKNOWN;
static char payload[PAYLOAD_BUFFER_SIZE];
static uint16_t offset = 0;

/*! @brief MQTT broker IP address. */
static ip_addr_t mqtt_addr;

/*! @brief Indicates connection to MQTT broker. */
static volatile bool connected = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Called when subscription request finishes.
 */
static void mqtt_topic_subscribed_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Subscribed to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to subscribe to the topic \"%s\": %d.\r\n", topic, err);
    }
}


static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    LWIP_UNUSED_ARG(arg);

    if (strcmp(topic, "monitor/msg") == 0)
		current_topic_id = TOPIC_MSG;

	else if (strcmp(topic, "monitor/med") == 0)
		current_topic_id = TOPIC_MED;

	else
		current_topic_id = TOPIC_UNKNOWN;

    PRINTF("Received %u bytes from the topic \"%s\": \"", tot_len, topic);
}


static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	LWIP_UNUSED_ARG(arg);

	for (int i = 0; i < len; i++)
	{
		if (offset < PAYLOAD_BUFFER_SIZE - 1)  // Dejas espacio para '\0'
		{
			payload[offset++] = data[i];
		}
	}

	if (flags & MQTT_DATA_FLAG_LAST)
	{
		payload[offset] = '\0';
		offset = 0; // Reiniciar offset para el siguiente mensaje
		PRINTF("/nData: %s\n", payload);

		if (current_topic_id == TOPIC_MSG)
		{
			if (strcmp(payload, "Assisted") == 0)
				RGB_select_color_on(Yellow);
		}
		else if (current_topic_id == TOPIC_MED)
		{
			if(strcmp(payload, "Medicate") == 0)
				RGB_select_color_on(Green);
		}
		else
		{
			RGB_select_color_on(Black);
		}
	}
}


/*!
 * @brief Subscribe to MQTT topics.
 */

static void mqtt_subscribe_topics(mqtt_client_t *client)
{
    static const char *topics[] = {"monitor/alarm", "monitor/msg", "monitor/med"};
    int qos[] = {0, 0, 0};

    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb,
                            LWIP_CONST_CAST(void *, &mqtt_client_info));

    for (int i = 0; i < ARRAY_SIZE(topics); i++)
    {
        err_t err = mqtt_subscribe(client, topics[i], qos[i], mqtt_topic_subscribed_cb, LWIP_CONST_CAST(void *, topics[i]));

        if (err == ERR_OK)
            PRINTF("Subscribing to the topic \"%s\" with QoS %d...\r\n", topics[i], qos[i]);
        else
            PRINTF("Failed to subscribe to the topic \"%s\" with QoS %d: %d.\r\n", topics[i], qos[i], err);
    }

}


/*!
 * @brief Called when connection state changes.
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;

    connected = (status == MQTT_CONNECT_ACCEPTED);

    switch (status)
    {
        case MQTT_CONNECT_ACCEPTED:
            PRINTF("MQTT client \"%s\" connected.\r\n", client_info->client_id);
            mqtt_subscribe_topics(client);
            break;

        case MQTT_CONNECT_DISCONNECTED:
            PRINTF("MQTT client \"%s\" not connected.\r\n", client_info->client_id);
            /* Try to reconnect 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_TIMEOUT:
            PRINTF("MQTT client \"%s\" connection timeout.\r\n", client_info->client_id);
            /* Try again 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:
        case MQTT_CONNECT_REFUSED_IDENTIFIER:
        case MQTT_CONNECT_REFUSED_SERVER:
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:
            PRINTF("MQTT client \"%s\" connection refused: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;

        default:
            PRINTF("MQTT client \"%s\" connection status: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;
    }
}

/*!
 * @brief Starts connecting to MQTT broker. To be called on tcpip_thread.
 */
static void connect_to_mqtt(void *ctx)
{
    LWIP_UNUSED_ARG(ctx);

    PRINTF("Connecting to MQTT broker at %s...\r\n", ipaddr_ntoa(&mqtt_addr));

    mqtt_client_connect(mqtt_client, &mqtt_addr, EXAMPLE_MQTT_SERVER_PORT, mqtt_connection_cb,
                        LWIP_CONST_CAST(void *, &mqtt_client_info), &mqtt_client_info);
}

/*!
 * @brief Called when publish request finishes.
 */
static void mqtt_message_published_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Published to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to publish to the topic \"%s\": %d.\r\n", topic, err);
    }
}


static void publish_message(void *ctx)
{

	static const char *topic_1 = "lwip_topic/Pc";
	static char message_Pc[64];

	static const char *topic_2 = "lwip_topic/Temp";
	static char message_Temp[64];

	 GPIO_PortInit(GPIO, APP_SW_PORT);


	    while (1)
	    {
	        if (!(GPIO_PortRead(GPIO, APP_SW_PORT) & (1 << APP_SW_PIN)))
	        {
	            vTaskDelay(pdMS_TO_TICKS(100));

	            if (!(GPIO_PortRead(GPIO, APP_SW_PORT) & (1 << APP_SW_PIN)))
	            {
	                // Actualizar pulso
	                switch (pulso)
	                {
	                    case 1: pulso = 250; break;
	                    case 250: pulso = 500; break;
	                    case 500: pulso = 750; break;
	                    case 750: pulso = 1000; break;
	                    case 1000: pulso = 1500; break;
	                    default: pulso = 1; break;
	                }

	                // Actualizar temperatura
	                switch (temperatura)
	                {
	                    case 1: temperatura = 20; break;
	                    case 20: temperatura = 29; break;
	                    case 29: temperatura = 35; break;
	                    case 35: temperatura = 44; break;
	                    case 44: temperatura = 58; break;
	                    default: temperatura = 1; break;
	                }

	                PRINTF("\r\nPulso: %d, Temperatura: %d\r\n", pulso, temperatura);

	                // Esperar liberación del botón
	                while (!(GPIO_PortRead(GPIO, APP_SW_PORT) & (1 << APP_SW_PIN)))
	                {
	                    vTaskDelay(pdMS_TO_TICKS(100));
	                }
	            }
	        }

	        vTaskDelay(pdMS_TO_TICKS(100));
	    }

    LWIP_UNUSED_ARG(ctx);

    //Pc
    snprintf(message_Pc, sizeof(message_Pc), "%d\r\n", pulso);
    //Temp
    snprintf(message_Temp, sizeof(message_Temp), "%d\r\n",temperatura);

    mqtt_publish(mqtt_client, topic_1, message_Pc, strlen(message_Pc), 1, 0, mqtt_message_published_cb, (void *)topic_1);

    mqtt_publish(mqtt_client, topic_2, message_Temp, strlen(message_Temp), 1, 0, mqtt_message_published_cb, (void *)topic_2);

}
//
//void vThread_paciente(void *pvParameters)
//{
//    GPIO_PortInit(GPIO, APP_SW_PORT);
//
//
//    while (1)
//    {
//        if (!(GPIO_PortRead(GPIO, APP_SW_PORT) & (1 << APP_SW_PIN)))
//        {
//            vTaskDelay(pdMS_TO_TICKS(100));
//
//            if (!(GPIO_PortRead(GPIO, APP_SW_PORT) & (1 << APP_SW_PIN)))
//            {
//                // Actualizar pulso
//                switch (pulso)
//                {
//                    case 1: pulso = 250; break;
//                    case 250: pulso = 500; break;
//                    case 500: pulso = 750; break;
//                    case 750: pulso = 1000; break;
//                    case 1000: pulso = 1500; break;
//                    default: pulso = 1; break;
//                }
//
//                // Actualizar temperatura
//                switch (temperatura)
//                {
//                    case 1: temperatura = 20; break;
//                    case 20: temperatura = 29; break;
//                    case 29: temperatura = 35; break;
//                    case 35: temperatura = 44; break;
//                    case 44: temperatura = 58; break;
//                    default: temperatura = 1; break;
//                }
//
//                PRINTF("\r\nPulso: %d, Temperatura: %d\r\n", pulso, temperatura);
//
//                // Esperar liberación del botón
//                while (!(GPIO_PortRead(GPIO, APP_SW_PORT) & (1 << APP_SW_PIN)))
//                {
//                    vTaskDelay(pdMS_TO_TICKS(100));
//                }
//            }
//        }
//
//        vTaskDelay(pdMS_TO_TICKS(100));
//    }
//}



/*!
 * @brief Application thread.
 */
static void app_thread(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    err_t err;
    int i;

    PRINTF("\r\nIPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
    PRINTF("IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
    PRINTF("IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));

    /*
     * Check if we have an IP address or host name string configured.
     * Could just call netconn_gethostbyname() on both IP address or host name,
     * but we want to print some info if goint to resolve it.
     */
    if (ipaddr_aton(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr) && IP_IS_V4(&mqtt_addr))
    {
        /* Already an IP address */
        err = ERR_OK;
    }
    else
    {
        /* Resolve MQTT broker's host name to an IP address */
        PRINTF("Resolving \"%s\"...\r\n", EXAMPLE_MQTT_SERVER_HOST);
        err = netconn_gethostbyname(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr);
    }

    if (err == ERR_OK)
    {
        /* Start connecting to MQTT broker from tcpip_thread */
        err = tcpip_callback(connect_to_mqtt, NULL);
        if (err != ERR_OK)
        {
            PRINTF("Failed to invoke broker connection on the tcpip_thread: %d.\r\n", err);
        }
    }
    else
    {
        PRINTF("Failed to obtain IP address: %d.\r\n", err);
    }

    /* Publish some messages */
    for (i = 0; i < 5;)
    {
        if (connected)
        {
            err = tcpip_callback(publish_message, NULL);
            if (err != ERR_OK)
            {
                PRINTF("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
            }
            i++;
            //xTaskCreate(vThread_paciente, "PacienteTask", configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY + 1, NULL);

        }
    break;

    sys_msleep(1000U);
    }

    vTaskDelete(NULL);
}

static void generate_client_id(void)
{
    uint8_t silicon_id[SILICONID_MAX_LENGTH];
    const char *hex = "0123456789abcdef";
    status_t status;
    uint32_t id_len = sizeof(silicon_id);
    int idx         = 0;
    int i;
    bool id_is_zero = true;

    /* Get unique ID of SoC */
    status = SILICONID_GetID(&silicon_id[0], &id_len);
    assert(status == kStatus_Success);
    assert(id_len > 0U);
    (void)status;

    /* Covert unique ID to client ID string in form: nxp_hex-unique-id */

    /* Check if client_id can accomodate prefix, id and terminator */
    assert(sizeof(client_id) >= (5U + (2U * id_len)));

    /* Fill in prefix */
    client_id[idx++] = 'n';
    client_id[idx++] = 'x';
    client_id[idx++] = 'p';
    client_id[idx++] = '_';

    /* Append unique ID */
    for (i = (int)id_len - 1; i >= 0; i--)
    {
        uint8_t value    = silicon_id[i];
        client_id[idx++] = hex[value >> 4];
        client_id[idx++] = hex[value & 0xFU];

        if (value != 0)
        {
            id_is_zero = false;
        }
    }

    /* Terminate string */
    client_id[idx] = '\0';

    if (id_is_zero)
    {
        PRINTF(
            "WARNING: MQTT client id is zero. (%s)"
#ifdef OCOTP
            " This might be caused by blank OTP memory."
#endif
            "\r\n",
            client_id);
    }
}

/*!
 * @brief Create and run example thread
 *
 * @param netif  netif which example should use
 */
void mqtt_freertos_run_thread(struct netif *netif)
{
    LOCK_TCPIP_CORE();
    mqtt_client = mqtt_client_new();
    UNLOCK_TCPIP_CORE();
    if (mqtt_client == NULL)
    {
        PRINTF("mqtt_client_new() failed.\r\n");
        while (1)
        {
        }
    }

    generate_client_id();

    if (sys_thread_new("app_task", app_thread, netif, APP_THREAD_STACKSIZE, APP_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("mqtt_freertos_start_thread(): Task creation failed.", 0);
    }
}
