#include "app_wifi_manager.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "mdns.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"

#include "esp_http_server.h"

#define DEBUG_LVL   LVL_TRACE   /**< logging level */
#define MOD_NAME    "wifi_manager"       /**< module name */
#include "ez_logging.h"
#include "ez_event_bus.h"
#include "ez_state_machine.h"

#include "app/gpio/app_gpio.h"
#include "app_event_bus.h"


#define WIFI_PARTITION_NAME     "wifi_manager_part"
#define EXAMPLE_ESP_WIFI_SSID   "rgb_guitar_stand"
#define EXAMPLE_ESP_WIFI_PASS   "ionian_dorian_1234"
#define EXAMPLE_MAX_STA_CONN    2
#define WIFI_NVS_NAMESPACE      "wifi_creds"
#define TASK_PRIORITY           10
#define TASK_SIZE               4096
#define MAX_CONNECT_RETRY       10
#define DEVICE_HOSTNAME         "guitar-stand"
#define DNS_PORT                53
#define DNS_PACKET_MAX_SIZE     512

#define SM_EVENT_WIFI_CREDENTIALS_RECEIVED  0x01
#define SM_EVENT_START_CAPTIVE_PORTAL       0x02
#define SM_EVENT_WIFI_START                 0x03
#define SM_EVENT_WIFI_CONNECTED             0x04
#define SM_EVENT_WIFI_DISCONNECTED          0x05

typedef enum{
    SM_ERR_DISCONNECTED,
    SM_ERR_LOAD_CREDENTIALS,
    SM_ERR_END,
}SM_ERR_CODE;

typedef struct{
    httpd_handle_t http_server_h;
    httpd_config_t http_server_conf;
    uint32_t reconnect_count;
    uint32_t count_100ms;
    nvs_handle_t nvs;
    wifi_config_t wifi_ap_config;
    wifi_config_t wifi_sta_config;
    esp_netif_t *netif;
    SM_ERR_CODE error_code;
}WifiManagerContext_t;

extern const char captive_portal_start[] asm("_binary_captive_portal_html_start");
extern const char captive_portal_end[] asm("_binary_captive_portal_html_end");

static ezEventListener_t button_event_observer;
static uint8_t sm_event_buff[32];
static WifiManagerContext_t context;
static TaskHandle_t dns_redirect_task_h = NULL;
static int dns_redirect_sock = -1;
static bool mdns_started = false;

static void dhcp_set_captiveportal_url(void);
static void dns_redirect_start(void);
static void dns_redirect_stop(void);
static void dns_redirect_task(void *arg);
static esp_err_t wifiManager_InitNetStack(void);
static esp_err_t wifiManager_ConfigureApDns(void);
static esp_err_t wifiManager_StartMdns(void);
static void wifiManager_StopMdns(void);
static httpd_handle_t start_webserver(httpd_handle_t* handle);
static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err);
static esp_err_t captive_portal_probe_handler(httpd_req_t *req);
static esp_err_t handle_form_submit(httpd_req_t *req);
static esp_err_t root_get_handler(httpd_req_t *req);
static int wifiManager_ButtonEventCallback(uint32_t event_code, const void *data, size_t data_size);
static void wifiManager_Task(void* arg);
static void http_string_decode(char* str, size_t len);
static bool wifiManager_LoadCredentialsFromNVS(WifiManagerContext_t *ctx);

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data);

static const httpd_uri_t http_get_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};

static const httpd_uri_t http_form_post = {
    .uri = "/submit",
    .method = HTTP_POST,
    .handler = handle_form_submit
};

static const httpd_uri_t http_android_generate_204 = {
    .uri = "/generate_204",
    .method = HTTP_GET,
    .handler = captive_portal_probe_handler
};

static const httpd_uri_t http_android_gen_204 = {
    .uri = "/gen_204",
    .method = HTTP_GET,
    .handler = captive_portal_probe_handler
};

static const httpd_uri_t http_apple_hotspot_detect = {
    .uri = "/hotspot-detect.html",
    .method = HTTP_GET,
    .handler = captive_portal_probe_handler
};

static const httpd_uri_t http_windows_connecttest = {
    .uri = "/connecttest.txt",
    .method = HTTP_GET,
    .handler = captive_portal_probe_handler
};

static const httpd_uri_t http_windows_ncsi = {
    .uri = "/ncsi.txt",
    .method = HTTP_GET,
    .handler = captive_portal_probe_handler
};

static const httpd_uri_t http_windows_fwlink = {
    .uri = "/fwlink",
    .method = HTTP_GET,
    .handler = captive_portal_probe_handler
};

static ezStateMachine_t wifi_manager_sm;
INIT_STATE(StateInit, NULL);
INIT_STATE(StateAP, NULL);
INIT_STATE(StateSTA, NULL);
INIT_STATE(StateError, NULL);


/**
 * @brief Initialize the WiFi manager.
 */
bool wifi_manager_Init(void) {
    EZTRACE("Initializing WiFi Manager...");

    ezEventBus_CreateListener(&button_event_observer, wifiManager_ButtonEventCallback);
    appEventBus_Subscribe(&button_event_observer);

    //Initialize NVS partition for WiFi manager
    ESP_ERROR_CHECK(nvs_flash_init());

    xTaskCreate(wifiManager_Task,
        "wifi_manager_task",
        TASK_SIZE, NULL,
        TASK_PRIORITY, NULL);
    return true;
}

static void dhcp_set_captiveportal_url(void)
{
    // get the IP of the access point to redirect to
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    EZDEBUG("Set up softAP with IP: %s", ip_addr);

    // turn the IP into a URI
    char* captiveportal_uri = (char*) malloc(32 * sizeof(char));
    assert(captiveportal_uri && "Failed to allocate captiveportal_uri");
    strcpy(captiveportal_uri, "http://");
    strcat(captiveportal_uri, ip_addr);

    // get a handle to configure DHCP with
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    // set the DHCP option 114
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(netif));
    ESP_ERROR_CHECK(esp_netif_dhcps_option(netif, ESP_NETIF_OP_SET, ESP_NETIF_CAPTIVEPORTAL_URI, captiveportal_uri, strlen(captiveportal_uri)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(netif));

    free(captiveportal_uri);
}


static esp_err_t wifiManager_InitNetStack(void)
{
    esp_err_t err = esp_netif_init();
    if((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE))
    {
        EZERROR("esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE))
    {
        EZERROR("esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}


static esp_err_t wifiManager_ConfigureApDns(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ip_info;
    esp_netif_dns_info_t dns_info;

    if(netif == NULL)
    {
        EZERROR("Failed to get AP netif for DNS setup");
        return ESP_FAIL;
    }

    if(esp_netif_get_ip_info(netif, &ip_info) != ESP_OK)
    {
        EZERROR("Failed to read AP IP info for DNS setup");
        return ESP_FAIL;
    }

    memset(&dns_info, 0, sizeof(dns_info));
    dns_info.ip.type = IPADDR_TYPE_V4;
    dns_info.ip.u_addr.ip4.addr = ip_info.ip.addr;

    if(esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info) != ESP_OK)
    {
        EZERROR("Failed to set AP DNS info");
        return ESP_FAIL;
    }

    return ESP_OK;
}


static esp_err_t wifiManager_StartMdns(void)
{
    esp_err_t err;

    if(mdns_started)
    {
        return ESP_OK;
    }

    err = mdns_init();
    if(err != ESP_OK)
    {
        EZERROR("mDNS init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = mdns_hostname_set(DEVICE_HOSTNAME);
    if(err != ESP_OK)
    {
        EZERROR("mDNS hostname set failed: %s", esp_err_to_name(err));
        mdns_free();
        return err;
    }

    err = mdns_instance_name_set("Guitar Stand");
    if(err != ESP_OK)
    {
        EZERROR("mDNS instance name set failed: %s", esp_err_to_name(err));
        mdns_free();
        return err;
    }

    mdns_started = true;
    EZINFO("mDNS started: %s.local", DEVICE_HOSTNAME);
    return ESP_OK;
}


static void wifiManager_StopMdns(void)
{
    if(mdns_started)
    {
        mdns_free();
        mdns_started = false;
        EZDEBUG("mDNS stopped");
    }
}


static void dns_redirect_task(void *arg)
{
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    uint8_t rx_buf[DNS_PACKET_MAX_SIZE];
    uint8_t tx_buf[DNS_PACKET_MAX_SIZE];
    uint32_t ap_ip;

    (void)arg;

    if(netif == NULL)
    {
        EZERROR("DNS redirect: AP netif is NULL");
        dns_redirect_task_h = NULL;
        vTaskDelete(NULL);
        return;
    }

    if(esp_netif_get_ip_info(netif, &ip_info) != ESP_OK)
    {
        EZERROR("DNS redirect: failed to get AP IP info");
        dns_redirect_task_h = NULL;
        vTaskDelete(NULL);
        return;
    }

    ap_ip = ip_info.ip.addr;

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(DNS_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    dns_redirect_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(dns_redirect_sock < 0)
    {
        EZERROR("DNS redirect: failed to create socket");
        dns_redirect_sock = -1;
        dns_redirect_task_h = NULL;
        vTaskDelete(NULL);
        return;
    }

    if(bind(dns_redirect_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        EZERROR("DNS redirect: failed to bind UDP socket");
        close(dns_redirect_sock);
        dns_redirect_sock = -1;
        dns_redirect_task_h = NULL;
        vTaskDelete(NULL);
        return;
    }

    EZDEBUG("DNS redirect server started on port %d", DNS_PORT);

    while(1)
    {
        int rx_len = recvfrom(dns_redirect_sock,
            rx_buf,
            sizeof(rx_buf),
            0,
            (struct sockaddr *)&client_addr,
            &client_addr_len);

        if(rx_len < 12)
        {
            if(dns_redirect_sock < 0)
            {
                break;
            }
            continue;
        }

        memcpy(tx_buf, rx_buf, rx_len);

        tx_buf[2] = 0x81;
        tx_buf[3] = 0x80;
        tx_buf[6] = 0x00;
        tx_buf[7] = 0x01;
        tx_buf[8] = 0x00;
        tx_buf[9] = 0x00;
        tx_buf[10] = 0x00;
        tx_buf[11] = 0x00;

        if((rx_len + 16) > (int)sizeof(tx_buf))
        {
            continue;
        }

        tx_buf[rx_len + 0] = 0xC0;
        tx_buf[rx_len + 1] = 0x0C;
        tx_buf[rx_len + 2] = 0x00;
        tx_buf[rx_len + 3] = 0x01;
        tx_buf[rx_len + 4] = 0x00;
        tx_buf[rx_len + 5] = 0x01;
        tx_buf[rx_len + 6] = 0x00;
        tx_buf[rx_len + 7] = 0x00;
        tx_buf[rx_len + 8] = 0x00;
        tx_buf[rx_len + 9] = 0x3C;
        tx_buf[rx_len + 10] = 0x00;
        tx_buf[rx_len + 11] = 0x04;
        memcpy(&tx_buf[rx_len + 12], &ap_ip, sizeof(ap_ip));

        if(sendto(dns_redirect_sock,
            tx_buf,
            rx_len + 16,
            0,
            (struct sockaddr *)&client_addr,
            client_addr_len) < 0)
        {
            EZERROR("DNS redirect: failed to send response");
        }
    }

    if(dns_redirect_sock >= 0)
    {
        close(dns_redirect_sock);
        dns_redirect_sock = -1;
    }

    dns_redirect_task_h = NULL;
    EZDEBUG("DNS redirect server stopped");
    vTaskDelete(NULL);
}


static void dns_redirect_start(void)
{
    if(dns_redirect_task_h != NULL)
    {
        EZDEBUG("DNS redirect server already running");
        return;
    }

    if(xTaskCreate(dns_redirect_task,
        "dns_redirect",
        TASK_SIZE,
        NULL,
        TASK_PRIORITY,
        &dns_redirect_task_h) != pdPASS)
    {
        dns_redirect_task_h = NULL;
        EZERROR("Failed to start DNS redirect task");
    }
}


static void dns_redirect_stop(void)
{
    if(dns_redirect_sock >= 0)
    {
        close(dns_redirect_sock);
        dns_redirect_sock = -1;
    }
}


static httpd_handle_t start_webserver(httpd_handle_t* handle)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;

    // Start the httpd server
    EZDEBUG("Starting server on port: '%d'", config.server_port);
    if (httpd_start(handle, &config) == ESP_OK) {
        // Set URI handlers
        EZDEBUG("Registering URI handlers");
        httpd_register_uri_handler(*handle, &http_form_post);
        httpd_register_uri_handler(*handle, &http_get_root);
        httpd_register_uri_handler(*handle, &http_android_generate_204);
        httpd_register_uri_handler(*handle, &http_android_gen_204);
        httpd_register_uri_handler(*handle, &http_apple_hotspot_detect);
        httpd_register_uri_handler(*handle, &http_windows_connecttest);
        httpd_register_uri_handler(*handle, &http_windows_ncsi);
        httpd_register_uri_handler(*handle, &http_windows_fwlink);
        httpd_register_err_handler(*handle, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    return *handle;
}


static esp_err_t root_get_handler(httpd_req_t *req)
{
    const uint32_t root_len = captive_portal_end - captive_portal_start;

    EZDEBUG("Serve root");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, captive_portal_start, root_len);

    return ESP_OK;
}


static esp_err_t captive_portal_probe_handler(httpd_req_t *req)
{
    EZDEBUG("Captive portal probe hit: %s", req->uri);
    httpd_resp_set_status(req, "302 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Redirect to captive portal", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


static esp_err_t handle_form_submit(httpd_req_t *req)
{
    char buf[100];
    memset(buf, 0, sizeof(buf));
    int recv_len = httpd_req_recv(req, buf, sizeof(buf));
    if (recv_len <= 0) return ESP_FAIL;

    EZDEBUG("Received form data: %s", buf);
    // Extract SSID & Password
    char ssid[32];
    char password[64];
    memset(ssid, 0, sizeof(ssid));
    memset(password, 0, sizeof(password));

    sscanf(buf, "ssid=%31[^&]&password=%63s", ssid, password);
    http_string_decode(password, strlen(password) + 1);
    http_string_decode(ssid, strlen(ssid) + 1);
    EZDEBUG("Received SSID: %s, Password: %s", ssid, password);

    // Store in NVS
    ESP_ERROR_CHECK(nvs_open("wifi_creds", NVS_READWRITE, &context.nvs));
    ESP_ERROR_CHECK(nvs_set_str(context.nvs, "ssid", ssid));
    ESP_ERROR_CHECK(nvs_set_str(context.nvs, "password", password));
    ESP_ERROR_CHECK(nvs_commit(context.nvs));
    nvs_close(context.nvs);

    httpd_resp_send(req, "Wi-Fi credentials saved! Connecting...", HTTPD_RESP_USE_STRLEN);
    ezSM_SetEvent(&wifi_manager_sm, SM_EVENT_WIFI_CREDENTIALS_RECEIVED);
    return ESP_OK;
}


static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    EZDEBUG("Redirecting to root");
    return ESP_OK;
}


static void wifi_event_handler(void *arg, esp_event_base_t event_base,
    int32_t event_id, void *event_data)
{
    if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_AP_STACONNECTED)) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        EZDEBUG("station " MACSTR " join, AID=%d",
            MAC2STR(event->mac), event->aid);
    }
    else if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_AP_STADISCONNECTED))
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        EZDEBUG("station " MACSTR " leave, AID=%d, reason=%d",
            MAC2STR(event->mac), event->aid, event->reason);
    }
    else if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_CONNECTED))
    {
        ezSM_SetEvent(&wifi_manager_sm, SM_EVENT_WIFI_CONNECTED);
    }
    else if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_DISCONNECTED))
    {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        EZERROR("STA disconnected, reason=%d", event->reason);
        wifiManager_StopMdns();
        ezSM_SetEvent(&wifi_manager_sm, SM_EVENT_WIFI_DISCONNECTED);
    }
    else if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_START))
    {
        ezSM_SetEvent(&wifi_manager_sm, SM_EVENT_WIFI_START);
    }
    else if((event_base == IP_EVENT) && (event_id == IP_EVENT_STA_GOT_IP))
    {
        (void)event_data;
        EZINFO("STA got IP address");
        if(wifiManager_StartMdns() != ESP_OK)
        {
            EZERROR("Failed to start mDNS responder");
        }
        else
        {
            EZINFO("Hostname reachable as: %s.local", DEVICE_HOSTNAME);
        }
    }
    else
    {
        /* Unhandled event. Do nothing */
    }
}

static int wifiManager_ButtonEventCallback(uint32_t event_code, const void *data, size_t data_size)
{
    if(event_code == APP_EVENT_GPIO_PRESS)
    {
        //EZDEBUG("Button pressed");
    }
    else if(event_code == APP_EVENT_GPIO_LONG_PRESS)
    {
        EZDEBUG("Button long pressed");
        ezSM_SetEvent(&wifi_manager_sm, SM_EVENT_START_CAPTIVE_PORTAL);
    }
    else
    {
        //EZDEBUG("Unknown event type");
    }

    return 0;
}


static void wifiManager_Task(void* arg)
{
    EZTRACE("Starting WiFi Manager task...");

    // Initialize the state machine
    if(ezSM_Init(&wifi_manager_sm, &StateInit, sm_event_buff, sizeof(sm_event_buff), &context) == false)
    {
        EZERROR("Failed to initialize state machine");
        return;
    }

    // Run the state machine
    while (true) {
        ezSM_Run(&wifi_manager_sm);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static bool wifiManager_LoadCredentialsFromNVS(WifiManagerContext_t *ctx)
{
    EZTRACE("wifiManager_LoadCredentialsFromNVS()");
    if(ctx == NULL)
    {
        EZERROR("Context is NULL");
        return false;
    }

    uint8_t ssid[32];
    uint8_t password[64];
    size_t ssid_len = sizeof(ssid);
    size_t password_len = sizeof(password);

    memset(ssid, 0, sizeof(ssid));
    memset(password, 0, sizeof(password));

    if(nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &ctx->nvs) != ESP_OK)
    {
        EZERROR("Failed to open NVS namespace");
        return false;
    }

    if(nvs_get_str(ctx->nvs, "ssid", (char*)ssid, &ssid_len) != ESP_OK)
    {
        EZERROR("Failed to get SSID from nvs");
        nvs_close(ctx->nvs);
        return false;
    }

    if(nvs_get_str(ctx->nvs, "password", (char*)password, &password_len) != ESP_OK)
    {
        EZERROR("Failed to get password from nvs");
        nvs_close(ctx->nvs);
        return false;
    }

    nvs_close(ctx->nvs);
    if(ssid_len == 0 || password_len == 0)
    {
        EZERROR("SSID or password is empty");
        return false;
    }

    memcpy(ctx->wifi_sta_config.sta.ssid, ssid, 32);
    memcpy(ctx->wifi_sta_config.sta.password, password, 64);

    EZDEBUG("ssid: %s", ctx->wifi_sta_config.sta.ssid);
    EZDEBUG("ssid_len: %d", ssid_len);
    EZDEBUG("password: %s", ctx->wifi_sta_config.sta.password);
    EZDEBUG("password_len: %d", password_len);
    return true;
}

static void http_string_decode(char* str, size_t len)
{
    char *read_ptr = str;
    char *write_ptr = str;
    for(size_t i = 0; i < len; i++)
    {
        if (read_ptr[i] == '%')
        {
            if(i + 2 < len)
            {
                char hex[3] = {read_ptr[i+1], read_ptr[i+2], '\0'};
                *write_ptr++ = (char)strtol(hex, NULL, 16);
                i += 2;
            }
        }
        else if(read_ptr[i] == '+')
        {
            *write_ptr++ = ' ';
        }
        else
        {
            *write_ptr++ = read_ptr[i];
        }
    }
}

DEFINE_ENTRY_FUNCTION(StateInit)
{
    WifiManagerContext_t *ctx = (WifiManagerContext_t*)sm->data;
    ctx->reconnect_count = 0;
    ctx->count_100ms = 0;
    ctx->http_server_h = NULL;
    ctx->netif = NULL;

    strcpy((char*)ctx->wifi_ap_config.ap.ssid, EXAMPLE_ESP_WIFI_SSID);
    strcpy((char*)ctx->wifi_ap_config.ap.password, EXAMPLE_ESP_WIFI_PASS);

    /* Access point configuration */
    ctx->wifi_ap_config.ap.ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID);
    ctx->wifi_ap_config.ap.channel = 1;
    ctx->wifi_ap_config.ap.max_connection = EXAMPLE_MAX_STA_CONN;
    ctx->wifi_ap_config.ap.ssid_hidden = 0;
    ctx->wifi_ap_config.ap.beacon_interval = 100;
    ctx->wifi_ap_config.ap.pmf_cfg.required = false;
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        ctx->wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    else {
        ctx->wifi_ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    return NULL;
}

DEFINE_ACTION_FUNCTION(StateInit)
{
    EZDEBUG("Enter action function of StateInit");
    WifiManagerContext_t *ctx = (WifiManagerContext_t*)sm->data;

    if(wifiManager_LoadCredentialsFromNVS(ctx) == false)
    {
        EZERROR("Failed to load credentials from NVS");
        return &StateAP;
    }
    else
    {
        EZDEBUG("Loaded credentials from NVS");
        EZDEBUG("SSID: %s", ctx->wifi_sta_config.sta.ssid);
        EZDEBUG("Password: %s", ctx->wifi_sta_config.sta.password);
        return &StateSTA;
    }

    return NULL;
}

DEFINE_EXIT_FUNCTION(StateInit)
{
    /* Unused */
    return NULL;
}

DEFINE_EVENT_HANDLER_FUNCTION(StateInit)
{
    /* Unused */
    return NULL;
}


DEFINE_ENTRY_FUNCTION(StateAP)
{
    EZTRACE("Starting captive portal...");
    WifiManagerContext_t *ctx = (WifiManagerContext_t*)sm->data;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(wifiManager_InitNetStack());
    ctx->netif = esp_netif_create_default_wifi_ap();
    if(ctx->netif == NULL)
    {
        EZERROR("Failed to create default wifi AP");
        return &StateError;
    }
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ctx->wifi_ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    EZDEBUG("Set up softAP with IP: %s", ip_addr);

    EZDEBUG("wifi_init_softap finished. SSID:'%s' password:'%s'",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    ESP_ERROR_CHECK(wifiManager_ConfigureApDns());
    dhcp_set_captiveportal_url();
    dns_redirect_start();

    ctx->http_server_h = start_webserver(&ctx->http_server_h);
    return NULL;
}

DEFINE_ACTION_FUNCTION(StateAP)
{
    /* Unused */
    return NULL;
}

DEFINE_EXIT_FUNCTION(StateAP)
{
    WifiManagerContext_t *ctx = (WifiManagerContext_t*)sm->data;

    dns_redirect_stop();

    /* Stop the server */
    if(ctx->http_server_h != NULL)
    {
        httpd_stop(ctx->http_server_h);
        ctx->http_server_h = NULL;
    }

    /* stop dhcp server */
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(netif));

    ESP_ERROR_CHECK(esp_wifi_stop());
    esp_wifi_deinit();
    if(ctx->netif != NULL)
    {
        esp_netif_destroy_default_wifi(ctx->netif);
        ctx->netif = NULL;
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_delete_default());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_deinit());
    return NULL;
}

DEFINE_EVENT_HANDLER_FUNCTION(StateAP)
{
    if(event == SM_EVENT_START_CAPTIVE_PORTAL)
    {
        EZDEBUG("Captive portal is already started.");
    }
    else if (event == SM_EVENT_WIFI_CREDENTIALS_RECEIVED)
    {
        EZDEBUG("WiFi credentials received. Starting connection...");
        return &StateSTA;
    }
    else
    {
        /* Unhndled event */
    }

    return NULL;
}

DEFINE_ENTRY_FUNCTION(StateSTA)
{
    WifiManagerContext_t *ctx = (WifiManagerContext_t*)sm->data;
    EZTRACE("Connecting to WiFi...");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if(wifiManager_LoadCredentialsFromNVS(ctx) == false)
    {
        EZERROR("Failed to load credentials from NVS");
        return &StateError;
    }

    EZDEBUG("Loaded credentials from NVS");
    EZDEBUG("SSID: %s", ctx->wifi_sta_config.sta.ssid);
    EZDEBUG("Password: %s", ctx->wifi_sta_config.sta.password);

    ESP_ERROR_CHECK(wifiManager_InitNetStack());
    ctx->netif = esp_netif_create_default_wifi_sta();
    if(ctx->netif == NULL)
    {
        EZERROR("Failed to create default wifi STA");
        return &StateError;
    }
    if(esp_netif_set_hostname(ctx->netif, DEVICE_HOSTNAME) != ESP_OK)
    {
        EZERROR("Failed to set STA hostname");
        return &StateError;
    }
    if(esp_wifi_init(&cfg) != ESP_OK)
    {
        EZERROR("Failed to initialize WiFi");
        return &StateError;
    }

    if(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL) != ESP_OK)
    {
        EZERROR("Failed to register handle");
        return &StateError;
    }
    if(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL) != ESP_OK)
    {
        EZERROR("Failed to register IP event handler");
        return &StateError;
    }

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // default is WIFI_PS_MIN_MODEM
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // default is WIFI_STORAGE_FLASH
    if(esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK)
    {
        EZERROR("Failed to set wifi mode");
        return &StateError;
    }

    if(esp_wifi_set_config(WIFI_IF_STA, &ctx->wifi_sta_config) != ESP_OK)
    {
        EZERROR("Failed to config wifi");
        return &StateError;
    }

    if(esp_wifi_start() != ESP_OK)
    {
        EZERROR("Failed to start STA");
        return &StateError;
    }

    return NULL;
}

DEFINE_ACTION_FUNCTION(StateSTA)
{
    return NULL;
}

DEFINE_EXIT_FUNCTION(StateSTA)
{
    WifiManagerContext_t *ctx = (WifiManagerContext_t*)sm->data;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
    wifiManager_StopMdns();
    esp_wifi_stop();
    esp_wifi_deinit();
    if(ctx->netif != NULL)
    {
        esp_netif_destroy_default_wifi(ctx->netif);
        ctx->netif = NULL;
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_delete_default());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_deinit());
    return NULL;
}

DEFINE_EVENT_HANDLER_FUNCTION(StateSTA)
{
    
    if(event == SM_EVENT_START_CAPTIVE_PORTAL)
    {
        esp_wifi_disconnect();
        return &StateAP;
    }
    else if(event == SM_EVENT_WIFI_DISCONNECTED)
    {
        if(context.reconnect_count < MAX_CONNECT_RETRY)
        {
            EZDEBUG("WiFi disconnected, retrying connection...");
            context.reconnect_count++;
            esp_wifi_connect();
        }
        else
        {
            EZERROR("Failed to connect to WiFi after %d attempts", MAX_CONNECT_RETRY);
            context.error_code = SM_ERR_DISCONNECTED;
            appEventBus_Notify(APP_EVENT_WIFI_DISCONNECTED, (void*)&context.http_server_h, sizeof(httpd_handle_t));
            return &StateError;
        }
    }
    else if(event == SM_EVENT_WIFI_CONNECTED)
    {
        EZDEBUG("Connected to WiFi");
        if(context.reconnect_count > 0)
        {
            context.reconnect_count = 0;
        }
        appEventBus_Notify(APP_EVENT_WIFI_CONNECTED, (void*)&context.http_server_h, sizeof(httpd_handle_t));
    }
    else if(event == SM_EVENT_WIFI_START)
    {
        EZDEBUG("Wifi started");
        if(esp_wifi_connect() != ESP_OK)
        {
            return &StateError;
        }
    }
    else
    {
        /* Unhandled event, do nothing */
    }
    return NULL;
}

DEFINE_ENTRY_FUNCTION(StateError)
{
    return NULL;
}

DEFINE_ACTION_FUNCTION(StateError)
{
    WifiManagerContext_t *ctx = (WifiManagerContext_t*)sm->data;
    if(ctx->error_code == SM_ERR_DISCONNECTED)
    {
        EZERROR("Disconnected from WiFi");
        esp_wifi_stop();
        esp_wifi_deinit();
        esp_netif_destroy_default_wifi(ctx->netif);
        esp_event_loop_delete_default();
        esp_netif_deinit();
        return &StateAP;
    }
    else if(ctx->error_code == SM_ERR_LOAD_CREDENTIALS)
    {
        EZERROR("Failed to load credentials from NVS");
        return &StateAP;
    }
    else
    {
        /* Unhndled event */
    }
    return NULL;
}

DEFINE_EXIT_FUNCTION(StateError)
{
    return NULL;
}

DEFINE_EVENT_HANDLER_FUNCTION(StateError)
{
    return NULL;
}
