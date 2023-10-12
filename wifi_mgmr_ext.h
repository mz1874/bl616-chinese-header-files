#ifndef __WIFI_MGMR_EXT_H_
#define __WIFI_MGMR_EXT_H_

#include "stdint.h"
#include "stdbool.h"

#define MAX_FIXED_CHANNELS_LIMIT (14) // 最大固定信道数量
#define MAX_AP_SCAN 50 // 最大AP扫描数量
#define MGMR_SSID_LEN 32 // SSID长度
#define MGMR_KEY_LEN 64 // 密钥长度

/* WiFi异步事件 */
#define  EV_WIFI                  0x0002
#define  CODE_WIFI_ON_INIT_DONE   1 // WiFi初始化完成事件
#define  CODE_WIFI_ON_MGMR_DONE   2 // WiFi管理器初始化完成事件
#define  CODE_WIFI_CMD_RECONNECT  3 // WiFi重新连接命令
#define  CODE_WIFI_ON_CONNECTED   4 // WiFi已连接事件
#define  CODE_WIFI_ON_DISCONNECT  5 // WiFi断开连接事件
#define  CODE_WIFI_ON_PRE_GOT_IP  6 // WiFi获取IP前事件
#define  CODE_WIFI_ON_GOT_IP      7 // WiFi获取IP事件
#define  CODE_WIFI_ON_CONNECTING  8 // WiFi正在连接事件
#define  CODE_WIFI_ON_SCAN_DONE   9 // WiFi扫描完成事件
#define  CODE_WIFI_ON_SCAN_DONE_ONJOIN  10 // WiFi扫描完成后连接事件
#define  CODE_WIFI_ON_AP_STARTED        11 // WiFi AP启动事件
#define  CODE_WIFI_ON_AP_STOPPED        12 // WiFi AP停止事件
#define  CODE_WIFI_ON_PROV_SSID         13 // WiFi配置SSID事件
#define  CODE_WIFI_ON_PROV_BSSID        14 // WiFi配置BSSID事件
#define  CODE_WIFI_ON_PROV_PASSWD       15 // WiFi配置密码事件
#define  CODE_WIFI_ON_PROV_CONNECT      16 // WiFi配置连接事件
#define  CODE_WIFI_ON_PROV_DISCONNECT   17 // WiFi配置断开连接事件
#define  CODE_WIFI_ON_PROV_SCAN_START   18 // WiFi配置扫描开始事件
#define  CODE_WIFI_ON_PROV_STATE_GET    19 // WiFi配置状态获取事件
#define  CODE_WIFI_ON_MGMR_DENOISE      20 // WiFi管理器去噪事件
#define  CODE_WIFI_ON_AP_STA_ADD        21 // WiFi AP添加STA事件
#define  CODE_WIFI_ON_AP_STA_DEL        22 // WiFi AP删除STA事件
#define  CODE_WIFI_ON_EMERGENCY_MAC     23 // WiFi紧急模式MAC事件
#define  CODE_WIFI_ON_EXIT_PS           24 // 退出电源保存模式事件
#define  CODE_WIFI_ON_GOT_IP6           25 // WiFi获取IPv6事件

#define WIFI_EVENT_BEACON_IND_AUTH_OPEN            0 // 信标认证方式：开放
#define WIFI_EVENT_BEACON_IND_AUTH_WEP             1 // 信标认证方式：WEP
#define WIFI_EVENT_BEACON_IND_AUTH_WPA_PSK         2 // 信标认证方式：WPA-PSK
#define WIFI_EVENT_BEACON_IND_AUTH_WPA2_PSK        3 // 信标认证方式：WPA2-PSK
#define WIFI_EVENT_BEACON_IND_AUTH_WPA_WPA2_PSK    4 // 信标认证方式：WPA/WPA2-PSK
#define WIFI_EVENT_BEACON_IND_AUTH_WPA_ENT         5 // 信标认证方式：WPA-ENT
#define WIFI_EVENT_BEACON_IND_AUTH_WPA3_SAE        6 // 信标认证方式：WPA3-SAE
#define WIFI_EVENT_BEACON_IND_AUTH_WPA2_PSK_WPA3_SAE 7 // 信标认证方式：WPA2-PSK/WPA3-SAE
#define WIFI_EVENT_BEACON_IND_AUTH_UNKNOWN      0xff // 信标认证方式：未知

#define WIFI_EVENT_BEACON_IND_CIPHER_NONE           0 // 信标加密方式：无
#define WIFI_EVENT_BEACON_IND_CIPHER_WEP            1 // 信标加密方式：WEP
#define WIFI_EVENT_BEACON_IND_CIPHER_AES            2 // 信标加密方式：AES
#define WIFI_EVENT_BEACON_IND_CIPHER_TKIP           3 // 信标加密方式：TKIP
#define WIFI_EVENT_BEACON_IND_CIPHER_TKIP_AES       4 // 信标加密方式：TKIP/AES


/// 网络接口类型
typedef enum
{
    /// STA接口（无线客户端接口）
    MGMR_VIF_STA = 0,
    /// AP接口（无线热点接口）
    MGMR_VIF_AP
} wifi_mgmr_vif_type;


typedef struct wifi_mgmr_scan_item {
    uint32_t mode;               // 模式
    uint32_t timestamp_lastseen; // 上次发现的时间戳
    int ssid_len;                // SSID长度
    uint8_t channel;             // 信道
    int8_t rssi;                 // 信号强度
    char ssid[32];               // SSID
    char ssid_tail[1];           // 总是放在SSID后面
    uint8_t bssid[6];            // BSSID（MAC地址）
    int8_t ppm_abs;              // 绝对PPM
    int8_t ppm_rel;              // 相对PPM
    uint8_t auth;                // 认证方式
    uint8_t cipher;              // 加密方式
    uint8_t is_used;             // 是否已使用
    uint8_t wps;                 // WPS支持
} wifi_mgmr_scan_item_t;

typedef struct wifi_mgmr_sniffer_item {
    /// 接口索引
    char *itf;
    /// 信道类型（@ref mac_chan_bandwidth）
    uint8_t type;
    /// 主要20MHz信道的频率（单位：MHz）
    uint16_t prim20_freq;
    /// 连续信道或主要80+80的中心频率（单位：MHz）
    uint16_t center1_freq;
    /// 不连续的80+80的中心频率（单位：MHz）
    uint16_t center2_freq;
    /// 帧接收回调函数
    void *cb;
    /// 用于监视回调的参数
    void *cb_arg;
} wifi_mgmr_sniffer_item_t;

/// STA连接配置参数
/*
 * 参数:
 *  ssid     : 目标AP的SSID
 *  key      : AP的密码
 *  bssid    : AP的BSSID（MAC地址）
 *  akm_str  : AP的AKM（认证和密钥管理方法）
 *  pmf_cfg  : PMF（Protected Management Frames）配置
 *  freq1    : AP的频率
 *  freq2    : AP的频率（可以指定最多两个将要扫描的频率）
 *  use_dhcp : 是否使用AP提供的DHCP服务器
 *  listen_interval : 监视AP信标的时间间隔
 *                    范围:[1, 100]
 *  scan_mode : 扫描模式，默认为1，扫描所有信道；
 *              如果为0，快速扫描，连接到第一个匹配SSID的AP
 *  quick_connect : 连接模式，默认为正常连接；
 *                如果为1，快速连接
 */
typedef struct wifi_mgmr_sta_connect_params {
    uint8_t ssid[MGMR_SSID_LEN]; // 目标AP的SSID
    uint8_t ssid_tail[1];
    char key[MGMR_KEY_LEN]; // AP的密码
    char key_tail[1];
    uint8_t ssid_len; // SSID长度
    uint8_t key_len; // 密码长度
    uint8_t bssid[6]; // AP的BSSID（MAC地址）
    char akm_str[10]; // AP的AKM（认证和密钥管理方法）
    uint8_t akm_len; // AKM长度
    uint16_t freq1; // AP的频率
    uint16_t freq2; // AP的频率
    uint8_t pmf_cfg; // PMF配置
    uint8_t use_dhcp; // 是否使用AP提供的DHCP服务器
    uint8_t listen_interval; // 监视AP信标的时间间隔
    uint8_t scan_mode; // 扫描模式
    uint8_t quick_connect; // 连接模式
} wifi_mgmr_sta_connect_params_t;


/// 扫描参数
typedef struct wifi_mgmr_scan_params {
    uint8_t ssid_length; // SSID长度
    uint8_t ssid_array[MGMR_SSID_LEN]; // SSID数组
    uint8_t bssid[6]; // BSSID（MAC地址）
    uint8_t bssid_set_flag; // BSSID设置标志
    int channels_cnt; // 信道计数
    uint8_t channels[MAX_FIXED_CHANNELS_LIMIT]; // 信道数组
    uint32_t duration; // 扫描持续时间
} wifi_mgmr_scan_params_t;

/// 原始数据发送参数
typedef struct wifi_mgmr_raw_send_params {
    void *pkt; // 要发送的数据包的起始地址
    uint32_t len; // 要发送的数据包长度
    uint8_t channel; // 数据包将要发送的信道
} wifi_mgmr_raw_send_params_t;

/// AP启动参数
typedef struct wifi_mgmr_ap_params {
    char *ssid; // 必须设置的SSID
    char *key; // 如果为NULL且akm不为NULL，将设置默认值 "12345678"
    char *akm; // 支持OPEN/WPA/WPA2，如果为NULL且key不为NULL，将设置默认值 "WPA2"
    uint8_t channel; // 如果为零，将设置默认值6
    uint8_t type; // 信道类型
    bool use_dhcpd; // 是否使用AP提供的DHCP服务器
    uint32_t ap_max_inactivity; // AP的最大非活动时间
} wifi_mgmr_ap_params_t;


/**
 * 完全托管的帧信息
 */
struct bl_frame_info
{
    /**
     * 接收帧的接口索引。(-1表示未知)
     */
    int fvif_idx;
    /**
     * 帧的长度（以字节为单位）。
     */
    uint16_t length;
    /**
     * 接收帧的主要信道频率（以MHz为单位）。
     */
    uint16_t freq;
    /**
     * 接收信号强度（以dBm为单位）
     */
    int8_t rssi;
    /**
     * 帧有效载荷。如果以uf参数设置为true启动监控模式，则可能为NULL。在这种情况下，所有其他字段仍然有效。
     */
    uint8_t *payload;
};

typedef struct wifi_mgmr_connect_ind_stat_info {
    uint16_t status_code;
    uint16_t reason_code;
    char ssid[33];
    char passphr[65];
    /// BSSID
    uint8_t bssid[6];
    uint8_t type_ind;
    uint8_t chan_band;
    uint8_t channel;
    uint8_t security;
    /// AP为此连接分配的关联ID
    uint16_t aid;
    /// 完成关联过程的VIF的索引
    uint8_t vif_idx;
    /// 为AP分配的STA条目的索引
    uint8_t ap_idx;
    /// 连接附加的LMAC信道上下文的索引
    uint8_t ch_idx;
    /// 指示AP是否支持QoS的标志
    bool qos;
} wifi_mgmr_connect_ind_stat_info_t;

typedef struct wifi_conf {
    char country_code[3];
    int channel_nums;
} wifi_conf_t;

typedef struct wifi_sta_basic_info {
    uint8_t sta_idx;
    uint8_t is_used;
    uint8_t sta_mac[6];
    uint16_t aid;
} wifi_sta_basic_info_t;

typedef struct rf_pwr_table {
    int8_t     pwr_11b[4];
    int8_t     pwr_11g[8];
    int8_t     pwr_11n_ht20[8];
    int8_t     pwr_11n_ht40[8];
    int8_t     pwr_11ac_vht20[10];
    int8_t     pwr_11ac_vht40[10];
    int8_t     reserved[10];
    int8_t     pwr_11ax_he20[12];
    int8_t     pwr_11ax_he40[12];
    int8_t     reserved2[12];
    int8_t     reserved3[12];
} rf_pwr_table_t;

typedef struct
{
    uint8_t ucSSID[32];         /**< Wi-Fi网络的SSID。 */
    uint8_t ucSSIDLength;       /**< SSID长度。 */
    uint8_t ucBSSID[6];         /**< Wi-Fi网络的BSSID。 */
    uint8_t ucSecurity;         /**< Wi-Fi安全性。 */
    uint8_t ucPWD[65];          /**< WPA/WPA2密码。 */
    uint8_t ucChannel;          /**< 信道号。 */
} wifi_mgmr_ap_info_t;

typedef void (*scan_item_cb_t)(void *env, void *arg, wifi_mgmr_scan_item_t *item);

/**
 * wifi_sta_connect
 * 连接到AP
 * 参数:
 *  ssid     : 目标AP的SSID
 *  key      : AP的密码
 *  bssid    : AP的BSSID
 *  akm_str  : AP的AKM
 *  pmf_cfg  : PMF配置
 *  freq1    : AP的频率
 *  freq2    : AP的频率（可以指定最多两个扫描AP的频率）
 *  use_dhcp : 是否使用AP提供的DHCP服务器
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_sta_connect(char *ssid, char *key, char *bssid, char *akm_str, uint8_t pmf_cfg, uint16_t freq1, uint16_t freq2, uint8_t use_dhcp);

/**
 * wifi_mgmr_sta_quickconnect
 * 连接到AP
 * 参数:
 *  ssid     : 目标AP的SSID
 *  key      : AP的密码
 *  freq1    : AP的频率
 *  freq2    : AP的频率（最多可以指定两个扫描AP的频率）
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_quickconnect(char *ssid, char *key, uint16_t freq1, uint16_t freq2);

/**
 * wifi_mgmr_sta_connect
 * 连接到AP
 * 参数:
 *  param1 : 连接的配置
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_connect(const wifi_mgmr_sta_connect_params_t *config);

/**
 * wifi_sta_disconnect
 * 从AP断开连接
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_sta_disconnect(void);

/**
 * 获取STA接口的IPv4地址
 * 返回 0 表示成功，-1 表示错误
 */
int wifi_sta_ip4_addr_get(uint32_t *addr, uint32_t *mask, uint32_t *gw, uint32_t *dns);

#if NX_FHOST_MONITOR
/**
 * wifi_mgmr_sniffer_enable
 * 启动嗅探器模式
 * 参数:
 *  param1 : 嗅探器模式的配置
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sniffer_enable(wifi_mgmr_sniffer_item_t sniffer_item);

/**
 * wifi_mgmr_sniffer_disable
 * 停止嗅探器模式
 * 参数:
 *  param1 : 嗅探器模式的句柄（来自wifi_mgmr_sniffer_enable()）
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sniffer_disable(wifi_mgmr_sniffer_item_t sniffer_item);
#endif



/**
 * wifi_mgmr_state_get
 * 获取AP/STA状态
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_state_get(void);

/**
 * wifi_mgmr_sta_rssi_get
 * 获取最后一个信标的RSSI
 * 参数:
 *  param1 : rssi的指针
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_rssi_get(int *rssi);

/**
 * wifi_mgmr_sta_channel_get
 * 获取STA模式的信道
 * 参数:
 *  param1 : 信道的指针
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_channel_get(int *channel);

/**
 * wifi_mgmr_sta_ssid_set
 * 设置STA模式的SSID
 * 参数:
 *  param1 : SSID的指针
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_ssid_set(char *ssid);

/**
 * wifi_mgmr_sta_passphr_set
 * 设置STA模式的密码
 * 参数:
 *  param1 : 密码的指针
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_passphr_set(char *passphr);

/**
 * wifi_mgmr_sta_connect_ind_stat_get
 * 获取STA模式的状态（SSID密码）
 * 参数:
 *  param1 : 指向wifi_mgmr_connect_ind_stat_info_t结构的指针
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_connect_ind_stat_get(wifi_mgmr_connect_ind_stat_info_t *wifi_mgmr_ind_stat);

/**
 * wifi_mgmr_sta_scan
 * 扫描可用的AP
 * 参数:
 *  param1 : 扫描的配置
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_scan(const wifi_mgmr_scan_params_t *config);

/**
 * wifi_mgmr_sta_scanlist
 * 列出最后一次扫描的扫描结果
 * 返回:
 *  0 : 成功
 *  其他是失败
 */
int wifi_mgmr_sta_scanlist(void);

/**
 * wifi_mgmr_sta_scanlist_nums_get
 * 获取扫描结果的数量
 * 返回:
 *  扫描结果的数量
 */
uint32_t wifi_mgmr_sta_scanlist_nums_get(void);

/**
 * wifi_mgmr_sta_scanlist_dump
 * 获取扫描结果
 * 参数:
 *  param1 : 用于存储扫描结果的地址
 *  param2 : 用于存储扫描结果的最大项数
 * 返回:
 *  扫描结果的数量
 */
uint32_t wifi_mgmr_sta_scanlist_dump(void *results, uint32_t resultNums);

/**
 * @brief 获取扫描结果
 *
 * @param env 用于回调的环境
 * @param arg 用于回调的参数
 * @param cb 用于接收扫描结果的回调函数
 *
 * @return 0
 */
int wifi_mgmr_scan_ap_all(void *env, void *arg, scan_item_cb_t cb);

/**
 * wifi_mgmr_ap_start
 * 启动AP模式
 * 参数:
 *  param1 : AP模式的配置
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_ap_start(const wifi_mgmr_ap_params_t *config);

/**
 * wifi_mgmr_ap_stop
 * 停止AP模式
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_ap_stop(void);

/**
 * wifi_mgmr_mode_to_str
 * 获取eht模式的字符串表示
 * 参数:
 *  param1 : 物理模式
 * 返回:
 *  "未知" : 无法转换为字符串的模式
 *  其他是模式的字符串表示
 */
char *wifi_mgmr_mode_to_str(uint32_t mode);

/**
 * show_auth_cipher
 * 打印扫描结果的认证和加密方式
 * 参数:
 *  param1 : 扫描结果的实例
 */
// void show_auth_cipher(struct mac_scan_result *result);

/**
 * wifi_mgmr_mac_set
 * 设置MAC地址
 * 参数:
 *  param1 : MAC地址数组
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_mac_set(uint8_t mac[6]);

/**
 * wifi_mgmr_sta_mac_get
 * 获取STA的MAC地址
 * 参数:
 *  param1 : MAC地址数组
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_mac_get(uint8_t mac[6]);

/**
 * wifi_mgmr_ap_mac_get
 * 获取AP的MAC地址
 * 参数:
 *  param1 : MAC地址数组
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_ap_mac_get(uint8_t mac[6]);

/**
 * wifi_mgmr_set_country_code
 * 设置国家代码
 * 参数:
 *  param1 : 国家代码的指针，例如"CN"，"US"，"JP"，"EU"
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_set_country_code(char *country_code);

/**
 * wifi_mgmr_get_country_code
 * 获取国家代码
 * 参数:
 *  country_code : 用于获取国家代码的指针，例如"CN"，"US"，"JP"，"EU"
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_get_country_code(char *country_code);

/**
 * wifi_mgmr_set_autoconnect_enable
 * 启用站点自动连接
 * 返回:
 *   0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_autoconnect_enable(void);

/**
 * wifi_mgmr_set_autoconnect_disable
 * 禁用站点自动连接
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_autoconnect_disable(void);

/**
 * wifi_mgmr_sta_ps_enter
 * 进入省电模式
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_ps_enter(void);

/**
 * wifi_mgmr_sta_ps_exit
 * 退出省电模式
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_ps_exit(void);

/**
 * wifi_mgmr_sta_set_listen_itv
 * 设置监听间隔
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_set_listen_itv(uint8_t itv);

/**
 * wifi_mgmr_sta_get_listen_itv
 * 获取监听间隔
 * 返回: 监听间隔
 */
uint8_t wifi_mgmr_sta_get_listen_itv(void);

/**
 * wifi_mgmr_sta_aid_get
 * 获取AID
 * 返回: AID
 */
int wifi_mgmr_sta_aid_get(void);

/**
 * wifi_mgmr_sta_get_bssid
 * 获取STA的BSSID
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_get_bssid(uint8_t bssid[6]);

/**
 * wifi_mgmr_ap_sta_info_get
 * 获取AP模式下的STA列表
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_ap_sta_info_get(struct wifi_sta_basic_info *sta_info, uint8_t idx);

/**
 * wifi_mgmr_tpc_pwr_set
 * 设置功率表
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_tpc_pwr_set(rf_pwr_table_t *power_table);

/**
 * wifi_mgmr_tpc_pwr_get
 * 获取功率表
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_tpc_pwr_get(rf_pwr_table_t *power_table);

/**
 * wifi_mgmr_connection_info
 * 获取连接的AP/路由器信息
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_connection_info(wifi_mgmr_connect_ind_stat_info_t *connection_info);

/**
 * wifi_mgmr_get_ap_info
 * 获取AP信息
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_get_ap_info(wifi_mgmr_ap_info_t *ap);

/**
 ****************************************************************************************
 * @brief 为STA模式设置保持活动时间
 *
 * @param[in] 用于STA模式的保持活动时间（单位：秒）
 ****************************************************************************************
 */
int wifi_mgmr_sta_keepalive_time_set(uint8_t time_seconds);

/**
 ****************************************************************************************
 * @brief 获取STA模式的连接状态（已连接或未连接）
 * 返回:
 *  0 : 未连接
 *  1 : 已连接
 ****************************************************************************************
 */
int wifi_mgmr_sta_state_get(void);

/**
 * wifi_mgmr_sta_info_status_code_get
 * 获取状态码
 * 返回: 状态码
 */
uint16_t wifi_mgmr_sta_info_status_code_get();
/**
 ****************************************************************************************
 * @brief 获取AP模式是否已启动
 * 返回:
 *  0 : 未启动
 *  1 : 已启动
 ****************************************************************************************
 */
int wifi_mgmr_ap_state_get(void);

/**
 * wifi_mgmr_conf_max_sta
 * 设置AP支持的STA的最大数量
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_conf_max_sta(uint8_t max_sta_supported);

/**
 * wifi_mgmr_ap_sta_delete
 * 删除AP模式下的STA
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_ap_sta_delete(uint8_t sta_idx);

/**
 * wifi_mgmr_raw_80211_send
 * 发送原始数据包
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_raw_80211_send(const wifi_mgmr_raw_send_params_t *config);

/**
 * wifi_mgmr_psk_cal
 * 通过密码计算PSK
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_psk_cal(char *password, const uint8_t *ssid, int ssid_len, char *output);

/**
 * wifi_mgmr_wifi_pwr_off
 * 关闭Wi-Fi电源
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_wifi_pwr_off(void);

/**
 * wifi_mgmr_wifi_pwr_on
 * 打开Wi-Fi电源
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_wifi_pwr_on(void);

#ifdef CFG_BL_WIFI_PS_ENABLE
/**
 * wifi_mgmr_sta_start_keep_alive
 * 启动保持活动。
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_start_keep_alive(uint16_t seconds);

/**
 * wifi_mgmr_sta_stop_keep_alive
 * 停止保持活动。
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  其他是失败
 */
int wifi_mgmr_sta_stop_keep_alive(void);
#endif


/**
 * wifi_mgmr_rate_config
 * 返回:
 *  0 : 成功
 *  -1 : 失败
 *  0xFFFF: 禁用固
 *  * 返回速率
 */
int wifi_mgmr_rate_config(uint16_t fixed_rate_cfg);


/**
 *
 * wifi_mgmr_sta_extra_timcnt_get
 * 返回:
 *  Tim计数
 */
uint8_t wifi_mgmr_sta_extra_timcnt_get(void);
#endif