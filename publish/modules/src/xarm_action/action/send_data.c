#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/wait.h>

#define DEFAULT_CAN_IF "can0"
#define MAX_LOG_LINE 256
#define SEND_FAILED ((void *)1)

static volatile int running = 1;

typedef struct {
    char log_file[MAX_LOG_LINE];
    char can_if[16];
    long long send_interval_ns;
    int use_high_precision;
    int send_count;
    int line_count;
    int use_two_freq;
    long long first_interval_ns;
    long long second_interval_ns;
    int switch_frame_count;
    int error_count;
} send_thread_params_t;

static void print_usage(const char *prog) {
    printf("用法: %s [选项]\n", prog);
    printf("\n说明:\n");
    printf("  1. 无参数启动时会自动配置 CAN 接口并显示交互菜单\n");
    printf("  2. 带动作文件参数时直接发送；调用前须确保 CAN 已配置\n");
    printf("\n选项:\n");
    printf("  -h, --help           显示此帮助信息\n");
    printf("  -f, --file <文件>    指定日志文件路径\n");
    printf("  -c, --can <接口>     指定 CAN 接口（默认: can0）\n");
    printf("  -f2, --file2 <文件>  指定第二个日志文件路径\n");
    printf("  -c2, --can2 <接口>   指定第二个 CAN 接口\n");
}

static int open_can_socket(const char *ifname) {
    struct ifreq ifr;
    struct sockaddr_can addr;
    int sock;

    if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        close(sock);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return -1;
    }

    return sock;
}

static int validate_can_interface(const char *can_if) {
    struct ifreq ifr;
    int sock;
    
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        return -1;
    }
    
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, can_if, IFNAMSIZ - 1);
    
    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
        close(sock);
        return -1;
    }
    
    if (!(ifr.ifr_flags & IFF_UP)) {
        close(sock);
        return -1;
    }
    
    close(sock);
    return 0;
}

static int send_can_frame(int sock, struct can_frame *frame) {
    int ret;
    int retry = 0;
    const int max_retry = 3;
    
    while (retry < max_retry) {
        ret = write(sock, frame, sizeof(struct can_frame));
        if (ret > 0) {
            return ret;
        }
        
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            usleep(10);
            retry++;
            continue;
        }
        
        return ret;
    }
    
    return -1;
}

static void *send_thread_func(void *arg) {
    send_thread_params_t *params = (send_thread_params_t *)arg;
    FILE *fin;
    char line[MAX_LOG_LINE];
    int sock;
    struct can_frame frame;
    char *hash;
    int can_id;
    struct timespec next_send_time;
    
    if (!(fin = fopen(params->log_file, "r"))) {
        perror("[ERROR] fopen");
        params->error_count++;
        return SEND_FAILED;
    }
    
    if ((sock = open_can_socket(params->can_if)) < 0) {
        fprintf(stderr, "[ERROR] 无法打开 CAN 接口: %s\n", params->can_if);
        fclose(fin);
        params->error_count++;
        return SEND_FAILED;
    }
    
    int sndbuf = 65536;
    if (setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
        perror("[WARN] setsockopt SO_SNDBUF");
    }
    
    if (params->use_high_precision) {
        clock_gettime(CLOCK_MONOTONIC, &next_send_time);
    }
    
    while (running && fgets(line, sizeof(line), fin)) {
        line[strcspn(line, "\n")] = '\0';
        
        if (strlen(line) == 0 || line[0] == ' ') continue;
        
        params->line_count++;
        
        frame.can_id = 0;
        frame.can_dlc = 0;
        memset(frame.data, 0, 8);
        
        if ((hash = strchr(line, '#'))) {
            *hash = '\0';
            can_id = strtol(line, NULL, 16);
            hash++;
            
            frame.can_id = can_id & CAN_EFF_MASK;
            
            int len = strlen(hash);
            frame.can_dlc = len / 2;
            if (frame.can_dlc > 8) frame.can_dlc = 8;
            
            for (int i = 0; i < frame.can_dlc; i++) {
                sscanf(&hash[i * 2], "%02X", (unsigned int *)&frame.data[i]);
            }
        } else if (sscanf(line, "%*s %X [%hhd] %63[^\n]", &can_id, &frame.can_dlc, line) == 3) {
            frame.can_id = can_id & CAN_EFF_MASK;
            if (frame.can_dlc > 8) frame.can_dlc = 8;
            
            char *p = line;
            for (int i = 0; i < frame.can_dlc; i++) {
                while (*p == ' ') p++;
                sscanf(p, "%02X", (unsigned int *)&frame.data[i]);
                p += 2;
            }
        } else {
            continue;
        }
        
        if (params->use_high_precision) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            
            if (now.tv_sec < next_send_time.tv_sec ||
                (now.tv_sec == next_send_time.tv_sec && now.tv_nsec < next_send_time.tv_nsec)) {
                
                struct timespec sleep_time = {
                    .tv_sec = next_send_time.tv_sec - now.tv_sec,
                    .tv_nsec = next_send_time.tv_nsec - now.tv_nsec
                };
                
                if (sleep_time.tv_nsec < 0) {
                    sleep_time.tv_sec--;
                    sleep_time.tv_nsec += 1000000000LL;
                }
                
                nanosleep(&sleep_time, NULL);
            }
            
            if (send_can_frame(sock, &frame) < 0) {
                perror("[ERROR] CAN 帧发送失败");
                params->error_count++;
                break;
            }
            
            long long interval = params->send_interval_ns;
            if (params->use_two_freq && params->send_count >= params->switch_frame_count) {
                interval = params->second_interval_ns;
            } else if (params->use_two_freq) {
                interval = params->first_interval_ns;
            }
            
            next_send_time.tv_nsec += interval;
            if (next_send_time.tv_nsec >= 1000000000LL) {
                next_send_time.tv_sec++;
                next_send_time.tv_nsec -= 1000000000LL;
            }
        } else {
            long long interval = params->send_interval_ns;
            if (params->use_two_freq && params->send_count >= params->switch_frame_count) {
                interval = params->second_interval_ns;
            } else if (params->use_two_freq) {
                interval = params->first_interval_ns;
            }
            
            if (send_can_frame(sock, &frame) < 0) {
                perror("[ERROR] CAN 帧发送失败");
                params->error_count++;
                break;
            }
            usleep((useconds_t)(interval / 1000));
        }
        
        params->send_count++;
    }
    
    close(sock);
    fclose(fin);
    
    return params->error_count > 0 ? SEND_FAILED : NULL;
}

/* 命令行单臂回放入口，供 CAN 监听器非交互式调用。 */
static int run_send_with_args(const char *log_file, const char *can_if) {
    send_thread_params_t params;
    memset(&params, 0, sizeof(params));

    strncpy(params.log_file, log_file, sizeof(params.log_file) - 1);
    strncpy(params.can_if, can_if, sizeof(params.can_if) - 1);

    long long send_interval_ns = 2500000LL;
    const int use_high_precision = 1;

    if (validate_can_interface(params.can_if) < 0) {
        fprintf(stderr, "[ERROR] CAN 接口无效: %s\n", params.can_if);
        return -1;
    }

    params.send_interval_ns = send_interval_ns;
    params.use_high_precision = use_high_precision;

    printf("[INFO] 开始发送数据帧...\n");
    printf("[INFO] 日志文件: %s\n", params.log_file);
    printf("[INFO] CAN 口: %s\n", params.can_if);
    printf("[INFO] 发送频率: %.2f kHz (间隔: %lld ns)\n",
           1000000000.0 / send_interval_ns / 1000.0, send_interval_ns);
    printf("[INFO] 高精度定时: %s\n", use_high_precision ? "是" : "否");

    void *thread_result = send_thread_func(&params);

    printf("\n[INFO] 发送完成\n");
    printf("[INFO] 总行数: %d\n", params.line_count);
    printf("[INFO] 成功发送: %d\n", params.send_count);
    printf("[INFO] 失败数量: %d\n", params.line_count - params.send_count);

    if (thread_result != NULL) {
        fprintf(stderr, "[ERROR] 动作发送失败\n");
        return 1;
    }

    return 0;
}

/* 命令行双臂回放入口；两个发送线程分别向两个 CAN 接口发送。 */
static int run_send_with_two_args(const char *log_file1, const char *can_if1,
                                   const char *log_file2, const char *can_if2) {
    send_thread_params_t params[2];
    pthread_t threads[2];
    void *thread_results[2] = {NULL, NULL};

    memset(&params[0], 0, sizeof(params[0]));
    memset(&params[1], 0, sizeof(params[1]));

    strncpy(params[0].log_file, log_file1, sizeof(params[0].log_file) - 1);
    strncpy(params[0].can_if, can_if1, sizeof(params[0].can_if) - 1);
    strncpy(params[1].log_file, log_file2, sizeof(params[1].log_file) - 1);
    strncpy(params[1].can_if, can_if2, sizeof(params[1].can_if) - 1);

    long long send_interval_ns = 2500000LL;
    const int use_high_precision = 1;

    if (validate_can_interface(params[0].can_if) < 0) {
        fprintf(stderr, "[ERROR] CAN 接口无效: %s\n", params[0].can_if);
        return -1;
    }

    if (validate_can_interface(params[1].can_if) < 0) {
        fprintf(stderr, "[ERROR] CAN 接口无效: %s\n", params[1].can_if);
        return -1;
    }

    params[0].send_interval_ns = send_interval_ns;
    params[0].use_high_precision = use_high_precision;
    params[1].send_interval_ns = send_interval_ns;
    params[1].use_high_precision = use_high_precision;

    printf("[INFO] 开始同时发送数据帧...\n");
    printf("[INFO] 文件1: %s -> CAN: %s\n", params[0].log_file, params[0].can_if);
    printf("[INFO] 文件2: %s -> CAN: %s\n", params[1].log_file, params[1].can_if);
    printf("[INFO] 发送频率: %.2f kHz (间隔: %lld ns)\n",
           1000000000.0 / send_interval_ns / 1000.0, send_interval_ns);
    printf("[INFO] 高精度定时: %s\n", use_high_precision ? "是" : "否");

    if (pthread_create(&threads[0], NULL, send_thread_func, &params[0]) != 0) {
        perror("[ERROR] 无法创建第一个发送线程");
        return 1;
    }

    if (pthread_create(&threads[1], NULL, send_thread_func, &params[1]) != 0) {
        perror("[ERROR] 无法创建第二个发送线程");
        pthread_join(threads[0], &thread_results[0]);
        return 1;
    }

    pthread_join(threads[0], &thread_results[0]);
    pthread_join(threads[1], &thread_results[1]);

    printf("\n[INFO] 发送完成\n");
    printf("[INFO] 文件1 - 总行数: %d, 成功发送: %d, 失败: %d\n",
           params[0].line_count, params[0].send_count, params[0].line_count - params[0].send_count);
    printf("[INFO] 文件2 - 总行数: %d, 成功发送: %d, 失败: %d\n",
           params[1].line_count, params[1].send_count, params[1].line_count - params[1].send_count);
    printf("[INFO] 总计 - 成功发送: %d, 失败: %d\n",
           params[0].send_count + params[1].send_count,
           (params[0].line_count - params[0].send_count) + (params[1].line_count - params[1].send_count));

    if (thread_results[0] != NULL || thread_results[1] != NULL) {
        fprintf(stderr, "[ERROR] 一个或多个动作发送失败\n");
        return 1;
    }

    return 0;
}
static void run_send_from_log(void) {
    char choice[8];
    char line[MAX_LOG_LINE];
    long long send_interval_ns = 2500000LL;
    const int use_high_precision = 1;
    const int use_two_freq = 0;
    
    printf("\n从日志文件发送 CAN 数据帧\n");
    printf("  1) 单文件单 CAN 口发送\n");
    printf("  2) 双文件双 CAN 口同时发送\n");
    printf("请选择 [1-2]: ");
    
    if (!fgets(choice, sizeof(choice), stdin)) return;
    choice[strcspn(choice, "\n")] = '\0';
    
    if (choice[0] == '1') {
        send_thread_params_t params;
        memset(&params, 0, sizeof(params));
        
        printf("请输入日志文件路径: ");
        fgets(params.log_file, sizeof(params.log_file), stdin);
        params.log_file[strcspn(params.log_file, "\n")] = '\0';
        
        printf("请输入要发送的 CAN 口 [%s]: ", DEFAULT_CAN_IF);
        fgets(params.can_if, sizeof(params.can_if), stdin);
        params.can_if[strcspn(params.can_if, "\n")] = '\0';
        if (strlen(params.can_if) == 0) strcpy(params.can_if, DEFAULT_CAN_IF);
        
        if (validate_can_interface(params.can_if) < 0) {
            fprintf(stderr, "[ERROR] CAN 接口无效: %s\n", params.can_if);
            return;
        }
        
        params.send_interval_ns = send_interval_ns;
        params.use_high_precision = use_high_precision;

        printf("[INFO] 开始发送数据帧...\n");
        printf("[INFO] 日志文件: %s\n", params.log_file);
        printf("[INFO] CAN 口: %s\n", params.can_if);
        printf("[INFO] 发送频率: %.2f kHz (间隔: %lld ns)\n",
               1000000000.0 / send_interval_ns / 1000.0, send_interval_ns);
        printf("[INFO] 高精度定时: %s\n", use_high_precision ? "是" : "否");
        printf("[INFO] 按 Ctrl+C 停止发送\n");
        
        send_thread_func(&params);
        
        printf("\n[INFO] 发送完成\n");
        printf("[INFO] 总行数: %d\n", params.line_count);
        printf("[INFO] 成功发送: %d\n", params.send_count);
        printf("[INFO] 失败数量: %d\n", params.line_count - params.send_count);
        
    } else if (choice[0] == '2') {
        send_thread_params_t params[2];
        pthread_t threads[2];
        
        memset(&params[0], 0, sizeof(params[0]));
        memset(&params[1], 0, sizeof(params[1]));
        
        printf("请输入第一个日志文件路径: ");
        fgets(params[0].log_file, sizeof(params[0].log_file), stdin);
        params[0].log_file[strcspn(params[0].log_file, "\n")] = '\0';
        
        printf("请输入第一个 CAN 口 [%s]: ", DEFAULT_CAN_IF);
        fgets(params[0].can_if, sizeof(params[0].can_if), stdin);
        params[0].can_if[strcspn(params[0].can_if, "\n")] = '\0';
        if (strlen(params[0].can_if) == 0) strcpy(params[0].can_if, DEFAULT_CAN_IF);
        
        printf("请输入第二个日志文件路径: ");
        fgets(params[1].log_file, sizeof(params[1].log_file), stdin);
        params[1].log_file[strcspn(params[1].log_file, "\n")] = '\0';
        
        printf("请输入第二个 CAN 口 [%s]: ", DEFAULT_CAN_IF);
        fgets(params[1].can_if, sizeof(params[1].can_if), stdin);
        params[1].can_if[strcspn(params[1].can_if, "\n")] = '\0';
        if (strlen(params[1].can_if) == 0) strcpy(params[1].can_if, DEFAULT_CAN_IF);
        
        if (validate_can_interface(params[0].can_if) < 0) {
            fprintf(stderr, "[ERROR] CAN 接口无效: %s\n", params[0].can_if);
            return;
        }
        
        if (validate_can_interface(params[1].can_if) < 0) {
            fprintf(stderr, "[ERROR] CAN 接口无效: %s\n", params[1].can_if);
            return;
        }
        
        params[0].send_interval_ns = send_interval_ns;
        params[0].use_high_precision = use_high_precision;
        params[1].send_interval_ns = send_interval_ns;
        params[1].use_high_precision = use_high_precision;
        

        
        printf("[INFO] 开始同时发送数据帧...\n");
        printf("[INFO] 文件1: %s -> CAN: %s\n", params[0].log_file, params[0].can_if);
        printf("[INFO] 文件2: %s -> CAN: %s\n", params[1].log_file, params[1].can_if);
        printf("[INFO] 发送频率: %.2f kHz (间隔: %lld ns)\n",
               1000000000.0 / send_interval_ns / 1000.0, send_interval_ns);
        printf("[INFO] 高精度定时: %s\n", use_high_precision ? "是" : "否");
        printf("[INFO] 按 Ctrl+C 停止发送\n");
        
        pthread_create(&threads[0], NULL, send_thread_func, &params[0]);
        pthread_create(&threads[1], NULL, send_thread_func, &params[1]);
        
        pthread_join(threads[0], NULL);
        pthread_join(threads[1], NULL);
        
        printf("\n[INFO] 发送完成\n");
        printf("[INFO] 文件1 - 总行数: %d, 成功发送: %d, 失败: %d\n", 
               params[0].line_count, params[0].send_count, params[0].line_count - params[0].send_count);
        printf("[INFO] 文件2 - 总行数: %d, 成功发送: %d, 失败: %d\n", 
               params[1].line_count, params[1].send_count, params[1].line_count - params[1].send_count);
        printf("[INFO] 总计 - 成功发送: %d, 失败: %d\n", 
               params[0].send_count + params[1].send_count, 
               (params[0].line_count - params[0].send_count) + (params[1].line_count - params[1].send_count));
    } else {
        printf("[WARN] 无效选择\n");
    }
}

static void signal_handler(int sig) {
    running = 0;
    printf("\n[INFO] 用户取消操作\n");
    exit(0);
}

/* 手动模式从当前可执行文件位置定位仓库内的 CAN 配置脚本，避免依赖旧部署路径。 */
static int run_setup_can(void) {
    char executable_path[PATH_MAX];
    char setup_script[PATH_MAX];
    char *last_slash;
    ssize_t path_length;
    pid_t child_pid;
    int child_status;

    path_length = readlink("/proc/self/exe", executable_path, sizeof(executable_path) - 1);
    if (path_length < 0) {
        perror("[ERROR] 无法定位 send_data 可执行文件");
        return -1;
    }
    executable_path[path_length] = '\0';

    last_slash = strrchr(executable_path, '/');
    if (last_slash == NULL) {
        fprintf(stderr, "[ERROR] send_data 可执行文件路径无效\n");
        return -1;
    }
    *last_slash = '\0';

    if (snprintf(setup_script, sizeof(setup_script),
                 "%s/../../../../xarm_can/package/libexec/setup_can_interfaces.sh",
                 executable_path) >= (int)sizeof(setup_script)) {
        fprintf(stderr, "[ERROR] CAN 配置脚本路径过长\n");
        return -1;
    }
    if (access(setup_script, R_OK) != 0) {
        fprintf(stderr, "[ERROR] 找不到 CAN 配置脚本: %s\n", setup_script);
        return -1;
    }

    printf("[INFO] 配置 CAN 接口: %s\n", setup_script);
    child_pid = fork();
    if (child_pid < 0) {
        perror("[ERROR] 无法启动 CAN 配置脚本");
        return -1;
    }
    if (child_pid == 0) {
        execl("/bin/bash", "bash", setup_script, (char *)NULL);
        perror("[ERROR] 无法执行 CAN 配置脚本");
        _exit(127);
    }
    if (waitpid(child_pid, &child_status, 0) < 0) {
        perror("[ERROR] 等待 CAN 配置脚本失败");
        return -1;
    }
    if (!WIFEXITED(child_status) || WEXITSTATUS(child_status) != 0) {
        fprintf(stderr, "[ERROR] CAN 配置脚本执行失败\n");
        return -1;
    }

    printf("[INFO] CAN 接口配置完成\n");
    return 0;
}

static void post_setup_menu(void) {
    char choice[8];
    
    while (running) {
        printf("\nCAN 接口配置完成，可选择:\n");
        printf("  1) 从日志文件发送数据帧\n");
        printf("  2) 退出\n");
        printf("请选择 [1-2]: ");
        
        if (!fgets(choice, sizeof(choice), stdin)) break;
        choice[strcspn(choice, "\n")] = '\0';
        
        switch (choice[0]) {
            case '1':
                run_send_from_log();
                break;
            case '2':
            case 'q':
            case 'Q':
                exit(0);
            default:
                printf("[WARN] 无效选择，请重新输入。\n");
        }
    }
}

/*
 * 无动作文件参数时保留人工菜单；带文件参数时供监听器自动回放。
 * 自动回放依赖外部已完成 CAN 配置，避免每次触发动作都重复配置接口。
 */
int main(int argc, char *argv[]) {
    const char *log_file = NULL;
    const char *can_if = DEFAULT_CAN_IF;
    const char *log_file2 = NULL;
    const char *can_if2 = DEFAULT_CAN_IF;
    int skip_setup = 0;
    int has_can_option = 0;
    int has_can2_option = 0;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)) {
            print_usage(argv[0]);
            return 0;
        } else if ((strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--file") == 0)) {
            if (i + 1 < argc) {
                log_file = argv[++i];
                skip_setup = 1;
            } else {
                fprintf(stderr, "[ERROR] -f/--file 参数需要指定文件路径\n");
                print_usage(argv[0]);
                return 1;
            }
        } else if ((strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--can") == 0)) {
            if (i + 1 < argc) {
                can_if = argv[++i];
                has_can_option = 1;
            } else {
                fprintf(stderr, "[ERROR] -c/--can 参数需要指定 CAN 接口\n");
                print_usage(argv[0]);
                return 1;
            }
        } else if ((strcmp(argv[i], "-f2") == 0 || strcmp(argv[i], "--file2") == 0)) {
            if (i + 1 < argc) {
                log_file2 = argv[++i];
                skip_setup = 1;
            } else {
                fprintf(stderr, "[ERROR] -f2/--file2 参数需要指定文件路径\n");
                print_usage(argv[0]);
                return 1;
            }
        } else if ((strcmp(argv[i], "-c2") == 0 || strcmp(argv[i], "--can2") == 0)) {
            if (i + 1 < argc) {
                can_if2 = argv[++i];
                has_can2_option = 1;
            } else {
                fprintf(stderr, "[ERROR] -c2/--can2 参数需要指定 CAN 接口\n");
                print_usage(argv[0]);
                return 1;
            }
        } else {
            fprintf(stderr, "[ERROR] 未知参数: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    if (log_file == NULL && (log_file2 != NULL || has_can_option || has_can2_option)) {
        fprintf(stderr, "[ERROR] -c/-c2/-f2 必须与主文件参数 -f/--file 一起使用\n");
        print_usage(argv[0]);
        return 1;
    }
    if (log_file2 != NULL && !has_can2_option) {
        fprintf(stderr, "[ERROR] 双文件模式必须同时指定 -c2/--can2，避免两个动作误发到同一接口\n");
        print_usage(argv[0]);
        return 1;
    }
    if (log_file2 == NULL && has_can2_option) {
        fprintf(stderr, "[ERROR] -c2/--can2 必须与 -f2/--file2 一起使用\n");
        print_usage(argv[0]);
        return 1;
    }

    if (!skip_setup && run_setup_can() != 0) {
        return 1;
    }

    if (log_file2 != NULL && log_file != NULL) {
        return run_send_with_two_args(log_file, can_if, log_file2, can_if2);
    } else if (log_file != NULL) {
        return run_send_with_args(log_file, can_if);
    }

    post_setup_menu();

    return 0;
}
