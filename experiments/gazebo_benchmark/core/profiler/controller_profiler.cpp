#include <chrono>
#include <vector>
#include <dlfcn.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <csignal>
#include <unistd.h>
#include <pthread.h>

typedef bool (*sleep_fn_t)(void*);
static sleep_fn_t real_sleep = nullptr;

typedef int (*publish_fn_t)(const void*, const void*, void*);
static publish_fn_t real_publish = nullptr;

static std::vector<double> g_durations_ms;
static std::mutex g_mutex;
static bool g_dumped = false;

static thread_local std::chrono::steady_clock::time_point tl_start;
static thread_local bool tl_active = false;

static void dump_data() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_dumped) return;
    g_dumped = true;
    const char* out_path = std::getenv("CONTROLLER_PROFILER_OUT");
    if (!out_path) return;
    std::ofstream ofs(out_path);
    for (double d : g_durations_ms) {
        ofs << d << "\n";
    }
    ofs.flush();
}

static void sig_handler(int sig) {
    dump_data();
    _exit(0);
}

__attribute__((constructor))
static void on_load() {
    struct sigaction sa;
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT, &sa, nullptr);
}

__attribute__((destructor))
static void on_unload() {
    dump_data();
}

// Hook rclcpp::Rate::sleep()
extern "C" bool _ZN6rclcpp4Rate5sleepEv(void* this_ptr) {
    if (!real_sleep) {
        real_sleep = (sleep_fn_t)dlsym(RTLD_NEXT, "_ZN6rclcpp4Rate5sleepEv");
    }
    bool res = real_sleep(this_ptr);
    tl_start = std::chrono::steady_clock::now();
    tl_active = true;
    return res;
}

// Hook rcl_publish
extern "C" int rcl_publish(const void* publisher, const void* ros_message, void* allocation) {
    if (!real_publish) {
        real_publish = (publish_fn_t)dlsym(RTLD_NEXT, "rcl_publish");
    }
    if (tl_active) {
        auto t_end = std::chrono::steady_clock::now();
        double dt_ms = std::chrono::duration<double, std::milli>(t_end - tl_start).count();
        tl_active = false;
        
        std::lock_guard<std::mutex> lock(g_mutex);
        g_durations_ms.push_back(dt_ms);
    }
    return real_publish(publisher, ros_message, allocation);
}
