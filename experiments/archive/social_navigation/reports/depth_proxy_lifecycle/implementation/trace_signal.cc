// Diagnostic-only, owned process: dump native stacks after shutdown has already timed out.
#include <execinfo.h>
#include <signal.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <cstdio>
static void dump(int) {
  char label[96];int n=snprintf(label,sizeof(label),"\n[LifecycleTrace] tid=%ld\n",syscall(SYS_gettid));
  write(STDERR_FILENO,label,n);
  void *frames[48];int size=backtrace(frames,48);backtrace_symbols_fd(frames,size,STDERR_FILENO);
}
__attribute__((constructor)) static void install() {
  void *frames[2];backtrace(frames,2); // Warm libgcc unwinder before any diagnostic signal.
  struct sigaction action{};action.sa_handler=dump;sigemptyset(&action.sa_mask);action.sa_flags=SA_RESTART;
  sigaction(SIGUSR2,&action,nullptr);
}
