// Lets any process ptrace this one (Yama ptrace_scope = 1 otherwise allows only ancestors), so
// eu-stack can sample it. Built and preloaded by scripts/profile.sh; never linked into the app.
#include <sys/prctl.h>
__attribute__((constructor)) static void allow_ptrace(void) { prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0); }
