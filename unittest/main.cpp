#include "AYTest.h"
#include <cstdio>
#include <cstdlib>

int main(int argc, char* argv[]) {
    // Force unbuffered stdout so per-case output is flushed immediately.
    // Without this, MSVC's CRT keeps the FILE buffer until 4 KiB is filled
    // or fflush is called, which causes the runner's tail (summary)
    // to be lost when stdout is redirected to a non-tty pipe / file.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    return ayt::test::runTests("AYPhysics", argc, argv);
}
