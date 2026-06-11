#include <windows.h>

#include "Simulator\simulator.h"
#include "config.h"

/*
 * Made by Devin Frandsen, Jan/Feb 2026
 *
 * Found on github: https://github.com/Watermeloncl/slam-simulator
 * 
 * Last updated 2/27/26
*/

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    Simulator* sim = new Simulator(hInstance, nCmdShow);
    sim->RunMainLoop();
    return 0;
}
