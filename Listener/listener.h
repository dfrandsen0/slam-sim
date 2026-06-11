#ifndef LISTENER_H_
#define LISTENER_H_

#include <windows.h>

class ListenerModule {
public:
private:
    static MSG msg;
public:
    static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
private:
    ListenerModule();
};

#endif