#include <windows.h>
#include <windowsx.h>

#include "listener.h"
#include "userInput.h"

MSG ListenerModule::msg = {};

ListenerModule::ListenerModule() {
    //constructor; empty
    // set private so that no instantiations can exist
}

//act on messages
// - pause/unpause (space)
// - Change Trajectory (click top right)
LRESULT CALLBACK ListenerModule::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    UserInput* userInput = (UserInput*)GetWindowLongPtr(hwnd, GWLP_USERDATA);

    switch (msg) {
        case WM_LBUTTONDOWN: {
            if(userInput == nullptr) {
                break;
            }

            userInput->SetClicked();
            userInput->SetX(GET_X_LPARAM(lParam));
            userInput->SetY(GET_Y_LPARAM(lParam));

            break;
        }
        case WM_KEYDOWN: {
            if(wParam == VK_SPACE) {
                userInput->SetSpaced();
            }
            return 0;
        }
        case WM_DESTROY: {
            PostQuitMessage(0);
            return 0;
        }
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}