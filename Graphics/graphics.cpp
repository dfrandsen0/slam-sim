#include <windows.h>
#include <dwrite.h>
#include <d2d1_1.h>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <cmath>
#include <vector>
#include <mutex>
#include <memory>
#include <cwchar>

#include "graphics.h"
#include "..\Listener\userInput.h"
#include "..\Listener\listener.h"
#include "..\World\map.h"
#include "..\World\Objects\oline.h"
#include "..\World\Objects\opoint.h"
#include "..\Utilities\mathUtilities.h"
#include "../config.h"


GraphicsModule::GraphicsModule(HINSTANCE hInstance, int nCmdShow) {
    this->CreateWindowModule(hInstance, nCmdShow);

    this->InitD2D();
}

GraphicsModule::~GraphicsModule() {
    this->CleanupD2D();
    delete this->currentPacket;
    delete this->userInput;
}

void GraphicsModule::InitD2D() {
    D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, __uuidof(ID2D1Factory1), (void**)&factory);
    DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED, __uuidof(IDWriteFactory), reinterpret_cast<IUnknown**>(&writeFactory));

    RECT rc;
    GetClientRect(this->hwnd, &rc);

    ID2D1HwndRenderTarget* hwndRT = nullptr;
    D2D1_RENDER_TARGET_PROPERTIES props = D2D1::RenderTargetProperties();
    D2D1_HWND_RENDER_TARGET_PROPERTIES hwndProps = D2D1::HwndRenderTargetProperties(this->hwnd, D2D1::SizeU(rc.right, rc.bottom));

    factory->CreateHwndRenderTarget(&props, &hwndProps, &hwndRT);

    hwndRT->QueryInterface(__uuidof(ID2D1DeviceContext), (void**)&deviceContext);
    
    this->deviceContext->GetTarget(&this->targetBitmap);

    hwndRT->Release();

    for(int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        ID2D1SolidColorBrush* tempBrush = nullptr;
        this->deviceContext->CreateSolidColorBrush(D2D1::ColorF(COLOR_PALETTE_VALUES[i]), &tempBrush);
        this->brushes[COLOR_PALETTE_VALUES[i]] = tempBrush;
    }

    this->writeFactory->CreateTextFormat(
        L"Segoe UI",
        NULL,           // font collection; unneeded since Segoe UI is standard
        DWRITE_FONT_WEIGHT_NORMAL,
        DWRITE_FONT_STYLE_NORMAL,
        DWRITE_FONT_STRETCH_NORMAL,
        15.0f,            // font size (dips)
        L"en-us",       // language locale; interpretation basically
        &(this->neffTextFormat)
    );

    this->neffNumLayout = D2D1::RectF(
        (CLIENT_SCREEN_WIDTH / 2.0) + neffOffset + neffNumOffset, // width needed a small boost
        (CLIENT_SCREEN_HEIGHT / 2.0) + neffOffset,
        (CLIENT_SCREEN_WIDTH / 2.0) + neffOffset + 75 + neffNumOffset, // ballpark width/height maxima
        (CLIENT_SCREEN_HEIGHT / 2.0) + neffOffset + 40
    );

    this->sensorClip = D2D1::RectF(
        BACKGROUND_LINE_WIDTH,
        CLIENT_SCREEN_HEIGHT / 2.0,
        (CLIENT_SCREEN_WIDTH / 2.0) - (BACKGROUND_LINE_WIDTH / 2.0),
        CLIENT_SCREEN_HEIGHT - BACKGROUND_LINE_WIDTH
    );

    this->slamClip = D2D1::RectF(
        BACKGROUND_LINE_WIDTH,
        BACKGROUND_LINE_WIDTH,
        (CLIENT_SCREEN_WIDTH / 2.0) - (BACKGROUND_LINE_WIDTH / 2.0),
        (CLIENT_SCREEN_HEIGHT / 2.0) - (BACKGROUND_LINE_WIDTH / 2.0)
    );

    this->confidenceClip = D2D1::RectF(
        (CLIENT_SCREEN_WIDTH / 2.0) + (BACKGROUND_LINE_WIDTH / 2.0),
        (CLIENT_SCREEN_HEIGHT / 2.0) + (BACKGROUND_LINE_WIDTH / 2.0),
        CLIENT_SCREEN_WIDTH - (BACKGROUND_LINE_WIDTH / 2.0),
        CLIENT_SCREEN_HEIGHT - (BACKGROUND_LINE_WIDTH / 2.0)
    );

    D2D1_BITMAP_PROPERTIES props2 = D2D1::BitmapProperties(
        D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_PREMULTIPLIED)
    );

    this->deviceContext->CreateBitmap(
        D2D1::SizeU(COLOR_PALETTE_WALLS_SIZE, 1),
        COLOR_PALETTE_WALLS,
        COLOR_PALETTE_WALLS_SIZE*4,
        &props2,
        &(this->whitePixelBitmap)
    );

    this->wallColorSources = new D2D1_RECT_F[COLOR_PALETTE_WALLS_SIZE];
    for (int i = 0; i < COLOR_PALETTE_WALLS_SIZE; i++) {
        this->wallColorSources[i] = D2D1::RectF((float)i, 0.0F, (float)i + 1.0F, 1.0F);
    }

}

void GraphicsModule::CleanupD2D() {
    for(int i = 0; i < COLOR_PALETTE_SIZE; i++) {
        if(this->brushes[COLOR_PALETTE_VALUES[i]]) this->brushes[COLOR_PALETTE_VALUES[i]]->Release();
    }
    this->brushes.clear();

    if(this->commandList) this->commandList->Release();
    if(this->targetBitmap) this->targetBitmap->Release();
    if (this->deviceContext) this->deviceContext->Release();
    if (this->factory) this->factory->Release();
}

void GraphicsModule::RenderFrame() {
    this->deviceContext->BeginDraw();
    this->deviceContext->Clear(D2D1::ColorF(COLOR_PALETTE_BACKGROUND));

    this->DrawStaticElements();

    //draw dynamic elements. consider push axis aligned clip and setTransform
    this->DrawRobot(TOP_RIGHT, this->currentPacket->realX, this->currentPacket->realY, this->currentPacket->realTheta, false);
    this->DrawRobot(BOTTOM_LEFT, this->currentPacket->realX, this->currentPacket->realY, this->currentPacket->realTheta, false);
    this->DrawPointCloud();
    this->DrawMap();
    this->DrawPoses(); // must check for nullptr
    this->DrawNeff();

    this->deviceContext->EndDraw(); // BLOCKS for VSync
}

void GraphicsModule::CreateBackground(Map* map) {
    // Begin Context
    deviceContext->CreateCommandList(&this->commandList);
    deviceContext->SetTarget(this->commandList);
    deviceContext->BeginDraw();

    // Background Elements
    deviceContext->DrawLine(D2D1::Point2F(0, BACKGROUND_LINE_WIDTH / 2), D2D1::Point2F(CLIENT_SCREEN_WIDTH, BACKGROUND_LINE_WIDTH / 2), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);
    deviceContext->DrawLine(D2D1::Point2F(CLIENT_SCREEN_WIDTH - 2, 0), D2D1::Point2F(CLIENT_SCREEN_WIDTH - 2, CLIENT_SCREEN_HEIGHT), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);
    deviceContext->DrawLine(D2D1::Point2F(0, CLIENT_SCREEN_HEIGHT - 2), D2D1::Point2F(CLIENT_SCREEN_WIDTH, CLIENT_SCREEN_HEIGHT - 2), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);
    deviceContext->DrawLine(D2D1::Point2F(BACKGROUND_LINE_WIDTH / 2, 0), D2D1::Point2F(BACKGROUND_LINE_WIDTH / 2, CLIENT_SCREEN_HEIGHT), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);

    deviceContext->DrawLine(D2D1::Point2F(CLIENT_SCREEN_WIDTH / 2, 0), D2D1::Point2F(CLIENT_SCREEN_WIDTH / 2, CLIENT_SCREEN_HEIGHT), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);
    deviceContext->DrawLine(D2D1::Point2F(0, CLIENT_SCREEN_HEIGHT / 2), D2D1::Point2F(CLIENT_SCREEN_WIDTH, CLIENT_SCREEN_HEIGHT / 2), this->brushes[COLOR_PALETTE_BLACK], BACKGROUND_LINE_WIDTH);

    //draw map top right
    OLine** lines = map->GetLines();
    std::pair<float, float> temp1, temp2;

    for(int i = 0; i < map->GetLinesSize(); i++) {
        temp1 = this->XYToDipsBackground(TOP_RIGHT, lines[i]->point1->x, lines[i]->point1->y);
        temp2 = this->XYToDipsBackground(TOP_RIGHT, lines[i]->point2->x, lines[i]->point2->y);        

        deviceContext->DrawLine(
            D2D1::Point2F(temp1.first, temp1.second),
            D2D1::Point2F(temp2.first, temp2.second),
            this->brushes[COLOR_PALETTE_BLACK],
            MAP_LINE_WIDTH
        );
    }

    D2D1_RECT_F neffLayout = D2D1::RectF(
        (CLIENT_SCREEN_WIDTH / 2.0) + neffOffset + 2, // width needed a small boost
        (CLIENT_SCREEN_HEIGHT / 2.0) + neffOffset,
        (CLIENT_SCREEN_WIDTH / 2.0) + neffOffset + 42, // ballpark width/height maxima
        (CLIENT_SCREEN_HEIGHT / 2.0) + neffOffset + 40
    );

    deviceContext->DrawText(
        neffText,
        (UINT32)wcslen(neffText),
        neffTextFormat,
        neffLayout,
        this->brushes[COLOR_PALETTE_BLACK]
    );

    this->DrawBigRobot();

    if(SHOW_POSSIBLE_STARTING_LOCATIONS) {
        OPoint** starts = map->GetStarts();
        for(int i = 0; i < map->GetStartsSize(); i++) {
            temp1 = this->XYToDipsBackground(TOP_RIGHT, starts[i]->x, starts[i]->y);
            deviceContext->DrawEllipse(
                D2D1::Ellipse(D2D1::Point2F(temp1.first, temp1.second), ROBOT_RADIUS, ROBOT_RADIUS),
                this->brushes[COLOR_PALETTE_BLACK],
                1.5,
                nullptr
            );
        }
    }

    // End context
    deviceContext->EndDraw();
    this->commandList->Close();
    deviceContext->SetTarget(this->targetBitmap); 
}

void GraphicsModule::UpdateRenderInfo(RenderPacket* incoming) {
    delete this->currentPacket;
    this->currentPacket = incoming;
}

void GraphicsModule::DrawRobot(int quadrant, double x, double y, double theta, bool varsInScreenForm) {
    std::pair<float, float> temp;
    if(!varsInScreenForm) {
        temp = this->XYToDipsBackground(quadrant, x, y);
    } else {
        temp = {(float)x, (float)y};
    }

    deviceContext->DrawEllipse(
        D2D1::Ellipse(D2D1::Point2F(temp.first, temp.second), ROBOT_RADIUS, ROBOT_RADIUS),
        this->brushes[COLOR_PALETTE_BLACK],
        1.5,
        nullptr
    );

    deviceContext->DrawLine(
        D2D1::Point2F(temp.first, temp.second),
        D2D1::Point2F((float)(temp.first + (ROBOT_RADIUS * std::cos(theta))),
                        (float)(temp.second - (ROBOT_RADIUS * std::sin(theta)))),
        this->brushes[COLOR_PALETTE_BLACK],
        2,
        nullptr
    );
}

void GraphicsModule::DrawBigRobot() {
    deviceContext->DrawEllipse(
        D2D1::Ellipse(D2D1::Point2F(CLIENT_SCREEN_WIDTH * 0.75, CLIENT_SCREEN_HEIGHT * 0.75), ROBOT_REAL_RADIUS, ROBOT_REAL_RADIUS),
        this->brushes[COLOR_PALETTE_LIGHT_GRAY],
        3.5,
        nullptr
    );

    deviceContext->DrawLine(
        D2D1::Point2F(CLIENT_SCREEN_WIDTH * 0.75, CLIENT_SCREEN_HEIGHT * 0.75),
        D2D1::Point2F((CLIENT_SCREEN_WIDTH * 0.75) + ROBOT_REAL_RADIUS, CLIENT_SCREEN_HEIGHT * 0.75),
        this->brushes[COLOR_PALETTE_LIGHT_GRAY],
        3.5,
        nullptr
    );
}

void GraphicsModule::DrawPointCloud() {
    if(this->currentPacket->pointCloud == nullptr) {
        return;
    }

    this->deviceContext->PushAxisAlignedClip(this->sensorClip, D2D1_ANTIALIAS_MODE_PER_PRIMITIVE);

    OPoint* tempPoint;
    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        if(this->currentPacket->pointCloud[i] == nullptr) {
            break;
        }

        tempPoint = this->currentPacket->pointCloud[i];
        std::pair<float, float> temp = XYToDipsBackground(BOTTOM_LEFT, tempPoint->x, tempPoint->y);
        deviceContext->FillEllipse(
            D2D1::Ellipse(D2D1::Point2F(temp.first, temp.second), LIDAR_POINT_RADIUS, LIDAR_POINT_RADIUS),
            this->brushes[COLOR_PALETTE_RED]
        );
    }

    this->deviceContext->PopAxisAlignedClip();
}


void GraphicsModule::DrawMap() {
    std::lock_guard<std::mutex> lock(*(this->guardRenderMap));

    if((this->renderMapAddress == nullptr) || (*(this->renderMapAddress) == nullptr)) {
        return;
    }

    this->deviceContext->PushAxisAlignedClip(this->slamClip, D2D1_ANTIALIAS_MODE_PER_PRIMITIVE);

    std::vector<float>* map = *(this->renderMapAddress);
    int colorIndex;
    double colorMult = (1.0 / GMAPPING_MAX_LOG_ODDS) * COLOR_PALETTE_WALLS_SIZE;

    for (long long unsigned int i = 0; i < map->size(); i += 3) {
        if((*map)[i + 2] == GMAPPING_MAX_LOG_ODDS) {
            colorIndex = 9;
        } else {
            colorIndex = (int)(((*map)[i + 2]) * colorMult);
        }
        
        D2D1_RECT_F wallRect = D2D1::RectF(
            (*map)[i], 
            (*map)[i + 1], 
            (*map)[i] + GMAPPING_CELL_SIZE_DIPS, 
            (*map)[i + 1] + GMAPPING_CELL_SIZE_DIPS
        );

        this->deviceContext->DrawBitmap(
            this->whitePixelBitmap,
            &wallRect,
            1.0F,
            D2D1_INTERPOLATION_MODE_NEAREST_NEIGHBOR, 
            &(this->wallColorSources[colorIndex])
        );
    }

    this->deviceContext->PopAxisAlignedClip();
}

void GraphicsModule::DrawPoses() {
    if(this->currentPacket->poses == nullptr) {
        return;
    }

    if(STARTING_SLAM == SLAM_OPTION_GMAPPING) {
        PoseRenderPacket* posePacket = this->currentPacket->poses;
        PoseRenderPacket* extendedPoses = this->currentPacket->extendedPoses;

        //skips the first one (to do last, so it shows on top)
        for(int index = posePacket->valueSetSize; index < posePacket->numValues; index += posePacket->valueSetSize) {
            if(SHOW_PARTICLES_TOP_LEFT) {
                this->DrawParticle(
                    (float)(posePacket->poses[index]),
                    (float)(posePacket->poses[index + 1]),
                    posePacket->poses[index + 2],
                    COLOR_PALETTE_BLUE,
                    false
                );
            }
        }

        //draw most confident pose
        this->DrawRobot(TOP_LEFT, posePacket->poses[0], posePacket->poses[1], posePacket->poses[2], true);

        // Although this loop is identical, pushing axisalignedclip is expensive, so this is designed for minimizing the clipping calls
        this->deviceContext->PushAxisAlignedClip(this->confidenceClip, D2D1_ANTIALIAS_MODE_PER_PRIMITIVE);
        for(int index = posePacket->valueSetSize; index < posePacket->numValues; index += posePacket->valueSetSize) {
            this->DrawParticle(
                (float)(extendedPoses->poses[index]),
                (float)(extendedPoses->poses[index + 1]),
                extendedPoses->poses[index + 2],
                COLOR_PALETTE_BLUE,
                true
            );
        }

        int strongestColor = COLOR_PALETTE_BLUE;
        if(HIGHLIGHT_STRONGEST_POSE_BOTTOM_RIGHT) {
            strongestColor = COLOR_PALETTE_RED;
        }

        this->DrawParticle(
            (float)(extendedPoses->poses[0]),
            (float)(extendedPoses->poses[1]),
            extendedPoses->poses[2],
            strongestColor,
            true
        );

        this->deviceContext->PopAxisAlignedClip();
    }
}

void GraphicsModule::DrawNeff() {
    wchar_t neffText[16];
    swprintf(neffText, 16, L"%.5f", this->currentPacket->neff);

    deviceContext->DrawText(
        neffText,
        (UINT32)wcslen(neffText),
        neffTextFormat,
        this->neffNumLayout,
        this->brushes[COLOR_PALETTE_BLACK]
    );
}

void GraphicsModule::DrawParticle(float x, float y, double theta, int color, bool bigger) {
    //todo determine lengths/sizes/colors
    float length = GMAPPING_PARTICLE_POINTER_LENGTH;
    float width = GMAPPING_PARTICLE_POINTER_WIDTH;
    float radius = GMAPPING_PARTICLE_RADIUS;

    if(bigger) {
        length = GMAPPING_BIG_PARTICLE_POINTER_LENGTH;
        width = GMAPPING_BIG_PARTICLE_POINTER_WIDTH;
        radius = GMAPPING_BIG_PARTICLE_RADIUS;
    }

    deviceContext->FillEllipse(
        D2D1::Ellipse(D2D1::Point2F(x, y), radius, radius),
        this->brushes[color]
    );

    float endX = x + ((float)(std::cos(theta) * length));
    float endY = y - ((float)(std::sin(theta) * length)); //flip screen upside down (hence minus)
    deviceContext->DrawLine(D2D1::Point2F(x, y), D2D1::Point2F(endX, endY), this->brushes[color], width);
}

void GraphicsModule::GiveRenderMapAddress(std::vector<float>** address) {
    // (*(this->renderMapAddress)) access like this
    this->renderMapAddress = address;
}

void GraphicsModule::GiveRenderMapGuard(std::shared_ptr<std::mutex> guard) {
    // (*(this->renderManGuard)) access like this
    this->guardRenderMap = guard;
}

UserInput* GraphicsModule::GetUserInput() {
    return this->userInput;
}

void GraphicsModule::CreateWindowModule(HINSTANCE hInstance, int nCmdShow) {
    const wchar_t CLASS_NAME[]  = L"SLAM Window Class";

    WNDCLASSW wc = { };

    wc.lpfnWndProc = ListenerModule::WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = CLASS_NAME;

    RegisterClassW(&wc);

    RECT rect = {0, 0, CLIENT_SCREEN_WIDTH, CLIENT_SCREEN_HEIGHT};
    AdjustWindowRectEx(&rect, WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX, FALSE, 0);

    int screenWidth = GetSystemMetrics(SM_CXSCREEN);
    int screenHeight = GetSystemMetrics(SM_CYSCREEN);

    this->hwnd = CreateWindowExW(
        0,
        CLASS_NAME,
        L"SLAM Simulator", //Only S shows up anyways. 
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,

        (screenWidth - (rect.right - rect.left)) / 2, (screenHeight - (rect.bottom - rect.top)) / 2, rect.right - rect.left, rect.bottom - rect.top,

        NULL,   
        NULL,
        hInstance,
        NULL
    );

    if (hwnd == NULL) {
        std::cout << "Failed to create window handle." << std::endl;
        return;
    }

    UserInput* userInput = new UserInput();
    SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)userInput);
    this->userInput = userInput;

    ShowWindow(hwnd, nCmdShow);
}

void GraphicsModule::DrawStaticElements() {
    if(this->commandList) {
        this->deviceContext->DrawImage(this->commandList);
    }
}

// adds or subtracts 1 appropriately to center background lines on specified points
std::pair<float, float> GraphicsModule::XYToDipsBackground(int quadrant, double x, double y) {
    switch(quadrant) {
        case TOP_LEFT:
            return {
                (CLIENT_SCREEN_WIDTH * 0.25) + (x / MM_PER_DIP) + 1,
                (CLIENT_SCREEN_HEIGHT * 0.25) + (y / MM_PER_DIP * -1) + 1
            };
        case TOP_RIGHT:
            return {
                (CLIENT_SCREEN_WIDTH * 0.75) + (x / MM_PER_DIP) - 1,
                (CLIENT_SCREEN_HEIGHT * 0.25) + (y / MM_PER_DIP * -1) + 1
            };
        case BOTTOM_LEFT:
            return {
                (CLIENT_SCREEN_WIDTH * 0.25) + (x / MM_PER_DIP) + 1,
                (CLIENT_SCREEN_HEIGHT * 0.75) + (y / MM_PER_DIP * -1) - 1
            };
        default:
            return {
                (CLIENT_SCREEN_WIDTH * 0.75) + (x / MM_PER_DIP) - 1,
                (CLIENT_SCREEN_HEIGHT * 0.75) + (y / MM_PER_DIP * -1) - 1
            };
    }
}
