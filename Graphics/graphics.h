#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include <windows.h>
#include <dwrite.h>
#include <d2d1_1.h>
#include <unordered_map>
#include <utility>
#include <vector>
#include <mutex>
#include <memory>

#include "renderPacket.h"
#include "..\Listener\userInput.h"
#include "..\World\map.h"
#include "..\config.h"

class GraphicsModule {
private:
    ID2D1Factory1* factory = nullptr;
    IDWriteFactory* writeFactory = nullptr;
    ID2D1DeviceContext* deviceContext = nullptr;
    ID2D1CommandList* commandList = nullptr;
    ID2D1Image* targetBitmap = nullptr;

    IDWriteTextFormat* neffTextFormat = nullptr;
    D2D1_RECT_F neffNumLayout;

    D2D1_RECT_F sensorClip;
    D2D1_RECT_F slamClip;
    D2D1_RECT_F confidenceClip;

    std::unordered_map<int, ID2D1SolidColorBrush*> brushes;

    //wall stuff
    D2D1_RECT_F* wallColorSources = nullptr;
    ID2D1Bitmap* whitePixelBitmap = nullptr;

    HWND hwnd = NULL;
    RenderPacket* currentPacket = nullptr;

    std::vector<float>** renderMapAddress = nullptr;
    std::shared_ptr<std::mutex> guardRenderMap;

    UserInput* userInput = nullptr;
public:
    GraphicsModule(HINSTANCE hInstance, int nCmdShow);
    ~GraphicsModule();

    void RenderFrame();
    void CreateBackground(Map* map);

    void UpdateRenderInfo(RenderPacket* incoming);

    void DrawRobot(int quadrant, double x, double y, double theta, bool varsInScreenForm);
    void DrawBigRobot();

    void DrawPointCloud();
    void DrawMap();
    void DrawPoses();
    void DrawNeff();

    void DrawParticle(float x, float y, double theta, int color, bool bigger);

    void GiveRenderMapAddress(std::vector<float>** address);
    void GiveRenderMapGuard(std::shared_ptr<std::mutex> guard);
    UserInput* GetUserInput();

private:
    void InitD2D();
    void CleanupD2D();

    void CreateWindowModule(HINSTANCE hInstance, int nCmdShow);

    void DrawStaticElements();

    std::pair<float, float> XYToDipsBackground(int quadrant, double x, double y);
};

#endif