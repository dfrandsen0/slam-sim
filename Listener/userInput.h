#ifndef LISTENER_CLICK_INPUT_H_
#define LISTENER_CLICK_INPUT_H_

class UserInput {
private:
    bool clicked = false;
    bool spaced = false;
    
    int x = 0;
    int y = 0;
public:
    UserInput();
    ~UserInput();

    bool GetClicked();
    void SetClicked();
    void SetUnclicked();

    bool GetSpaced();
    void SetSpaced();
    void SetUnspaced();

    int GetX();
    void SetX(int x);

    int GetY();
    void SetY(int y);
};

#endif