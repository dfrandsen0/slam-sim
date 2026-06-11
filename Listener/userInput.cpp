#include "userInput.h"

UserInput::UserInput() {

}

UserInput::~UserInput() {
    
}

bool UserInput::GetClicked() {
    return this->clicked;
}

void UserInput::SetClicked() {
    this->clicked = true;
}

void UserInput::SetUnclicked() {
    this->clicked = false;
}

bool UserInput::GetSpaced() {
    return this->spaced;
}

void UserInput::SetSpaced() {
    this->spaced = true;
}

void UserInput::SetUnspaced() {
    this->spaced = false;
}

int UserInput::GetX() {
    return this->x;
}

void UserInput::SetX(int x) {
    this->x = x;
}

int UserInput::GetY() {
    return this->y;
}

void UserInput::SetY(int y) {
    this->y = y;
}
