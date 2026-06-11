#include "logField.h"

LogField::LogField() {

}

//doesn't delete it's data except inverseSigmas
LogField::~LogField() {
    delete[] this->inverseSigmas;
}