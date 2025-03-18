#include "main.h"

void prevAuto() {
    --autoNum;
}

void selectAuto() {
    selecting = false;
}

void nextAuto() {
    ++autoNum;
}

void nextScreen() {
    currentScreen = (currentScreen + 1) % 3;
}

void prevScreen() {
    currentScreen = (currentScreen + 2) % 3;
}
