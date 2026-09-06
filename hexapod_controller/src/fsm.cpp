#include "fsm.hpp"

FSM_INITIAL_STATE(StateMachine, Rest)
std::function<void()> HexapodBridge::sendStandPose = []() {};
std::function<void(bool)> HexapodBridge::startWalkCycle = [](bool is_reversed) {
};
std::function<void()> HexapodBridge::cancelLegActions = []() {};
std::function<bool()> HexapodBridge::isWalkingReversed = []() { return false; };
