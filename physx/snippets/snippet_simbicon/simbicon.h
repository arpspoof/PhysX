#pragma once

constexpr int SIMBICON_RFOOT_CONTACT = 1;
constexpr int SIMBICON_LFOOT_CONTACT = 2;

constexpr float SIMBICON_TIMER = 0.2f;

void simbicon_tick(float dt, int contact);
void simbicon_setTargets();
void simbicon_updateForces();
