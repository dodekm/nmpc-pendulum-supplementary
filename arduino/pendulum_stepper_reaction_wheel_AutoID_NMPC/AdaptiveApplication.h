#ifndef ADAPTIVE_ADAPTIVEAPPLICATION_H
#define ADAPTIVE_ADAPTIVEAPPLICATION_H
#pragma once

#include "application/Lifecycle.h"

// Arduino-facing application boundary. All workflow state is privately owned by
// the application composition root; callers may only initialize, poll, or inspect it.
namespace adaptive_application {
void initialize();
void poll();
Status status();
} // namespace adaptive_application

#endif // ADAPTIVE_ADAPTIVEAPPLICATION_H
