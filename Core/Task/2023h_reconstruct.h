//
// Rebuild separated 2023H signals on the two DAC outputs.
//

#ifndef IIT6_OSCILLISCOPE_2023H_RECONSTRUCT_H
#define IIT6_OSCILLISCOPE_2023H_RECONSTRUCT_H

#include "2023h_signal_seperate.h"

void Reconstruct2023H_Output(const SignalSeparationResult *result);
SignalSeparationResult Execute_Separation_And_Reconstruct2023H(void);

#endif // IIT6_OSCILLISCOPE_2023H_RECONSTRUCT_H
