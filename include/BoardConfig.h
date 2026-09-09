// Selects the active board's hardware definition based on a `BOARD_*` preprocessor define set in
// platformio.ini's build_flags for the environment being built. Application code should include
// this header (not any boards/board_*.h directly) and use crowpanel::board::kWhatever - never
// hardcode a resolution, pin number, or product name inline. See boards/board_template.h for the
// full list of what a board must define and how to add a new one.
#pragma once

#if defined(BOARD_CROWPANEL_4_2)
#include "boards/board_crowpanel_4_2.h"
#else
#error "No board selected - define a BOARD_* macro (e.g. BOARD_CROWPANEL_4_2) in platformio.ini's build_flags. See firmware/include/boards/board_template.h."
#endif
