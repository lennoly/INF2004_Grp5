/* Exercise the actual vehicle junction decision without robot hardware.
 * Whole-program optimisation discards unrelated hardware routines. */
#include <stdio.h>
#include "../../../app_program/vehicle.c"

int main(void)
{
    unsigned checks = 0u;
    for (int cmd = NAV_LEFT; cmd <= NAV_UTURN; cmd++)
    {
        for (int line = LINE_ON; line <= LINE_JUNCTION; line++)
        {
            g_st.pending_nav = (barcode_decode_nav_t) cmd;
            g_st.line_state = (line_follow_state_t) line;
            if (nav_due() != (line == LINE_JUNCTION))
            {
                printf("FAIL: cmd=%d line=%d\n", cmd, line);
                return 1;
            }
            checks++;
        }
    }
    g_st.pending_nav = NAV_NONE;
    g_st.line_state = LINE_JUNCTION;
    if (nav_due()) { return 1; }
    printf("navigation: %u junction decisions passed\n", checks + 1u);
    return 0;
}
