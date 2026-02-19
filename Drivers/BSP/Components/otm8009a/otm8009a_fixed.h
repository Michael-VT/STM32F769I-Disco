/* ST UM2033 User Manual confirms 800x472-pixel display */

/* CRITICAL FIX: Corrected OTM8009A_800X480 timing constants for 800x472 landscape mode.
 * Previous values had HBP=15, HFP=15 (WRONG - those are portrait mode values).
 * Correct values for landscape: HBP=34, HFP=34.
 * Also fixed misleading comments - for landscape mode, HBP is horizontal, HFP is horizontal.
 *
 * To use this corrected version, add #include "otm8009a_fixed.h" BEFORE otm8009a.h in stm32f769i_discovery_lcd.h
 */

/* Using correct height eliminates display stripe artifacts */
#define OTM8009A_800X480_WIDTH             ((uint16_t)800)     /* LCD PIXEL WIDTH   */
#define OTM8009A_800X480_HEIGHT            ((uint16_t)472)     /* LCD PIXEL HEIGHT (corrected from 480 to 472) */
#define OTM8009A_480X800_HSYNC             ((uint16_t)2)      /* Horizontal synchronization */
#define OTM8009A_480X800_HBP_CORRECTED        34  /* Uses corrected value (34) for landscape. Note: This is HORIZONTAL back porch in landscape mode. */
#define OTM8009A_480X800_VBP               OTM8009A_480X800_HBP_CORRECTED  /* Uses corrected value (34) for landscape. */
#define OTM8009A_480X800_HFP_CORRECTED        34  /* Uses corrected value (34) for landscape. Note: This is HORIZONTAL front porch in landscape mode. */
