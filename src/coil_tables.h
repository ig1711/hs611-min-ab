/*
 * coil_tables.h — receive-coil mux tables.
 *
 * Each entry is 6 bytes: [index u16][GPIOB ODR mask u16][GPIOC ODR value u16].
 * A measurement selects one receive loop by writing
 *   GPIOB->ODR = (ODR | MUX_B_IDLE_MASK) & mask_b;
 *   GPIOC->ODR = (ODR & MUX_C_KEEP_MASK) | val_c;
 * (see afe_mux()).
 *
 * Extracted from the vendor init data:
 *   axis A (27 loops, Y on this firmware) RAM 0x2000081A
 *   axis B (41 loops, X on this firmware) RAM 0x20000724
 */

#ifndef MIN_COIL_TABLES_H
#define MIN_COIL_TABLES_H

#include <stdint.h>
#include "board.h"

extern const uint8_t axis_a_table[AXIS_A_N * 6U];
extern const uint8_t axis_b_table[AXIS_B_N * 6U];

/* Table entry field accessors (entry points at byte 0 of the 6-byte record). */
static inline uint16_t coil_mask(const uint8_t *entry) { return (uint16_t)(entry[2] | (entry[3] << 8)); }
static inline uint16_t coil_value(const uint8_t *entry) { return (uint16_t)(entry[4] | (entry[5] << 8)); }
static inline const uint8_t *coil_entry(const uint8_t *table, uint8_t idx) {
    return table + (uint16_t)(idx - 1U) * 6U;   /* idx is 1-based */
}

#endif /* MIN_COIL_TABLES_H */
