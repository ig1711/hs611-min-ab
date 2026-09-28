/*
 * estimator.h — sub-pixel position estimators (runtime-selectable).
 *
 *   ESTIMATOR_NCENTROID  N-point centroid over all window coils after
 *                        subtracting an additive noise base (no calibration).
 *   ESTIMATOR_VENDOR     the vendor rational interpolator with a per-channel
 *                        calibration table and amplitude-band corrections.
 *   ESTIMATOR_GAOMON     Gaomon-style 5-point parabolic + 3-point linear,
 *                        with the shared noise base subtracted as a floor.
 *   ESTIMATOR_LOGAUSS    3-point log-Gaussian, amplitude-independent.
 *
 * All return the position in 1/256-channel units (1-based).
 */

#ifndef MIN_ESTIMATOR_H
#define MIN_ESTIMATOR_H

#include <stdint.h>

#define ESTIMATOR_NCENTROID   0U
#define ESTIMATOR_VENDOR      1U
#define ESTIMATOR_GAOMON      2U
#define ESTIMATOR_LOGAUSS     3U

/* Peak amplitude at/above which the vendor rejects the read (0xF3C). */
#define ESTIMATOR_VENDOR_SAT  0xF3CU

/* Initialize the calibration table to a uniform pitch. */
void estimator_init(void);

void estimator_set(uint8_t mode);
uint8_t estimator_get(void);

/* Position of the peak within w[0..m-1] (consecutive coils starting at 1-based
 * channel base+1). `axis` selects the per-axis log-Gaussian coefficients
 * (0 = X / axis B, 1 = Y / axis A). */
int32_t estimator_position(const uint16_t *w, uint8_t m, uint8_t base, uint8_t axis);

/* Additive noise base (counts), shared by the N-point centroid and Gaomon. */
void estimator_set_ncentroid(uint16_t noise);
uint16_t estimator_get_ncentroid(void);

/* Log-Gaussian baseline and per-axis Q8 coefficients. */
void estimator_set_loggauss(uint8_t axis, uint16_t baseline, int16_t a1_q8, int16_t a3_q8);
uint16_t estimator_get_loggauss_base(void);
int16_t estimator_get_loggauss_a1(uint8_t axis);
int16_t estimator_get_loggauss_a3(uint8_t axis);

#endif /* MIN_ESTIMATOR_H */
