#include "App.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ti_msp_dl_config.h"

#include "AD9959.h"
#include "AdcCapture.h"
#include "BTN.h"
#include "DDS.h"
#include "DacOutput.h"
#include "Encoder.h"
#include "RGBLED.h"
#include "SoundLight.h"
#include "Tick.h"
#include "TSL1401.h"
#include "UserUART.h"

#if USER_UART_DEBUG_PORT == 0
#define APP_UART_PRINTF UART0_printf
#elif USER_UART_DEBUG_PORT == 2
#define APP_UART_PRINTF UART2_printf
#endif

#define APP_COMMAND_SIZE 64U
#define F_FREQ_MIN_HZ 1000U
#define F_FREQ_MAX_HZ 100000U
#define F_FREQ_STEP_HZ 100U
#define F_PHASE_90_WORD 4096U
#define F_PHASE_MAX_WORD 16383U
/*
 * Infinity-mode display calibration.  The mathematical target is
 * output = sin(2wt + 2phi), but the complete PFD/DDS/output/scope chain
 * displays a parabola at a raw zero offset.  Bench verification at 33 kHz
 * showed that a -90-degree correction at 2f (equivalent to -45 degrees in
 * fundamental time phase) produces the required symmetric infinity trace.
 */
#define F_INFINITY_PHASE_LOW_MAX_HZ 2000U
#define F_INFINITY_PHASE_NORMAL_HZ 2100U
#define F_INFINITY_PHASE_LOW_WORD 11200U
#define F_INFINITY_PHASE_WORD 12288U
#define F_PROBE_DEFAULT_ASF 220U
#define F_RAMP_MIN_CODE 256U
#define F_RAMP_MAX_CODE 3840U
/*
 * The installed PA15 path is unipolar and the scope centre must remain fixed.
 * Use nearly the full DAC range, then spread the inactive interval over narrow
 * 64-code bands outside the useful range. This keeps the AC average at
 * midscale without painting bright single-pixel idle lines.
 */
#define F_WINDOW_RAMP_MIN_CODE 64U
#define F_WINDOW_RAMP_MAX_CODE 4031U
#define F_WINDOW_IDLE_LOW_CODE 0U
#define F_WINDOW_IDLE_HIGH_CODE 4095U
#define F_WINDOW_IDLE_BAND_CODES 64U
#define F_DK_TIMEOUT_MS 3000U
#define F_RELAY_SETTLE_MS 20U
#define F_AMP_ANCHOR_COUNT 6U
#define F_CIRCLE_PHASE_ANCHOR_COUNT 5U
#define PLOCK_DEFAULT_ASF 512U
#define PLOCK_ADC_TARGET 3072U
#define PLOCK_ADC_RATE_HZ 200000U
#define PLOCK_PERIOD_MS 20U
#define PLOCK_KP 4
#define PLOCK_KI_Q8 13
#define PLOCK_I_LIMIT 1024
#define PLOCK_STEP_LIMIT 2048
#define PLOCK_REACQUIRE_STEP_LIMIT 1024
#define PLOCK_HOLD_KP 2
#define PLOCK_HOLD_KI_Q8 8
#define PLOCK_HOLD_I_LIMIT 1024
#define PLOCK_HOLD_DEADBAND 4
#define PLOCK_HOLD_STEP_LIMIT 1024
#define PLOCK_FTW_TRIM_SAMPLES 100U
#define PLOCK_FTW_TRIM_MAX_ERROR 128U
#define PLOCK_FTW_TRIM_TOTAL_LIMIT 10240
/*
 * At a 25 MHz DDS clock and a 20 ms control period:
 *   delta_FTW = phase_step * 262144 / 500000.
 */
#define PLOCK_FTW_TRIM_NUMERATOR 262144L
#define PLOCK_FTW_TRIM_DENOMINATOR 500000L
#define PLOCK_LOCK_ERROR 32
#define PLOCK_HOLD_ERROR 768
#define PLOCK_LOCK_RIPPLE 320U
#define PLOCK_HOLD_RIPPLE 420U
#define PLOCK_VERY_LOW_FREQ_HZ 2000U
#define PLOCK_VERY_LOW_LOCK_RIPPLE 2200U
#define PLOCK_VERY_LOW_HOLD_RIPPLE 2600U
#define PLOCK_LOW_FREQ_HZ 5000U
#define PLOCK_LOW_LOCK_RIPPLE 700U
#define PLOCK_LOW_HOLD_RIPPLE 900U
#define PLOCK_LOCK_SAMPLES 25U
#define PLOCK_HOLD_SAMPLES 5U
#define PLOCK_SCAN_COARSE_STEP 512U
#define PLOCK_SCAN_COARSE_COUNT 33U
#define PLOCK_SCAN_FINE_STEP 64U
#define PLOCK_SCAN_FINE_RADIUS 512U
#define PLOCK_SCAN_FINE_COUNT 17U
#define PLOCK_SCAN_SETTLE_MS 2U
#define PLOCK_SCAN_MIN_SPAN 400U
#define PLOCK_RECALIBRATE_SAMPLES 110U
#define F_FREQ_CAPTURE_LOW_RATE_HZ 200000U
#define F_FREQ_CAPTURE_HIGH_RATE_HZ 500000U
#define F_FREQ_CAPTURE_HIGH_BAND_HZ 40000U
#define F_FREQ_CAPTURE_MIN_SPAN 100U
#define F_FREQ_CAPTURE_TIMEOUT_MS 50U
#define CCD_AUTO_FRAME_US 10000U
#define CCD_FREQ_IDLE_CODE DAC_OUTPUT_MIDCODE
#define CCD_AUTO_THRESHOLD_PERCENT 25U
#define CCD_AUTO_SEARCH_THRESHOLD_PERCENT 40U
#define CCD_PHASE_THRESHOLD_PERCENT 40U
#define CCD_AUTO_SETTLE_MS 35U
#define CCD_AUTO_FREQ_SETTLE_MS 100U
#define CCD_AUTO_SEARCH_EXPOSURE_MS 40U
#define CCD_FINE_SEARCH_RADIUS_HZ 600U
#define CCD_FINE_PRIME_MS 500U
#define CCD_FINE_MIN_WIDTH 18U
#define CCD_FINE_DIFF_EXPOSURE_MS 40U
#define CCD_FINE_DIFF_ACTIVE_SETTLE_MS 500U
#define CCD_FINE_DIFF_THRESHOLD_PERCENT 40U
#define CCD_FINE_DIFF_GOOD_SCORE 2000U
#define CCD_FINE_DIFF_LOW_RADIUS_HZ 700U
#define CCD_FINE_DIFF_MID_RADIUS_HZ 1500U
#define CCD_FINE_DIFF_HIGH_RADIUS_HZ 2000U
#define CCD_AUTO_MIN_PEAKS 1U
#define CCD_AUTO_MAX_PEAKS 6U
#define CCD_FREQ_MIN_POINTS 4U
#define CCD_FREQ_MAX_POINTS 16U
#define CCD_FREQ_MIN_PROMINENCE 30U
#define CCD_FREQ_PROMINENCE_DIVISOR 25U
#define CCD_FREQ_PEAK_RADIUS 5U
#define CCD_FREQ_MIN_PEAK_DISTANCE 6U
#define CCD_FREQ_ROI_FIRST_PIXEL 5U
#define CCD_FREQ_ROI_LAST_PIXEL 102U
#define CCD_FREQ_SETTLE_MS 500U
#define CCD_FREQ_EXPOSURE_MS 40U
#define CCD_FREQ_HIGH_EXPOSURE_MS CCD_FREQ_EXPOSURE_MS
#define CCD_FREQ_SAMPLE_FRAMES 10U
#define CCD_FREQ_WARMUP_FRAMES 2U
#define CCD_FREQ_MIN_CONSENSUS_FRAMES 3U
#define CCD_FREQ_CONSENSUS_TOLERANCE_HZ 100U
#define CCD_FREQ_MIN_TRAIN_SPAN 60U
#define CCD_FREQ_HIGH_MIN_FIT_POINTS 5U
#define CCD_FREQ_HIGH_CONSENSUS_PERCENT 5U
#define CCD_FREQ_MODEL_AGREEMENT_PERCENT 5U
#define CCD_FREQ_MODEL_SUPPORT_ADVANTAGE 2U
#define CCD_FREQ_SPLIT_REJECT_PERCENT 5U
#define CCD_FREQ_RAMP_VERIFY_MIN_SPAN 30U
#define CCD_FREQ_100US_DEFER_BELOW_HZ 25000U
#define CCD_FREQ_200US_LOW_MIN_HZ 8000U
#define CCD_FREQ_200US_LOW_MAX_HZ 12000U
#define CCD_FREQ_HIGH_ROUTE_HZ 85000U
#define CCD_FREQ_BOUNDARY_VERIFY_HZ 98000U
#define CCD_FREQ_BOUNDARY_OVERSHOOT_HZ 110000U
#define CCD_FREQ_BOUNDARY_PAIR_PERCENT 3U
#define CCD_FREQ_BOUNDARY_CROSS_PERCENT 8U
#define CCD_FREQ_CONFIRM_MIN_HZ 85000U
#define CCD_FREQ_CONFIRM_SETTLE_MS 1000U
#define CCD_FREQ_CONFIRM_AGREEMENT_PERCENT 2U
#define CCD_FREQ_50US_LOW_CAL_MIN_HZ 35000U
#define CCD_FREQ_50US_LOW_CAL_MAX_HZ 65000U
#define CCD_FREQ_50US_LOW_CAL_NUMERATOR 960U
#define CCD_FREQ_50US_LOW_CAL_DENOMINATOR 1000U
#define CCD_FREQ_50US_MID_CAL_MIN_HZ 65001U
#define CCD_FREQ_50US_MID_CAL_MAX_HZ 84999U
#define CCD_FREQ_50US_MID_CAL_NUMERATOR 961U
#define CCD_FREQ_50US_MID_CAL_DENOMINATOR 1000U
#define CCD_FREQ_HIGH_CAL_MIN_HZ 85000U
#define CCD_FREQ_HIGH_CAL_MAX_HZ 97999U
#define CCD_FREQ_HIGH_CAL_NUMERATOR 960U
#define CCD_FREQ_HIGH_CAL_DENOMINATOR 1000U
/*
 * PA15 and/or the scope input is AC coupled, so static DAC endpoint codes all
 * collapse onto the same horizontal center line and cannot calibrate the
 * optical span. Complete ramp frames measure the installed geometry at
 * 82.25 pixels; keep the quarter pixel in Q8 for the subpixel fit.
 */
#define CCD_FREQ_ACTIVE_SPAN_Q8 21056U
/*
 * The two reliable V37 high-band observations had the same scale error:
 * 50.000 kHz -> 49.3 kHz and 100.001 kHz -> 98.6 kHz. Correct that 1.42
 * percent optical compression only for the 20/50/100 us windows; longer
 * windows retain the independently validated 82.25-pixel geometry.
 */
#define CCD_FREQ_HIGH_ACTIVE_SPAN_Q8 21355U
#define CCD_FREQ_CLUSTER_THRESHOLD_PERCENT 10U
#define CCD_AUTO_TARGET_PEAKS 2U
#define CCD_AUTO_BAD_SCORE 1000000UL
#define CCD_TRACK_INTERVAL_MS 40U
#define CCD_TRACK_EXPOSURE_MS 40U
#define CCD_TRACK_PHASE_STEP 64U
#define CCD_TRACK_PHASE_COARSE_STEP 512U
#define CCD_TRACK_LINE_TIGHT_SCORE 1500U
#define CCD_TRACK_GOOD_LINE_SCORE 4500U
#define CCD_TRACK_GOOD_CURVE_SCORE 4000U
#define CCD_PHASE_STRONG_LINE_SCORE 4000U

typedef enum FMode {
    F_MODE_OFF = 0,
    F_MODE_THRU,
    F_MODE_SAME,
    F_MODE_QUAD,
    F_MODE_DOUBLE,
    F_MODE_PROBE4,
    F_MODE_RAMP,
    F_MODE_AUTO_LINE,
    F_MODE_AUTO_CIRCLE,
    F_MODE_AUTO_INFINITY
} FMode;

typedef enum FState {
    F_STATE_READY = 0,
    F_STATE_WAIT_DK,
    F_STATE_SEARCHING,
    F_STATE_LOCKING,
    F_STATE_STABLE,
    F_STATE_ERROR
} FState;

typedef struct FControl {
    FMode mode;
    FMode autoTarget;
    FState state;
    uint32_t inputFreqHz;
    uint32_t outputFreqHz[4];
    uint16_t outputAmp[4];
    uint16_t outputPhase[4];
    uint16_t singlePhaseWord;
    uint8_t targetDiv;
    uint32_t requestId;
    uint32_t lastDKTick;
} FControl;

typedef enum PhaseLockOutputMode {
    PLOCK_OUTPUT_OFF = 0,
    PLOCK_OUTPUT_SAME,
    PLOCK_OUTPUT_DOUBLE
} PhaseLockOutputMode;

typedef enum PhaseLockScanState {
    PLOCK_SCAN_IDLE = 0,
    PLOCK_SCAN_COARSE,
    PLOCK_SCAN_FINE_ZERO,
    PLOCK_SCAN_FINE_LOCK
} PhaseLockScanState;

typedef struct PhaseLockControl {
    bool enabled;
    bool capturePending;
    bool locked;
    bool phaseCalibrated;
    uint32_t frequencyHz;
    uint32_t nominalFTW;
    uint32_t referenceFTW;
    int32_t frequencyTrimFTW;
    uint16_t amplitude;
    uint16_t phaseWord;
    uint16_t adcTarget;
    int8_t controlSign;
    PhaseLockScanState scanState;
    uint8_t scanIndex;
    uint8_t calibrationPass;
    uint16_t calibrationHoldSamples;
    uint32_t scanSetTick;
    int32_t scanStartWord;
    uint16_t coarseZeroWord;
    uint16_t coarseAnchorWord;
    uint16_t coarseFixedTargetWord;
    bool fixedTargetValid;
    uint16_t phaseZeroWord;
    uint16_t phaseAnchorWord;
    uint16_t fixedTargetPhaseWord;
    uint16_t zeroCorrection;
    uint16_t adcMean;
    uint16_t adcMin;
    uint16_t adcMax;
    int16_t error;
    int32_t integratorQ8;
    int16_t lastStep;
    uint16_t stableSamples;
    uint8_t holdBadSamples;
    uint32_t updateCount;
    int32_t trimStepSum;
    uint16_t trimSampleCount;
    uint32_t lastCaptureTick;
    PhaseLockOutputMode outputMode;
    bool outputArmed;
    bool outputConnected;
    uint16_t outputAmplitude;
    uint16_t outputOffset;
    uint16_t outputPhase;
} PhaseLockControl;

static bool gRainbowEnabled;
static bool gDDSInitialized;
static bool gRelayDDSSelected;
static uint32_t gDDSRecoveryCount;
static char gCommand[APP_COMMAND_SIZE];
static FControl gFControl;
static PhaseLockControl gPhaseLock;
static uint16_t gPhaseScanCoarse[PLOCK_SCAN_COARSE_COUNT];
static uint16_t gPhaseScanFine[PLOCK_SCAN_FINE_COUNT];
static uint16_t gCCDPixels[TSL1401_PIXEL_COUNT];
static uint16_t gCCDBackground[TSL1401_PIXEL_COUNT];
static uint16_t gCCDCorrected[TSL1401_PIXEL_COUNT];
static uint32_t gCCDActiveSpanQ8 = CCD_FREQ_ACTIVE_SPAN_Q8;
static uint32_t gCCDExposureMs = 5U;
static uint32_t gCCDTrackTick;
static int8_t gCCDTrackDirection;
static bool gCCDTrackTrialActive;
static uint16_t gCCDTrackTrialBasePhase;
static uint32_t gCCDTrackTrialBaseScore;
static uint8_t gCCDTrackBadFrames;
static const uint32_t gAmpAnchorHz[F_AMP_ANCHOR_COUNT] = {
    1000U, 1500U, 2000U, 3000U, 5000U, 10000U
};
static const uint16_t gAmpAnchorAsf[4][F_AMP_ANCHOR_COUNT] = {
    {230U, 176U, 152U, 132U, 121U, 116U},
    {464U, 365U, 312U, 274U, 251U, 240U},
    {704U, 554U, 480U, 415U, 380U, 364U},
    {940U, 744U, 642U, 558U, 511U, 489U}
};
/*
 * Manual-mode starting offsets. The encoder remains authoritative and can
 * trim these values in 64-word steps after the output mode is selected.
 */
static const uint32_t
gCirclePhaseAnchorHz[F_CIRCLE_PHASE_ANCHOR_COUNT] = {
    1000U, 5000U, 10000U, 50000U, 100000U
};
static const uint16_t
gCirclePhaseAnchorWord[F_CIRCLE_PHASE_ANCHOR_COUNT] = {
    1813U, 3375U, 3666U, 3863U, 3740U
};
/*
 * Calibrated CH0 phase offsets for a 90-degree CH1-to-CH2 relationship.
 * Signed interpolation is used because the small high-frequency correction
 * reverses direction between 50 kHz and 100 kHz.
 */
static void processCommand(char *command);
static bool processFCommand(const char *name, size_t nameLen, const char *args);
static void showHelp(void);
static void showStatus(void);
static void showFHelp(void);
static void showFStatus(void);
static void serviceControls(void);
static void servicePhaseLock(void);
static void serviceADC(void);
static void serviceFWatchdog(void);
static void serviceCCDVisualLock(void);
static void phaseLockStart(uint32_t freqHz, uint16_t amp);
static void phaseLockStop(bool turnOffDDS);
static void phaseLockProcessADC(
    uint16_t mean, uint16_t minimum, uint16_t maximum);
static void phaseLockBeginCalibration(void);
static void phaseLockProcessCalibrationADC(
    uint16_t mean, uint16_t minimum, uint16_t maximum);
static bool phaseLockAnalyzeCoarse(void);
static bool phaseLockFinishFineScan(void);
static bool phaseLockFindCrossing(const uint16_t *samples, uint8_t count,
    int32_t startWord, uint16_t stepWord, uint16_t target,
    int8_t preferredSlope, uint16_t maximumDelta,
    uint16_t *phaseWord, int8_t *slopeSign);
static void phaseLockSetScanPhase(uint16_t phaseWord);
static uint16_t phaseLockWrapWord(int32_t phaseWord);
static void phaseLockServiceFTWTrim(
    uint32_t absoluteError, uint16_t ripple, uint16_t holdRippleLimit);
static void phaseLockApplyReferenceFTW(void);
static void phaseLockArmOutput(
    PhaseLockOutputMode mode, uint16_t amp, uint16_t offset);
static void phaseLockSetOutputAmplitude(uint16_t amp);
static void phaseLockSetOutputOffset(uint16_t offset);
static void phaseLockConnectOutput(void);
static void phaseLockDisconnectOutput(void);
static void phaseLockUpdateOutputPhase(void);
static uint16_t phaseLockCircleOutputOffset(void);
static uint16_t phaseLockInfinityOutputOffset(void);
static uint16_t circlePhaseOffset(uint32_t inputFreqHz);
static void phaseLockSelectDirect(void);
static void showPhaseLockStatus(void);
static void captureCCD(
    uint32_t exposureMs, uint8_t thresholdPercent, bool dumpPixels);
static bool runCCDFrequencyRecognition(void);
static bool runCCDFineFrequency(uint32_t centerFrequencyHz);
static bool runCCDAutoMode(FMode mode);
static bool ccdEstimateFrequency(uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint32_t *frequencyHz, uint32_t *windowUs, uint8_t *peakCount);
static bool ccdEstimateFrequencyConfirmed(uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint32_t *frequencyHz, uint32_t *windowUs, uint8_t *peakCount);
static uint32_t ccdEvaluateFrequency(uint32_t frequencyHz,
    uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT]);
static uint32_t ccdEvaluateFrequencyDifferential(uint32_t frequencyHz);
static uint32_t ccdSearchFrequency(uint32_t coarseFrequencyHz,
    uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT]);
static uint32_t ccdSearchPass(uint32_t centerFrequencyHz,
    uint32_t radiusHz, uint32_t stepHz, uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint32_t *bestScore);
static uint16_t ccdSearchPhase(FMode mode, uint32_t inputFrequencyHz,
    uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT]);
static uint32_t ccdEvaluatePhase(FMode mode, uint32_t inputFrequencyHz,
    uint16_t phaseWord, uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT]);
static uint32_t ccdMeasureShapeScore(FMode mode, uint32_t exposureMs);
static uint32_t ccdStatsWidth(const TSL1401_Stats *stats);
static uint8_t ccdCountProminentPeaks(
    const uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint8_t centers[TSL1401_MAX_PEAKS],
    uint16_t *prominenceThreshold);
static uint8_t ccdSelectRegularPeaks(
    uint8_t centers[TSL1401_MAX_PEAKS], uint8_t count,
    uint8_t *minimumGap, uint8_t *maximumGap);
static uint32_t __attribute__((noinline)) ccdEstimateSubpixelFit(
    const uint16_t pixels[TSL1401_PIXEL_COUNT],
    const uint8_t centers[TSL1401_MAX_PEAKS], uint8_t count,
    uint32_t windowUs);
static uint32_t __attribute__((noinline)) ccdRefineFrequencyComb(
    const uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint32_t coarseFrequencyHz, uint32_t windowUs);
static bool __attribute__((noinline)) ccdSelectFrequencyConsensus(
    const uint32_t candidates[CCD_FREQ_SAMPLE_FRAMES],
    uint8_t candidateCount, uint32_t *frequencyHz, uint8_t *support);
static bool ccdSelectRelativeConsensus(
    const uint32_t candidates[CCD_FREQ_SAMPLE_FRAMES],
    uint8_t candidateCount, uint8_t tolerancePercent,
    uint32_t *frequencyHz, uint8_t *support,
    uint32_t *minimumHz, uint32_t *maximumHz);
static uint32_t ccdCalibrateShortWindowFrequency(
    uint32_t frequencyHz, uint32_t windowUs);
static uint32_t roundFrequency100(uint32_t frequencyHz);
static uint32_t roundFrequency100WithHint(
    uint32_t frequencyHz, uint32_t hintHz);
static bool measureInputFrequency(uint32_t *frequencyHz);
static bool captureInputFrequency(
    uint32_t sampleRateHz, uint32_t *frequencyHz);
static void ensureDDS(void);
static void forceDDSReinitialize(void);
static bool recoverDDSAfterDAC(void);
static void ddsAllOff(void);
static void relaySelectDirect(void);
static void relaySelectDDS(void);
static void applySingleOutput(uint32_t freqHz, uint16_t amp, uint16_t phase);
static void applyPFDReference(uint32_t freqHz, uint16_t amp, uint16_t phase);
static bool applyManualMode(FMode mode);
static bool applyProbe4(const uint32_t freqHz[4], uint16_t amp);
static void startAutoMode(FMode mode);
static void requestDKAutoMode(FMode mode, const char *keyName);
static void setFState(FState state);
static uint16_t ampFromDiv(uint8_t div, uint32_t outputFreqHz);
static const char *fModeName(FMode mode);
static const char *fStateName(FState state);
static bool parseU32(const char **text, uint32_t *value);
static const char *skipSpaces(const char *text);

void App_init(void)
{
    SoundLight_init();
    BTN_init();
    ENC_init();
    AdcCapture_init();
    DacOutput_init();
    TSL1401_init();

    /*
     * The DDS supply can rise after the MCU supply. Keep every AD9959 control
     * line high impedance until the first real DDS initialization so the MCU
     * cannot phantom-power the external board through its input clamps.
     */
    AD9959_busHiZ();

    gRainbowEnabled = false;
    gDDSInitialized = false;
    gRelayDDSSelected = false;
    gDDSRecoveryCount = 0U;
    DL_GPIO_clearPins(GPIO_RELAY_PORT, GPIO_RELAY_CTRL_PIN);
    memset(&gFControl, 0, sizeof(gFControl));
    memset(&gPhaseLock, 0, sizeof(gPhaseLock));
    gFControl.mode = F_MODE_OFF;
    gFControl.autoTarget = F_MODE_OFF;
    gFControl.state = F_STATE_READY;
    gFControl.inputFreqHz = 10000U;
    gFControl.targetDiv = 8U;
    gFControl.singlePhaseWord = 0U;
    gFControl.lastDKTick = Tick_now();
    gCCDTrackDirection = -1;
    gCCDTrackTrialActive = false;
    gCCDTrackBadFrames = 0U;
    RGBLED_setColor(0U, 24U, 0U);
    SoundLight_play(SOUND_LIGHT_CUE_POWER_ON);

    UART0_sendStr("\r\n2026 F Lissajous MCU READY\r\n");
    UART0_sendStr("MSPM0G3507 / 80 MHz / SDK 2.10.00.04\r\n");
    UART0_sendStr(
        "Buttons: UP=auto-frequency lock LEFT=direct "
        "MID=circle RIGHT=infinity\r\n");
    UART0_sendStr(
        "MID/RIGHT require an active electrical lock; press UP first.\r\n");
    UART0_sendStr("Type fhelp for the F-controller protocol.\r\n");
}

void App_loop(void)
{
    UserUART_task();

    if (UserUART_readLine(gCommand, sizeof(gCommand))) {
        processCommand(gCommand);
    }

    serviceControls();
    servicePhaseLock();
    serviceADC();
    serviceCCDVisualLock();
    serviceFWatchdog();

    if (gRainbowEnabled) {
        RGBLED_taskRainbow();
    }

    /*
     * Add the 2026 problem state machine here. Keep interrupt handlers short;
     * move calculations, display updates, and command processing into this loop.
     */
}

static void processCommand(char *command)
{
    const char *args;
    uint32_t first;
    uint32_t second;
    uint32_t third;

    command = (char *)skipSpaces(command);
    args = command;
    while ((*args != '\0') && (*args != ' ')) {
        args++;
    }

    if (processFCommand(command, (size_t)(args - command), args)) {
        return;
    } else if (((size_t)(args - command) == 4U) &&
        (strncmp(command, "help", 4U) == 0) && (*args == '\0')) {
        showHelp();
    } else if (((size_t)(args - command) == 6U) &&
        (strncmp(command, "status", 6U) == 0) && (*args == '\0')) {
        showStatus();
    } else if (((size_t)(args - command) == 3U) &&
        (strncmp(command, "led", 3U) == 0)) {
        if (parseU32(&args, &first) && parseU32(&args, &second) &&
            parseU32(&args, &third) &&
            (*skipSpaces(args) == '\0') &&
            (first <= 255U) && (second <= 255U) && (third <= 255U)) {
            gRainbowEnabled = false;
            RGBLED_setColor(
                (uint8_t)first, (uint8_t)second, (uint8_t)third);
            UserUART_write("OK led\r\n");
        } else {
            UserUART_write("ERR usage: led R G B\r\n");
        }
    } else if (((size_t)(args - command) == 5U) &&
        (strncmp(command, "sound", 5U) == 0)) {
        args = skipSpaces(args);
        if (strcmp(args, "key") == 0) {
            SoundLight_play(SOUND_LIGHT_CUE_KEY);
            UserUART_write("OK sound key pa1\r\n");
        } else if (strcmp(args, "start") == 0) {
            SoundLight_play(SOUND_LIGHT_CUE_START);
            UserUART_write("OK sound start pa1\r\n");
        } else if (strcmp(args, "success") == 0) {
            SoundLight_play(SOUND_LIGHT_CUE_SUCCESS);
            UserUART_write("OK sound success pa1\r\n");
        } else if (strcmp(args, "error") == 0) {
            SoundLight_play(SOUND_LIGHT_CUE_ERROR);
            UserUART_write("OK sound error pa1\r\n");
        } else if (strcmp(args, "off") == 0) {
            SoundLight_forceOff();
            UserUART_write("OK sound off pa1\r\n");
        } else {
            UserUART_write(
                "ERR usage: sound key|start|success|error|off\r\n");
        }
    } else if (((size_t)(args - command) == 7U) &&
        (strncmp(command, "rainbow", 7U) == 0)) {
        args = skipSpaces(args);
        if (strcmp(args, "on") == 0) {
            gRainbowEnabled = true;
            UserUART_write("OK rainbow on\r\n");
        } else if (strcmp(args, "off") == 0) {
            gRainbowEnabled = false;
            UserUART_write("OK rainbow off\r\n");
        } else {
            UserUART_write("ERR usage: rainbow on|off\r\n");
        }
    } else if (((size_t)(args - command) == 3U) &&
        (strncmp(command, "ccd", 3U) == 0)) {
        const char *ccdArgs = skipSpaces(args);
        bool dumpPixels = false;

        if (strcmp(ccdArgs, "status") == 0) {
            UserUART_write(
                "CCD_STATUS pixels=128 ao=PA14 adc=ADC0_CH12 "
                "clk=PA17 si=PA18 pa27=preserved\r\n");
            return;
        }
        if (strcmp(ccdArgs, "freq") == 0) {
            (void)runCCDFrequencyRecognition();
            return;
        }
        if (strncmp(ccdArgs, "fine", 4U) == 0) {
            ccdArgs = skipSpaces(ccdArgs + 4);
            first = gFControl.inputFreqHz;
            if ((*ccdArgs != '\0') &&
                (!parseU32(&ccdArgs, &first) ||
                 (*skipSpaces(ccdArgs) != '\0'))) {
                UserUART_write(
                    "ERR usage: ccd fine [COARSE_FREQ_HZ]\r\n");
                return;
            }
            if ((first < F_FREQ_MIN_HZ) ||
                (first > F_FREQ_MAX_HZ)) {
                UserUART_write(
                    "ERR no coarse frequency; run ccd freq first or "
                    "use ccd fine FREQ_HZ\r\n");
                return;
            }
            (void)runCCDFineFrequency(first);
            return;
        }
        if (strncmp(ccdArgs, "auto", 4U) == 0) {
            ccdArgs = skipSpaces(ccdArgs + 4);
            if (strcmp(ccdArgs, "line") == 0) {
                startAutoMode(F_MODE_AUTO_LINE);
            } else if (strcmp(ccdArgs, "circle") == 0) {
                startAutoMode(F_MODE_AUTO_CIRCLE);
            } else if (strcmp(ccdArgs, "infinity") == 0) {
                startAutoMode(F_MODE_AUTO_INFINITY);
            } else {
                UserUART_write(
                    "ERR usage: ccd auto line|circle|infinity\r\n");
            }
            return;
        }
        if (strncmp(ccdArgs, "capture", 7U) == 0) {
            ccdArgs += 7;
        } else if (strncmp(ccdArgs, "dump", 4U) == 0) {
            ccdArgs += 4;
            dumpPixels = true;
        } else {
            UserUART_write(
                "ERR usage: ccd capture [1..100ms] [10..90pct] | "
                "ccd dump [1..100ms] [10..90pct] | "
                "ccd freq | ccd fine [COARSE_FREQ_HZ] | "
                "ccd status\r\n");
            return;
        }

        first = 5U;
        second = 45U;
        if ((*skipSpaces(ccdArgs) != '\0') &&
            !parseU32(&ccdArgs, &first)) {
            UserUART_write("ERR invalid CCD exposure\r\n");
            return;
        }
        if ((*skipSpaces(ccdArgs) != '\0') &&
            !parseU32(&ccdArgs, &second)) {
            UserUART_write("ERR invalid CCD threshold\r\n");
            return;
        }
        if ((*skipSpaces(ccdArgs) != '\0') || (first < 1U) ||
            (first > 100U) || (second < 10U) || (second > 90U)) {
            UserUART_write(
                "ERR CCD range: exposure=1..100ms threshold=10..90pct\r\n");
            return;
        }

        captureCCD(first, (uint8_t)second, dumpPixels);
    } else if (((size_t)(args - command) == 3U) &&
        (strncmp(command, "adc", 3U) == 0)) {
        if (gPhaseLock.enabled) {
            UserUART_write("ERR ADC reserved by phase lock\r\n");
            return;
        }
        args = skipSpaces(args);
        if (strncmp(args, "once", 4U) != 0) {
            UserUART_write("ERR usage: adc once [rate_hz]\r\n");
            return;
        }
        args += 4;
        first = ADC_CAPTURE_DEFAULT_RATE_HZ;
        if ((*skipSpaces(args) != '\0') && !parseU32(&args, &first)) {
            UserUART_write("ERR invalid ADC rate\r\n");
            return;
        }
        first = AdcCapture_setSampleRate(first);
        if (AdcCapture_start()) {
            UserUART_printf("OK adc started rate=%lu count=%u\r\n",
                (unsigned long)first, (unsigned int)ADC_CAPTURE_SIZE);
        } else {
            UserUART_write("ERR ADC busy\r\n");
        }
    } else if (((size_t)(args - command) == 7U) &&
        (strncmp(command, "dacstat", 7U) == 0) && (*args == '\0')) {
        APP_UART_PRINTF(
            "DAC_STATUS dma_en=%u dma_size=%u dma_mode=0x%lx "
            "src=0x%08lx dst=0x%08lx dac_en=%u out_en=%u "
            "fifo_en=%u dma_trig=%u sample_timer=%u\r\n",
            (unsigned int)DL_DMA_isChannelEnabled(
                DMA, DMA_CH2_CHAN_ID),
            (unsigned int)DL_DMA_getTransferSize(
                DMA, DMA_CH2_CHAN_ID),
            (unsigned long)DL_DMA_getTransferMode(
                DMA, DMA_CH2_CHAN_ID),
            (unsigned long)DL_DMA_getSrcAddr(
                DMA, DMA_CH2_CHAN_ID),
            (unsigned long)DL_DMA_getDestAddr(
                DMA, DMA_CH2_CHAN_ID),
            (unsigned int)DL_DAC12_isEnabled(DAC0),
            (unsigned int)DL_DAC12_isOutputPinEnabled(DAC0),
            (unsigned int)DL_DAC12_isFIFOEnabled(DAC0),
            (unsigned int)DL_DAC12_isDMATriggerEnabled(DAC0),
            (unsigned int)DL_DAC12_isSampleTimeGeneratorEnabled(DAC0));
    } else if (((size_t)(args - command) == 3U) &&
        (strncmp(command, "dac", 3U) == 0)) {
        if (parseU32(&args, &first) && (*skipSpaces(args) == '\0') &&
            (first <= 4095U)) {
            DacOutput_setCode((uint16_t)first);
            UserUART_printf("OK dac code=%lu\r\n", (unsigned long)first);
        } else {
            UserUART_write("ERR usage: dac CODE(0..4095)\r\n");
        }
    } else if (((size_t)(args - command) == 3U) &&
        (strncmp(command, "dds", 3U) == 0)) {
        phaseLockStop(false);
        args = skipSpaces(args);
        if (strcmp(args, "init") == 0) {
            relaySelectDirect();
            forceDDSReinitialize();
            UserUART_write(
                "OK dds reinitialized clock_mode=refclk_bypass "
                "refclk_hz=25000000\r\n");
            return;
        } else if (strcmp(args, "off") == 0) {
            relaySelectDirect();
            UserUART_write("OK dds off relay=direct\r\n");
            return;
        }

        second = 512U;
        third = 0U;
        if (!parseU32(&args, &first) ||
            (first > (AD9959_SYSTEM_CLOCK_HZ / 2U))) {
            UserUART_write(
                "ERR usage: dds FREQ_HZ [AMP_0_1023] [PHASE_0_16383]\r\n");
            return;
        }
        if ((*skipSpaces(args) != '\0') && !parseU32(&args, &second)) {
            UserUART_write("ERR invalid DDS amplitude\r\n");
            return;
        }
        if ((*skipSpaces(args) != '\0') && !parseU32(&args, &third)) {
            UserUART_write("ERR invalid DDS phase\r\n");
            return;
        }
        if ((*skipSpaces(args) != '\0') ||
            (second > 1023U) || (third > 16383U)) {
            UserUART_write("ERR DDS parameter out of range\r\n");
            return;
        }
        /*
         * Normal frequency changes must not restart the PLL. The first DDS
         * use initializes it once; "dds init" remains the explicit recovery
         * command when a hardware reset is actually requested.
         */
        relaySelectDirect();
        (void)recoverDDSAfterDAC();
        ensureDDS();
        relaySelectDDS();
        {
            DDS_SingleToneParam_t tone;
            tone.freq = (float)first;
            tone.amp = (uint16_t)second;
            tone.phase = (uint16_t)third;
            DDS_singleTone(AD9959_CH0, &tone);
            DDS_update();
        }
        UserUART_printf("OK dds ch0 freq=%lu amp=%lu phase=%lu\r\n",
            (unsigned long)first, (unsigned long)second,
            (unsigned long)third);
    } else {
        UserUART_write("ERR unknown command; type help\r\n");
    }
}

static bool processFCommand(const char *name, size_t nameLen, const char *args)
{
    uint32_t value[5];
    uint16_t amp;
    uint32_t freq[4];
    uint32_t outputFreqHz;
    uint32_t rampPeriodUs;
    size_t rampSampleCount;
    const char *text;
    uint8_t index;
    bool rampUsesMicroseconds;

    if ((name == NULL) || (nameLen == 0U)) return false;

    if ((nameLen == 5U) && (strncmp(name, "fhelp", 5U) == 0)) {
        if (*skipSpaces(args) == '\0') {
            showFHelp();
        } else {
            UserUART_write("ERR usage: fhelp\r\n");
        }
        return true;
    }

    if ((nameLen == 7U) && (strncmp(name, "fstatus", 7U) == 0)) {
        if (*skipSpaces(args) == '\0') {
            showFStatus();
        } else {
            UserUART_write("ERR usage: fstatus\r\n");
        }
        return true;
    }

    if ((nameLen == 5U) && (strncmp(name, "fping", 5U) == 0)) {
        gFControl.lastDKTick = Tick_now();
        UserUART_printf("F_PONG uptime_ms=%lu request=%lu\r\n",
                        (unsigned long)Tick_now(),
                        (unsigned long)gFControl.requestId);
        return true;
    }

    if ((nameLen == 5U) && (strncmp(name, "plock", 5U) == 0)) {
        text = skipSpaces(args);
        if (strcmp(text, "stop") == 0) {
            phaseLockStop(true);
            UserUART_write("OK plock stopped relay=direct\r\n");
        } else if (strcmp(text, "status") == 0) {
            showPhaseLockStatus();
        } else if (strcmp(text, "circle") == 0) {
            uint16_t phaseOffset;

            if (!gPhaseLock.enabled || !gPhaseLock.phaseCalibrated ||
                !gPhaseLock.locked) {
                UserUART_write(
                    "ERR wait for phase calibration and green lock\r\n");
                return true;
            }
            amp = ampFromDiv(
                gFControl.targetDiv, gPhaseLock.frequencyHz);
            phaseOffset = phaseLockCircleOutputOffset();
            phaseLockArmOutput(PLOCK_OUTPUT_SAME, amp, phaseOffset);
            UserUART_printf(
                "OK plock circle freq=%lu div=%u amp=%u offset=%u "
                "state=%s\r\n",
                (unsigned long)gPhaseLock.frequencyHz,
                (unsigned int)gFControl.targetDiv,
                (unsigned int)amp,
                (unsigned int)phaseOffset,
                gPhaseLock.outputConnected ? "connected" : "armed");
        } else if (strncmp(text, "phase", 5U) == 0) {
            text += 5;
            if (!gPhaseLock.enabled || !gPhaseLock.outputArmed) {
                UserUART_write(
                    "ERR start phase lock and arm output before phase\r\n");
                return true;
            }
            if (!parseU32(&text, &value[0]) ||
                (*skipSpaces(text) != '\0') ||
                (value[0] > F_PHASE_MAX_WORD)) {
                UserUART_write("ERR usage: plock phase OFFSET(0..16383)\r\n");
                return true;
            }
            phaseLockSetOutputOffset((uint16_t)value[0]);
            UserUART_printf(
                "OK plock phase offset=%u phase=%u state=%s\r\n",
                (unsigned int)gPhaseLock.outputOffset,
                (unsigned int)gPhaseLock.outputPhase,
                gPhaseLock.outputConnected ? "connected" : "armed");
        } else if (strncmp(text, "output", 6U) == 0) {
            PhaseLockOutputMode outputMode;

            text = skipSpaces(text + 6);
            if (strcmp(text, "off") == 0) {
                gPhaseLock.outputArmed = false;
                gPhaseLock.outputMode = PLOCK_OUTPUT_OFF;
                phaseLockDisconnectOutput();
                UserUART_write(
                    "OK plock output off relay=direct lock=active\r\n");
                return true;
            }
            if (!gPhaseLock.enabled || !gPhaseLock.phaseCalibrated ||
                !gPhaseLock.locked) {
                UserUART_write(
                    "ERR wait for phase calibration and green lock\r\n");
                return true;
            }
            if ((strncmp(text, "same", 4U) == 0) &&
                ((text[4] == ' ') || (text[4] == '\0'))) {
                outputMode = PLOCK_OUTPUT_SAME;
                text += 4;
            } else if ((strncmp(text, "double", 6U) == 0) &&
                       ((text[6] == ' ') || (text[6] == '\0'))) {
                outputMode = PLOCK_OUTPUT_DOUBLE;
                text += 6;
            } else {
                UserUART_write(
                    "ERR usage: plock output same|double AMP OFFSET\r\n");
                return true;
            }
            if (!parseU32(&text, &value[0]) ||
                !parseU32(&text, &value[1]) ||
                (*skipSpaces(text) != '\0') ||
                (value[0] > 1023U) ||
                (value[1] > F_PHASE_MAX_WORD)) {
                UserUART_write(
                    "ERR usage: plock output same|double "
                    "AMP(0..1023) OFFSET(0..16383)\r\n");
                return true;
            }
            phaseLockArmOutput(
                outputMode, (uint16_t)value[0], (uint16_t)value[1]);
            UserUART_printf(
                "OK plock output mode=%s amp=%lu offset=%lu state=%s\r\n",
                (outputMode == PLOCK_OUTPUT_DOUBLE) ? "double" : "same",
                (unsigned long)value[0], (unsigned long)value[1],
                gPhaseLock.outputConnected ? "connected" : "armed");
        } else if (strncmp(text, "start", 5U) == 0) {
            text += 5;
            amp = PLOCK_DEFAULT_ASF;
            if (!parseU32(&text, &value[0])) {
                UserUART_write(
                    "ERR usage: plock start FREQ(1000..100000) [AMP]\r\n");
                return true;
            }
            if (*skipSpaces(text) != '\0') {
                if (!parseU32(&text, &value[1]) || (value[1] > 1023U)) {
                    UserUART_write("ERR invalid plock amplitude\r\n");
                    return true;
                }
                amp = (uint16_t)value[1];
            }
            if ((*skipSpaces(text) != '\0') ||
                (value[0] < F_FREQ_MIN_HZ) ||
                (value[0] > F_FREQ_MAX_HZ)) {
                UserUART_write(
                    "ERR usage: plock start FREQ(1000..100000) [AMP]\r\n");
                return true;
            }
            phaseLockStart(value[0], amp);
            UserUART_printf(
                "OK plock started freq=%lu amp=%u target=%u period_ms=%u\r\n",
                (unsigned long)value[0], (unsigned int)amp,
                (unsigned int)gPhaseLock.adcTarget,
                (unsigned int)PLOCK_PERIOD_MS);
        } else {
            UserUART_write("ERR usage: plock start FREQ [AMP] | "
                           "plock output same|double AMP OFFSET | "
                           "plock circle | plock phase OFFSET | "
                           "plock status|stop\r\n");
        }
        return true;
    }

    if ((nameLen == 5U) && (strncmp(name, "fmode", 5U) == 0)) {
        phaseLockStop(false);
        text = skipSpaces(args);
        if (strcmp(text, "off") == 0) {
            relaySelectDirect();
            gFControl.mode = F_MODE_OFF;
            gFControl.autoTarget = F_MODE_OFF;
            setFState(F_STATE_READY);
            UserUART_write("OK fmode off relay=direct\r\n");
        } else if (strcmp(text, "thru") == 0) {
            relaySelectDirect();
            gFControl.mode = F_MODE_THRU;
            gFControl.autoTarget = F_MODE_OFF;
            setFState(F_STATE_READY);
            UserUART_write("OK fmode thru relay=direct pa13=0\r\n");
        } else if (strcmp(text, "same") == 0) {
            gFControl.singlePhaseWord = 0U;
            (void)applyManualMode(F_MODE_SAME);
        } else if (strcmp(text, "quad") == 0) {
            gFControl.singlePhaseWord = F_PHASE_90_WORD;
            (void)applyManualMode(F_MODE_QUAD);
        } else if (strcmp(text, "double") == 0) {
            gFControl.singlePhaseWord = 0U;
            (void)applyManualMode(F_MODE_DOUBLE);
        } else {
            UserUART_write(
                "ERR usage: fmode off|thru|same|quad|double\r\n");
        }
        return true;
    }

    if ((nameLen == 5U) && (strncmp(name, "ffreq", 5U) == 0)) {
        text = args;
        if (parseU32(&text, &value[0]) &&
            (*skipSpaces(text) == '\0') &&
            (value[0] >= F_FREQ_MIN_HZ) &&
            (value[0] <= F_FREQ_MAX_HZ)) {
            gFControl.inputFreqHz = value[0];
            if ((gFControl.mode == F_MODE_SAME) ||
                (gFControl.mode == F_MODE_QUAD) ||
                (gFControl.mode == F_MODE_DOUBLE)) {
                (void)applyManualMode(gFControl.mode);
            }
            UserUART_printf("OK ffreq input_hz=%lu\r\n",
                            (unsigned long)gFControl.inputFreqHz);
        } else {
            UserUART_write("ERR usage: ffreq 1000..100000\r\n");
        }
        return true;
    }

    if ((nameLen == 4U) && (strncmp(name, "fdiv", 4U) == 0)) {
        text = args;
        if (parseU32(&text, &value[0]) &&
            (*skipSpaces(text) == '\0') &&
            ((value[0] == 2U) || (value[0] == 4U) ||
             (value[0] == 6U) || (value[0] == 8U))) {
            gFControl.targetDiv = (uint8_t)value[0];
            if (gPhaseLock.enabled && gPhaseLock.outputArmed) {
                outputFreqHz = gPhaseLock.frequencyHz;
                if (gPhaseLock.outputMode == PLOCK_OUTPUT_DOUBLE) {
                    outputFreqHz *= 2U;
                }
            } else {
                outputFreqHz = gFControl.inputFreqHz;
                if (gFControl.mode == F_MODE_DOUBLE) {
                    outputFreqHz *= 2U;
                }
            }
            amp = ampFromDiv(gFControl.targetDiv, outputFreqHz);
            if (gPhaseLock.enabled && gPhaseLock.outputArmed) {
                phaseLockSetOutputAmplitude(amp);
            } else if ((gFControl.mode == F_MODE_SAME) ||
                (gFControl.mode == F_MODE_QUAD) ||
                (gFControl.mode == F_MODE_DOUBLE)) {
                (void)applyManualMode(gFControl.mode);
            }
            UserUART_printf("OK fdiv target=%u asf=%u output_hz=%lu\r\n",
                            (unsigned int)gFControl.targetDiv,
                            (unsigned int)amp,
                            (unsigned long)outputFreqHz);
        } else {
            UserUART_write("ERR usage: fdiv 2|4|6|8\r\n");
        }
        return true;
    }

    if ((nameLen == 6U) && (strncmp(name, "fphase", 6U) == 0)) {
        phaseLockStop(false);
        text = args;
        if (parseU32(&text, &value[0]) &&
            (*skipSpaces(text) == '\0') &&
            (value[0] <= F_PHASE_MAX_WORD)) {
            gFControl.singlePhaseWord = (uint16_t)value[0];
            if ((gFControl.mode == F_MODE_SAME) ||
                (gFControl.mode == F_MODE_QUAD) ||
                (gFControl.mode == F_MODE_DOUBLE)) {
                (void)applyManualMode(gFControl.mode);
            }
            UserUART_printf("OK fphase word=%u\r\n",
                            (unsigned int)gFControl.singlePhaseWord);
        } else {
            UserUART_write("ERR usage: fphase 0..16383\r\n");
        }
        return true;
    }

    if ((nameLen == 4U) && (strncmp(name, "fopt", 4U) == 0)) {
        uint32_t minimumFTW =
            (uint32_t)((float)F_FREQ_MIN_HZ * AD9959_FTW_PER_HZ + 0.5F);
        uint32_t maximumFTW =
            (uint32_t)((float)(2U * F_FREQ_MAX_HZ) *
                AD9959_FTW_PER_HZ + 0.5F);
        bool recovered;

        phaseLockStop(false);
        text = args;
        if (parseU32(&text, &value[0]) &&
            parseU32(&text, &value[1]) &&
            parseU32(&text, &value[2]) &&
            (*skipSpaces(text) == '\0') &&
            (value[0] >= minimumFTW) &&
            (value[0] <= maximumFTW) &&
            (value[1] <= 1023U) &&
            (value[2] <= F_PHASE_MAX_WORD)) {
            /*
             * Optical PLL fast path.  Keep the relay and DDS configuration
             * untouched while changing the raw CH0 frequency tuning word and
             * phase in one I/O update.  One FTW LSB is about 0.00582 Hz, so
             * video-measured sub-hertz drift can be removed instead of chased
             * forever with phase-only corrections.
             */
            recovered = recoverDDSAfterDAC();
            relaySelectDDS();
            ensureDDS();
            AD9959_setAmp(AD9959_CH1 | AD9959_CH2 | AD9959_CH3, 0U);
            AD9959_setFTW(AD9959_CH0, value[0]);
            AD9959_setAmp(AD9959_CH0, (uint16_t)value[1]);
            AD9959_setPhase(AD9959_CH0, (uint16_t)value[2]);
            DDS_update();
            if (recovered) {
                Tick_delay(2U);
                AD9959_setFTW(AD9959_CH0, value[0]);
                AD9959_setAmp(AD9959_CH0, (uint16_t)value[1]);
                AD9959_setPhase(AD9959_CH0, (uint16_t)value[2]);
                DDS_update();
            }

            gFControl.outputFreqHz[0] =
                (uint32_t)((float)value[0] / AD9959_FTW_PER_HZ + 0.5F);
            gFControl.outputAmp[0] = (uint16_t)value[1];
            gFControl.outputPhase[0] = (uint16_t)value[2];
            gFControl.outputFreqHz[1] = 0U;
            gFControl.outputFreqHz[2] = 0U;
            gFControl.outputFreqHz[3] = 0U;
            gFControl.outputAmp[1] = 0U;
            gFControl.outputAmp[2] = 0U;
            gFControl.outputAmp[3] = 0U;
            gFControl.outputPhase[1] = 0U;
            gFControl.outputPhase[2] = 0U;
            gFControl.outputPhase[3] = 0U;
            gFControl.singlePhaseWord = (uint16_t)value[2];
            gFControl.mode = F_MODE_SAME;
            setFState(F_STATE_READY);
            gFControl.lastDKTick = Tick_now();
            UserUART_printf(
                "OK fopt ftw=%lu amp=%lu phase=%lu freq_hz=%lu\r\n",
                (unsigned long)value[0], (unsigned long)value[1],
                (unsigned long)value[2],
                (unsigned long)gFControl.outputFreqHz[0]);
        } else {
            UserUART_write(
                "ERR usage: fopt FTW AMP(0..1023) PHASE(0..16383)\r\n");
        }
        return true;
    }

    if ((nameLen == 4U) && (strncmp(name, "fset", 4U) == 0)) {
        phaseLockStop(false);
        text = args;
        if (parseU32(&text, &value[0]) &&
            parseU32(&text, &value[1]) &&
            parseU32(&text, &value[2]) &&
            (*skipSpaces(text) == '\0') &&
            (value[0] >= F_FREQ_MIN_HZ) &&
            (value[0] <= (2U * F_FREQ_MAX_HZ)) &&
            (value[1] <= 1023U) &&
            (value[2] <= F_PHASE_MAX_WORD)) {
            applySingleOutput(value[0], (uint16_t)value[1],
                              (uint16_t)value[2]);
            if ((gFControl.autoTarget == F_MODE_AUTO_LINE) ||
                (gFControl.autoTarget == F_MODE_AUTO_CIRCLE) ||
                (gFControl.autoTarget == F_MODE_AUTO_INFINITY)) {
                gFControl.mode = gFControl.autoTarget;
                setFState(F_STATE_LOCKING);
            } else {
                gFControl.mode = F_MODE_SAME;
                setFState(F_STATE_READY);
            }
            gFControl.lastDKTick = Tick_now();
            UserUART_printf(
                "OK fset freq=%lu amp=%lu phase=%lu\r\n",
                (unsigned long)value[0], (unsigned long)value[1],
                (unsigned long)value[2]);
        } else {
            UserUART_write(
                "ERR usage: fset FREQ(1000..200000) AMP(0..1023) "
                "PHASE(0..16383)\r\n");
        }
        return true;
    }

    if ((nameLen == 6U) && (strncmp(name, "pfdref", 6U) == 0)) {
        phaseLockStop(false);
        text = skipSpaces(args);
        if (strcmp(text, "off") == 0) {
            relaySelectDirect();
            UserUART_write("OK pfdref off relay=direct\r\n");
        } else if (parseU32(&text, &value[0]) &&
            parseU32(&text, &value[1]) &&
            parseU32(&text, &value[2]) &&
            (*skipSpaces(text) == '\0') &&
            (value[0] >= F_FREQ_MIN_HZ) &&
            (value[0] <= F_FREQ_MAX_HZ) &&
            (value[1] <= 1023U) &&
            (value[2] <= F_PHASE_MAX_WORD)) {
            applyPFDReference(value[0], (uint16_t)value[1],
                              (uint16_t)value[2]);
            UserUART_printf(
                "OK pfdref ch1 freq=%lu amp=%lu phase=%lu relay=direct\r\n",
                (unsigned long)value[0], (unsigned long)value[1],
                (unsigned long)value[2]);
        } else {
            UserUART_write(
                "ERR usage: pfdref FREQ(1000..100000) AMP(0..1023) "
                "PHASE(0..16383) | pfdref off\r\n");
        }
        return true;
    }

    if (((nameLen == 5U) && (strncmp(name, "framp", 5U) == 0)) ||
        ((nameLen == 7U) && (strncmp(name, "frampus", 7U) == 0))) {
        rampUsesMicroseconds = (nameLen == 7U);
        phaseLockStop(false);
        text = skipSpaces(args);
        if (strcmp(text, "off") == 0) {
            DacOutput_stop();
            gFControl.mode = F_MODE_OFF;
            setFState(F_STATE_READY);
            UserUART_write("OK framp off dac=high_z\r\n");
            return true;
        }

        if (!parseU32(&text, &value[0]) ||
            (*skipSpaces(text) != '\0')) {
            UserUART_write(
                "ERR usage: framp 1|2|5|10|off | "
                "frampus 20|50|100|200|500|1000|2000|5000|10000\r\n");
            return true;
        }
        if (rampUsesMicroseconds) {
            if ((value[0] != 20U) && (value[0] != 50U) &&
                (value[0] != 100U) && (value[0] != 200U) &&
                (value[0] != 500U) && (value[0] != 1000U) &&
                (value[0] != 2000U) && (value[0] != 5000U) &&
                (value[0] != 10000U)) {
                UserUART_write("ERR unsupported frampus window\r\n");
                return true;
            }
            rampPeriodUs = value[0];
        } else {
            if ((value[0] != 1U) && (value[0] != 2U) &&
                (value[0] != 5U) && (value[0] != 10U)) {
                UserUART_write("ERR unsupported framp window\r\n");
                return true;
            }
            rampPeriodUs = value[0] * 1000U;
        }
        rampSampleCount = (size_t)(
            ((uint64_t)rampPeriodUs * (uint64_t)DAC_OUTPUT_RATE_HZ) /
            1000000ULL);

        relaySelectDirect();
        if (AdcCapture_isBusy()) {
            AdcCapture_abort();
        }
        if (!DacOutput_playRamp(
                rampSampleCount,
                F_RAMP_MIN_CODE, F_RAMP_MAX_CODE)) {
            DacOutput_stop();
            setFState(F_STATE_ERROR);
            UserUART_write("ERR framp start failed\r\n");
            return true;
        }

        gFControl.mode = F_MODE_RAMP;
        gFControl.lastDKTick = Tick_now();
        setFState(F_STATE_SEARCHING);
        UserUART_printf(
            "OK framp window_us=%lu samples=%lu rate_hz=%lu "
            "codes=%u..%u relay=direct\r\n",
            (unsigned long)rampPeriodUs,
            (unsigned long)rampSampleCount,
            (unsigned long)DAC_OUTPUT_RATE_HZ,
            (unsigned int)F_RAMP_MIN_CODE,
            (unsigned int)F_RAMP_MAX_CODE);
        return true;
    }

    if ((nameLen == 7U) && (strncmp(name, "fwindow", 7U) == 0)) {
        phaseLockStop(false);
        text = skipSpaces(args);
        if (strcmp(text, "off") == 0) {
            DacOutput_stop();
            gFControl.mode = F_MODE_OFF;
            setFState(F_STATE_READY);
            UserUART_write("OK fwindow off dac=high_z\r\n");
            return true;
        }
        if (!parseU32(&text, &value[0]) ||
            (*skipSpaces(text) != '\0') ||
            ((value[0] != 20U) && (value[0] != 50U) &&
             (value[0] != 100U) && (value[0] != 200U) &&
             (value[0] != 500U) && (value[0] != 1000U) &&
             (value[0] != 2000U) && (value[0] != 5000U) &&
             (value[0] != 10000U))) {
            UserUART_write(
                "ERR usage: fwindow "
                "20|50|100|200|500|1000|2000|5000|10000|off\r\n");
            return true;
        }

        relaySelectDirect();
        if (AdcCapture_isBusy()) {
            AdcCapture_abort();
        }
        if (!DacOutput_playWindowedRampEdgeBands(
                value[0], CCD_AUTO_FRAME_US,
                F_WINDOW_RAMP_MIN_CODE, F_WINDOW_RAMP_MAX_CODE,
                F_WINDOW_IDLE_LOW_CODE, F_WINDOW_IDLE_HIGH_CODE,
                F_WINDOW_IDLE_BAND_CODES)) {
            DacOutput_stop();
            setFState(F_STATE_ERROR);
            UserUART_write("ERR fwindow start failed\r\n");
            return true;
        }
        gFControl.mode = F_MODE_RAMP;
        gFControl.lastDKTick = Tick_now();
        setFState(F_STATE_SEARCHING);
        UserUART_printf(
            "OK fwindow window_us=%lu frame_us=%u frame_hz=100 "
            "codes=%u..%u idle=edge_bands rails=%u/%u band_codes=%u "
            "transition=step "
            "repeat=hardware fallback=dac_irq relay=direct\r\n",
            (unsigned long)value[0], (unsigned int)CCD_AUTO_FRAME_US,
            (unsigned int)F_WINDOW_RAMP_MIN_CODE,
            (unsigned int)F_WINDOW_RAMP_MAX_CODE,
            (unsigned int)F_WINDOW_IDLE_LOW_CODE,
            (unsigned int)F_WINDOW_IDLE_HIGH_CODE,
            (unsigned int)F_WINDOW_IDLE_BAND_CODES);
        return true;
    }

    if ((nameLen == 6U) && (strncmp(name, "fprobe", 6U) == 0)) {
        phaseLockStop(false);
        text = args;
        for (index = 0U; index < 4U; index++) {
            if (!parseU32(&text, &value[index])) {
                UserUART_write(
                    "ERR usage: fprobe F0 F1 F2 F3 [AMP]\r\n");
                return true;
            }
            freq[index] = value[index];
        }
        amp = F_PROBE_DEFAULT_ASF;
        if (*skipSpaces(text) != '\0') {
            if (!parseU32(&text, &value[4]) || (value[4] > 1023U)) {
                UserUART_write("ERR invalid probe amplitude\r\n");
                return true;
            }
            amp = (uint16_t)value[4];
        }
        if (*skipSpaces(text) != '\0') {
            UserUART_write("ERR too many fprobe arguments\r\n");
            return true;
        }
        if (applyProbe4(freq, amp)) {
            gFControl.lastDKTick = Tick_now();
            setFState(F_STATE_SEARCHING);
            UserUART_printf(
                "OK fprobe f0=%lu f1=%lu f2=%lu f3=%lu amp=%u\r\n",
                (unsigned long)freq[0], (unsigned long)freq[1],
                (unsigned long)freq[2], (unsigned long)freq[3],
                (unsigned int)amp);
        } else {
            UserUART_write(
                "ERR fprobe frequencies must be 1000..100000 Hz\r\n");
        }
        return true;
    }

    if ((nameLen == 5U) && (strncmp(name, "fauto", 5U) == 0)) {
        phaseLockStop(false);
        text = skipSpaces(args);
        if (strcmp(text, "line") == 0) {
            startAutoMode(F_MODE_AUTO_LINE);
        } else if (strcmp(text, "circle") == 0) {
            startAutoMode(F_MODE_AUTO_CIRCLE);
        } else if (strcmp(text, "infinity") == 0) {
            startAutoMode(F_MODE_AUTO_INFINITY);
        } else {
            UserUART_write("ERR usage: fauto line|circle|infinity\r\n");
        }
        return true;
    }

    if ((nameLen == 6U) && (strncmp(name, "fstate", 6U) == 0)) {
        text = skipSpaces(args);
        gFControl.lastDKTick = Tick_now();
        if (strcmp(text, "ready") == 0) {
            setFState(F_STATE_READY);
        } else if (strcmp(text, "wait") == 0) {
            setFState(F_STATE_WAIT_DK);
        } else if (strcmp(text, "search") == 0) {
            setFState(F_STATE_SEARCHING);
        } else if (strcmp(text, "lock") == 0) {
            setFState(F_STATE_LOCKING);
        } else if (strcmp(text, "stable") == 0) {
            setFState(F_STATE_STABLE);
        } else if (strcmp(text, "error") == 0) {
            relaySelectDirect();
            setFState(F_STATE_ERROR);
        } else {
            UserUART_write(
                "ERR usage: fstate ready|wait|search|lock|stable|error\r\n");
            return true;
        }
        UserUART_printf("OK fstate %s\r\n", fStateName(gFControl.state));
        return true;
    }

    return false;
}

static void showHelp(void)
{
    UserUART_write(
        "help | status | led R G B | rainbow on|off | "
        "sound key|start|success|error|off\r\n");
    UserUART_write("adc once [1000..1000000] | dac CODE(0..4095)\r\n");
    UserUART_write(
        "ccd capture [1..100ms] [10..90pct] | ccd dump [...] | "
        "ccd freq | ccd fine [COARSE_FREQ_HZ] | ccd status\r\n");
    UserUART_write("ccd auto line|circle|infinity\r\n");
    UserUART_write(
        "dds init | dds FREQ_HZ [AMP_0_1023] [PHASE_0_16383] | "
        "dds off\r\n");
    UserUART_write("F controller: fhelp\r\n");
}

static void captureCCD(
    uint32_t exposureMs, uint8_t thresholdPercent, bool dumpPixels)
{
    TSL1401_Stats stats;
    uint8_t localCenters[TSL1401_MAX_PEAKS];
    uint8_t localCount;
    uint16_t localProminence;
    uint32_t i;

    if (gPhaseLock.enabled) {
        phaseLockStop(false);
    }
    AdcCapture_abort();

    UserUART_task();
    APP_UART_PRINTF(
        "CCD_CAPTURE exposure_ms=%lu threshold_pct=%u phase_lock=stopped\r\n",
        (unsigned long)exposureMs, (unsigned int)thresholdPercent);

    if (!TSL1401_capture(gCCDPixels, exposureMs)) {
        APP_UART_PRINTF("ERR CCD ADC timeout; check AO=PA14 and power\r\n");
        return;
    }

    TSL1401_analyze(gCCDPixels, thresholdPercent, &stats);
    localCount = ccdCountProminentPeaks(
        gCCDPixels, localCenters, &localProminence);
    APP_UART_PRINTF(
        "CCD_STATS min=%u max=%u span=%u threshold=%u peaks=%u\r\n",
        (unsigned int)stats.minimum, (unsigned int)stats.maximum,
        (unsigned int)(stats.maximum - stats.minimum),
        (unsigned int)stats.threshold, (unsigned int)stats.peakCount);

    APP_UART_PRINTF("CCD_PEAKS");
    for (i = 0U; i < stats.peakCount; i++) {
        APP_UART_PRINTF(" %u:%u", (unsigned int)stats.peakCenter[i],
            (unsigned int)stats.peakWidth[i]);
    }
    APP_UART_PRINTF("\r\n");
    APP_UART_PRINTF(
        "CCD_LOCAL prominence=%u peaks=%u",
        (unsigned int)localProminence, (unsigned int)localCount);
    for (i = 0U; i < localCount; i++) {
        APP_UART_PRINTF(" %u", (unsigned int)localCenters[i]);
    }
    APP_UART_PRINTF("\r\n");

    if (dumpPixels) {
        for (i = 0U; i < TSL1401_PIXEL_COUNT; i++) {
            if ((i % 16U) == 0U) {
                APP_UART_PRINTF("CCD_DATA %03u", (unsigned int)i);
            }
            APP_UART_PRINTF(" %u", (unsigned int)gCCDPixels[i]);
            if ((i % 16U) == 15U) {
                APP_UART_PRINTF("\r\n");
            }
        }
    }
    APP_UART_PRINTF("OK ccd capture complete; ADC0 restored to PA27\r\n");
}

static bool runCCDFrequencyRecognition(void)
{
    uint32_t startTick = Tick_now();
    uint32_t coarseFrequencyHz;
    uint32_t fineFrequencyHz;
    uint32_t detectedFrequencyHz;
    uint32_t probeWindowUs;
    uint16_t outputAmplitude;
    uint8_t coarsePeaks;

    gFControl.requestId++;
    phaseLockStop(true);
    AdcCapture_abort();
    gFControl.mode = F_MODE_OFF;
    gFControl.autoTarget = F_MODE_OFF;
    setFState(F_STATE_SEARCHING);
    APP_UART_PRINTF(
        "CCD_FREQ start request=%lu phase_lock=disabled\r\n",
        (unsigned long)gFControl.requestId);

    /*
     * Capture the optical background at the same midpoint code used between
     * ramp windows. The ramp and idle output are both centered at code 2048,
     * so the display center remains unchanged. Subtracting this frame removes
     * the unavoidable idle horizontal line in software.
    */
    relaySelectDirect();
    DacOutput_setCode(CCD_FREQ_IDLE_CODE);
    Tick_delay(80U);
    gCCDExposureMs = CCD_FREQ_EXPOSURE_MS;
    if (!TSL1401_captureFiltered(
            gCCDBackground, gCCDExposureMs, 3U)) {
        DacOutput_stop();
        gDDSInitialized = false;
        relaySelectDirect();
        setFState(F_STATE_ERROR);
        APP_UART_PRINTF(
            "CCD_FREQ error=background_capture "
            "check=ccd_power_and_alignment\r\n");
        return false;
    }
    gCCDActiveSpanQ8 = CCD_FREQ_ACTIVE_SPAN_Q8;
    APP_UART_PRINTF(
        "CCD_FREQ exposure_ms=%lu background_subtraction=on idle_code=%u "
        "active_span_q8=%lu calibration=fixed_ac_coupled "
        "algorithm=coarse_guarded_v46 high_exposure_ms=%u "
        "high_active_span_q8=%u sample_frames=%u warmup_frames=%u\r\n",
        (unsigned long)gCCDExposureMs,
        (unsigned int)CCD_FREQ_IDLE_CODE,
        (unsigned long)gCCDActiveSpanQ8,
        (unsigned int)CCD_FREQ_HIGH_EXPOSURE_MS,
        (unsigned int)CCD_FREQ_HIGH_ACTIVE_SPAN_Q8,
        (unsigned int)CCD_FREQ_SAMPLE_FRAMES,
        (unsigned int)CCD_FREQ_WARMUP_FRAMES);

    if (!ccdEstimateFrequencyConfirmed(gCCDExposureMs, gCCDBackground,
            &coarseFrequencyHz, &probeWindowUs, &coarsePeaks)) {
        DacOutput_stop();
        gDDSInitialized = false;
        relaySelectDirect();
        setFState(F_STATE_ERROR);
        APP_UART_PRINTF(
            "CCD_FREQ error=coarse_frequency "
            "reason=insufficient_optical_consensus "
            "check=alignment_focus_persistence\r\n");
        return false;
    }
    APP_UART_PRINTF(
        "CCD_FREQ coarse_hz=%lu window_us=%lu peaks=%u elapsed_ms=%lu\r\n",
        (unsigned long)coarseFrequencyHz,
        (unsigned long)probeWindowUs, (unsigned int)coarsePeaks,
        (unsigned long)Tick_elapsed(startTick));

    /*
     * The spacing estimate already resolves the specified 100 Hz grid.
     * Do not sweep DDS candidates here: the oscilloscope's long persistence
     * accumulates old candidates, so "narrowest trace" selected an unrelated
     * frequency even when the optical crossing train was correct.
     */
    setFState(F_STATE_LOCKING);
    fineFrequencyHz = coarseFrequencyHz;
    detectedFrequencyHz = coarseFrequencyHz;
    APP_UART_PRINTF(
        "CCD_FREQ spacing_selected_hz=%lu fine_search=disabled\r\n",
        (unsigned long)detectedFrequencyHz);

    /*
     * "ccd freq" is deliberately a measurement-only operation. Keep the DDS
     * isolated after PA15 activity so frequency development is independent
     * of the final output/lock stages. A later DDS or automatic-mode command
     * consumes the pending recovery request and rebuilds the DDS once.
     */
    DacOutput_stop();
    gDDSInitialized = false;
    relaySelectDirect();
    outputAmplitude = ampFromDiv(8U, detectedFrequencyHz);
    gFControl.inputFreqHz = detectedFrequencyHz;
    gFControl.targetDiv = 8U;
    gFControl.singlePhaseWord = 0U;
    memset(gFControl.outputFreqHz, 0, sizeof(gFControl.outputFreqHz));
    memset(gFControl.outputAmp, 0, sizeof(gFControl.outputAmp));
    memset(gFControl.outputPhase, 0, sizeof(gFControl.outputPhase));
    gFControl.mode = F_MODE_OFF;
    gFControl.autoTarget = F_MODE_OFF;
    gFControl.lastDKTick = Tick_now();
    setFState(F_STATE_READY);

    APP_UART_PRINTF(
        "CCD_FREQ complete detected_hz=%lu coarse_hz=%lu fine_hz=%lu "
        "output=deferred suggested_amp=%u lock=off elapsed_ms=%lu\r\n",
        (unsigned long)detectedFrequencyHz,
        (unsigned long)coarseFrequencyHz,
        (unsigned long)fineFrequencyHz,
        (unsigned int)outputAmplitude,
        (unsigned long)Tick_elapsed(startTick));
    return true;
}

static bool runCCDFineFrequency(uint32_t centerFrequencyHz)
{
    uint32_t center;
    uint32_t radius;
    uint32_t offset;
    uint32_t candidate;
    uint32_t score;
    uint32_t confirmScore;
    uint32_t bestFrequency = 0U;
    uint32_t bestScore = CCD_AUTO_BAD_SCORE;
    uint8_t direction;
    bool confirmed = false;

    if ((centerFrequencyHz < F_FREQ_MIN_HZ) ||
        (centerFrequencyHz > F_FREQ_MAX_HZ)) {
        return false;
    }

    center = roundFrequency100(centerFrequencyHz);
    if (center <= 15000U) {
        radius = CCD_FINE_DIFF_LOW_RADIUS_HZ;
    } else if (center <= 70000U) {
        radius = CCD_FINE_DIFF_MID_RADIUS_HZ;
    } else {
        radius = CCD_FINE_DIFF_HIGH_RADIUS_HZ;
    }

    phaseLockStop(true);
    AdcCapture_abort();
    gFControl.mode = F_MODE_OFF;
    gFControl.autoTarget = F_MODE_OFF;
    setFState(F_STATE_LOCKING);
    APP_UART_PRINTF(
        "CCD_FINE_DIFF start center_hz=%lu radius_hz=%lu step_hz=%u "
        "algorithm=fine_raw_width_v50 active_settle_ms=%u "
        "active_frames=3 max_width_px=20 confirm_required=2 "
        "scope_persistence_required=MIN "
        "source=scope_ccd_only pa16=unused pa27=unused\r\n",
        (unsigned long)center, (unsigned long)radius,
        (unsigned int)F_FREQ_STEP_HZ,
        (unsigned int)CCD_FINE_DIFF_ACTIVE_SETTLE_MS);

    /*
     * Search from the coarse estimate outwards. Each candidate builds its own
     * zero-amplitude optical baseline, so an old high-persistence trace is
     * subtracted instead of being compared with a later candidate.
     */
    for (offset = 0U; offset <= radius; offset += F_FREQ_STEP_HZ) {
        for (direction = 0U; direction < ((offset == 0U) ? 1U : 2U);
             direction++) {
            if (direction == 0U) {
                candidate = center + offset;
                if (candidate > F_FREQ_MAX_HZ) {
                    continue;
                }
            } else {
                if ((offset > center) ||
                    ((center - offset) < F_FREQ_MIN_HZ)) {
                    continue;
                }
                candidate = center - offset;
            }

            score = ccdEvaluateFrequencyDifferential(candidate);
            if (score < bestScore) {
                bestScore = score;
                bestFrequency = candidate;
            }
            if (score <= CCD_FINE_DIFF_GOOD_SCORE) {
                confirmScore =
                    ccdEvaluateFrequencyDifferential(candidate);
                APP_UART_PRINTF(
                    "CCD_FINE_DIFF confirm candidate_hz=%lu "
                    "score=%lu confirm_score=%lu limit=%u\r\n",
                    (unsigned long)candidate,
                    (unsigned long)score,
                    (unsigned long)confirmScore,
                    (unsigned int)CCD_FINE_DIFF_GOOD_SCORE);
                if (confirmScore <= CCD_FINE_DIFF_GOOD_SCORE) {
                    bestFrequency = candidate;
                    bestScore = (score + confirmScore + 1U) / 2U;
                    confirmed = true;
                    break;
                }
            }
        }
        if (confirmed) {
            break;
        }
    }

    relaySelectDirect();
    if (!confirmed) {
        setFState(F_STATE_ERROR);
        APP_UART_PRINTF(
            "CCD_FINE_DIFF error=no_confirmed_candidate "
            "center_hz=%lu radius_hz=%lu diagnostic_best_hz=%lu "
            "diagnostic_score=%lu\r\n",
            (unsigned long)center, (unsigned long)radius,
            (unsigned long)bestFrequency,
            (unsigned long)bestScore);
        return false;
    }

    gFControl.inputFreqHz = bestFrequency;
    setFState(F_STATE_READY);
    APP_UART_PRINTF(
        "CCD_FINE_DIFF complete detected_hz=%lu best_score=%lu "
        "confidence=confirmed output=deferred\r\n",
        (unsigned long)bestFrequency,
        (unsigned long)bestScore);
    return true;
}

static bool runCCDAutoMode(FMode mode)
{
    const char *target;
    uint32_t startTick;
    uint32_t coarseFrequencyHz;
    uint32_t fineFrequencyHz;
    uint32_t detectedFrequencyHz;
    uint32_t probeWindowUs;
    uint32_t searchExposureMs = CCD_AUTO_SEARCH_EXPOSURE_MS;
    uint16_t finalPhase;
    uint16_t finalAmplitude;
    uint32_t outputFrequencyHz;
    uint8_t coarsePeaks;

    if ((mode != F_MODE_AUTO_LINE) &&
        (mode != F_MODE_AUTO_CIRCLE) &&
        (mode != F_MODE_AUTO_INFINITY)) {
        return false;
    }
    target = (mode == F_MODE_AUTO_LINE) ? "line" :
        ((mode == F_MODE_AUTO_CIRCLE) ? "circle" : "infinity");
    startTick = Tick_now();

    phaseLockStop(true);
    AdcCapture_abort();
    setFState(F_STATE_SEARCHING);
    APP_UART_PRINTF(
        "CCD_AUTO start target=%s request=%lu electrical_input=disconnected\r\n",
        target, (unsigned long)gFControl.requestId);

    /*
     * Capture a fixed optical background at the idle code. The same
     * background is subtracted from every ramp frame, so the idle horizontal
     * line cannot dominate the CCD estimate.
     */
    relaySelectDirect();
    DacOutput_setCode(CCD_FREQ_IDLE_CODE);
    Tick_delay(80U);
    gCCDExposureMs = CCD_FREQ_EXPOSURE_MS;
    if (!TSL1401_captureFiltered(
            gCCDBackground, gCCDExposureMs, 3U)) {
        APP_UART_PRINTF("CCD_AUTO error=background_capture\r\n");
        return false;
    }
    APP_UART_PRINTF(
        "CCD_AUTO exposure_ms=%lu background_subtraction=on idle_code=%u\r\n",
        (unsigned long)gCCDExposureMs,
        (unsigned int)CCD_FREQ_IDLE_CODE);

    if (!ccdEstimateFrequencyConfirmed(gCCDExposureMs, gCCDBackground,
            &coarseFrequencyHz, &probeWindowUs, &coarsePeaks)) {
        DacOutput_stop();
        APP_UART_PRINTF(
            "CCD_AUTO error=coarse_frequency "
            "reason=insufficient_optical_consensus "
            "check=alignment_focus_persistence\r\n");
        return false;
    }
    APP_UART_PRINTF(
        "CCD_AUTO coarse_hz=%lu window_us=%lu peaks=%u elapsed_ms=%lu\r\n",
        (unsigned long)coarseFrequencyHz,
        (unsigned long)probeWindowUs, (unsigned int)coarsePeaks,
        (unsigned long)Tick_elapsed(startTick));
    setFState(F_STATE_LOCKING);

    fineFrequencyHz = ccdSearchFrequency(
        coarseFrequencyHz, searchExposureMs, gCCDBackground);
    detectedFrequencyHz = coarseFrequencyHz;
    if ((fineFrequencyHz < F_FREQ_MIN_HZ) ||
        (fineFrequencyHz > F_FREQ_MAX_HZ)) {
        APP_UART_PRINTF(
            "CCD_AUTO fine_fallback_hz=%lu reason=no_valid_candidate\r\n",
            (unsigned long)detectedFrequencyHz);
    } else if (fineFrequencyHz != coarseFrequencyHz) {
        APP_UART_PRINTF(
            "CCD_AUTO fine_rejected_hz=%lu anchor_hz=%lu "
            "reason=single_line_phase_ambiguity\r\n",
            (unsigned long)fineFrequencyHz,
            (unsigned long)coarseFrequencyHz);
    } else {
        APP_UART_PRINTF(
            "CCD_AUTO fine_confirmed_hz=%lu\r\n",
            (unsigned long)fineFrequencyHz);
    }
    APP_UART_PRINTF(
        "CCD_AUTO detected_hz=%lu elapsed_ms=%lu\r\n",
        (unsigned long)detectedFrequencyHz,
        (unsigned long)Tick_elapsed(startTick));

    /*
     * The frequency probe background used the auto-selected exposure. Phase
     * recognition uses a longer exposure, so refresh the background at the
     * same exposure to remove the scope grid and the collapsed baseline.
     */
    relaySelectDirect();
    DacOutput_setCode(F_RAMP_MIN_CODE);
    Tick_delay(50U);
    if (!TSL1401_captureFiltered(
            gCCDBackground, searchExposureMs, 1U)) {
        APP_UART_PRINTF("CCD_AUTO error=phase_background_capture\r\n");
        return false;
    }
    APP_UART_PRINTF(
        "CCD_AUTO phase_background_ms=%lu\r\n",
        (unsigned long)searchExposureMs);

    finalPhase = ccdSearchPhase(
        mode, detectedFrequencyHz, searchExposureMs, gCCDBackground);
    outputFrequencyHz = detectedFrequencyHz;
    if (mode == F_MODE_AUTO_INFINITY) {
        outputFrequencyHz *= 2U;
    }
    finalAmplitude = ampFromDiv(8U, outputFrequencyHz);

    /*
     * Continuous tracking uses a shorter exposure for higher loop bandwidth.
     * Refresh the grid/baseline background at that same exposure before
     * enabling the final DDS output; otherwise 100 ms background values
     * cannot be subtracted reliably from a 40 ms tracking frame.
     */
    relaySelectDirect();
    DacOutput_setCode(F_RAMP_MIN_CODE);
    Tick_delay(30U);
    if (!TSL1401_captureFiltered(
            gCCDBackground, CCD_TRACK_EXPOSURE_MS, 1U)) {
        APP_UART_PRINTF("CCD_AUTO error=track_background_capture\r\n");
        return false;
    }
    APP_UART_PRINTF(
        "CCD_AUTO track_background_ms=%u interval_ms=%u\r\n",
        (unsigned int)CCD_TRACK_EXPOSURE_MS,
        (unsigned int)CCD_TRACK_INTERVAL_MS);

    applySingleOutput(
        outputFrequencyHz, finalAmplitude, finalPhase);

    gFControl.inputFreqHz = detectedFrequencyHz;
    gFControl.targetDiv = 8U;
    gFControl.singlePhaseWord = finalPhase;
    gFControl.mode = mode;
    gFControl.autoTarget = mode;
    gFControl.lastDKTick = Tick_now();
    gCCDTrackTick = Tick_now();
    gCCDTrackDirection = -1;
    gCCDTrackTrialActive = false;
    gCCDTrackBadFrames = 0U;
    setFState(F_STATE_STABLE);

    APP_UART_PRINTF(
        "CCD_AUTO complete target=%s input_hz=%lu output_hz=%lu "
        "phase=%u amp=%u control_ms=%lu stable=continuous\r\n",
        target, (unsigned long)detectedFrequencyHz,
        (unsigned long)outputFrequencyHz, (unsigned int)finalPhase,
        (unsigned int)finalAmplitude,
        (unsigned long)Tick_elapsed(startTick));
    return true;
}

static bool ccdFrequencyTrainAccept(uint32_t windowUs, uint8_t pointCount,
    uint8_t minimumGap, uint8_t maximumGap, uint32_t trainSpan)
{
    uint32_t minimumSpan = CCD_FREQ_MIN_TRAIN_SPAN;

    /*
     * At 60..100 kHz a 50 us trace contains closely spaced crossings, so a
     * good five-point train can be shorter than the legacy 60-pixel
     * requirement. Keep the original gate everywhere else.
     */
    if (windowUs <= 100U) {
        if ((windowUs == 20U) && (pointCount == 4U)) {
            minimumSpan = 58U;
        } else if (pointCount >= 7U) {
            minimumSpan = 40U;
        } else if (pointCount >= 5U) {
            minimumSpan = 45U;
        }
    }
    if (trainSpan < minimumSpan) {
        return false;
    }

    /*
     * A four-point trace has only three gaps, so one merged shoulder can move
     * the estimate substantially. Keep the tighter uniformity check there.
     * Five-point traces are instead protected by the multi-frame relative
     * consensus below: the 60 kHz bench trace repeatedly produced a stable
     * 56..57-pixel total span even though individual gaps varied by more than
     * 20 percent. Six or more points already passed
     * ccdSelectRegularPeaks()'s 5/3 gate.
     */
    if ((windowUs <= 100U) && (pointCount == 4U) &&
        !((windowUs == 20U) && (trainSpan >= 58U))) {
        if (maximumGap < minimumGap) {
            return false;
        }
        if (((uint32_t)(maximumGap - minimumGap) * 5U) >
            ((uint32_t)maximumGap + 1U)) {
            return false;
        }
    }
    return true;
}

static bool ccdEstimateFrequency(uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint32_t *frequencyHz, uint32_t *windowUs, uint8_t *peakCount)
{
    /*
     * Each source cycle produces two zero crossings. Four to sixteen visible
     * crossings give the most reliable spacing estimate, so cover the full
     * 1..100 kHz range with a small geometric set instead of repeating one
     * unsuitable window five times:
     *   50 us  -> 40..100 kHz primary observation
     *   20 us  -> 80..100 kHz fallback
     *   100 us -> 20..80 kHz overlap
     *   2 ms   -> 1..4 kHz
     *   300 us -> 7..25 kHz (about ten crossings near 17 kHz)
     *   200 us -> 10..40 kHz fallback
     *   500 us -> 4..16 kHz
     * Longer and shorter windows remain only as optical fallbacks.
     */
    static const uint16_t probeWindowsUs[] = {
        50U, 20U, 100U, 300U, 200U, 500U,
        2000U, 1000U, 5000U, 10U, 10000U
    };
    /*
     * Frequency recognition is a blocking, non-reentrant operation. Keep its
     * large workspace in BSS: the generated linker reserves only a 512-byte
     * stack, while three TSL1401 statistics records plus peak arrays alone
     * exceed half of that budget.
     */
    static const uint16_t *frameBackground;
    static TSL1401_Stats stats;
    static TSL1401_Stats frameStats;
    static TSL1401_Stats verifyStats;
    static uint8_t localCenters[TSL1401_MAX_PEAKS];
    static uint8_t frameCenters[TSL1401_MAX_PEAKS];
    static uint8_t clusterCenters[TSL1401_MAX_PEAKS];
    static uint8_t frameClusterCenters[TSL1401_MAX_PEAKS];
    static uint32_t estimated;
    static uint32_t bestFrequency;
    static uint32_t sortValue;
    static uint32_t bestWindow;
    static uint32_t frameExposureMs;
    static uint32_t width;
    static uint32_t frameWidth;
    static uint32_t trainSpan;
    static uint32_t clusterTrainSpan;
    static uint32_t frameClusterTrainSpan;
    static uint32_t frameClusterEstimated;
    static uint32_t frameLocalEstimated;
    static uint32_t clusterRawHz;
    static uint32_t clusterHintHz;
    static uint32_t localRawHz;
    static uint32_t localHintHz;
    static uint32_t fitHz;
    static uint32_t combHz;
    static uint32_t clusterConsensusHz;
    static uint32_t highbandConsensusHz;
    static uint32_t localHighbandConsensusHz;
    static uint32_t fitHighbandConsensusHz;
    static uint32_t localHighbandMinimumHz;
    static uint32_t localHighbandMaximumHz;
    static uint32_t fitHighbandMinimumHz;
    static uint32_t fitHighbandMaximumHz;
    static uint32_t modelDifference;
    static uint32_t modelAgreementLimit;
    static uint32_t localCandidateSum;
    static uint32_t localCandidateMinimum;
    static uint32_t localCandidateMaximum;
    static uint32_t localStrictConsensusHz;
    static uint32_t combStrictConsensusHz;
    static uint32_t boundaryPairHz;
    static uint32_t boundaryPairDifference;
    static uint32_t boundaryPairLimit;
    static uint32_t boundaryCrossDifference;
    static uint32_t boundaryCrossLimit;
    static uint32_t localSparsePairHz;
    static uint32_t fitSparsePairHz;
    static uint32_t localSparseDifference;
    static uint32_t fitSparseDifference;
    static uint32_t sparsePairLimit;
    static uint32_t calibratedFrequency;
    static uint32_t clusterCandidates[CCD_FREQ_SAMPLE_FRAMES];
    static uint32_t localCandidates[CCD_FREQ_SAMPLE_FRAMES];
    static uint32_t fitCandidates[CCD_FREQ_SAMPLE_FRAMES];
    static uint32_t combCandidates[CCD_FREQ_SAMPLE_FRAMES];
    static uint32_t highbandCandidates[CCD_FREQ_SAMPLE_FRAMES];
    static uint32_t midbandCandidates[CCD_FREQ_SAMPLE_FRAMES * 2U];
    static uint8_t fitCandidatePoints[CCD_FREQ_SAMPLE_FRAMES];
    static uint64_t estimateNumerator;
    static uint64_t estimateDenominator;
    static uint16_t prominenceThreshold;
    static uint16_t frameProminenceThreshold;
    static uint16_t activeSpanTimes2;
    static uint8_t rawLocalPointCount;
    static uint8_t frameRawLocalPointCount;
    static uint8_t localPointCount;
    static uint8_t frameLocalPointCount;
    static uint8_t clusterCount;
    static uint8_t frameClusterCount;
    static uint8_t frameEffectivePointCount;
    static uint8_t clusterCandidateCount;
    static uint8_t localCandidateCount;
    static uint8_t fitCandidateCount;
    static uint8_t combCandidateCount;
    static uint8_t highbandCandidateCount;
    static uint8_t midbandCandidateCount;
    static uint8_t midbandBeforeCount;
    static uint8_t midbandMinimumPoints;
    static uint8_t percentileIndex;
    static uint8_t consensusSupport;
    static uint8_t clusterConsensusSupport;
    static uint8_t highbandConsensusSupport;
    static uint8_t localHighbandSupport;
    static uint8_t fitHighbandSupport;
    static uint8_t localStrictSupport;
    static uint8_t combStrictSupport;
    static uint8_t minimumGap;
    static uint8_t maximumGap;
    static uint8_t frameMinimumGap;
    static uint8_t frameMaximumGap;
    static uint8_t clusterMinimumGap;
    static uint8_t clusterMaximumGap;
    static uint8_t frameClusterMinimumGap;
    static uint8_t frameClusterMaximumGap;
    static uint8_t bestPeaks;
    static uint8_t index;
    static uint8_t point;
    static uint8_t frame;
    static bool estimateAccepted;
    static bool clusterConsensusAccepted;
    static bool highbandConsensusAccepted;
    static bool localHighbandAccepted;
    static bool fitHighbandAccepted;
    static bool highbandFusionSelected;
    static bool highbandFitSelected;
    static bool highbandLocalSelected;
    static bool highbandSparseSelected;
    static bool highbandAliasOverride;
    static bool modelHighbandSelected;
    static bool localMeanSelected;
    static bool localStrictSelected;
    static bool localStrictAccepted;
    static bool combStrictAccepted;
    static bool boundaryPairSelected;
    static bool highRouteEvidence;
    static bool fitSelected;

    if (background == NULL) {
        return false;
    }
    bestFrequency = 0U;
    bestWindow = 0U;
    bestPeaks = 0U;
    midbandCandidateCount = 0U;
    highRouteEvidence = false;
    /*
     * V38 proved that 20 ms high-band exposure halves the useful optical
     * train and makes 100 kHz undetectable. Use the validated 40 ms exposure
     * and its matching background for every window; only the high-band span
     * calibration changes below.
     */
    for (index = 0U;
         index < (sizeof(probeWindowsUs) / sizeof(probeWindowsUs[0]));
         index++) {
        if (probeWindowsUs[index] <= 100U) {
            frameExposureMs = CCD_FREQ_HIGH_EXPOSURE_MS;
            frameBackground = background;
            gCCDActiveSpanQ8 = CCD_FREQ_HIGH_ACTIVE_SPAN_Q8;
        } else {
            frameExposureMs = exposureMs;
            frameBackground = background;
            gCCDActiveSpanQ8 = CCD_FREQ_ACTIVE_SPAN_Q8;
        }
        activeSpanTimes2 = (uint16_t)(
            (gCCDActiveSpanQ8 + 64U) / 128U);
        /*
         * A preceding continuous DDS/CCD tracking session can leave the DAC
         * trigger path armed but not advancing. Stop the old waveform before
         * rebuilding each probe window so the first request after a frequency
         * change cannot inherit that stale state.
         */
        DacOutput_stop();
        Tick_delay(5U);
        if (!DacOutput_playWindowedRamp(probeWindowsUs[index],
                CCD_AUTO_FRAME_US, F_RAMP_MIN_CODE, F_RAMP_MAX_CODE,
                CCD_FREQ_IDLE_CODE)) {
            continue;
        }
        /*
         * After a static DAC output, the first DMA start can leave the DAC
         * sample generator armed without issuing its initial FIFO request.
         * Repeating the same start once is harmless and was verified to turn
         * the otherwise static first probe into the requested ramp.
         */
        Tick_delay(500U);
        if (!DacOutput_playWindowedRamp(probeWindowsUs[index],
                CCD_AUTO_FRAME_US, F_RAMP_MIN_CODE, F_RAMP_MAX_CODE,
                CCD_FREQ_IDLE_CODE)) {
            continue;
        }
        /*
         * The oscilloscope intensity display needs several refreshes after
         * changing the ramp window. A short 35 ms delay leaves remnants from
         * the preceding wider window and merges real crossings.
         */
        Tick_delay(CCD_FREQ_SETTLE_MS);
        /*
         * The first CCD exposures after a scope clear or window transition
         * still contain a partially painted trace. Do not let those startup
         * frames create a false majority (observed as 54.1 kHz at 70 kHz).
         */
        for (frame = 0U; frame < CCD_FREQ_WARMUP_FRAMES; frame++) {
            if (!TSL1401_capture(gCCDPixels, frameExposureMs)) {
                DacOutput_stop();
                return false;
            }
        }
        APP_UART_PRINTF(
            "CCD_WARMUP window_us=%u exposure_ms=%lu "
            "active_span_q8=%lu discarded=%u\r\n",
            (unsigned int)probeWindowsUs[index],
            (unsigned long)frameExposureMs,
            (unsigned long)gCCDActiveSpanQ8,
            (unsigned int)CCD_FREQ_WARMUP_FRAMES);
        /*
         * Detect the complete-zero condition seen on the first 100 kHz run.
         * One controlled stop/restart is cheaper and safer than consuming all
         * eleven probe windows with a frozen DAC. This validation frame is
         * discarded so it cannot vote in the frequency result.
         */
        if (!TSL1401_capture(gCCDPixels, frameExposureMs)) {
            DacOutput_stop();
            return false;
        }
        TSL1401_subtractBackground(
            gCCDPixels, frameBackground, gCCDCorrected);
        TSL1401_analyze(gCCDCorrected,
            CCD_FREQ_CLUSTER_THRESHOLD_PERCENT, &verifyStats);
        if ((probeWindowsUs[index] == 50U) &&
            ((verifyStats.maximum - verifyStats.minimum) <
             CCD_FREQ_RAMP_VERIFY_MIN_SPAN)) {
            APP_UART_PRINTF(
                "CCD_RAMP_RESTART window_us=%u verify_span=%u\r\n",
                (unsigned int)probeWindowsUs[index],
                (unsigned int)(verifyStats.maximum - verifyStats.minimum));
            DacOutput_stop();
            Tick_delay(5U);
            if (!DacOutput_playWindowedRamp(probeWindowsUs[index],
                    CCD_AUTO_FRAME_US, F_RAMP_MIN_CODE, F_RAMP_MAX_CODE,
                    CCD_FREQ_IDLE_CODE)) {
                continue;
            }
            Tick_delay(500U);
            if (!DacOutput_playWindowedRamp(probeWindowsUs[index],
                    CCD_AUTO_FRAME_US, F_RAMP_MIN_CODE, F_RAMP_MAX_CODE,
                    CCD_FREQ_IDLE_CODE)) {
                continue;
            }
            Tick_delay(CCD_FREQ_SETTLE_MS);
            for (frame = 0U; frame < CCD_FREQ_WARMUP_FRAMES; frame++) {
                if (!TSL1401_capture(gCCDPixels, frameExposureMs)) {
                    DacOutput_stop();
                    return false;
                }
            }
        }
        rawLocalPointCount = 0U;
        localPointCount = 0U;
        clusterCount = 0U;
        prominenceThreshold = 0U;
        minimumGap = 0U;
        maximumGap = 0U;
        clusterMinimumGap = 0U;
        clusterMaximumGap = 0U;
        clusterTrainSpan = 0U;
        clusterCandidateCount = 0U;
        localCandidateCount = 0U;
        fitCandidateCount = 0U;
        combCandidateCount = 0U;
        width = 0U;
        memset(&stats, 0, sizeof(stats));
        for (frame = 0U; frame < CCD_FREQ_SAMPLE_FRAMES; frame++) {
            /*
             * Evaluate independent frames. Pixel-wise median filtering erases
             * narrow optical peaks when the scope trace moves by one pixel.
             */
            if (!TSL1401_capture(gCCDPixels, frameExposureMs)) {
                DacOutput_stop();
                return false;
            }
            /*
             * The idle part of every 10 ms frame intentionally draws a
             * horizontal line at the DAC midpoint. Remove the matching
             * idle-only optical frame before detecting crossings; the scope
             * display center is left unchanged.
             */
            TSL1401_subtractBackground(
                gCCDPixels, frameBackground, gCCDCorrected);
            TSL1401_analyze(gCCDCorrected,
                CCD_FREQ_CLUSTER_THRESHOLD_PERCENT, &frameStats);
            frameWidth = ccdStatsWidth(&frameStats);
            for (point = 0U; point < frameStats.peakCount; point++) {
                frameClusterCenters[point] =
                    frameStats.peakCenter[point];
            }
            frameClusterCount = ccdSelectRegularPeaks(
                frameClusterCenters, frameStats.peakCount,
                &frameClusterMinimumGap, &frameClusterMaximumGap);
            frameClusterTrainSpan = (frameClusterCount >= 2U) ?
                (uint32_t)(
                    frameClusterCenters[frameClusterCount - 1U] -
                    frameClusterCenters[0]) : 0U;
            frameRawLocalPointCount = ccdCountProminentPeaks(
                gCCDCorrected, frameCenters, &frameProminenceThreshold);
            frameLocalPointCount = ccdSelectRegularPeaks(
                frameCenters, frameRawLocalPointCount,
                &frameMinimumGap, &frameMaximumGap);
            trainSpan = (frameLocalPointCount >= 2U) ?
                (uint32_t)(frameCenters[frameLocalPointCount - 1U] -
                    frameCenters[0]) : 0U;
            frameClusterEstimated = 0U;
            frameLocalEstimated = 0U;
            clusterRawHz = 0U;
            clusterHintHz = 0U;
            localRawHz = 0U;
            localHintHz = 0U;
            fitHz = 0U;
            combHz = 0U;

            /*
             * Turn every frame into an independent frequency vote. Do not
             * combine the point count from one frame with the span from
             * another, and do not let one unusually wide persistence frame
             * decide the result for the whole probe window.
             */
            if ((frameClusterCount >= CCD_FREQ_MIN_POINTS) &&
                (frameClusterCount <= CCD_FREQ_MAX_POINTS) &&
                ccdFrequencyTrainAccept(
                    probeWindowsUs[index], frameClusterCount,
                    frameClusterMinimumGap, frameClusterMaximumGap,
                    frameClusterTrainSpan) &&
                ((frameStats.maximum - frameStats.minimum) >= 100U)) {
                estimateNumerator =
                    (uint64_t)(frameClusterCount - 1U) *
                    activeSpanTimes2 * 250000ULL;
                estimateDenominator =
                    (uint64_t)frameClusterTrainSpan *
                    probeWindowsUs[index];
                clusterRawHz = (uint32_t)(
                    (estimateNumerator + (estimateDenominator / 2ULL)) /
                    estimateDenominator);
                clusterHintHz = (uint32_t)(
                    (((uint64_t)frameClusterCount * 500000ULL) +
                     (probeWindowsUs[index] / 2U)) /
                    probeWindowsUs[index]);
                frameClusterEstimated = roundFrequency100WithHint(
                    clusterRawHz, clusterHintHz);
                /*
                 * At the specified 100 kHz upper boundary, lens compression
                 * can make a correct four-crossing 20 us train calculate as
                 * 102..106 kHz. Preserve a coherent boundary vote instead of
                 * discarding it as out of range. Lower in-band frequencies
                 * remain untouched.
                 */
                if ((probeWindowsUs[index] == 20U) &&
                    (frameClusterCount == 4U) &&
                    (clusterRawHz > F_FREQ_MAX_HZ) &&
                    (clusterRawHz <=
                     CCD_FREQ_BOUNDARY_OVERSHOOT_HZ)) {
                    frameClusterEstimated = F_FREQ_MAX_HZ;
                }
                if ((frameClusterEstimated >= F_FREQ_MIN_HZ) &&
                    (frameClusterEstimated <= F_FREQ_MAX_HZ)) {
                    clusterCandidates[clusterCandidateCount++] =
                        frameClusterEstimated;
                } else {
                    frameClusterEstimated = 0U;
                }
            }

            if ((frameLocalPointCount >= CCD_FREQ_MIN_POINTS) &&
                (frameLocalPointCount <= CCD_FREQ_MAX_POINTS) &&
                ccdFrequencyTrainAccept(
                    probeWindowsUs[index], frameLocalPointCount,
                    frameMinimumGap, frameMaximumGap, trainSpan) &&
                ((frameStats.maximum - frameStats.minimum) >= 100U)) {
                frameEffectivePointCount = (uint8_t)(
                    (((uint32_t)activeSpanTimes2 *
                      (uint32_t)(frameLocalPointCount - 1U)) +
                     trainSpan) / (2U * trainSpan));
                if ((frameEffectivePointCount >= CCD_FREQ_MIN_POINTS) &&
                    (frameEffectivePointCount <= CCD_FREQ_MAX_POINTS)) {
                    estimateNumerator =
                        (uint64_t)(frameLocalPointCount - 1U) *
                        activeSpanTimes2 * 250000ULL;
                    estimateDenominator =
                        (uint64_t)trainSpan * probeWindowsUs[index];
                    localRawHz = (uint32_t)(
                        (estimateNumerator +
                         (estimateDenominator / 2ULL)) /
                        estimateDenominator);
                    localHintHz = (uint32_t)(
                        (((uint64_t)frameEffectivePointCount * 500000ULL) +
                         (probeWindowsUs[index] / 2U)) /
                        probeWindowsUs[index]);
                    frameLocalEstimated = roundFrequency100WithHint(
                        localRawHz, localHintHz);
                    if ((frameLocalEstimated >= F_FREQ_MIN_HZ) &&
                        (frameLocalEstimated <= F_FREQ_MAX_HZ)) {
                        localCandidates[localCandidateCount++] =
                            frameLocalEstimated;
                    } else {
                        frameLocalEstimated = 0U;
                    }

                    fitHz = ccdEstimateSubpixelFit(
                        gCCDCorrected, frameCenters,
                        frameLocalPointCount, probeWindowsUs[index]);
                    if ((fitHz >= F_FREQ_MIN_HZ) &&
                        (fitHz <= F_FREQ_MAX_HZ)) {
                        fitCandidates[fitCandidateCount] = fitHz;
                        fitCandidatePoints[fitCandidateCount] =
                            frameLocalPointCount;
                        fitCandidateCount++;
                        combHz = ccdRefineFrequencyComb(
                            gCCDCorrected, fitHz,
                            probeWindowsUs[index]);
                        if ((combHz >= F_FREQ_MIN_HZ) &&
                            (combHz <= F_FREQ_MAX_HZ)) {
                            combCandidates[combCandidateCount++] =
                                combHz;
                        } else {
                            combHz = 0U;
                        }
                    } else {
                        fitHz = 0U;
                    }
                }
            }

            APP_UART_PRINTF(
                "CCD_FRAME window_us=%u frame=%u peaks=%u span=%u clusters=%u "
                "cluster_gap=%u..%u cluster_span=%lu raw=%u "
                "regular=%u gap=%u..%u train_span=%lu "
                "cluster_hz=%lu cluster_hint_hz=%lu local_hz=%lu "
                "fit_hz=%lu comb_hz=%lu\r\n",
                (unsigned int)probeWindowsUs[index],
                (unsigned int)(frame + 1U),
                (unsigned int)frameStats.peakCount,
                (unsigned int)(frameStats.maximum - frameStats.minimum),
                (unsigned int)frameClusterCount,
                (unsigned int)frameClusterMinimumGap,
                (unsigned int)frameClusterMaximumGap,
                (unsigned long)frameClusterTrainSpan,
                (unsigned int)frameRawLocalPointCount,
                (unsigned int)frameLocalPointCount,
                (unsigned int)frameMinimumGap,
                (unsigned int)frameMaximumGap,
                (unsigned long)trainSpan,
                (unsigned long)frameClusterEstimated,
                (unsigned long)clusterHintHz,
                (unsigned long)frameLocalEstimated,
                (unsigned long)fitHz,
                (unsigned long)combHz);

            if ((frameClusterCount > clusterCount) ||
                ((frameClusterCount == clusterCount) &&
                 (frameClusterTrainSpan > clusterTrainSpan))) {
                clusterCount = frameClusterCount;
                clusterMinimumGap = frameClusterMinimumGap;
                clusterMaximumGap = frameClusterMaximumGap;
                clusterTrainSpan = frameClusterTrainSpan;
                for (point = 0U; point < frameClusterCount; point++) {
                    clusterCenters[point] = frameClusterCenters[point];
                }
            }
            if ((frameLocalPointCount > localPointCount) ||
                ((frameLocalPointCount == localPointCount) &&
                 (trainSpan > ((localPointCount >= 2U) ?
                    (uint32_t)(localCenters[localPointCount - 1U] -
                        localCenters[0]) : 0U)))) {
                stats = frameStats;
                width = frameWidth;
                rawLocalPointCount = frameRawLocalPointCount;
                localPointCount = frameLocalPointCount;
                prominenceThreshold = frameProminenceThreshold;
                minimumGap = frameMinimumGap;
                maximumGap = frameMaximumGap;
                for (point = 0U; point < frameLocalPointCount; point++) {
                    localCenters[point] = frameCenters[point];
                }
            }
        }
        trainSpan = (localPointCount >= 2U) ?
            (uint32_t)(localCenters[localPointCount - 1U] -
                localCenters[0]) : 0U;

        APP_UART_PRINTF(
            "CCD_PROBE window_us=%u clusters=%u cluster_gap=%u..%u "
            "cluster_span=%lu "
            "raw_points=%u "
            "regular_points=%u prominence=%u gap=%u..%u width=%lu "
            "span=%u train_span=%lu centers=",
            (unsigned int)probeWindowsUs[index],
            (unsigned int)clusterCount,
            (unsigned int)clusterMinimumGap,
            (unsigned int)clusterMaximumGap,
            (unsigned long)clusterTrainSpan,
            (unsigned int)rawLocalPointCount,
            (unsigned int)localPointCount,
            (unsigned int)prominenceThreshold,
            (unsigned int)minimumGap, (unsigned int)maximumGap,
            (unsigned long)width,
            (unsigned int)(stats.maximum - stats.minimum),
            (unsigned long)trainSpan);
        for (point = 0U; point < localPointCount; point++) {
            APP_UART_PRINTF("%u%s", (unsigned int)localCenters[point],
                ((point + 1U) < localPointCount) ? "," : "");
        }
        APP_UART_PRINTF("\r\n");

        APP_UART_PRINTF(
            "CCD_CONSENSUS window_us=%u cluster_votes=",
            (unsigned int)probeWindowsUs[index]);
        for (frame = 0U; frame < clusterCandidateCount; frame++) {
            APP_UART_PRINTF("%lu%s",
                (unsigned long)clusterCandidates[frame],
                ((frame + 1U) < clusterCandidateCount) ? "," : "");
        }
        APP_UART_PRINTF(" local_votes=");
        for (frame = 0U; frame < localCandidateCount; frame++) {
            APP_UART_PRINTF("%lu%s",
                (unsigned long)localCandidates[frame],
                ((frame + 1U) < localCandidateCount) ? "," : "");
        }
        APP_UART_PRINTF(" fit_votes=");
        for (frame = 0U; frame < fitCandidateCount; frame++) {
            APP_UART_PRINTF("%lu%s",
                (unsigned long)fitCandidates[frame],
                ((frame + 1U) < fitCandidateCount) ? "," : "");
        }
        APP_UART_PRINTF(" comb_votes=");
        for (frame = 0U; frame < combCandidateCount; frame++) {
            APP_UART_PRINTF("%lu%s",
                (unsigned long)combCandidates[frame],
                ((frame + 1U) < combCandidateCount) ? "," : "");
        }
        APP_UART_PRINTF("\r\n");

        /*
         * Preserve early high-frequency evidence even when it has not yet
         * reached the normal three-frame quorum. If the following 20 us probe
         * also cannot decide, the caller should retry the short windows
         * instead of allowing long-window persistence aliases to report a
         * false 50/60 kHz result for a 90 kHz source.
         */
        if (probeWindowsUs[index] == 50U) {
            for (frame = 0U; frame < localCandidateCount; frame++) {
                if (localCandidates[frame] >= CCD_FREQ_HIGH_ROUTE_HZ) {
                    highRouteEvidence = true;
                }
            }
            for (frame = 0U; frame < fitCandidateCount; frame++) {
                if (fitCandidates[frame] >= CCD_FREQ_HIGH_ROUTE_HZ) {
                    highRouteEvidence = true;
                }
            }
        } else if (probeWindowsUs[index] == 20U) {
            for (frame = 0U; frame < clusterCandidateCount; frame++) {
                if (clusterCandidates[frame] >= CCD_FREQ_HIGH_ROUTE_HZ) {
                    highRouteEvidence = true;
                }
            }
        }

        /*
         * The regular optical clusters are zero crossings: there are two per
         * input cycle. Accept a window only when at least three independent
         * frames agree within one 100 Hz source step. This removes the old
         * bias toward a single persistence-smeared frame with a large span.
         * Keep the two peak detectors as separate voting populations because
         * local shoulders can otherwise double the apparent crossing count.
         */
        consensusSupport = 0U;
        clusterConsensusSupport = 0U;
        highbandConsensusSupport = 0U;
        localHighbandSupport = 0U;
        fitHighbandSupport = 0U;
        localMeanSelected = false;
        localStrictSelected = false;
        localStrictAccepted = false;
        combStrictAccepted = false;
        boundaryPairSelected = false;
        fitSelected = false;
        highbandFusionSelected = false;
        highbandFitSelected = false;
        highbandLocalSelected = false;
        highbandSparseSelected = false;
        highbandAliasOverride = false;
        modelHighbandSelected = false;
        clusterConsensusAccepted = false;
        highbandConsensusAccepted = false;
        localHighbandAccepted = false;
        fitHighbandAccepted = false;
        estimateAccepted = false;

        /*
         * Do not decide the 10..40 kHz band from one persistence state.
         * Collect dense least-squares fits from both 300 us and 200 us. Most
         * optical failures merge a shoulder and bias frequency downward, so
         * the 80th percentile rejects the low tail while still discarding the
         * highest 20 percent as positive outliers.
         */
        if ((probeWindowsUs[index] == 300U) ||
            (probeWindowsUs[index] == 200U)) {
            /*
             * A 10 kHz source paints six reliable crossings in 300 us.
             * Requiring eight discarded nine tightly grouped 9.93..10.21 kHz
             * fits. Six points still provide five intervals for regression.
             */
            midbandMinimumPoints = 6U;
            midbandBeforeCount = midbandCandidateCount;
            for (frame = 0U; frame < fitCandidateCount; frame++) {
                if ((fitCandidatePoints[frame] >= midbandMinimumPoints) &&
                    (midbandCandidateCount <
                     (CCD_FREQ_SAMPLE_FRAMES * 2U))) {
                    midbandCandidates[midbandCandidateCount++] =
                        fitCandidates[frame];
                }
            }
            APP_UART_PRINTF(
                "CCD_MIDBAND collect window_us=%u added=%u total=%u\r\n",
                (unsigned int)probeWindowsUs[index],
                (unsigned int)(
                    midbandCandidateCount - midbandBeforeCount),
                (unsigned int)midbandCandidateCount);

            if (probeWindowsUs[index] == 300U) {
                APP_UART_PRINTF(
                    "CCD_MIDBAND pending=200us required=%u\r\n",
                    (unsigned int)CCD_FREQ_MIN_CONSENSUS_FRAMES);
                continue;
            }

            /*
             * At 10 kHz the 200 us trace gives only four crossing points, but
             * its three subpixel fits repeatedly agreed at 9.96..10.12 kHz.
             * Prefer that strict 100 Hz consensus over the positively biased
             * 10.43..10.55 kHz fits from the 300 us persistence state.
             */
            if (ccdSelectFrequencyConsensus(
                    fitCandidates, fitCandidateCount,
                    &localStrictConsensusHz, &localStrictSupport) &&
                (localStrictConsensusHz >=
                 CCD_FREQ_200US_LOW_MIN_HZ) &&
                (localStrictConsensusHz <=
                 CCD_FREQ_200US_LOW_MAX_HZ)) {
                estimated = roundFrequency100(localStrictConsensusHz);
                consensusSupport = localStrictSupport;
                estimateAccepted = true;
                fitSelected = true;
                localStrictSelected = true;
                APP_UART_PRINTF(
                    "CCD_LOWBAND_200US selected_hz=%lu "
                    "support=%u/%u\r\n",
                    (unsigned long)estimated,
                    (unsigned int)consensusSupport,
                    (unsigned int)fitCandidateCount);
            }

            /*
             * Do not let a 300 us-only population decide 10 kHz when 200 us
             * contributed no dense fits. Continue to the 500 us strict local
             * consensus instead.
             */
            if (!estimateAccepted &&
                (midbandCandidateCount == midbandBeforeCount)) {
                APP_UART_PRINTF(
                    "CCD_MIDBAND defer=500us reason=no_200us_addition\r\n");
                continue;
            }

            if (!estimateAccepted &&
                (midbandCandidateCount >=
                 CCD_FREQ_MIN_CONSENSUS_FRAMES)) {
                for (frame = 1U;
                     frame < midbandCandidateCount; frame++) {
                    sortValue = midbandCandidates[frame];
                    point = frame;
                    while ((point > 0U) &&
                        (midbandCandidates[point - 1U] > sortValue)) {
                        midbandCandidates[point] =
                            midbandCandidates[point - 1U];
                        point--;
                    }
                    midbandCandidates[point] = sortValue;
                }
                percentileIndex = (uint8_t)(
                    (((uint32_t)midbandCandidateCount - 1U) * 4U) / 5U);
                estimated = roundFrequency100(
                    midbandCandidates[percentileIndex]);
                APP_UART_PRINTF(
                    "CCD_MIDBAND sorted=");
                for (frame = 0U;
                     frame < midbandCandidateCount; frame++) {
                    APP_UART_PRINTF("%lu%s",
                        (unsigned long)midbandCandidates[frame],
                        ((frame + 1U) < midbandCandidateCount) ? "," : "");
                }
                APP_UART_PRINTF(
                    " percentile=80 index=%u raw_hz=%lu selected_hz=%lu\r\n",
                    (unsigned int)percentileIndex,
                    (unsigned long)midbandCandidates[percentileIndex],
                    (unsigned long)estimated);
                consensusSupport = midbandCandidateCount;
                estimateAccepted = true;
                fitSelected = true;
            }
        }
        if (!estimateAccepted &&
            (probeWindowsUs[index] != 200U) &&
            (probeWindowsUs[index] != 300U)) {
            clusterConsensusAccepted = ccdSelectFrequencyConsensus(
                clusterCandidates, clusterCandidateCount,
                &clusterConsensusHz, &clusterConsensusSupport);
        }

        /*
         * At 10 kHz the 500 us endpoint detector repeatedly produced four
         * exact 10.0 kHz votes while the broad cluster detector saw only the
         * persistence center line. Accept this narrow-window local result
         * only through the strict 100 Hz consensus helper. Its split check
         * still rejects the earlier mixed 8.2/10.0 kHz topology.
         */
        if (!estimateAccepted && (probeWindowsUs[index] == 500U)) {
            localStrictAccepted = ccdSelectFrequencyConsensus(
                localCandidates, localCandidateCount,
                &localStrictConsensusHz, &localStrictSupport);
            combStrictAccepted = ccdSelectFrequencyConsensus(
                combCandidates, combCandidateCount,
                &combStrictConsensusHz, &combStrictSupport);

            /*
             * The final 10 kHz bench run gave a stable 10.5 kHz endpoint
             * population but a 10.4 kHz comb population. They describe the
             * same optical train; prefer the conservative member only when
             * the two independent models agree within five percent. This
             * removes the repeatable positive edge bias without introducing
             * another frequency calibration table.
             */
            if (localStrictAccepted && combStrictAccepted) {
                modelDifference =
                    (localStrictConsensusHz >= combStrictConsensusHz) ?
                        (localStrictConsensusHz - combStrictConsensusHz) :
                        (combStrictConsensusHz - localStrictConsensusHz);
                modelAgreementLimit =
                    ((localStrictConsensusHz >= combStrictConsensusHz) ?
                        localStrictConsensusHz : combStrictConsensusHz) *
                    CCD_FREQ_MODEL_AGREEMENT_PERCENT / 100U;
                if (modelDifference <= modelAgreementLimit) {
                    estimated =
                        (localStrictConsensusHz <= combStrictConsensusHz) ?
                            localStrictConsensusHz :
                            combStrictConsensusHz;
                    consensusSupport =
                        (localStrictSupport <= combStrictSupport) ?
                            localStrictSupport : combStrictSupport;
                    estimateAccepted = true;
                    localStrictSelected = true;
                    APP_UART_PRINTF(
                        "CCD_LOWBAND_DUAL local_hz=%lu local_support=%u/%u "
                        "comb_hz=%lu comb_support=%u/%u selected_hz=%lu\r\n",
                        (unsigned long)localStrictConsensusHz,
                        (unsigned int)localStrictSupport,
                        (unsigned int)localCandidateCount,
                        (unsigned long)combStrictConsensusHz,
                        (unsigned int)combStrictSupport,
                        (unsigned int)combCandidateCount,
                        (unsigned long)estimated);
                }
            } else if (localStrictAccepted) {
                estimated = localStrictConsensusHz;
                consensusSupport = localStrictSupport;
                estimateAccepted = true;
                localStrictSelected = true;
                APP_UART_PRINTF(
                    "CCD_LOWBAND_LOCAL selected_hz=%lu support=%u/%u\r\n",
                    (unsigned long)estimated,
                    (unsigned int)consensusSupport,
                    (unsigned int)localCandidateCount);
            } else if (combStrictAccepted) {
                estimated = combStrictConsensusHz;
                consensusSupport = combStrictSupport;
                estimateAccepted = true;
                localStrictSelected = true;
                APP_UART_PRINTF(
                    "CCD_LOWBAND_COMB selected_hz=%lu support=%u/%u\r\n",
                    (unsigned long)estimated,
                    (unsigned int)consensusSupport,
                    (unsigned int)combCandidateCount);
            }
        }

        /*
         * A 20 us boundary trace may contain only one or two complete
         * four-point trains. They are not sufficient on their own, but they
         * become safe when a preceding >=98 kHz 50 us result independently
         * identifies the 100 kHz endpoint. Two short-window votes select
         * their mean; one vote is averaged with the independent 50 us result.
         */
        if (!estimateAccepted &&
            (probeWindowsUs[index] == 20U) &&
            (bestFrequency >= CCD_FREQ_BOUNDARY_VERIFY_HZ) &&
            (clusterCandidateCount >= 1U) &&
            (clusterCandidateCount <= 2U)) {
            if (clusterCandidateCount == 2U) {
                boundaryPairHz = roundFrequency100(
                    (clusterCandidates[0] +
                     clusterCandidates[1] + 1U) / 2U);
                boundaryPairDifference =
                    (clusterCandidates[0] >= clusterCandidates[1]) ?
                        (clusterCandidates[0] - clusterCandidates[1]) :
                        (clusterCandidates[1] - clusterCandidates[0]);
            } else {
                boundaryPairHz = clusterCandidates[0];
                boundaryPairDifference = 0U;
            }
            boundaryPairLimit =
                boundaryPairHz * CCD_FREQ_BOUNDARY_PAIR_PERCENT / 100U;
            boundaryCrossDifference =
                (boundaryPairHz >= bestFrequency) ?
                    (boundaryPairHz - bestFrequency) :
                    (bestFrequency - boundaryPairHz);
            boundaryCrossLimit =
                bestFrequency * CCD_FREQ_BOUNDARY_CROSS_PERCENT / 100U;
            if ((boundaryPairDifference <= boundaryPairLimit) &&
                (boundaryCrossDifference <= boundaryCrossLimit)) {
                estimated = (clusterCandidateCount == 2U) ?
                    boundaryPairHz :
                    roundFrequency100(
                        (bestFrequency + boundaryPairHz + 1U) / 2U);
                consensusSupport = clusterCandidateCount;
                estimateAccepted = true;
                boundaryPairSelected = true;
                APP_UART_PRINTF(
                    "CCD_BOUNDARY_VOTE votes=%u vote_hz=%lu "
                    "selected_hz=%lu pair_diff_hz=%lu "
                    "pair_limit_hz=%lu prior_50us_hz=%lu "
                    "cross_diff_hz=%lu cross_limit_hz=%lu\r\n",
                    (unsigned int)clusterCandidateCount,
                    (unsigned long)boundaryPairHz,
                    (unsigned long)estimated,
                    (unsigned long)boundaryPairDifference,
                    (unsigned long)boundaryPairLimit,
                    (unsigned long)bestFrequency,
                    (unsigned long)boundaryCrossDifference,
                    (unsigned long)boundaryCrossLimit);
            }
        }

        /*
         * Dense high-frequency trains do not always form broad optical
         * clusters. Build independent relative-consensus estimates from the
         * endpoint spacing and subpixel fits. Unlike a full-range gate, this
         * keeps the three repeated 99.7 kHz votes when one 78.8 kHz shoulder
         * appears. At most one distant outlier is tolerated by the helper.
         */
        if (!estimateAccepted && (probeWindowsUs[index] <= 100U)) {
            highbandCandidateCount = 0U;
            for (frame = 0U; frame < fitCandidateCount; frame++) {
                if ((fitCandidatePoints[frame] >=
                     ((probeWindowsUs[index] == 50U) ? 4U :
                        CCD_FREQ_HIGH_MIN_FIT_POINTS)) &&
                    (highbandCandidateCount < CCD_FREQ_SAMPLE_FRAMES)) {
                    highbandCandidates[highbandCandidateCount++] =
                        fitCandidates[frame];
                }
            }
            fitHighbandAccepted = ccdSelectRelativeConsensus(
                highbandCandidates, highbandCandidateCount,
                CCD_FREQ_HIGH_CONSENSUS_PERCENT,
                &fitHighbandConsensusHz, &fitHighbandSupport,
                &fitHighbandMinimumHz, &fitHighbandMaximumHz);
            localHighbandAccepted = ccdSelectRelativeConsensus(
                localCandidates, localCandidateCount,
                CCD_FREQ_HIGH_CONSENSUS_PERCENT,
                &localHighbandConsensusHz, &localHighbandSupport,
                &localHighbandMinimumHz, &localHighbandMaximumHz);

            /*
             * The moving persistence trace sometimes leaves only two valid
             * endpoint trains. Two frames are too weak as a single model, but
             * two independent local votes plus two independent subpixel-fit
             * votes form four consistent observations. Admit this reduced
             * quorum on the 50/100 us windows only when each pair and the two
             * model means agree within five percent.
             */
            if (((probeWindowsUs[index] == 50U) ||
                 (probeWindowsUs[index] == 100U)) &&
                !localHighbandAccepted && !fitHighbandAccepted &&
                (localCandidateCount == 2U) &&
                (highbandCandidateCount == 2U)) {
                localSparsePairHz = roundFrequency100(
                    (localCandidates[0] + localCandidates[1] + 1U) / 2U);
                fitSparsePairHz = roundFrequency100(
                    (highbandCandidates[0] +
                     highbandCandidates[1] + 1U) / 2U);
                localSparseDifference =
                    (localCandidates[0] >= localCandidates[1]) ?
                        (localCandidates[0] - localCandidates[1]) :
                        (localCandidates[1] - localCandidates[0]);
                fitSparseDifference =
                    (highbandCandidates[0] >= highbandCandidates[1]) ?
                        (highbandCandidates[0] - highbandCandidates[1]) :
                        (highbandCandidates[1] - highbandCandidates[0]);
                modelDifference =
                    (localSparsePairHz >= fitSparsePairHz) ?
                        (localSparsePairHz - fitSparsePairHz) :
                        (fitSparsePairHz - localSparsePairHz);
                modelAgreementLimit =
                    ((localSparsePairHz >= fitSparsePairHz) ?
                        localSparsePairHz : fitSparsePairHz) *
                    CCD_FREQ_MODEL_AGREEMENT_PERCENT / 100U;
                sparsePairLimit =
                    localSparsePairHz *
                    CCD_FREQ_HIGH_CONSENSUS_PERCENT / 100U;
                if ((localSparseDifference <= sparsePairLimit) &&
                    (fitSparseDifference <=
                     (fitSparsePairHz *
                      CCD_FREQ_HIGH_CONSENSUS_PERCENT / 100U)) &&
                    (modelDifference <= modelAgreementLimit)) {
                    localHighbandConsensusHz = localSparsePairHz;
                    fitHighbandConsensusHz = fitSparsePairHz;
                    localHighbandMinimumHz =
                        (localCandidates[0] <= localCandidates[1]) ?
                            localCandidates[0] : localCandidates[1];
                    localHighbandMaximumHz =
                        (localCandidates[0] >= localCandidates[1]) ?
                            localCandidates[0] : localCandidates[1];
                    fitHighbandMinimumHz =
                        (highbandCandidates[0] <=
                         highbandCandidates[1]) ?
                            highbandCandidates[0] :
                            highbandCandidates[1];
                    fitHighbandMaximumHz =
                        (highbandCandidates[0] >=
                         highbandCandidates[1]) ?
                            highbandCandidates[0] :
                            highbandCandidates[1];
                    localHighbandSupport = 2U;
                    fitHighbandSupport = 2U;
                    localHighbandAccepted = true;
                    fitHighbandAccepted = true;
                    highbandSparseSelected = true;
                    APP_UART_PRINTF(
                        "CCD_DUAL_MODEL_SPARSE window_us=%u "
                        "local_hz=%lu fit_hz=%lu "
                        "local_diff_hz=%lu fit_diff_hz=%lu "
                        "model_diff_hz=%lu accepted=1\r\n",
                        (unsigned int)probeWindowsUs[index],
                        (unsigned long)localSparsePairHz,
                        (unsigned long)fitSparsePairHz,
                        (unsigned long)localSparseDifference,
                        (unsigned long)fitSparseDifference,
                        (unsigned long)modelDifference);
                }
            }
            APP_UART_PRINTF(
                "CCD_HIGHBAND_LOCAL accepted=%u median_hz=%lu "
                "support=%u/%u range=%lu..%lu\r\n",
                localHighbandAccepted ? 1U : 0U,
                (unsigned long)(localHighbandAccepted ?
                    localHighbandConsensusHz : 0U),
                (unsigned int)localHighbandSupport,
                (unsigned int)localCandidateCount,
                (unsigned long)(localHighbandAccepted ?
                    localHighbandMinimumHz : 0U),
                (unsigned long)(localHighbandAccepted ?
                    localHighbandMaximumHz : 0U));
            APP_UART_PRINTF(
                "CCD_HIGHBAND_FIT accepted=%u median_hz=%lu "
                "support=%u/%u range=%lu..%lu\r\n",
                fitHighbandAccepted ? 1U : 0U,
                (unsigned long)(fitHighbandAccepted ?
                    fitHighbandConsensusHz : 0U),
                (unsigned int)fitHighbandSupport,
                (unsigned int)highbandCandidateCount,
                (unsigned long)(fitHighbandAccepted ?
                    fitHighbandMinimumHz : 0U),
                (unsigned long)(fitHighbandAccepted ?
                    fitHighbandMaximumHz : 0U));

            if (localHighbandAccepted && fitHighbandAccepted) {
                modelDifference =
                    (localHighbandConsensusHz >=
                     fitHighbandConsensusHz) ?
                    (localHighbandConsensusHz -
                     fitHighbandConsensusHz) :
                    (fitHighbandConsensusHz -
                     localHighbandConsensusHz);
                modelAgreementLimit =
                    ((localHighbandConsensusHz >=
                      fitHighbandConsensusHz) ?
                        localHighbandConsensusHz :
                        fitHighbandConsensusHz) *
                    CCD_FREQ_MODEL_AGREEMENT_PERCENT / 100U;
                if (modelDifference <= modelAgreementLimit) {
                    highbandConsensusHz = roundFrequency100(
                        (localHighbandConsensusHz +
                         fitHighbandConsensusHz) / 2U);
                    highbandConsensusSupport =
                        (localHighbandSupport <= fitHighbandSupport) ?
                            localHighbandSupport : fitHighbandSupport;
                    highbandFusionSelected = true;
                    highbandConsensusAccepted = true;
                } else if (fitHighbandSupport >
                           localHighbandSupport) {
                    highbandConsensusHz =
                        roundFrequency100(fitHighbandConsensusHz);
                    highbandConsensusSupport = fitHighbandSupport;
                    highbandFitSelected = true;
                    highbandConsensusAccepted = true;
                } else {
                    highbandConsensusHz =
                        roundFrequency100(localHighbandConsensusHz);
                    highbandConsensusSupport = localHighbandSupport;
                    highbandLocalSelected = true;
                    highbandConsensusAccepted = true;
                }
            } else if (fitHighbandAccepted) {
                highbandConsensusHz =
                    roundFrequency100(fitHighbandConsensusHz);
                highbandConsensusSupport = fitHighbandSupport;
                highbandFitSelected = true;
                highbandConsensusAccepted = true;
            } else if (localHighbandAccepted) {
                highbandConsensusHz =
                    roundFrequency100(localHighbandConsensusHz);
                highbandConsensusSupport = localHighbandSupport;
                highbandLocalSelected = true;
                highbandConsensusAccepted = true;
            }
            if (highbandConsensusAccepted) {
                APP_UART_PRINTF(
                    "CCD_HIGHBAND selected=%s estimate_hz=%lu "
                    "support=%u\r\n",
                    highbandSparseSelected ?
                        "sparse_local_fit_fusion" :
                    (highbandFusionSelected ? "local_fit_fusion" :
                        (highbandFitSelected ? "subpixel" : "local")),
                    (unsigned long)highbandConsensusHz,
                    (unsigned int)highbandConsensusSupport);
            }
        }

        /*
         * Model support is evidence, not just a diagnostic. The 80 kHz trace
         * produced ten coherent high-band frames versus three incorrect broad
         * clusters; V40 still selected 74.8 kHz solely because the estimates
         * differed by more than five percent. V41 lets a two-frame support
         * advantage override that disagreement. At 90 kHz, however, three
         * broad-cluster votes aliased to 64.2 kHz while three independent
         * local/fit votes correctly gave 89.5 kHz. When the 50 us models land
         * on opposite sides of the 85 kHz routing boundary, equal support
         * must prefer the high-band model. The 85..98 kHz range is calibrated
         * and batch-confirmed at 50 us; only the 100 kHz boundary uses 20 us.
         * For the sparse 20 us fallback an equal-support four-cluster train
         * remains preferable.
         */
        if (!estimateAccepted && (probeWindowsUs[index] <= 100U) &&
            clusterConsensusAccepted && highbandConsensusAccepted) {
            modelDifference =
                (clusterConsensusHz >= highbandConsensusHz) ?
                (clusterConsensusHz - highbandConsensusHz) :
                (highbandConsensusHz - clusterConsensusHz);
            modelAgreementLimit =
                ((clusterConsensusHz >= highbandConsensusHz) ?
                    clusterConsensusHz : highbandConsensusHz) *
                CCD_FREQ_MODEL_AGREEMENT_PERCENT / 100U;
            highbandAliasOverride =
                (probeWindowsUs[index] == 50U) &&
                (highbandConsensusHz >= CCD_FREQ_HIGH_ROUTE_HZ) &&
                (clusterConsensusHz < CCD_FREQ_HIGH_ROUTE_HZ) &&
                (modelDifference > modelAgreementLimit) &&
                (highbandConsensusSupport >= clusterConsensusSupport);
            if (((modelDifference <= modelAgreementLimit) &&
                 !((probeWindowsUs[index] == 20U) &&
                   (clusterConsensusSupport >=
                    highbandConsensusSupport))) ||
                highbandAliasOverride ||
                (highbandConsensusSupport >=
                 (uint8_t)(clusterConsensusSupport +
                    CCD_FREQ_MODEL_SUPPORT_ADVANTAGE))) {
                estimated = highbandConsensusHz;
                consensusSupport = highbandConsensusSupport;
                fitSelected = true;
                modelHighbandSelected = true;
                estimateAccepted = true;
            } else {
                estimated = clusterConsensusHz;
                consensusSupport = clusterConsensusSupport;
                highbandFusionSelected = false;
                highbandFitSelected = false;
                highbandLocalSelected = false;
                estimateAccepted = true;
            }
            APP_UART_PRINTF(
                "CCD_MODEL_FUSION cluster_hz=%lu cluster_support=%u "
                "highband_hz=%lu highband_support=%u difference_hz=%lu "
                "limit_hz=%lu alias_override=%u "
                "selected=%s selected_hz=%lu\r\n",
                (unsigned long)clusterConsensusHz,
                (unsigned int)clusterConsensusSupport,
                (unsigned long)highbandConsensusHz,
                (unsigned int)highbandConsensusSupport,
                (unsigned long)modelDifference,
                (unsigned long)modelAgreementLimit,
                highbandAliasOverride ? 1U : 0U,
                modelHighbandSelected ? "highband" : "cluster",
                (unsigned long)estimated);
        } else if (!estimateAccepted && (probeWindowsUs[index] <= 100U) &&
            highbandConsensusAccepted) {
            estimated = highbandConsensusHz;
            consensusSupport = highbandConsensusSupport;
            fitSelected = true;
            modelHighbandSelected = true;
            estimateAccepted = true;
        } else if (!estimateAccepted && clusterConsensusAccepted &&
            (probeWindowsUs[index] <= 100U) &&
            ((localCandidateCount >= 2U) ||
             (fitCandidateCount >= 2U))) {
            /*
             * A stable 60 kHz run produced six local/fit observations split
             * around 56 and 63 kHz while the broad persistence band aliased
             * to 45.1 kHz. Once an independent model has repeated evidence,
             * a rejected model consensus is a conflict, not permission for
             * the broad cluster to win alone. Route to the next exposure.
             */
            APP_UART_PRINTF(
                "CCD_MODEL_SPLIT defer=next_window cluster_hz=%lu "
                "cluster_support=%u local_count=%u fit_count=%u\r\n",
                (unsigned long)clusterConsensusHz,
                (unsigned int)clusterConsensusSupport,
                (unsigned int)localCandidateCount,
                (unsigned int)fitCandidateCount);
        } else if (!estimateAccepted && clusterConsensusAccepted) {
            estimated = clusterConsensusHz;
            consensusSupport = clusterConsensusSupport;
            highbandLocalSelected = false;
            estimateAccepted = true;
        }

        /*
         * Around 10..40 kHz the 200 us trace is narrow enough that the broad
         * cluster detector can merge or split alternating crossings. The
         * independent local detector still returned five regular trains on
         * the bench at 17 kHz. Accept its rounded five-frame mean only when
         * every frame is valid and the total spread remains bounded; this
         * prevents a single shoulder or persistence remnant from deciding the
         * result.
         */
        if (!estimateAccepted &&
            ((probeWindowsUs[index] == 200U) ||
             (probeWindowsUs[index] == 300U)) &&
            (localCandidateCount == CCD_FREQ_SAMPLE_FRAMES)) {
            localCandidateSum = 0U;
            localCandidateMinimum = localCandidates[0];
            localCandidateMaximum = localCandidates[0];
            for (frame = 0U; frame < localCandidateCount; frame++) {
                localCandidateSum += localCandidates[frame];
                if (localCandidates[frame] < localCandidateMinimum) {
                    localCandidateMinimum = localCandidates[frame];
                }
                if (localCandidates[frame] > localCandidateMaximum) {
                    localCandidateMaximum = localCandidates[frame];
                }
            }
            if ((localCandidateMaximum - localCandidateMinimum) <= 1000U) {
                estimated = roundFrequency100(
                    (localCandidateSum + (localCandidateCount / 2U)) /
                    localCandidateCount);
                consensusSupport = localCandidateCount;
                estimateAccepted = true;
                localMeanSelected = true;
            }
        }

        /*
         * The 100 us overlap window resolves 30 kHz well, but at 20 kHz its
         * four-crossing estimate was biased to 20.9 kHz and prevented the
         * purpose-built 300/200 us joint fit from running. Treat sub-25 kHz
         * results as routing hints only.
         */
        if (estimateAccepted &&
            (probeWindowsUs[index] == 100U) &&
            (estimated < CCD_FREQ_100US_DEFER_BELOW_HZ)) {
            APP_UART_PRINTF(
                "CCD_PROBE defer window_us=100 estimate_hz=%lu "
                "reason=route_to_midband\r\n",
                (unsigned long)estimated);
            estimateAccepted = false;
        }

        if (estimateAccepted) {
            bestFrequency = estimated;
            bestWindow = probeWindowsUs[index];
            bestPeaks = (fitSelected || localMeanSelected) ?
                localPointCount : clusterCount;
            APP_UART_PRINTF(
                "CCD_PROBE selected model=%s "
                "window_us=%lu clusters=%u estimate_hz=%lu support=%u/%u\r\n",
                highbandFusionSelected ? "highband_local_fit_fusion" :
                (highbandLocalSelected ? "highband_local_consensus" :
                (highbandFitSelected ? "highband_subpixel_median" :
                (boundaryPairSelected ? "cross_window_boundary_pair" :
                (localStrictSelected ? "local_strict_consensus" :
                (fitSelected ? "joint_subpixel_percentile80" :
                    (localMeanSelected ? "local_endpoint_mean" :
                        "cycle_clusters_consensus")))))),
                (unsigned long)bestWindow,
                (unsigned int)bestPeaks,
                (unsigned long)bestFrequency,
                (unsigned int)consensusSupport,
                (unsigned int)(fitSelected ? consensusSupport :
                    (localMeanSelected ? localCandidateCount :
                        clusterCandidateCount)));
            /*
             * Only a >=98 kHz 50 us estimate needs the sparse 20 us train as
             * an independent 100 kHz boundary check. Keep the 50 us value as
             * a fallback if that check cannot form consensus.
             */
            if ((probeWindowsUs[index] == 50U) &&
                (bestFrequency >= CCD_FREQ_BOUNDARY_VERIFY_HZ)) {
                APP_UART_PRINTF(
                    "CCD_BOUNDARY pending_hz=%lu verify_window_us=20\r\n",
                    (unsigned long)bestFrequency);
                continue;
            }
            if ((probeWindowsUs[index] == 20U) ||
                (probeWindowsUs[index] == 50U) ||
                (probeWindowsUs[index] == 100U)) {
                calibratedFrequency =
                    ccdCalibrateShortWindowFrequency(
                        bestFrequency, probeWindowsUs[index]);
                if (calibratedFrequency != bestFrequency) {
                    APP_UART_PRINTF(
                        "CCD_CAL window_us=%u raw_hz=%lu "
                        "calibrated_hz=%lu band=%s\r\n",
                        (unsigned int)probeWindowsUs[index],
                        (unsigned long)bestFrequency,
                        (unsigned long)calibratedFrequency,
                        (bestFrequency >=
                         CCD_FREQ_HIGH_CAL_MIN_HZ) ?
                            "85_98k" :
                        ((bestFrequency <=
                         CCD_FREQ_50US_LOW_CAL_MAX_HZ) ?
                            "35_65k" : "65_85k"));
                    bestFrequency = calibratedFrequency;
                }
            }
            break;
        }

        APP_UART_PRINTF(
            "CCD_PROBE reject=no_cluster_consensus "
            "local_only_not_frequency_safe required=%u tolerance_hz=%u\r\n",
            (unsigned int)CCD_FREQ_MIN_CONSENSUS_FRAMES,
            (unsigned int)CCD_FREQ_CONSENSUS_TOLERANCE_HZ);
        if ((probeWindowsUs[index] == 20U) &&
            (bestFrequency >= CCD_FREQ_BOUNDARY_VERIFY_HZ)) {
            APP_UART_PRINTF(
                "CCD_BOUNDARY fallback_hz=%lu reason=no_20us_consensus\r\n",
                (unsigned long)bestFrequency);
            break;
        }
        if ((probeWindowsUs[index] == 20U) &&
            (bestFrequency == 0U) && highRouteEvidence) {
            APP_UART_PRINTF(
                "CCD_HIGH_ROUTE retry_short_windows=1 "
                "reason=high_evidence_without_consensus\r\n");
            break;
        }
    }

    gCCDActiveSpanQ8 = CCD_FREQ_ACTIVE_SPAN_Q8;
    DacOutput_setCode(CCD_FREQ_IDLE_CODE);
    if (bestFrequency == 0U) {
        return false;
    }
    DacOutput_setCode(CCD_FREQ_IDLE_CODE);
    *frequencyHz = bestFrequency;
    *windowUs = bestWindow;
    *peakCount = bestPeaks;
    return true;
}

static bool ccdEstimateFrequencyConfirmed(uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint32_t *frequencyHz, uint32_t *windowUs, uint8_t *peakCount)
{
    uint32_t firstFrequency;
    uint32_t firstWindow;
    uint32_t secondFrequency;
    uint32_t secondWindow;
    uint32_t difference;
    uint32_t agreementLimit;
    uint8_t firstPeaks;
    uint8_t secondPeaks;
    bool firstAccepted;
    bool secondAccepted;

    if ((frequencyHz == NULL) || (windowUs == NULL) ||
        (peakCount == NULL)) {
        return false;
    }

    firstAccepted = ccdEstimateFrequency(exposureMs, background,
        &firstFrequency, &firstWindow, &firstPeaks);
    if (!firstAccepted) {
        /*
         * A complete first pass can coincide with an old-persistence state.
         * Retry once after the scope has painted fresh frames. Normal valid
         * measurements never pay this extra delay.
         */
        APP_UART_PRINTF(
            "CCD_CONFIRM retry=1 reason=first_pass_no_consensus "
            "settle_ms=%u\r\n",
            (unsigned int)CCD_FREQ_CONFIRM_SETTLE_MS);
        Tick_delay(CCD_FREQ_CONFIRM_SETTLE_MS);
        return ccdEstimateFrequency(exposureMs, background,
            frequencyHz, windowUs, peakCount);
    }

    if ((firstWindow != 50U) ||
        (firstFrequency < CCD_FREQ_CONFIRM_MIN_HZ)) {
        *frequencyHz = firstFrequency;
        *windowUs = firstWindow;
        *peakCount = firstPeaks;
        return true;
    }

    /*
     * At 90 kHz two individually confident batches returned 87.7 and
     * 90.6 kHz because the first still represented the previous persistence
     * topology. Confirm only this vulnerable high-frequency 50 us result.
     */
    Tick_delay(CCD_FREQ_CONFIRM_SETTLE_MS);
    secondAccepted = ccdEstimateFrequency(exposureMs, background,
        &secondFrequency, &secondWindow, &secondPeaks);
    if (!secondAccepted) {
        APP_UART_PRINTF(
            "CCD_CONFIRM first_hz=%lu second=invalid selected=first\r\n",
            (unsigned long)firstFrequency);
        *frequencyHz = firstFrequency;
        *windowUs = firstWindow;
        *peakCount = firstPeaks;
        return true;
    }

    difference = (firstFrequency >= secondFrequency) ?
        (firstFrequency - secondFrequency) :
        (secondFrequency - firstFrequency);
    agreementLimit =
        ((firstFrequency >= secondFrequency) ?
            firstFrequency : secondFrequency) *
        CCD_FREQ_CONFIRM_AGREEMENT_PERCENT / 100U;
    if (difference <= agreementLimit) {
        *frequencyHz = roundFrequency100(
            (firstFrequency + secondFrequency) / 2U);
    } else {
        /* The later batch has one more full persistence-settle interval. */
        *frequencyHz = secondFrequency;
    }
    *windowUs = secondWindow;
    *peakCount = secondPeaks;
    APP_UART_PRINTF(
        "CCD_CONFIRM first_hz=%lu second_hz=%lu difference_hz=%lu "
        "limit_hz=%lu selected_hz=%lu rule=%s\r\n",
        (unsigned long)firstFrequency,
        (unsigned long)secondFrequency,
        (unsigned long)difference,
        (unsigned long)agreementLimit,
        (unsigned long)*frequencyHz,
        (difference <= agreementLimit) ? "average" : "settled_second");
    return true;
}

static uint32_t ccdEvaluateFrequency(uint32_t frequencyHz,
    uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT])
{
    TSL1401_Stats stats;
    uint32_t width;
    uint32_t score;

    if ((frequencyHz < F_FREQ_MIN_HZ) ||
        (frequencyHz > F_FREQ_MAX_HZ)) {
        return CCD_AUTO_BAD_SCORE;
    }

    (void)background;
    /*
     * At the correct 100 Hz-grid candidate the XY trace is stationary and
     * intersects the CCD in a narrow cluster. A +/-100 Hz error completes
     * four beat cycles during the 40 ms exposure and paints a broad band.
     */
    applySingleOutput(frequencyHz,
        ampFromDiv(8U, frequencyHz), F_PHASE_90_WORD);
    Tick_delay(CCD_AUTO_FREQ_SETTLE_MS);
    if (!TSL1401_capture(gCCDPixels, exposureMs)) {
        return CCD_AUTO_BAD_SCORE;
    }
    TSL1401_analyze(gCCDPixels,
        CCD_FREQ_CLUSTER_THRESHOLD_PERCENT, &stats);
    if ((stats.peakCount == 0U) ||
        ((stats.maximum - stats.minimum) < 100U)) {
        APP_UART_PRINTF(
            "CCD_FINE candidate=%lu peaks=%u width=0 span=%u score=%lu\r\n",
            (unsigned long)frequencyHz, (unsigned int)stats.peakCount,
            (unsigned int)(stats.maximum - stats.minimum),
            (unsigned long)(CCD_AUTO_BAD_SCORE - 1U));
        return CCD_AUTO_BAD_SCORE - 1U;
    }

    width = ccdStatsWidth(&stats);
    if (width < CCD_FINE_MIN_WIDTH) {
        APP_UART_PRINTF(
            "CCD_FINE candidate=%lu peaks=%u width=%lu span=%u "
            "score=%lu reason=no_dds_trace\r\n",
            (unsigned long)frequencyHz, (unsigned int)stats.peakCount,
            (unsigned long)width,
            (unsigned int)(stats.maximum - stats.minimum),
            (unsigned long)(CCD_AUTO_BAD_SCORE - 1U));
        return CCD_AUTO_BAD_SCORE - 1U;
    }
    score = width * 100U;
    APP_UART_PRINTF(
        "CCD_FINE candidate=%lu peaks=%u width=%lu min=%u max=%u "
        "span=%u score=%lu\r\n",
        (unsigned long)frequencyHz, (unsigned int)stats.peakCount,
        (unsigned long)width,
        (unsigned int)stats.minimum, (unsigned int)stats.maximum,
        (unsigned int)(stats.maximum - stats.minimum),
        (unsigned long)score);
    return score;
}

static uint32_t ccdEvaluateFrequencyDifferential(uint32_t frequencyHz)
{
    TSL1401_Stats stats;
    uint32_t width;
    uint32_t score;

    /*
     * With scope persistence set to MIN, a matched DDS candidate paints only
     * narrow intersections on the CCD while a +/-100 Hz error sweeps phase
     * four times during one 40 ms exposure and paints almost the full line.
     * Use the raw optical width. A zero-amplitude background is invalid here:
     * it paints the scope's center horizontal line directly onto the CCD.
     */
    applySingleOutput(frequencyHz,
        ampFromDiv(8U, frequencyHz), 0U);
    Tick_delay(CCD_FINE_DIFF_ACTIVE_SETTLE_MS);
    if (!TSL1401_captureFiltered(
            gCCDPixels, CCD_FINE_DIFF_EXPOSURE_MS, 3U)) {
        return CCD_AUTO_BAD_SCORE;
    }
    TSL1401_analyze(gCCDPixels,
        CCD_FINE_DIFF_THRESHOLD_PERCENT, &stats);
    if ((stats.peakCount == 0U) ||
        ((stats.maximum - stats.minimum) < 80U)) {
        APP_UART_PRINTF(
            "CCD_FINE_DIFF candidate_hz=%lu peaks=%u contrast=%u "
            "score=%lu reason=no_trace\r\n",
            (unsigned long)frequencyHz,
            (unsigned int)stats.peakCount,
            (unsigned int)(stats.maximum - stats.minimum),
            (unsigned long)(CCD_AUTO_BAD_SCORE - 1U));
        return CCD_AUTO_BAD_SCORE - 1U;
    }

    width = ccdStatsWidth(&stats);
    score = width * 100U;

    APP_UART_PRINTF(
        "CCD_FINE_DIFF candidate_hz=%lu peaks=%u width=%lu "
        "contrast=%u score=%lu\r\n",
        (unsigned long)frequencyHz,
        (unsigned int)stats.peakCount,
        (unsigned long)width,
        (unsigned int)(stats.maximum - stats.minimum),
        (unsigned long)score);
    return score;
}

static uint32_t ccdSearchFrequency(uint32_t coarseFrequencyHz,
    uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT])
{
    uint32_t bestFrequency;
    uint32_t bestScore;

    /*
     * The coarse probe leaves the relay on the DAC ramp. Prime the DDS path
     * before timing individual candidates so the first score is not a stale
     * ramp image.
     */
    applySingleOutput(coarseFrequencyHz,
        ampFromDiv(8U, coarseFrequencyHz), F_PHASE_90_WORD);
    Tick_delay(CCD_FINE_PRIME_MS);
    bestFrequency = ccdSearchPass(coarseFrequencyHz,
        CCD_FINE_SEARCH_RADIUS_HZ, F_FREQ_STEP_HZ,
        exposureMs, background, &bestScore);
    APP_UART_PRINTF(
        "CCD_SEARCH pass=fine center=%lu radius=%u step=100 "
        "best=%lu score=%lu\r\n",
        (unsigned long)coarseFrequencyHz,
        (unsigned int)CCD_FINE_SEARCH_RADIUS_HZ,
        (unsigned long)bestFrequency, (unsigned long)bestScore);
    if (bestScore >= (CCD_AUTO_BAD_SCORE - 1U)) {
        return 0U;
    }
    return bestFrequency;
}

static uint32_t ccdSearchPass(uint32_t centerFrequencyHz,
    uint32_t radiusHz, uint32_t stepHz, uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint32_t *bestScore)
{
    int32_t start;
    int32_t end;
    int32_t candidate;
    uint32_t score;
    uint32_t selected = centerFrequencyHz;
    uint32_t selectedScore = CCD_AUTO_BAD_SCORE;

    start = (int32_t)centerFrequencyHz - (int32_t)radiusHz;
    end = (int32_t)centerFrequencyHz + (int32_t)radiusHz;
    if (start < (int32_t)F_FREQ_MIN_HZ) {
        start = (int32_t)F_FREQ_MIN_HZ;
    }
    if (end > (int32_t)F_FREQ_MAX_HZ) {
        end = (int32_t)F_FREQ_MAX_HZ;
    }
    start = (int32_t)roundFrequency100((uint32_t)start);

    for (candidate = start; candidate <= end;
         candidate += (int32_t)stepHz) {
        score = ccdEvaluateFrequency(
            (uint32_t)candidate, exposureMs, background);
        if (score < (CCD_AUTO_BAD_SCORE - 1U)) {
            score += (uint32_t)abs(
                candidate - (int32_t)centerFrequencyHz) * 2U;
        }
        if ((score < selectedScore) ||
            ((score == selectedScore) &&
             (abs(candidate - (int32_t)centerFrequencyHz) <
              abs((int32_t)selected - (int32_t)centerFrequencyHz)))) {
            selectedScore = score;
            selected = (uint32_t)candidate;
        }
    }

    *bestScore = selectedScore;
    return roundFrequency100(selected);
}

static uint16_t ccdSearchPhase(FMode mode, uint32_t inputFrequencyHz,
    uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT])
{
    uint32_t score;
    uint32_t confirmScore;
    uint32_t bestScore = CCD_AUTO_BAD_SCORE;
    uint32_t acceptableScore;
    uint16_t bestPhase = 0U;
    uint16_t phase;

    /*
     * During the initial line search, accept only a strong, long optical
     * span. A merely recognizable line can still be a visibly wide ellipse.
     * If no phase reaches this stronger threshold, the complete scan still
     * returns the best measured phase.
     */
    acceptableScore = (mode == F_MODE_AUTO_LINE) ?
        CCD_PHASE_STRONG_LINE_SCORE : CCD_TRACK_GOOD_CURVE_SCORE;
    for (phase = 0U; phase < 16384U; phase = (uint16_t)(phase + 512U)) {
        score = ccdEvaluatePhase(mode, inputFrequencyHz,
            phase, exposureMs, background);
        if (score < bestScore) {
            bestScore = score;
            bestPhase = phase;
        }
        if (score <= acceptableScore) {
            /*
             * Reject a one-frame phosphor/camera coincidence. The requested
             * line must satisfy the optical geometry on two consecutive
             * exposures without rewriting the DDS between them.
             */
            confirmScore = ccdMeasureShapeScore(mode, exposureMs);
            if (confirmScore <= acceptableScore) {
                APP_UART_PRINTF(
                    "CCD_PHASE confirmed_lock=%u score=%lu confirm=%lu "
                    "threshold=%lu\r\n",
                    (unsigned int)phase, (unsigned long)score,
                    (unsigned long)confirmScore,
                    (unsigned long)acceptableScore);
                return phase;
            }
            APP_UART_PRINTF(
                "CCD_PHASE reject_single_frame=%u score=%lu confirm=%lu\r\n",
                (unsigned int)phase, (unsigned long)score,
                (unsigned long)confirmScore);
        }
    }
    APP_UART_PRINTF(
        "CCD_PHASE best=%u score=%lu\r\n",
        (unsigned int)bestPhase, (unsigned long)bestScore);
    return bestPhase;
}

static uint32_t ccdEvaluatePhase(FMode mode, uint32_t inputFrequencyHz,
    uint16_t phaseWord, uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT])
{
    uint32_t outputFrequencyHz = inputFrequencyHz;

    (void)background;
    if (mode == F_MODE_AUTO_INFINITY) {
        outputFrequencyHz *= 2U;
    }
    applySingleOutput(outputFrequencyHz,
        ampFromDiv(8U, outputFrequencyHz), phaseWord);
    Tick_delay(CCD_AUTO_SETTLE_MS);
    return ccdMeasureShapeScore(mode, exposureMs);
}

static uint32_t ccdMeasureShapeScore(FMode mode, uint32_t exposureMs)
{
    TSL1401_Stats stats;
    uint32_t score;
    uint32_t width;
    uint32_t separation = 0U;
    uint32_t midpointError = TSL1401_PIXEL_COUNT;

    if (!TSL1401_captureFiltered(gCCDPixels, exposureMs, 1U)) {
        return CCD_AUTO_BAD_SCORE;
    }
    TSL1401_subtractBackground(
        gCCDPixels, gCCDBackground, gCCDCorrected);
    TSL1401_analyze(gCCDCorrected,
        CCD_PHASE_THRESHOLD_PERCENT, &stats);
    if (stats.peakCount == 0U) {
        return CCD_AUTO_BAD_SCORE - 1U;
    }

    if ((mode == F_MODE_AUTO_LINE) ||
        (mode == F_MODE_AUTO_INFINITY)) {
        /*
         * The CCD is mounted vertically through X=0. A correct diagonal line
         * crosses it once at the screen center. For y=sin(2*x+phase), both
         * X=0 crossings also coincide at one point; that point is centered
         * only when the infinity figure has the required symmetric phase.
         * Therefore line and infinity both need one compact centered peak.
         */
        width = ccdStatsWidth(&stats);
        score = (uint32_t)abs((int)stats.peakCount - 1) * 5000U;
        score += width * 100U;
        midpointError = (uint32_t)abs(
            ((int)stats.peakCenter[0] * 2) -
            (int)(TSL1401_PIXEL_COUNT - 1U));
        score += midpointError * 40U;
        return score;
    }

    width = ccdStatsWidth(&stats);
    score = (uint32_t)abs(
        (int)stats.peakCount - 2) * 5000U;
    score += width * 100U;

    if (stats.peakCount >= 2U) {
        separation =
            stats.peakCenter[stats.peakCount - 1U] - stats.peakCenter[0];
        midpointError = (uint32_t)abs(
            (int)stats.peakCenter[0] +
            (int)stats.peakCenter[stats.peakCount - 1U] -
            (int)(TSL1401_PIXEL_COUNT - 1U));
    }
    score += (TSL1401_PIXEL_COUNT - 1U - separation) * 20U;
    score += midpointError * 40U;
    return score;
}

static void serviceCCDVisualLock(void)
{
    uint16_t currentPhase;
    uint16_t bestPhase;
    uint32_t currentScore;
    uint32_t bestScore;
    uint32_t goodScore;
    uint32_t phaseStep;
    uint32_t outputFrequencyHz;
    uint16_t outputAmplitude;

    if ((gFControl.state != F_STATE_STABLE) ||
        ((gFControl.mode != F_MODE_AUTO_LINE) &&
         (gFControl.mode != F_MODE_AUTO_CIRCLE) &&
         (gFControl.mode != F_MODE_AUTO_INFINITY)) ||
        !gRelayDDSSelected ||
        (Tick_elapsed(gCCDTrackTick) < CCD_TRACK_INTERVAL_MS)) {
        return;
    }

    /*
     * The external source and DDS have independent reference clocks. Even
     * when both read exactly the same frequency, their relative phase walks
     * slowly. Bench calibration on this clock pair showed that the measured
     * CH1-to-CH2 phase naturally increases, and increasing the AD9959 phase
     * word moves it in the same direction. Therefore correction must
     * decrement the phase word.
     */
    currentPhase = gFControl.singlePhaseWord;
    /*
     * Measure the already-running output without rewriting the DDS. Rewriting
     * an unchanged phase disturbs its accumulator and paints a transient
     * trace that can make a circle look like a high-contrast line.
     */
    currentScore = ccdMeasureShapeScore(
        gFControl.mode, CCD_TRACK_EXPOSURE_MS);
    bestPhase = currentPhase;
    bestScore = currentScore;
    goodScore = (gFControl.mode == F_MODE_AUTO_LINE) ?
        CCD_TRACK_GOOD_LINE_SCORE : CCD_TRACK_GOOD_CURVE_SCORE;

    /*
     * Use a bidirectional trial controller for the line. A fixed correction
     * direction is not reliable across source restarts. Try one fine step,
     * keep going only if the next optical score improves, otherwise restore
     * the base phase and reverse direction. Require three missing frames
     * before a coarse reacquisition step so one bad exposure cannot sweep
     * away a valid lock.
     */
    if (gFControl.mode == F_MODE_AUTO_LINE) {
        if (currentScore >= (CCD_AUTO_BAD_SCORE - 1U)) {
            if (gCCDTrackTrialActive) {
                bestPhase = gCCDTrackTrialBasePhase;
                gCCDTrackDirection = (int8_t)-gCCDTrackDirection;
                gCCDTrackTrialActive = false;
                gCCDTrackBadFrames = 0U;
            } else {
                if (gCCDTrackBadFrames < 255U) {
                    gCCDTrackBadFrames++;
                }
                if (gCCDTrackBadFrames >= 3U) {
                    phaseStep = CCD_TRACK_PHASE_COARSE_STEP;
                    if (gCCDTrackDirection < 0) {
                        bestPhase = (uint16_t)(
                            ((uint32_t)currentPhase + 16384U - phaseStep) &
                            0x3FFFU);
                    } else {
                        bestPhase = (uint16_t)(
                            ((uint32_t)currentPhase + phaseStep) & 0x3FFFU);
                    }
                    gCCDTrackBadFrames = 0U;
                }
            }
        } else {
            gCCDTrackBadFrames = 0U;
            if (currentScore <= CCD_TRACK_LINE_TIGHT_SCORE) {
                gCCDTrackTrialActive = false;
            } else if (gCCDTrackTrialActive) {
                if (currentScore < gCCDTrackTrialBaseScore) {
                    gCCDTrackTrialBasePhase = currentPhase;
                    gCCDTrackTrialBaseScore = currentScore;
                    if (gCCDTrackDirection < 0) {
                        bestPhase = (uint16_t)(
                            ((uint32_t)currentPhase + 16384U -
                                CCD_TRACK_PHASE_STEP) & 0x3FFFU);
                    } else {
                        bestPhase = (uint16_t)(
                            ((uint32_t)currentPhase + CCD_TRACK_PHASE_STEP) &
                            0x3FFFU);
                    }
                } else {
                    bestPhase = gCCDTrackTrialBasePhase;
                    gCCDTrackDirection = (int8_t)-gCCDTrackDirection;
                    gCCDTrackTrialActive = false;
                }
            } else {
                gCCDTrackTrialBasePhase = currentPhase;
                gCCDTrackTrialBaseScore = currentScore;
                if (gCCDTrackDirection < 0) {
                    bestPhase = (uint16_t)(
                        ((uint32_t)currentPhase + 16384U -
                            CCD_TRACK_PHASE_STEP) & 0x3FFFU);
                } else {
                    bestPhase = (uint16_t)(
                        ((uint32_t)currentPhase + CCD_TRACK_PHASE_STEP) &
                        0x3FFFU);
                }
                gCCDTrackTrialActive = true;
            }
        }
    } else if (currentScore > goodScore) {
        bestPhase = (uint16_t)(
            ((uint32_t)currentPhase + 16384U - CCD_TRACK_PHASE_STEP) &
            0x3FFFU);
    }

    if (bestPhase != currentPhase) {
        outputFrequencyHz = gFControl.inputFreqHz;
        if (gFControl.mode == F_MODE_AUTO_INFINITY) {
            outputFrequencyHz *= 2U;
        }
        outputAmplitude = ampFromDiv(gFControl.targetDiv, outputFrequencyHz);
        applySingleOutput(outputFrequencyHz, outputAmplitude, bestPhase);
        gFControl.singlePhaseWord = bestPhase;
    }
    gCCDTrackTick = Tick_now();

    APP_UART_PRINTF(
        "CCD_TRACK target=%s phase=%u score=%lu previous=%lu action=%s\r\n",
        fModeName(gFControl.mode), (unsigned int)bestPhase,
        (unsigned long)bestScore, (unsigned long)currentScore,
        (bestPhase == currentPhase) ? "hold" : "step");
}

static uint32_t ccdStatsWidth(const TSL1401_Stats *stats)
{
    uint32_t width = 0U;
    uint8_t index;

    for (index = 0U; index < stats->peakCount; index++) {
        width += stats->peakWidth[index];
    }
    return width;
}

static uint8_t ccdCountProminentPeaks(
    const uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint8_t centers[TSL1401_MAX_PEAKS],
    uint16_t *prominenceThreshold)
{
    uint16_t minimum = 4095U;
    uint16_t maximum = 0U;
    uint16_t threshold;
    uint16_t leftMinimum;
    uint16_t rightMinimum;
    uint16_t shoulder;
    uint32_t i;
    uint32_t offset;
    uint8_t count = 0U;

    if ((pixels == NULL) || (centers == NULL)) {
        return 0U;
    }

    for (i = CCD_FREQ_PEAK_RADIUS;
         i < (TSL1401_PIXEL_COUNT - CCD_FREQ_PEAK_RADIUS); i++) {
        if (pixels[i] < minimum) {
            minimum = pixels[i];
        }
        if (pixels[i] > maximum) {
            maximum = pixels[i];
        }
    }
    threshold = (uint16_t)(
        (maximum - minimum) / CCD_FREQ_PROMINENCE_DIVISOR);
    if (threshold < CCD_FREQ_MIN_PROMINENCE) {
        threshold = CCD_FREQ_MIN_PROMINENCE;
    }
    if (prominenceThreshold != NULL) {
        *prominenceThreshold = threshold;
    }

    for (i = CCD_FREQ_PEAK_RADIUS;
         i < (TSL1401_PIXEL_COUNT - CCD_FREQ_PEAK_RADIUS); i++) {
        if ((pixels[i] < pixels[i - 1U]) ||
            (pixels[i] < pixels[i + 1U])) {
            continue;
        }

        leftMinimum = pixels[i - 1U];
        rightMinimum = pixels[i + 1U];
        for (offset = 2U; offset <= CCD_FREQ_PEAK_RADIUS; offset++) {
            if (pixels[i - offset] < leftMinimum) {
                leftMinimum = pixels[i - offset];
            }
            if (pixels[i + offset] < rightMinimum) {
                rightMinimum = pixels[i + offset];
            }
        }
        shoulder = (leftMinimum > rightMinimum) ?
            leftMinimum : rightMinimum;
        if ((pixels[i] <= shoulder) ||
            ((pixels[i] - shoulder) < threshold)) {
            continue;
        }

        if ((count > 0U) &&
            ((i - centers[count - 1U]) < CCD_FREQ_MIN_PEAK_DISTANCE)) {
            if (pixels[i] > pixels[centers[count - 1U]]) {
                centers[count - 1U] = (uint8_t)i;
            }
            continue;
        }
        if (count >= TSL1401_MAX_PEAKS) {
            break;
        }
        centers[count] = (uint8_t)i;
        count++;
    }
    return count;
}

static uint8_t ccdSelectRegularPeaks(
    uint8_t centers[TSL1401_MAX_PEAKS], uint8_t count,
    uint8_t *minimumGap, uint8_t *maximumGap)
{
    uint8_t candidate[TSL1401_MAX_PEAKS];
    uint8_t trial[TSL1401_MAX_PEAKS];
    uint8_t selected[TSL1401_MAX_PEAKS];
    uint16_t bestRatio = 0xFFFFU;
    uint16_t ratio;
    uint8_t candidateCount = 0U;
    uint8_t trialCount;
    uint8_t trialMinimum;
    uint8_t trialMaximum;
    uint8_t bestMinimum = 0U;
    uint8_t bestMaximum = 0U;
    uint8_t remove;
    uint8_t i;
    uint8_t j;

    if ((centers == NULL) || (count == 0U)) {
        if (minimumGap != NULL) {
            *minimumGap = 0U;
        }
        if (maximumGap != NULL) {
            *maximumGap = 0U;
        }
        return 0U;
    }

    for (i = 0U; i < count; i++) {
        if ((centers[i] >= CCD_FREQ_ROI_FIRST_PIXEL) &&
            (centers[i] <= CCD_FREQ_ROI_LAST_PIXEL)) {
            candidate[candidateCount++] = centers[i];
        }
    }

    /*
     * First test all ROI peaks, then every version with one peak removed.
     * A valid crossing train has max gap / min gap <= 5 / 3. This tolerates
     * lens perspective while rejecting merged traces and a fixed grid mark.
     */
    for (remove = 0U; remove <= candidateCount; remove++) {
        trialCount = 0U;
        for (i = 0U; i < candidateCount; i++) {
            if ((remove < candidateCount) && (i == remove)) {
                continue;
            }
            trial[trialCount++] = candidate[i];
        }
        if ((trialCount < CCD_FREQ_MIN_POINTS) ||
            (trialCount > CCD_FREQ_MAX_POINTS)) {
            continue;
        }

        trialMinimum = 0xFFU;
        trialMaximum = 0U;
        for (i = 1U; i < trialCount; i++) {
            uint8_t gap = (uint8_t)(trial[i] - trial[i - 1U]);
            if (gap < trialMinimum) {
                trialMinimum = gap;
            }
            if (gap > trialMaximum) {
                trialMaximum = gap;
            }
        }
        if (((uint16_t)trialMaximum * 3U) >
            ((uint16_t)trialMinimum * 5U)) {
            continue;
        }

        ratio = (uint16_t)(((uint32_t)trialMaximum * 100U) /
            trialMinimum);
        if (remove == candidateCount) {
            /*
             * The unmodified train is valid, so never discard a real edge
             * merely because a shorter subset has a slightly better ratio.
             */
            bestRatio = ratio;
            bestMinimum = trialMinimum;
            bestMaximum = trialMaximum;
            for (j = 0U; j < trialCount; j++) {
                selected[j] = trial[j];
            }
            count = trialCount;
            break;
        }
        if ((trialCount > 0U) &&
            ((bestRatio == 0xFFFFU) || (ratio < bestRatio))) {
            bestRatio = ratio;
            bestMinimum = trialMinimum;
            bestMaximum = trialMaximum;
            for (j = 0U; j < trialCount; j++) {
                selected[j] = trial[j];
            }
            count = trialCount;
        }
    }

    if (bestRatio == 0xFFFFU) {
        if (minimumGap != NULL) {
            *minimumGap = 0U;
        }
        if (maximumGap != NULL) {
            *maximumGap = 0U;
        }
        return 0U;
    }
    for (i = 0U; i < count; i++) {
        centers[i] = selected[i];
    }
    if (minimumGap != NULL) {
        *minimumGap = bestMinimum;
    }
    if (maximumGap != NULL) {
        *maximumGap = bestMaximum;
    }
    return count;
}

static uint32_t ccdEstimateSubpixelFit(
    const uint16_t pixels[TSL1401_PIXEL_COUNT],
    const uint8_t centers[TSL1401_MAX_PEAKS], uint8_t count,
    uint32_t windowUs)
{
    int64_t sumX = 0;
    int64_t sumK = 0;
    int64_t sumK2 = 0;
    int64_t sumKX = 0;
    int64_t fitNumerator;
    int64_t fitDenominator;
    uint64_t frequencyNumerator;
    uint64_t frequencyDenominator;
    uint32_t frequencyHz;
    uint8_t index;

    if ((pixels == NULL) || (centers == NULL) ||
        (count < CCD_FREQ_MIN_POINTS) || (windowUs == 0U)) {
        return 0U;
    }

    for (index = 0U; index < count; index++) {
        int32_t center = centers[index];
        int32_t centerQ8 = center * 256;
        int32_t left;
        int32_t middle;
        int32_t right;
        int32_t curvature;
        int32_t deltaQ8;

        if ((center > 0) &&
            (center < ((int32_t)TSL1401_PIXEL_COUNT - 1))) {
            left = pixels[center - 1];
            middle = pixels[center];
            right = pixels[center + 1];
            curvature = left - (2 * middle) + right;
            if (curvature < 0) {
                /*
                 * Vertex of a parabola through the three samples. Q8 retains
                 * subpixel position while the clamp prevents an asymmetric
                 * shoulder from moving a peak beyond its adjacent samples.
                 */
                deltaQ8 = ((left - right) * 128) / curvature;
                if (deltaQ8 < -128) {
                    deltaQ8 = -128;
                } else if (deltaQ8 > 128) {
                    deltaQ8 = 128;
                }
                centerQ8 += deltaQ8;
            }
        }

        sumX += centerQ8;
        sumK += index;
        sumK2 += (int64_t)index * index;
        sumKX += (int64_t)index * centerQ8;
    }

    /*
     * Least-squares slope of subpixel position versus crossing index. This
     * uses every detected crossing instead of allowing the two edge peaks to
     * determine the entire frequency estimate.
     */
    fitNumerator = ((int64_t)count * sumKX) - (sumK * sumX);
    fitDenominator = ((int64_t)count * sumK2) - (sumK * sumK);
    if ((fitNumerator <= 0) || (fitDenominator <= 0)) {
        return 0U;
    }

    /*
     * Use the span measured from the static minimum/maximum DAC lines at the
     * start of this recognition request. Both values are Q8, so their scale
     * cancels without losing the subpixel endpoint calibration.
     */
    frequencyNumerator =
        (uint64_t)gCCDActiveSpanQ8 * 500000ULL *
        (uint64_t)fitDenominator;
    frequencyDenominator =
        (uint64_t)fitNumerator * (uint64_t)windowUs;
    frequencyHz = (uint32_t)(
        (frequencyNumerator + (frequencyDenominator / 2ULL)) /
        frequencyDenominator);
    /*
     * Preserve the sub-100 Hz information until independent frames have been
     * averaged. Rounding each frame first biases a 17.04 kHz mean upward when
     * two frames happen to land just above the 17.05 kHz boundary.
     */
    return frequencyHz;
}

static uint32_t ccdRefineFrequencyComb(
    const uint16_t pixels[TSL1401_PIXEL_COUNT],
    uint32_t coarseFrequencyHz, uint32_t windowUs)
{
    const uint32_t coordinateScale = 4096U;
    const uint32_t phaseStep = 512U; /* One eighth of a pixel. */
    uint32_t startFrequency;
    uint32_t endFrequency;
    uint32_t candidateFrequency;
    uint32_t spacingQ12;
    uint32_t phaseStartQ12;
    uint32_t phaseEndQ12;
    uint32_t phaseQ12;
    uint32_t positionQ12;
    uint32_t halfPositionQ12;
    uint32_t pixel;
    uint32_t fraction;
    uint32_t value;
    uint32_t peakSum;
    uint32_t valleySum;
    uint32_t peakCount;
    uint32_t valleyCount;
    uint32_t peakAverage;
    uint32_t valleyAverage;
    uint32_t score;
    uint32_t bestScore = 0U;
    uint32_t bestFrequency = 0U;
    uint32_t distance;
    uint32_t bestDistance = UINT32_MAX;

    if ((pixels == NULL) || (windowUs == 0U) ||
        (coarseFrequencyHz < F_FREQ_MIN_HZ) ||
        (coarseFrequencyHz > F_FREQ_MAX_HZ)) {
        return 0U;
    }

    startFrequency = (coarseFrequencyHz > 1200U) ?
        (coarseFrequencyHz - 1200U) : F_FREQ_MIN_HZ;
    endFrequency = coarseFrequencyHz + 1200U;
    if (endFrequency > F_FREQ_MAX_HZ) {
        endFrequency = F_FREQ_MAX_HZ;
    }
    startFrequency =
        ((startFrequency + (F_FREQ_STEP_HZ - 1U)) / F_FREQ_STEP_HZ) *
        F_FREQ_STEP_HZ;

    for (candidateFrequency = startFrequency;
         candidateFrequency <= endFrequency;
         candidateFrequency += F_FREQ_STEP_HZ) {
        /*
         * Convert the dynamically calibrated Q8 active span to the Q12
         * coordinate system used by the comb matcher (multiply by 16).
         */
        spacingQ12 = (uint32_t)(
            (((uint64_t)gCCDActiveSpanQ8 * 500000ULL * 16ULL) +
             (((uint64_t)candidateFrequency * windowUs) / 2ULL)) /
            ((uint64_t)candidateFrequency * windowUs));
        if ((spacingQ12 < (6U * coordinateScale)) ||
            (spacingQ12 > (24U * coordinateScale))) {
            continue;
        }

        phaseStartQ12 = CCD_FREQ_ROI_FIRST_PIXEL * coordinateScale;
        phaseEndQ12 = phaseStartQ12 + spacingQ12;
        for (phaseQ12 = phaseStartQ12;
             phaseQ12 < phaseEndQ12;
             phaseQ12 += phaseStep) {
            peakSum = 0U;
            valleySum = 0U;
            peakCount = 0U;
            valleyCount = 0U;

            for (positionQ12 = phaseQ12;
                 positionQ12 <
                    ((uint32_t)CCD_FREQ_ROI_LAST_PIXEL * coordinateScale);
                 positionQ12 += spacingQ12) {
                pixel = positionQ12 / coordinateScale;
                fraction = positionQ12 % coordinateScale;
                if ((pixel + 1U) >= TSL1401_PIXEL_COUNT) {
                    break;
                }
                value = (uint32_t)(
                    (((uint64_t)pixels[pixel] *
                       (coordinateScale - fraction)) +
                     ((uint64_t)pixels[pixel + 1U] * fraction)) /
                    coordinateScale);
                peakSum += value;
                peakCount++;

                halfPositionQ12 = positionQ12 + (spacingQ12 / 2U);
                pixel = halfPositionQ12 / coordinateScale;
                fraction = halfPositionQ12 % coordinateScale;
                if (((pixel + 1U) < TSL1401_PIXEL_COUNT) &&
                    (halfPositionQ12 <
                     ((uint32_t)CCD_FREQ_ROI_LAST_PIXEL *
                      coordinateScale))) {
                    value = (uint32_t)(
                        (((uint64_t)pixels[pixel] *
                           (coordinateScale - fraction)) +
                         ((uint64_t)pixels[pixel + 1U] * fraction)) /
                        coordinateScale);
                    valleySum += value;
                    valleyCount++;
                }
            }

            if ((peakCount < CCD_FREQ_MIN_POINTS) ||
                (valleyCount == 0U)) {
                continue;
            }
            peakAverage = peakSum / peakCount;
            valleyAverage = valleySum / valleyCount;
            score = (peakAverage > valleyAverage) ?
                (peakAverage - valleyAverage) : 0U;
            distance = (candidateFrequency >= coarseFrequencyHz) ?
                (candidateFrequency - coarseFrequencyHz) :
                (coarseFrequencyHz - candidateFrequency);
            if ((score > bestScore) ||
                ((score == bestScore) && (distance < bestDistance))) {
                bestScore = score;
                bestDistance = distance;
                bestFrequency = candidateFrequency;
            }
        }
    }

    /*
     * A weak comb match means the line contains too little periodic optical
     * information; let the caller fall back instead of returning a random
     * 100 Hz grid point.
     */
    return (bestScore >= 8U) ? bestFrequency : 0U;
}

static bool ccdSelectFrequencyConsensus(
    const uint32_t candidates[CCD_FREQ_SAMPLE_FRAMES],
    uint8_t candidateCount, uint32_t *frequencyHz, uint8_t *support)
{
    static uint32_t agreed[CCD_FREQ_SAMPLE_FRAMES];
    uint32_t bestDistance = UINT32_MAX;
    uint32_t distance;
    uint32_t difference;
    uint32_t splitThreshold;
    uint32_t value;
    uint8_t bestAnchor = 0U;
    uint8_t bestSupport = 0U;
    uint8_t distantCount = 0U;
    uint8_t agreedCount = 0U;
    uint8_t i;
    uint8_t j;

    if (support != NULL) {
        *support = 0U;
    }
    if ((candidates == NULL) || (frequencyHz == NULL) ||
        (candidateCount < CCD_FREQ_MIN_CONSENSUS_FRAMES) ||
        (candidateCount > CCD_FREQ_SAMPLE_FRAMES)) {
        return false;
    }

    /*
     * Find the candidate with the strongest neighborhood. The secondary
     * distance score prefers the tightest group when two neighborhoods have
     * equal support.
     */
    for (i = 0U; i < candidateCount; i++) {
        uint8_t currentSupport = 0U;

        distance = 0U;
        for (j = 0U; j < candidateCount; j++) {
            difference = (candidates[i] >= candidates[j]) ?
                (candidates[i] - candidates[j]) :
                (candidates[j] - candidates[i]);
            if (difference <= CCD_FREQ_CONSENSUS_TOLERANCE_HZ) {
                currentSupport++;
                distance += difference;
            }
        }
        if ((currentSupport > bestSupport) ||
            ((currentSupport == bestSupport) &&
             (distance < bestDistance))) {
            bestAnchor = i;
            bestSupport = currentSupport;
            bestDistance = distance;
        }
    }
    if (bestSupport < CCD_FREQ_MIN_CONSENSUS_FRAMES) {
        return false;
    }

    /*
     * A second, distant population means the optical trace changed topology
     * during this probe. Do not let a 4-vs-2 majority turn the observed
     * 54.1/68.8 kHz split into a confident 54.1 kHz result.
     */
    splitThreshold =
        (candidates[bestAnchor] * CCD_FREQ_SPLIT_REJECT_PERCENT) / 100U;
    if (splitThreshold < 1000U) {
        splitThreshold = 1000U;
    }
    for (i = 0U; i < candidateCount; i++) {
        difference = (candidates[bestAnchor] >= candidates[i]) ?
            (candidates[bestAnchor] - candidates[i]) :
            (candidates[i] - candidates[bestAnchor]);
        if (difference > splitThreshold) {
            distantCount++;
        }
    }
    if (distantCount >= 2U) {
        return false;
    }

    for (i = 0U; i < candidateCount; i++) {
        difference = (candidates[bestAnchor] >= candidates[i]) ?
            (candidates[bestAnchor] - candidates[i]) :
            (candidates[i] - candidates[bestAnchor]);
        if (difference <= CCD_FREQ_CONSENSUS_TOLERANCE_HZ) {
            /*
             * Insertion sort is sufficient for at most ten values and keeps
             * the helper independent of a runtime-library qsort.
             */
            value = candidates[i];
            j = agreedCount;
            while ((j > 0U) && (agreed[j - 1U] > value)) {
                agreed[j] = agreed[j - 1U];
                j--;
            }
            agreed[j] = value;
            agreedCount++;
        }
    }

    *frequencyHz = agreed[agreedCount / 2U];
    if (support != NULL) {
        *support = agreedCount;
    }
    return true;
}

static bool ccdSelectRelativeConsensus(
    const uint32_t candidates[CCD_FREQ_SAMPLE_FRAMES],
    uint8_t candidateCount, uint8_t tolerancePercent,
    uint32_t *frequencyHz, uint8_t *support,
    uint32_t *minimumHz, uint32_t *maximumHz)
{
    static uint32_t agreed[CCD_FREQ_SAMPLE_FRAMES];
    uint32_t bestDistance = UINT32_MAX;
    uint32_t distance;
    uint32_t difference;
    uint32_t tolerance;
    uint32_t value;
    uint8_t bestAnchor = 0U;
    uint8_t bestSupport = 0U;
    uint8_t agreedCount = 0U;
    uint8_t currentSupport;
    uint8_t i;
    uint8_t j;

    if (support != NULL) {
        *support = 0U;
    }
    if (minimumHz != NULL) {
        *minimumHz = 0U;
    }
    if (maximumHz != NULL) {
        *maximumHz = 0U;
    }
    if ((candidates == NULL) || (frequencyHz == NULL) ||
        (candidateCount < CCD_FREQ_MIN_CONSENSUS_FRAMES) ||
        (candidateCount > CCD_FREQ_SAMPLE_FRAMES) ||
        (tolerancePercent == 0U)) {
        return false;
    }

    /*
     * Find the densest relative-frequency neighborhood. A percentage window
     * follows the optical quantization from 50 through 100 kHz better than the
     * fixed 100 Hz cluster tolerance.
     */
    for (i = 0U; i < candidateCount; i++) {
        currentSupport = 0U;
        distance = 0U;
        tolerance =
            (candidates[i] * (uint32_t)tolerancePercent) / 100U;
        if (tolerance < CCD_FREQ_CONSENSUS_TOLERANCE_HZ) {
            tolerance = CCD_FREQ_CONSENSUS_TOLERANCE_HZ;
        }
        for (j = 0U; j < candidateCount; j++) {
            difference = (candidates[i] >= candidates[j]) ?
                (candidates[i] - candidates[j]) :
                (candidates[j] - candidates[i]);
            if (difference <= tolerance) {
                currentSupport++;
                distance += difference;
            }
        }
        if ((currentSupport > bestSupport) ||
            ((currentSupport == bestSupport) &&
             (distance < bestDistance))) {
            bestAnchor = i;
            bestSupport = currentSupport;
            bestDistance = distance;
        }
    }

    /*
     * One isolated shoulder may be ignored (99.7,99.7,99.7,78.8 kHz).
     * Two or more distant candidates indicate a real topology split such as
     * the earlier 54.1/68.8 kHz population and must force another window.
     */
    if ((bestSupport < CCD_FREQ_MIN_CONSENSUS_FRAMES) ||
        ((uint8_t)(candidateCount - bestSupport) > 1U)) {
        return false;
    }

    tolerance =
        (candidates[bestAnchor] * (uint32_t)tolerancePercent) / 100U;
    if (tolerance < CCD_FREQ_CONSENSUS_TOLERANCE_HZ) {
        tolerance = CCD_FREQ_CONSENSUS_TOLERANCE_HZ;
    }
    for (i = 0U; i < candidateCount; i++) {
        difference = (candidates[bestAnchor] >= candidates[i]) ?
            (candidates[bestAnchor] - candidates[i]) :
            (candidates[i] - candidates[bestAnchor]);
        if (difference <= tolerance) {
            value = candidates[i];
            j = agreedCount;
            while ((j > 0U) && (agreed[j - 1U] > value)) {
                agreed[j] = agreed[j - 1U];
                j--;
            }
            agreed[j] = value;
            agreedCount++;
        }
    }

    if ((agreedCount & 1U) != 0U) {
        *frequencyHz = agreed[agreedCount / 2U];
    } else {
        *frequencyHz = (agreed[(agreedCount / 2U) - 1U] +
            agreed[agreedCount / 2U]) / 2U;
    }
    if (support != NULL) {
        *support = agreedCount;
    }
    if (minimumHz != NULL) {
        *minimumHz = agreed[0];
    }
    if (maximumHz != NULL) {
        *maximumHz = agreed[agreedCount - 1U];
    }
    return true;
}

static uint32_t ccdCalibrateShortWindowFrequency(
    uint32_t frequencyHz, uint32_t windowUs)
{
    /*
     * The complete current-bench sweep measured 41.5/41.7 kHz at 40 kHz,
     * 52.2/51.8 kHz at 50 kHz, and 62.4/62.8 kHz at 60 kHz. Their combined
     * true/raw ratio is 0.960. Keep this correction inside the measured
     * 35..65 kHz interval and apply it to both the primary 50 us estimate and
     * its 100 us fallback.
     *
     * The next bench sweep measured 72.6/72.7 kHz at 70.000 kHz and
     * 83.7/83.1 kHz at 80.001 kHz. Their combined correction is 0.961.
     * The 90 kHz sweep returned 89.5..96.3 kHz across the 50/20 us models.
     * A conservative 0.960 correction keeps that whole observed population
     * within the coarse-measurement tolerance. Values at or above 98 kHz
     * remain reserved for the independently reliable 100 kHz endpoint.
     */
    if (((windowUs == 20U) || (windowUs == 50U)) &&
        (frequencyHz >= CCD_FREQ_HIGH_CAL_MIN_HZ) &&
        (frequencyHz <= CCD_FREQ_HIGH_CAL_MAX_HZ)) {
        return roundFrequency100((uint32_t)(
            (((uint64_t)frequencyHz * CCD_FREQ_HIGH_CAL_NUMERATOR) +
             (CCD_FREQ_HIGH_CAL_DENOMINATOR / 2U)) /
            CCD_FREQ_HIGH_CAL_DENOMINATOR));
    }
    if ((windowUs != 50U) && (windowUs != 100U)) {
        return frequencyHz;
    }
    if ((frequencyHz < CCD_FREQ_50US_LOW_CAL_MIN_HZ) ||
        (frequencyHz > CCD_FREQ_50US_LOW_CAL_MAX_HZ)) {
        if ((windowUs != 50U) ||
            (frequencyHz < CCD_FREQ_50US_MID_CAL_MIN_HZ) ||
            (frequencyHz > CCD_FREQ_50US_MID_CAL_MAX_HZ)) {
            return frequencyHz;
        }
        return roundFrequency100((uint32_t)(
            (((uint64_t)frequencyHz *
               CCD_FREQ_50US_MID_CAL_NUMERATOR) +
             (CCD_FREQ_50US_MID_CAL_DENOMINATOR / 2U)) /
            CCD_FREQ_50US_MID_CAL_DENOMINATOR));
    }
    return roundFrequency100((uint32_t)(
        (((uint64_t)frequencyHz * CCD_FREQ_50US_LOW_CAL_NUMERATOR) +
         (CCD_FREQ_50US_LOW_CAL_DENOMINATOR / 2U)) /
        CCD_FREQ_50US_LOW_CAL_DENOMINATOR));
}

static uint32_t roundFrequency100(uint32_t frequencyHz)
{
    return ((frequencyHz + 50U) / 100U) * 100U;
}

static uint32_t roundFrequency100WithHint(
    uint32_t frequencyHz, uint32_t hintHz)
{
    uint32_t lower = (frequencyHz / 100U) * 100U;
    uint32_t upper = lower + 100U;
    uint32_t lowerDistance = frequencyHz - lower;
    uint32_t upperDistance = upper - frequencyHz;
    uint32_t roundedHint;
    uint32_t hintToLower;
    uint32_t hintToUpper;

    if (lowerDistance != upperDistance) {
        return roundFrequency100(frequencyHz);
    }

    /*
     * Pixel quantization can put a correct spacing estimate exactly halfway
     * between two allowed 100 Hz values (2 kHz produced 2050 Hz on the bench).
     * Only at that exact tie, use the independently observed crossing count
     * to choose a side. Spacing remains authoritative everywhere else.
     */
    roundedHint = roundFrequency100(hintHz);
    hintToLower = (roundedHint >= lower) ?
        (roundedHint - lower) : (lower - roundedHint);
    hintToUpper = (roundedHint >= upper) ?
        (roundedHint - upper) : (upper - roundedHint);
    return (hintToLower <= hintToUpper) ? lower : upper;
}

static void showFHelp(void)
{
    UserUART_write(
        "keys: UP=auto-frequency/electrical lock | "
        "LEFT=direct | MID=circle\r\n");
    UserUART_write(
        "keys: RIGHT=infinity | DOWN=div 2/4/6/8 | "
        "ENC=phase | ENC press=direct\r\n");
    UserUART_write("fstatus | fping\r\n");
    UserUART_write(
        "plock start FREQ_HZ [AMP] (manual output phase mode) | "
        "plock status|stop\r\n");
    UserUART_write(
        "plock circle (load start offset, then trim with encoder)\r\n");
    UserUART_write(
        "plock output same|double AMP OFFSET | plock output off\r\n");
    UserUART_write("plock phase OFFSET\r\n");
    UserUART_write("fmode off|thru|same|quad|double\r\n");
    UserUART_write("ffreq 1000..100000 | fdiv 2|4|6|8\r\n");
    UserUART_write("fphase 0..16383\r\n");
    UserUART_write("fset FREQ_HZ AMP PHASE\r\n");
    UserUART_write("fopt FTW AMP PHASE (video optical PLL fast update)\r\n");
    UserUART_write("pfdref FREQ_HZ AMP PHASE | pfdref off\r\n");
    UserUART_write("framp 1|2|5|10|off (milliseconds)\r\n");
    UserUART_write(
        "frampus 20|50|100|200|500|1000|2000|5000|10000 (microseconds)\r\n");
    UserUART_write(
        "fwindow 20|50|100|200|500|1000|2000|5000|10000|off "
        "(10ms frame, edge-band idle)\r\n");
    UserUART_write("fprobe F0 F1 F2 F3 [AMP]\r\n");
    UserUART_write(
        "fauto line|circle|infinity (TSL1401 visual closed loop)\r\n");
    UserUART_write(
        "REQ5 keys when PA16 has no input: LEFT=line MID=circle "
        "RIGHT=infinity (DK full optical run)\r\n");
    UserUART_write(
        "fstate ready|wait|search|lock|stable|error\r\n");
}

static void showFStatus(void)
{
    UserUART_printf(
        "F_STATUS mode=%s auto=%s state=%s input_hz=%lu target_div=%u "
        "phase_word=%u request=%lu dk_age_ms=%lu "
        "relay_path=%s pa13=%u dds_recoveries=%lu "
        "dac_dma_recoveries=%lu\r\n",
        fModeName(gFControl.mode), fModeName(gFControl.autoTarget),
        fStateName(gFControl.state),
        (unsigned long)gFControl.inputFreqHz,
        (unsigned int)gFControl.targetDiv,
        (unsigned int)gFControl.singlePhaseWord,
        (unsigned long)gFControl.requestId,
        (unsigned long)Tick_elapsed(gFControl.lastDKTick),
        gRelayDDSSelected ? "dds" : "direct",
        gRelayDDSSelected ? 1U : 0U,
        (unsigned long)gDDSRecoveryCount,
        (unsigned long)DacOutput_getDMARecoveryCount());
    UserUART_printf(
        "F_OUTPUT f0=%lu a0=%u p0=%u f1=%lu a1=%u p1=%u "
        "f2=%lu a2=%u p2=%u f3=%lu a3=%u p3=%u\r\n",
        (unsigned long)gFControl.outputFreqHz[0],
        (unsigned int)gFControl.outputAmp[0],
        (unsigned int)gFControl.outputPhase[0],
        (unsigned long)gFControl.outputFreqHz[1],
        (unsigned int)gFControl.outputAmp[1],
        (unsigned int)gFControl.outputPhase[1],
        (unsigned long)gFControl.outputFreqHz[2],
        (unsigned int)gFControl.outputAmp[2],
        (unsigned int)gFControl.outputPhase[2],
        (unsigned long)gFControl.outputFreqHz[3],
        (unsigned int)gFControl.outputAmp[3],
        (unsigned int)gFControl.outputPhase[3]);
    showPhaseLockStatus();
}

static void showStatus(void)
{
    UserUART_printf(
        "STATUS uptime_ms=%lu adc_busy=%u adc_ready=%u adc_rate=%lu "
        "dds_init=%u rainbow=%u sound_active=%u sound_pin=PA1\r\n",
        (unsigned long)Tick_now(), AdcCapture_isBusy() ? 1U : 0U,
        AdcCapture_isReady() ? 1U : 0U,
        (unsigned long)AdcCapture_getSampleRate(),
        gDDSInitialized ? 1U : 0U, gRainbowEnabled ? 1U : 0U,
        SoundLight_isActive() ? 1U : 0U);
    showFStatus();
}

static void serviceControls(void)
{
    BTNData_t buttons;
    bool electricalInputPresent;
    int encoderDelta;
    uint8_t nextDiv;
    uint16_t nextAmp;
    uint16_t phaseOffset;
    uint32_t detectedFrequencyHz;
    uint32_t outputFreqHz;
    uint8_t encoderPressed;

    BTN_getData(&buttons);
    encoderPressed = ENC_getSW();
    if (buttons.left || buttons.down || buttons.right ||
        buttons.up || buttons.mid || encoderPressed) {
        SoundLight_play(SOUND_LIGHT_CUE_KEY);
    }
    if (buttons.left) {
        electricalInputPresent = gPhaseLock.enabled && gPhaseLock.locked;
        if (!electricalInputPresent) {
            electricalInputPresent =
                measureInputFrequency(&detectedFrequencyHz);
        }
        if (!electricalInputPresent) {
            requestDKAutoMode(F_MODE_AUTO_LINE, "left");
        } else {
            phaseLockSelectDirect();
            UserUART_write(
                "F_EVENT key=left requirement=1 relay=direct "
                "lock=preserved\r\n");
        }
    }
    else if (buttons.down) {
        nextDiv = (uint8_t)(gFControl.targetDiv + 2U);
        if (nextDiv > 8U) nextDiv = 2U;
        gFControl.targetDiv = nextDiv;
        if (gPhaseLock.enabled && gPhaseLock.outputArmed) {
            outputFreqHz = gPhaseLock.frequencyHz;
            if (gPhaseLock.outputMode == PLOCK_OUTPUT_DOUBLE) {
                outputFreqHz *= 2U;
            }
        } else {
            outputFreqHz = gFControl.inputFreqHz;
            if (gFControl.mode == F_MODE_DOUBLE) {
                outputFreqHz *= 2U;
            }
        }
        nextAmp = ampFromDiv(gFControl.targetDiv, outputFreqHz);
        if (gPhaseLock.enabled && gPhaseLock.outputArmed) {
            phaseLockSetOutputAmplitude(nextAmp);
        } else if ((gFControl.mode == F_MODE_SAME) ||
            (gFControl.mode == F_MODE_QUAD) ||
            (gFControl.mode == F_MODE_DOUBLE)) {
            (void)applyManualMode(gFControl.mode);
        }
        UserUART_printf(
                        "F_EVENT target_div=%u asf=%u output_hz=%lu\r\n",
                        (unsigned int)gFControl.targetDiv,
                        (unsigned int)nextAmp,
                        (unsigned long)outputFreqHz);
    }
    else if (buttons.right) {
        electricalInputPresent = gPhaseLock.enabled && gPhaseLock.locked;
        if (!electricalInputPresent) {
            electricalInputPresent =
                measureInputFrequency(&detectedFrequencyHz);
        }
        if (!electricalInputPresent) {
            requestDKAutoMode(F_MODE_AUTO_INFINITY, "right");
        } else if (!gPhaseLock.enabled) {
            UserUART_write(
                "F_EVENT key=right electrical_lock=off "
                "ignored=1 action=press_up_first\r\n");
        } else if (!gPhaseLock.locked ||
                   !gPhaseLock.phaseCalibrated) {
            UserUART_write(
                "F_EVENT key=right ignored=phase_calibrating "
                "action=wait_green\r\n");
        } else {
            outputFreqHz = gPhaseLock.frequencyHz * 2U;
            nextAmp = ampFromDiv(gFControl.targetDiv, outputFreqHz);
            phaseLockArmOutput(
                PLOCK_OUTPUT_DOUBLE, nextAmp,
                phaseLockInfinityOutputOffset());
            UserUART_printf(
                "F_EVENT key=right requirement=3 freq=%lu div=%u "
                "asf=%u offset=%u state=acquiring\r\n",
                (unsigned long)outputFreqHz,
                (unsigned int)gFControl.targetDiv,
                (unsigned int)nextAmp,
                (unsigned int)gPhaseLock.outputOffset);
        }
    }
    else if (buttons.up) {
        phaseLockStop(true);
        setFState(F_STATE_SEARCHING);
        if (measureInputFrequency(&detectedFrequencyHz)) {
            phaseLockStart(detectedFrequencyHz, PLOCK_DEFAULT_ASF);
            UserUART_printf(
                "F_EVENT key=up autofreq=%lu lock=start "
                "action=wait_green\r\n",
                (unsigned long)detectedFrequencyHz);
        } else {
            relaySelectDirect();
            setFState(F_STATE_ERROR);
            UserUART_write(
                "F_EVENT key=up pa16=no_signal lock=not_started "
                "relay=direct\r\n");
        }
    }
    else if (buttons.mid) {
        electricalInputPresent = gPhaseLock.enabled && gPhaseLock.locked;
        if (!electricalInputPresent) {
            electricalInputPresent =
                measureInputFrequency(&detectedFrequencyHz);
        }
        if (!electricalInputPresent) {
            requestDKAutoMode(F_MODE_AUTO_CIRCLE, "mid");
        } else if (!gPhaseLock.enabled) {
            UserUART_write(
                "F_EVENT key=mid electrical_lock=off "
                "ignored=1 action=press_up_first\r\n");
        } else if (!gPhaseLock.locked ||
                   !gPhaseLock.phaseCalibrated) {
            UserUART_write(
                "F_EVENT key=mid ignored=phase_calibrating "
                "action=wait_green\r\n");
        } else {
            nextAmp = ampFromDiv(
                gFControl.targetDiv, gPhaseLock.frequencyHz);
            phaseOffset = phaseLockCircleOutputOffset();
            phaseLockArmOutput(
                PLOCK_OUTPUT_SAME, nextAmp, phaseOffset);
            UserUART_printf(
                "F_EVENT key=mid requirement=2 freq=%lu div=%u "
                "asf=%u offset=%u state=acquiring\r\n",
                (unsigned long)gPhaseLock.frequencyHz,
                (unsigned int)gFControl.targetDiv,
                (unsigned int)nextAmp,
                (unsigned int)phaseOffset);
        }
    }
    if (encoderPressed) {
        phaseLockSelectDirect();
        UserUART_write(
            "F_EVENT encoder=press requirement=1 relay=direct "
            "lock=preserved\r\n");
    }

    encoderDelta = ENC_getIncVal();
    if ((encoderDelta != 0) &&
        ((gPhaseLock.enabled && gPhaseLock.outputArmed) ||
         (gFControl.mode == F_MODE_SAME) ||
         (gFControl.mode == F_MODE_QUAD) ||
         (gFControl.mode == F_MODE_DOUBLE))) {
        int32_t nextPhase;
        if (gPhaseLock.enabled && gPhaseLock.outputArmed) {
            nextPhase = (int32_t)gPhaseLock.outputOffset +
                (int32_t)encoderDelta * 64;
        } else {
            nextPhase = (int32_t)gFControl.singlePhaseWord +
                (int32_t)encoderDelta * 64;
        }
        while (nextPhase < 0) nextPhase += 16384;
        while (nextPhase > 16383) nextPhase -= 16384;
        if (gPhaseLock.enabled && gPhaseLock.outputArmed) {
            phaseLockSetOutputOffset((uint16_t)nextPhase);
            UserUART_printf("F_EVENT phase_offset=%u phase=%u\r\n",
                            (unsigned int)gPhaseLock.outputOffset,
                            (unsigned int)gPhaseLock.outputPhase);
        } else {
            gFControl.singlePhaseWord = (uint16_t)nextPhase;
            (void)applyManualMode(gFControl.mode);
            UserUART_printf("F_EVENT phase_word=%u\r\n",
                            (unsigned int)gFControl.singlePhaseWord);
        }
    }
}

static bool captureInputFrequency(
    uint32_t sampleRateHz, uint32_t *frequencyHz)
{
    const uint16_t *samples;
    uint16_t minimum = 4095U;
    uint16_t maximum = 0U;
    uint16_t midpoint;
    uint16_t hysteresis;
    uint16_t lowThreshold;
    uint16_t highThreshold;
    uint32_t actualRateHz;
    uint32_t firstCrossing = 0U;
    uint32_t lastCrossing = 0U;
    uint32_t crossingCount = 0U;
    uint32_t crossingSpan;
    uint32_t startTick;
    uint32_t index;
    bool armed = false;

    if (frequencyHz == NULL) {
        return false;
    }
    if (AdcCapture_isBusy()) {
        AdcCapture_abort();
    }
    if (AdcCapture_isReady()) {
        AdcCapture_clear();
    }

    actualRateHz = AdcCapture_setSampleRate(sampleRateHz);
    if (!AdcCapture_start()) {
        return false;
    }
    startTick = Tick_now();
    while (!AdcCapture_isReady()) {
        if (Tick_elapsed(startTick) > F_FREQ_CAPTURE_TIMEOUT_MS) {
            AdcCapture_abort();
            return false;
        }
    }

    samples = AdcCapture_getADC1();
    for (index = 0U; index < ADC_CAPTURE_SIZE; index++) {
        uint16_t value = samples[index];
        if (value < minimum) minimum = value;
        if (value > maximum) maximum = value;
    }
    if ((uint16_t)(maximum - minimum) < F_FREQ_CAPTURE_MIN_SPAN) {
        AdcCapture_clear();
        return false;
    }

    midpoint = (uint16_t)(((uint32_t)minimum + maximum) / 2U);
    hysteresis = (uint16_t)((maximum - minimum) / 8U);
    if (hysteresis < 8U) hysteresis = 8U;
    lowThreshold = (uint16_t)(midpoint - hysteresis);
    highThreshold = (uint16_t)(midpoint + hysteresis);

    for (index = 0U; index < ADC_CAPTURE_SIZE; index++) {
        uint16_t value = samples[index];
        if (!armed) {
            if (value <= lowThreshold) armed = true;
        } else if (value >= highThreshold) {
            if (crossingCount == 0U) firstCrossing = index;
            lastCrossing = index;
            crossingCount++;
            armed = false;
        }
    }
    AdcCapture_clear();

    if (crossingCount < 3U) {
        return false;
    }
    crossingSpan = lastCrossing - firstCrossing;
    if (crossingSpan == 0U) {
        return false;
    }
    *frequencyHz =
        (actualRateHz * (crossingCount - 1U) + (crossingSpan / 2U)) /
        crossingSpan;
    return true;
}

static bool measureInputFrequency(uint32_t *frequencyHz)
{
    uint32_t firstFrequencyHz;
    uint32_t secondFrequencyHz;
    uint32_t finalRateHz = F_FREQ_CAPTURE_LOW_RATE_HZ;
    uint32_t measuredFrequencyHz;

    if (frequencyHz == NULL) {
        return false;
    }

    if (!captureInputFrequency(
            F_FREQ_CAPTURE_LOW_RATE_HZ, &firstFrequencyHz)) {
        finalRateHz = F_FREQ_CAPTURE_HIGH_RATE_HZ;
        if (!captureInputFrequency(finalRateHz, &firstFrequencyHz)) {
            return false;
        }
    } else if (firstFrequencyHz > F_FREQ_CAPTURE_HIGH_BAND_HZ) {
        finalRateHz = F_FREQ_CAPTURE_HIGH_RATE_HZ;
        if (!captureInputFrequency(finalRateHz, &firstFrequencyHz)) {
            return false;
        }
    }

    if (!captureInputFrequency(finalRateHz, &secondFrequencyHz)) {
        return false;
    }
    if ((firstFrequencyHz > secondFrequencyHz) &&
        ((firstFrequencyHz - secondFrequencyHz) > 300U)) {
        return false;
    }
    if ((secondFrequencyHz > firstFrequencyHz) &&
        ((secondFrequencyHz - firstFrequencyHz) > 300U)) {
        return false;
    }

    measuredFrequencyHz =
        (firstFrequencyHz + secondFrequencyHz + 1U) / 2U;
    measuredFrequencyHz =
        ((measuredFrequencyHz + 50U) / 100U) * 100U;
    if ((measuredFrequencyHz < F_FREQ_MIN_HZ) ||
        (measuredFrequencyHz > F_FREQ_MAX_HZ)) {
        return false;
    }
    *frequencyHz = measuredFrequencyHz;
    return true;
}

static void servicePhaseLock(void)
{
    if (!gPhaseLock.enabled || gPhaseLock.capturePending) {
        return;
    }

    if (gPhaseLock.scanState != PLOCK_SCAN_IDLE) {
        if (Tick_elapsed(gPhaseLock.scanSetTick) < PLOCK_SCAN_SETTLE_MS) {
            return;
        }
    } else {
        if (Tick_elapsed(gPhaseLock.lastCaptureTick) < PLOCK_PERIOD_MS) {
            return;
        }
    }

    if (AdcCapture_start()) {
        gPhaseLock.capturePending = true;
        gPhaseLock.lastCaptureTick = Tick_now();
    }
}

static void serviceADC(void)
{
    const uint16_t *adc0;
    const uint16_t *adc1;
    uint32_t sum0 = 0U;
    uint32_t sum1 = 0U;
    uint16_t min0 = 4095U;
    uint16_t min1 = 4095U;
    uint16_t max0 = 0U;
    uint16_t max1 = 0U;
    uint16_t mean0;
    uint16_t mean1;
    uint32_t index;

    if (!AdcCapture_isReady()) {
        return;
    }

    adc0 = AdcCapture_getADC0();
    adc1 = AdcCapture_getADC1();
    for (index = 0U; index < ADC_CAPTURE_SIZE; index++) {
        uint16_t value0 = adc0[index];
        uint16_t value1 = adc1[index];

        sum0 += value0;
        sum1 += value1;
        if (value0 < min0) min0 = value0;
        if (value0 > max0) max0 = value0;
        if (value1 < min1) min1 = value1;
        if (value1 > max1) max1 = value1;
    }

    mean0 = (uint16_t)(sum0 / ADC_CAPTURE_SIZE);
    mean1 = (uint16_t)(sum1 / ADC_CAPTURE_SIZE);
    AdcCapture_clear();

    if (gPhaseLock.enabled && gPhaseLock.capturePending) {
        gPhaseLock.capturePending = false;
        if (gPhaseLock.scanState != PLOCK_SCAN_IDLE) {
            phaseLockProcessCalibrationADC(mean0, min0, max0);
        } else {
            phaseLockProcessADC(mean0, min0, max0);
        }
        return;
    }

    UserUART_printf(
        "ADC_RESULT rate=%lu count=%u "
        "adc0_min=%u adc0_max=%u adc0_mean=%lu "
        "adc1_min=%u adc1_max=%u adc1_mean=%lu\r\n",
        (unsigned long)AdcCapture_getSampleRate(),
        (unsigned int)ADC_CAPTURE_SIZE,
        (unsigned int)min0, (unsigned int)max0,
        (unsigned long)mean0,
        (unsigned int)min1, (unsigned int)max1,
        (unsigned long)mean1);
}

static void phaseLockStart(uint32_t freqHz, uint16_t amp)
{
    phaseLockStop(true);
    if (AdcCapture_isBusy()) {
        AdcCapture_abort();
    }
    if (AdcCapture_isReady()) {
        AdcCapture_clear();
    }
    /*
     * applyPFDReference() performs any pending DAC recovery and initializes
     * the DDS on first use. Do not initialize here first, otherwise a DAC
     * transition would unnecessarily run the sequence twice.
     */
    memset(&gPhaseLock, 0, sizeof(gPhaseLock));

    gPhaseLock.frequencyHz = freqHz;
    gFControl.inputFreqHz = freqHz;
    gPhaseLock.nominalFTW =
        (uint32_t)((float)freqHz * AD9959_FTW_PER_HZ + 0.5F);
    gPhaseLock.referenceFTW = gPhaseLock.nominalFTW;
    gPhaseLock.amplitude = amp;
    gPhaseLock.phaseWord = 0U;
    gPhaseLock.adcTarget = PLOCK_ADC_TARGET;
    gPhaseLock.controlSign = 1;
    gPhaseLock.adcMean = PLOCK_ADC_TARGET;
    gPhaseLock.phaseCalibrated = true;
    gPhaseLock.calibrationPass = 2U;
    gPhaseLock.scanState = PLOCK_SCAN_IDLE;
    gPhaseLock.lastCaptureTick = Tick_now() - PLOCK_PERIOD_MS;

    applyPFDReference(freqHz, amp, 0U);
    (void)AdcCapture_setSampleRate(PLOCK_ADC_RATE_HZ);

    gFControl.mode = F_MODE_THRU;
    gFControl.autoTarget = F_MODE_OFF;
    setFState(F_STATE_READY);
    gPhaseLock.enabled = true;
    RGBLED_setColor(24U, 12U, 0U);
    UserUART_printf(
        "PLOCK_MODE manual target=%u phase_step=64\r\n",
        (unsigned int)gPhaseLock.adcTarget);
}

static void phaseLockStop(bool turnOffDDS)
{
    if (gPhaseLock.enabled || gPhaseLock.capturePending) {
        if (AdcCapture_isBusy()) {
            AdcCapture_abort();
        }
        if (AdcCapture_isReady()) {
            AdcCapture_clear();
        }
    }

    gPhaseLock.enabled = false;
    gPhaseLock.capturePending = false;
    gPhaseLock.locked = false;
    gPhaseLock.phaseCalibrated = false;
    gPhaseLock.scanState = PLOCK_SCAN_IDLE;
    gPhaseLock.stableSamples = 0U;
    gPhaseLock.outputArmed = false;
    gPhaseLock.outputConnected = false;
    gPhaseLock.outputMode = PLOCK_OUTPUT_OFF;

    if (turnOffDDS) {
        relaySelectDirect();
        RGBLED_setColor(0U, 24U, 0U);
    }
}

static uint16_t phaseLockWrapWord(int32_t phaseWord)
{
    while (phaseWord < 0) {
        phaseWord += 16384;
    }
    while (phaseWord > (int32_t)F_PHASE_MAX_WORD) {
        phaseWord -= 16384;
    }
    return (uint16_t)phaseWord;
}

static void phaseLockSetScanPhase(uint16_t phaseWord)
{
    gPhaseLock.phaseWord = phaseWord;
    AD9959_setPhase(AD9959_CH1, phaseWord);
    DDS_update();
    gFControl.outputPhase[1] = phaseWord;
    gPhaseLock.scanSetTick = Tick_now();
}

static void phaseLockBeginCalibration(void)
{
    if (AdcCapture_isBusy()) {
        AdcCapture_abort();
    }
    if (AdcCapture_isReady()) {
        AdcCapture_clear();
    }

    gPhaseLock.capturePending = false;
    gPhaseLock.locked = false;
    gPhaseLock.stableSamples = 0U;
    gPhaseLock.holdBadSamples = 0U;
    gPhaseLock.integratorQ8 = 0;
    gPhaseLock.lastStep = 0;
    gPhaseLock.trimStepSum = 0;
    gPhaseLock.trimSampleCount = 0U;
    gPhaseLock.calibrationHoldSamples = 0U;
    gPhaseLock.scanState = PLOCK_SCAN_COARSE;
    gPhaseLock.scanIndex = 0U;
    gPhaseLock.scanStartWord = 0;
    gPhaseLock.phaseCalibrated = false;
    RGBLED_setColor(24U, 12U, 0U);
    phaseLockSetScanPhase(0U);

    UserUART_printf(
        "PLOCK_CAL start pass=%u coarse_step=%u points=%u\r\n",
        (unsigned int)(gPhaseLock.calibrationPass + 1U),
        (unsigned int)PLOCK_SCAN_COARSE_STEP,
        (unsigned int)PLOCK_SCAN_COARSE_COUNT);
}

static bool phaseLockFindCrossing(const uint16_t *samples, uint8_t count,
    int32_t startWord, uint16_t stepWord, uint16_t target,
    int8_t preferredSlope, uint16_t maximumDelta,
    uint16_t *phaseWord, int8_t *slopeSign)
{
    uint8_t index;
    uint16_t selectedDelta = 0U;
    int32_t selectedPhase = 0;
    int8_t selectedSlope = 0;
    bool found = false;

    if ((samples == NULL) || (phaseWord == NULL) ||
        (slopeSign == NULL) || (count < 2U)) {
        return false;
    }

    for (index = 0U; index < (uint8_t)(count - 1U); index++) {
        int32_t first = samples[index];
        int32_t second = samples[index + 1U];
        int32_t delta = second - first;
        uint16_t absoluteDelta;
        int8_t localSlope;
        int32_t interpolated;

        if (delta == 0) {
            continue;
        }
        if (!(((int32_t)target >= first && (int32_t)target <= second) ||
              ((int32_t)target <= first && (int32_t)target >= second))) {
            continue;
        }

        absoluteDelta =
            (uint16_t)((delta < 0) ? -delta : delta);
        if ((maximumDelta != 0U) && (absoluteDelta > maximumDelta)) {
            continue;
        }
        localSlope = (delta > 0) ? 1 : -1;
        if ((preferredSlope != 0) && (localSlope != preferredSlope)) {
            continue;
        }

        interpolated = startWord +
            (int32_t)index * (int32_t)stepWord +
            ((int32_t)stepWord *
             ((int32_t)target - first)) / delta;
        if (!found || (absoluteDelta > selectedDelta)) {
            found = true;
            selectedDelta = absoluteDelta;
            selectedPhase = interpolated;
            selectedSlope = localSlope;
        }
    }

    if (!found) {
        return false;
    }
    *phaseWord = phaseLockWrapWord(selectedPhase);
    *slopeSign = selectedSlope;
    return true;
}

static bool phaseLockAnalyzeCoarse(void)
{
    uint8_t index;
    uint8_t minimumIndex = 0U;
    uint8_t quarterIndex;
    const uint8_t cyclePointCount =
        (uint8_t)(PLOCK_SCAN_COARSE_COUNT - 1U);
    uint16_t minimum = gPhaseScanCoarse[0];
    uint16_t maximum = gPhaseScanCoarse[0];
    uint16_t span;
    uint16_t anchorWord;
    int32_t slopeDelta;
    int8_t slopeSign;

    /*
     * The last coarse point is the duplicated 360-degree sample. Exclude it
     * when locating the zero so that adding a quarter cycle can wrap cleanly.
     */
    for (index = 1U; index < cyclePointCount; index++) {
        if (gPhaseScanCoarse[index] < minimum) {
            minimum = gPhaseScanCoarse[index];
            minimumIndex = index;
        }
        if (gPhaseScanCoarse[index] > maximum) {
            maximum = gPhaseScanCoarse[index];
        }
    }

    span = (uint16_t)(maximum - minimum);
    if (span < PLOCK_SCAN_MIN_SPAN) {
        UserUART_printf(
            "PLOCK_CAL failed reason=small_span min=%u max=%u span=%u\r\n",
            (unsigned int)minimum, (unsigned int)maximum,
            (unsigned int)span);
        gPhaseLock.scanState = PLOCK_SCAN_IDLE;
        gPhaseLock.phaseCalibrated = false;
        RGBLED_setColor(96U, 0U, 0U);
        return false;
    }

    /*
     * Derive the lock point from this boot's measured transfer curve. The
     * quarter-cycle point is mathematical (90 degrees), not a stored bench
     * phase. A fine scan later measures its exact ADC target and slope.
     */
    quarterIndex = (uint8_t)(
        (minimumIndex +
         (uint8_t)(F_PHASE_90_WORD / PLOCK_SCAN_COARSE_STEP)) %
        cyclePointCount);
    anchorWord = (uint16_t)(
        (uint16_t)quarterIndex * PLOCK_SCAN_COARSE_STEP);
    gPhaseLock.adcTarget = gPhaseScanCoarse[quarterIndex];
    slopeDelta =
        (int32_t)gPhaseScanCoarse[
            (uint8_t)((quarterIndex + 1U) % cyclePointCount)] -
        (int32_t)gPhaseScanCoarse[
            (uint8_t)((quarterIndex + cyclePointCount - 1U) %
                      cyclePointCount)];
    slopeSign = (slopeDelta >= 0) ? 1 : -1;
    gPhaseLock.controlSign = (slopeSign > 0) ? -1 : 1;
    gPhaseLock.coarseZeroWord = phaseLockWrapWord(
        (int32_t)minimumIndex * (int32_t)PLOCK_SCAN_COARSE_STEP);
    gPhaseLock.coarseAnchorWord = anchorWord;
    gPhaseLock.coarseFixedTargetWord = anchorWord;
    gPhaseLock.fixedTargetValid = false;
    gPhaseLock.scanState = PLOCK_SCAN_FINE_ZERO;
    gPhaseLock.scanIndex = 0U;
    gPhaseLock.scanStartWord =
        (int32_t)gPhaseLock.coarseZeroWord -
        (int32_t)PLOCK_SCAN_FINE_RADIUS;
    phaseLockSetScanPhase(
        phaseLockWrapWord(gPhaseLock.scanStartWord));

    UserUART_printf(
        "PLOCK_CAL coarse mode=dynamic90 min=%u max=%u target=%u "
        "zero=%u anchor90=%u sign=%d\r\n",
        (unsigned int)minimum, (unsigned int)maximum,
        (unsigned int)gPhaseLock.adcTarget,
        (unsigned int)gPhaseLock.coarseZeroWord,
        (unsigned int)gPhaseLock.coarseAnchorWord,
        (int)gPhaseLock.controlSign);
    return true;
}

static bool phaseLockFinishFineScan(void)
{
    const uint8_t centerIndex =
        (uint8_t)(PLOCK_SCAN_FINE_RADIUS / PLOCK_SCAN_FINE_STEP);
    uint16_t refinedAnchor = phaseLockWrapWord(
        (int32_t)gPhaseLock.phaseZeroWord +
        (int32_t)F_PHASE_90_WORD);
    int8_t slopeSign;
    int8_t preferredSlope;
    int32_t slopeDelta;

    /*
     * Measure the target at the centre of the fine scan. Because that centre
     * is rebuilt from the newly measured zero plus exactly 90 degrees, no
     * historical phase table is involved.
     */
    gPhaseLock.adcTarget = gPhaseScanFine[centerIndex];
    slopeDelta =
        (int32_t)gPhaseScanFine[centerIndex + 1U] -
        (int32_t)gPhaseScanFine[centerIndex - 1U];
    preferredSlope = (slopeDelta >= 0) ? 1 : -1;
    if (!phaseLockFindCrossing(gPhaseScanFine,
            PLOCK_SCAN_FINE_COUNT, gPhaseLock.scanStartWord,
            PLOCK_SCAN_FINE_STEP, gPhaseLock.adcTarget,
            preferredSlope, 0U, &refinedAnchor, &slopeSign)) {
        UserUART_printf(
            "PLOCK_CAL fine90 fallback=%u reason=no_crossing\r\n",
            (unsigned int)refinedAnchor);
        slopeSign = preferredSlope;
    }

    gPhaseLock.controlSign = (slopeSign > 0) ? -1 : 1;
    gPhaseLock.phaseAnchorWord = refinedAnchor;
    gPhaseLock.fixedTargetPhaseWord = refinedAnchor;
    gPhaseLock.fixedTargetValid = false;
    gPhaseLock.zeroCorrection = phaseLockWrapWord(
        (int32_t)gPhaseLock.phaseZeroWord -
        (int32_t)gPhaseLock.phaseAnchorWord);
    gPhaseLock.scanState = PLOCK_SCAN_IDLE;
    gPhaseLock.phaseWord = refinedAnchor;
    gPhaseLock.integratorQ8 = 0;
    gPhaseLock.lastStep = 0;
    gPhaseLock.stableSamples = 0U;
    gPhaseLock.holdBadSamples = 0U;
    gPhaseLock.locked = false;
    gPhaseLock.lastCaptureTick = Tick_now() - PLOCK_PERIOD_MS;
    AD9959_setPhase(AD9959_CH1, refinedAnchor);
    DDS_update();
    gFControl.outputPhase[1] = refinedAnchor;

    if (gPhaseLock.calibrationPass == 0U) {
        gPhaseLock.calibrationPass = 1U;
        gPhaseLock.phaseCalibrated = false;
    } else {
        gPhaseLock.calibrationPass = 2U;
        gPhaseLock.phaseCalibrated = true;
    }

    UserUART_printf(
        "PLOCK_CAL complete pass=%u target=%u sign=%d zero=%u "
        "anchor=%u fixed_anchor=%u fixed_valid=%u "
        "zero_correction=%u final=%u\r\n",
        (unsigned int)gPhaseLock.calibrationPass,
        (unsigned int)gPhaseLock.adcTarget,
        (int)gPhaseLock.controlSign,
        (unsigned int)gPhaseLock.phaseZeroWord,
        (unsigned int)gPhaseLock.phaseAnchorWord,
        (unsigned int)gPhaseLock.fixedTargetPhaseWord,
        gPhaseLock.fixedTargetValid ? 1U : 0U,
        (unsigned int)gPhaseLock.zeroCorrection,
        gPhaseLock.phaseCalibrated ? 1U : 0U);
    return true;
}

static void phaseLockProcessCalibrationADC(
    uint16_t mean, uint16_t minimum, uint16_t maximum)
{
    uint8_t index;
    uint8_t minimumIndex;

    gPhaseLock.adcMean = mean;
    gPhaseLock.adcMin = minimum;
    gPhaseLock.adcMax = maximum;

    switch (gPhaseLock.scanState) {
        case PLOCK_SCAN_COARSE:
            gPhaseScanCoarse[gPhaseLock.scanIndex] = mean;
            gPhaseLock.scanIndex++;
            if (gPhaseLock.scanIndex >= PLOCK_SCAN_COARSE_COUNT) {
                (void)phaseLockAnalyzeCoarse();
            } else {
                phaseLockSetScanPhase(phaseLockWrapWord(
                    (int32_t)gPhaseLock.scanIndex *
                    (int32_t)PLOCK_SCAN_COARSE_STEP));
            }
            break;

        case PLOCK_SCAN_FINE_ZERO:
            gPhaseScanFine[gPhaseLock.scanIndex] = mean;
            gPhaseLock.scanIndex++;
            if (gPhaseLock.scanIndex >= PLOCK_SCAN_FINE_COUNT) {
                minimumIndex = 0U;
                for (index = 1U;
                     index < PLOCK_SCAN_FINE_COUNT; index++) {
                    if (gPhaseScanFine[index] <
                        gPhaseScanFine[minimumIndex]) {
                        minimumIndex = index;
                    }
                }
                gPhaseLock.phaseZeroWord = phaseLockWrapWord(
                    gPhaseLock.scanStartWord +
                    (int32_t)minimumIndex *
                    (int32_t)PLOCK_SCAN_FINE_STEP);
                gPhaseLock.scanState = PLOCK_SCAN_FINE_LOCK;
                gPhaseLock.scanIndex = 0U;
                gPhaseLock.scanStartWord =
                    (int32_t)gPhaseLock.phaseZeroWord +
                    (int32_t)F_PHASE_90_WORD -
                    (int32_t)PLOCK_SCAN_FINE_RADIUS;
                phaseLockSetScanPhase(
                    phaseLockWrapWord(gPhaseLock.scanStartWord));
            } else {
                phaseLockSetScanPhase(phaseLockWrapWord(
                    gPhaseLock.scanStartWord +
                    (int32_t)gPhaseLock.scanIndex *
                    (int32_t)PLOCK_SCAN_FINE_STEP));
            }
            break;

        case PLOCK_SCAN_FINE_LOCK:
            gPhaseScanFine[gPhaseLock.scanIndex] = mean;
            gPhaseLock.scanIndex++;
            if (gPhaseLock.scanIndex >= PLOCK_SCAN_FINE_COUNT) {
                (void)phaseLockFinishFineScan();
            } else {
                phaseLockSetScanPhase(phaseLockWrapWord(
                    gPhaseLock.scanStartWord +
                    (int32_t)gPhaseLock.scanIndex *
                    (int32_t)PLOCK_SCAN_FINE_STEP));
            }
            break;

        case PLOCK_SCAN_IDLE:
        default:
            break;
    }
}

static void phaseLockProcessADC(
    uint16_t mean, uint16_t minimum, uint16_t maximum)
{
    int32_t error;
    int32_t controlError;
    int32_t step;
    int32_t nextPhase;
    int32_t integralLimitQ8 = PLOCK_I_LIMIT * 256;
    uint32_t absoluteError;
    uint16_t ripple;
    uint16_t lockRippleLimit;
    uint16_t holdRippleLimit;
    int32_t stepLimit;

    gPhaseLock.adcMean = mean;
    gPhaseLock.adcMin = minimum;
    gPhaseLock.adcMax = maximum;
    ripple = (uint16_t)(maximum - minimum);
    if (gPhaseLock.frequencyHz <= PLOCK_VERY_LOW_FREQ_HZ) {
        lockRippleLimit = PLOCK_VERY_LOW_LOCK_RIPPLE;
        holdRippleLimit = PLOCK_VERY_LOW_HOLD_RIPPLE;
    } else if (gPhaseLock.frequencyHz <= PLOCK_LOW_FREQ_HZ) {
        lockRippleLimit = PLOCK_LOW_LOCK_RIPPLE;
        holdRippleLimit = PLOCK_LOW_HOLD_RIPPLE;
    } else {
        lockRippleLimit = PLOCK_LOCK_RIPPLE;
        holdRippleLimit = PLOCK_HOLD_RIPPLE;
    }
    error = (int32_t)mean - (int32_t)gPhaseLock.adcTarget;
    controlError = error * (int32_t)gPhaseLock.controlSign;
    gPhaseLock.error = (int16_t)error;

    if (gPhaseLock.locked) {
        /*
         * A small frequency difference between the external source and the
         * DDS requires a constant phase ramp.  Preserve that ramp estimate,
         * but leak and tightly limit it so a relay transient cannot wind the
         * loop up indefinitely.
         */
        gPhaseLock.integratorQ8 -= gPhaseLock.integratorQ8 / 256;
        if ((controlError < -PLOCK_HOLD_DEADBAND) ||
            (controlError > PLOCK_HOLD_DEADBAND)) {
            gPhaseLock.integratorQ8 +=
                controlError * PLOCK_HOLD_KI_Q8;
        }
        integralLimitQ8 = PLOCK_HOLD_I_LIMIT * 256;
        if (gPhaseLock.integratorQ8 > integralLimitQ8) {
            gPhaseLock.integratorQ8 = integralLimitQ8;
        } else if (gPhaseLock.integratorQ8 < -integralLimitQ8) {
            gPhaseLock.integratorQ8 = -integralLimitQ8;
        }
        stepLimit = PLOCK_HOLD_STEP_LIMIT;
        if ((controlError >= -PLOCK_HOLD_DEADBAND) &&
            (controlError <= PLOCK_HOLD_DEADBAND)) {
            step = gPhaseLock.integratorQ8 / 256;
        } else {
            step = (PLOCK_HOLD_KP * controlError) +
                (gPhaseLock.integratorQ8 / 256);
        }
    } else {
        gPhaseLock.integratorQ8 += controlError * PLOCK_KI_Q8;
        if (gPhaseLock.integratorQ8 > integralLimitQ8) {
            gPhaseLock.integratorQ8 = integralLimitQ8;
        } else if (gPhaseLock.integratorQ8 < -integralLimitQ8) {
            gPhaseLock.integratorQ8 = -integralLimitQ8;
        }

        /*
         * The initial search may use large phase steps, but after CH0/CH1 are
         * synchronised for a display mode the PFD lock window is narrow.  A
         * 45-degree step can repeatedly jump across that window, so use a
         * finer acquisition limit while an output mode is armed.
         */
        stepLimit = gPhaseLock.outputArmed ?
            PLOCK_REACQUIRE_STEP_LIMIT : PLOCK_STEP_LIMIT;
        step = (PLOCK_KP * controlError) +
            (gPhaseLock.integratorQ8 / 256);
    }
    if (step > stepLimit) {
        step = stepLimit;
    } else if (step < -stepLimit) {
        step = -stepLimit;
    }
    gPhaseLock.lastStep = (int16_t)step;

    nextPhase = (int32_t)gPhaseLock.phaseWord + step;
    while (nextPhase < 0) nextPhase += 16384;
    while (nextPhase > (int32_t)F_PHASE_MAX_WORD) nextPhase -= 16384;
    gPhaseLock.phaseWord = (uint16_t)nextPhase;

    AD9959_setPhase(AD9959_CH1, gPhaseLock.phaseWord);
    if (gPhaseLock.outputArmed) {
        phaseLockUpdateOutputPhase();
    }
    DDS_update();
    gFControl.outputPhase[1] = gPhaseLock.phaseWord;
    gPhaseLock.updateCount++;

    absoluteError = (error < 0) ? (uint32_t)(-error) : (uint32_t)error;
    if (!gPhaseLock.locked) {
        if ((absoluteError <= PLOCK_LOCK_ERROR) &&
            (ripple <= lockRippleLimit)) {
            if (gPhaseLock.stableSamples < PLOCK_LOCK_SAMPLES) {
                gPhaseLock.stableSamples++;
            }
        } else {
            gPhaseLock.stableSamples = 0U;
        }

        if (gPhaseLock.stableSamples >= PLOCK_LOCK_SAMPLES) {
            gPhaseLock.locked = true;
            gPhaseLock.holdBadSamples = 0U;
            if (gPhaseLock.phaseCalibrated) {
                RGBLED_setColor(0U, 24U, 0U);
                UserUART_printf(
                    "PLOCK_EVENT locked updates=%lu mean=%u "
                    "phase=%u calibrated=1\r\n",
                    (unsigned long)gPhaseLock.updateCount,
                    (unsigned int)gPhaseLock.adcMean,
                    (unsigned int)gPhaseLock.phaseWord);
                phaseLockConnectOutput();
            } else {
                RGBLED_setColor(24U, 12U, 0U);
                UserUART_printf(
                    "PLOCK_EVENT provisional_locked updates=%lu "
                    "mean=%u phase=%u action=ftw_settle\r\n",
                    (unsigned long)gPhaseLock.updateCount,
                    (unsigned int)gPhaseLock.adcMean,
                    (unsigned int)gPhaseLock.phaseWord);
            }
        }
    } else {
        if ((absoluteError > PLOCK_HOLD_ERROR) ||
            (ripple > holdRippleLimit)) {
            if (gPhaseLock.holdBadSamples < PLOCK_HOLD_SAMPLES) {
                gPhaseLock.holdBadSamples++;
            }
        } else {
            gPhaseLock.holdBadSamples = 0U;
        }

        if (gPhaseLock.holdBadSamples >= PLOCK_HOLD_SAMPLES) {
            gPhaseLock.locked = false;
            gPhaseLock.stableSamples = 0U;
            gPhaseLock.holdBadSamples = 0U;
            if (!gPhaseLock.phaseCalibrated) {
                gPhaseLock.calibrationHoldSamples = 0U;
            }
            gPhaseLock.trimStepSum = 0;
            gPhaseLock.trimSampleCount = 0U;
            phaseLockDisconnectOutput();
            RGBLED_setColor(24U, 12U, 0U);
            UserUART_printf(
                "PLOCK_EVENT tracking error=%ld mean=%u ripple=%u\r\n",
                (long)error, (unsigned int)gPhaseLock.adcMean,
                (unsigned int)ripple);
        }
    }

    phaseLockServiceFTWTrim(
        absoluteError, ripple, holdRippleLimit);

    if (gPhaseLock.locked && !gPhaseLock.phaseCalibrated &&
        (gPhaseLock.calibrationPass == 1U)) {
        if (gPhaseLock.calibrationHoldSamples <
            PLOCK_RECALIBRATE_SAMPLES) {
            gPhaseLock.calibrationHoldSamples++;
        }
        if (gPhaseLock.calibrationHoldSamples >=
            PLOCK_RECALIBRATE_SAMPLES) {
            UserUART_printf(
                "PLOCK_CAL rescan ftw=%lu trim=%ld\r\n",
                (unsigned long)gPhaseLock.referenceFTW,
                (long)gPhaseLock.frequencyTrimFTW);
            phaseLockBeginCalibration();
        }
    }
}

static void phaseLockServiceFTWTrim(
    uint32_t absoluteError, uint16_t ripple, uint16_t holdRippleLimit)
{
    int32_t averageStep;
    int32_t numerator;
    int32_t deltaFTW;
    int32_t nextTotalTrim;
    int32_t equivalentStep;
    int32_t integralLimitQ8 = PLOCK_HOLD_I_LIMIT * 256;

    if (!gPhaseLock.locked ||
        (absoluteError > PLOCK_FTW_TRIM_MAX_ERROR) ||
        (ripple > holdRippleLimit)) {
        gPhaseLock.trimStepSum = 0;
        gPhaseLock.trimSampleCount = 0U;
        return;
    }

    gPhaseLock.trimStepSum += gPhaseLock.lastStep;
    gPhaseLock.trimSampleCount++;
    if (gPhaseLock.trimSampleCount < PLOCK_FTW_TRIM_SAMPLES) {
        return;
    }

    averageStep =
        gPhaseLock.trimStepSum / (int32_t)gPhaseLock.trimSampleCount;
    gPhaseLock.trimStepSum = 0;
    gPhaseLock.trimSampleCount = 0U;

    numerator = averageStep * PLOCK_FTW_TRIM_NUMERATOR;
    if (numerator >= 0) {
        deltaFTW = (numerator + (PLOCK_FTW_TRIM_DENOMINATOR / 2)) /
            PLOCK_FTW_TRIM_DENOMINATOR;
    } else {
        deltaFTW = (numerator - (PLOCK_FTW_TRIM_DENOMINATOR / 2)) /
            PLOCK_FTW_TRIM_DENOMINATOR;
    }
    if (deltaFTW == 0) {
        return;
    }

    nextTotalTrim = gPhaseLock.frequencyTrimFTW + deltaFTW;
    if (nextTotalTrim > PLOCK_FTW_TRIM_TOTAL_LIMIT) {
        nextTotalTrim = PLOCK_FTW_TRIM_TOTAL_LIMIT;
    } else if (nextTotalTrim < -PLOCK_FTW_TRIM_TOTAL_LIMIT) {
        nextTotalTrim = -PLOCK_FTW_TRIM_TOTAL_LIMIT;
    }
    deltaFTW = nextTotalTrim - gPhaseLock.frequencyTrimFTW;
    if (deltaFTW == 0) {
        return;
    }

    gPhaseLock.frequencyTrimFTW = nextTotalTrim;
    gPhaseLock.referenceFTW =
        (uint32_t)((int32_t)gPhaseLock.nominalFTW + nextTotalTrim);

    numerator = deltaFTW * PLOCK_FTW_TRIM_DENOMINATOR;
    if (numerator >= 0) {
        equivalentStep =
            (numerator + (PLOCK_FTW_TRIM_NUMERATOR / 2)) /
            PLOCK_FTW_TRIM_NUMERATOR;
    } else {
        equivalentStep =
            (numerator - (PLOCK_FTW_TRIM_NUMERATOR / 2)) /
            PLOCK_FTW_TRIM_NUMERATOR;
    }
    gPhaseLock.integratorQ8 -= equivalentStep * 256;
    if (gPhaseLock.integratorQ8 > integralLimitQ8) {
        gPhaseLock.integratorQ8 = integralLimitQ8;
    } else if (gPhaseLock.integratorQ8 < -integralLimitQ8) {
        gPhaseLock.integratorQ8 = -integralLimitQ8;
    }

    phaseLockApplyReferenceFTW();
    UserUART_printf(
        "PLOCK_EVENT ftw_trim delta=%ld total=%ld ftw=%lu "
        "avg_step=%ld residual_i=%ld\r\n",
        (long)deltaFTW, (long)gPhaseLock.frequencyTrimFTW,
        (unsigned long)gPhaseLock.referenceFTW,
        (long)averageStep,
        (long)(gPhaseLock.integratorQ8 / 256));
}

static void phaseLockApplyReferenceFTW(void)
{
    uint32_t outputFTW;

    AD9959_setFTW(AD9959_CH1, gPhaseLock.referenceFTW);
    if (gPhaseLock.outputArmed) {
        outputFTW = gPhaseLock.referenceFTW;
        if (gPhaseLock.outputMode == PLOCK_OUTPUT_DOUBLE) {
            outputFTW *= 2U;
        }
        AD9959_setFTW(AD9959_CH0, outputFTW);
    }
    DDS_update();
}

static void phaseLockArmOutput(
    PhaseLockOutputMode mode, uint16_t amp, uint16_t offset)
{
    uint32_t outputFreqHz;
    uint32_t outputFTW;

    phaseLockDisconnectOutput();
    if (AdcCapture_isBusy()) {
        AdcCapture_abort();
    }
    if (AdcCapture_isReady()) {
        AdcCapture_clear();
    }
    gPhaseLock.trimStepSum = 0;
    gPhaseLock.trimSampleCount = 0U;
    gPhaseLock.outputMode = mode;
    gPhaseLock.outputArmed = true;
    gPhaseLock.outputAmplitude = amp;
    gPhaseLock.outputOffset = offset;

    outputFreqHz = gPhaseLock.frequencyHz;
    outputFTW = gPhaseLock.referenceFTW;
    if (mode == PLOCK_OUTPUT_DOUBLE) {
        outputFreqHz *= 2U;
        outputFTW *= 2U;
    }

    AD9959_setAmp(AD9959_CH0, 0U);
    AD9959_setFTW(AD9959_CH0, outputFTW);
    phaseLockUpdateOutputPhase();
    DDS_update();

    gFControl.outputFreqHz[0] = outputFreqHz;
    gFControl.outputAmp[0] = 0U;

    /*
     * Changing CH0 between f and 2f while its accumulator keeps running makes
     * the 2f orientation depend on the exact switching instant. Reset every
     * DDS accumulator here so CH0 and CH1 regain a common time origin. CH0
     * stays muted and the relay remains on the direct path while CH1
     * automatically reacquires the fixed PA27 target.
     */
    gPhaseLock.capturePending = false;
    gPhaseLock.locked = false;
    gPhaseLock.stableSamples = 0U;
    gPhaseLock.holdBadSamples = 0U;
    gPhaseLock.integratorQ8 = 0;
    gPhaseLock.lastStep = 0;
    gPhaseLock.error = 0;
    gPhaseLock.adcMean = gPhaseLock.adcTarget;
    gPhaseLock.adcMin = gPhaseLock.adcTarget;
    gPhaseLock.adcMax = gPhaseLock.adcTarget;
    AD9959_syncPhaseAccumulators();
    gPhaseLock.lastCaptureTick = Tick_now();
    RGBLED_setColor(24U, 12U, 0U);
    UserUART_printf(
        "PLOCK_EVENT phase_origin_reset mode=%s offset=%u "
        "action=reacquire\r\n",
        (mode == PLOCK_OUTPUT_DOUBLE) ? "double" : "same",
        (unsigned int)offset);
}

static void phaseLockSetOutputAmplitude(uint16_t amp)
{
    gPhaseLock.outputAmplitude = amp;
    if (gPhaseLock.outputConnected) {
        AD9959_setAmp(AD9959_CH0, amp);
        DDS_update();
        gFControl.outputAmp[0] = amp;
    }
}

static void phaseLockSetOutputOffset(uint16_t offset)
{
    gPhaseLock.outputOffset = offset;
    phaseLockUpdateOutputPhase();
    DDS_update();
}

static void phaseLockConnectOutput(void)
{
    uint32_t outputFreqHz;
    uint32_t outputFTW;

    if (!gPhaseLock.enabled || !gPhaseLock.locked ||
        !gPhaseLock.phaseCalibrated ||
        !gPhaseLock.outputArmed || gPhaseLock.outputConnected) {
        return;
    }

    outputFreqHz = gPhaseLock.frequencyHz;
    outputFTW = gPhaseLock.referenceFTW;
    if (gPhaseLock.outputMode == PLOCK_OUTPUT_DOUBLE) {
        outputFreqHz *= 2U;
        outputFTW *= 2U;
    }

    AD9959_setAmp(AD9959_CH0, 0U);
    AD9959_setFTW(AD9959_CH0, outputFTW);
    phaseLockUpdateOutputPhase();
    DDS_update();

    DL_GPIO_setPins(GPIO_RELAY_PORT, GPIO_RELAY_CTRL_PIN);
    gRelayDDSSelected = true;
    Tick_delay(F_RELAY_SETTLE_MS);
    AD9959_setAmp(AD9959_CH0, gPhaseLock.outputAmplitude);
    DDS_update();

    gPhaseLock.outputConnected = true;
    gFControl.mode = (gPhaseLock.outputMode == PLOCK_OUTPUT_DOUBLE) ?
        F_MODE_DOUBLE : F_MODE_SAME;
    gFControl.outputFreqHz[0] = outputFreqHz;
    gFControl.outputAmp[0] = gPhaseLock.outputAmplitude;
    UserUART_printf(
        "PLOCK_EVENT output_connected mode=%s freq=%lu amp=%u phase=%u\r\n",
        (gPhaseLock.outputMode == PLOCK_OUTPUT_DOUBLE) ? "double" : "same",
        (unsigned long)outputFreqHz,
        (unsigned int)gPhaseLock.outputAmplitude,
        (unsigned int)gPhaseLock.outputPhase);
}

static void phaseLockDisconnectOutput(void)
{
    bool wasConnected = gPhaseLock.outputConnected || gRelayDDSSelected;

    if (gDDSInitialized) {
        AD9959_setAmp(AD9959_CH0, 0U);
        DDS_update();
    }
    DL_GPIO_clearPins(GPIO_RELAY_PORT, GPIO_RELAY_CTRL_PIN);
    gRelayDDSSelected = false;
    gPhaseLock.outputConnected = false;
    gFControl.outputAmp[0] = 0U;
    if (gPhaseLock.enabled) {
        gFControl.mode = F_MODE_THRU;
    }
    if (wasConnected) {
        Tick_delay(F_RELAY_SETTLE_MS);
    }
}

static void phaseLockSelectDirect(void)
{
    if (gPhaseLock.enabled) {
        gPhaseLock.outputArmed = false;
        gPhaseLock.outputMode = PLOCK_OUTPUT_OFF;
        phaseLockDisconnectOutput();
    } else {
        relaySelectDirect();
    }
    gFControl.mode = F_MODE_THRU;
    gFControl.autoTarget = F_MODE_OFF;
    setFState(F_STATE_READY);
}

static void phaseLockUpdateOutputPhase(void)
{
    uint32_t phase = gPhaseLock.phaseWord;

    if (gPhaseLock.outputMode == PLOCK_OUTPUT_DOUBLE) {
        phase *= 2U;
    }
    phase += gPhaseLock.outputOffset;
    phase &= F_PHASE_MAX_WORD;
    gPhaseLock.outputPhase = (uint16_t)phase;
    AD9959_setPhase(AD9959_CH0, gPhaseLock.outputPhase);
    gFControl.outputPhase[0] = gPhaseLock.outputPhase;
}

static uint16_t phaseLockCircleOutputOffset(void)
{
    return circlePhaseOffset(gPhaseLock.frequencyHz);
}

static uint16_t phaseLockInfinityOutputOffset(void)
{
    uint32_t frequencyOffset;
    uint32_t frequencySpan;
    uint32_t phaseSpan;

    /*
     * The 2:1 phase relationship is maintained dynamically by
     * phaseLockUpdateOutputPhase(). At 1 kHz and 2 kHz, subtract the measured
     * 1088-word correction to move the crossing upward by one oscilloscope
     * division. Restore the nominal word at the next 100 Hz test point so
     * higher-frequency calibration remains unchanged.
     */
    if (gPhaseLock.frequencyHz <= F_INFINITY_PHASE_LOW_MAX_HZ) {
        return F_INFINITY_PHASE_LOW_WORD;
    }
    if (gPhaseLock.frequencyHz >= F_INFINITY_PHASE_NORMAL_HZ) {
        return F_INFINITY_PHASE_WORD;
    }

    frequencyOffset =
        gPhaseLock.frequencyHz - F_INFINITY_PHASE_LOW_MAX_HZ;
    frequencySpan =
        F_INFINITY_PHASE_NORMAL_HZ - F_INFINITY_PHASE_LOW_MAX_HZ;
    phaseSpan = F_INFINITY_PHASE_WORD - F_INFINITY_PHASE_LOW_WORD;
    return (uint16_t)(F_INFINITY_PHASE_LOW_WORD +
        (phaseSpan * frequencyOffset + (frequencySpan / 2U)) /
            frequencySpan);
}

static void showPhaseLockStatus(void)
{
    UserUART_printf(
        "PLOCK_STATUS enabled=%u state=%s freq=%lu ftw=%lu amp=%u target=%u "
        "mean=%u min=%u max=%u ripple=%u error=%d phase=%u step=%d i_q8=%ld "
        "stable=%u updates=%lu\r\n",
        gPhaseLock.enabled ? 1U : 0U,
        gPhaseLock.locked ? "locked" :
            (gPhaseLock.enabled ? "acquiring" : "off"),
        (unsigned long)gPhaseLock.frequencyHz,
        (unsigned long)gPhaseLock.referenceFTW,
        (unsigned int)gPhaseLock.amplitude,
        (unsigned int)gPhaseLock.adcTarget,
        (unsigned int)gPhaseLock.adcMean,
        (unsigned int)gPhaseLock.adcMin,
        (unsigned int)gPhaseLock.adcMax,
        (unsigned int)(gPhaseLock.adcMax - gPhaseLock.adcMin),
        (int)gPhaseLock.error,
        (unsigned int)gPhaseLock.phaseWord,
        (int)gPhaseLock.lastStep,
        (long)gPhaseLock.integratorQ8,
        (unsigned int)gPhaseLock.stableSamples,
        (unsigned long)gPhaseLock.updateCount);
    UserUART_printf(
        "PLOCK_CAL state=%u pass=%u calibrated=%u sign=%d "
        "zero=%u anchor=%u fixed_anchor=%u fixed_valid=%u "
        "correction=%u hold=%u\r\n",
        (unsigned int)gPhaseLock.scanState,
        (unsigned int)gPhaseLock.calibrationPass,
        gPhaseLock.phaseCalibrated ? 1U : 0U,
        (int)gPhaseLock.controlSign,
        (unsigned int)gPhaseLock.phaseZeroWord,
        (unsigned int)gPhaseLock.phaseAnchorWord,
        (unsigned int)gPhaseLock.fixedTargetPhaseWord,
        gPhaseLock.fixedTargetValid ? 1U : 0U,
        (unsigned int)gPhaseLock.zeroCorrection,
        (unsigned int)gPhaseLock.calibrationHoldSamples);
    UserUART_printf(
        "PLOCK_FTW nominal=%lu current=%lu trim=%ld "
        "trim_samples=%u trim_step_sum=%ld\r\n",
        (unsigned long)gPhaseLock.nominalFTW,
        (unsigned long)gPhaseLock.referenceFTW,
        (long)gPhaseLock.frequencyTrimFTW,
        (unsigned int)gPhaseLock.trimSampleCount,
        (long)gPhaseLock.trimStepSum);
    UserUART_printf(
        "PLOCK_OUTPUT mode=%s armed=%u connected=%u amp=%u "
        "offset=%u phase=%u relay=%s\r\n",
        (gPhaseLock.outputMode == PLOCK_OUTPUT_DOUBLE) ? "double" :
            ((gPhaseLock.outputMode == PLOCK_OUTPUT_SAME) ? "same" : "off"),
        gPhaseLock.outputArmed ? 1U : 0U,
        gPhaseLock.outputConnected ? 1U : 0U,
        (unsigned int)gPhaseLock.outputAmplitude,
        (unsigned int)gPhaseLock.outputOffset,
        (unsigned int)gPhaseLock.outputPhase,
        gRelayDDSSelected ? "dds" : "direct");
}

static void serviceFWatchdog(void)
{
    if ((gFControl.state != F_STATE_WAIT_DK) &&
        (gFControl.state != F_STATE_SEARCHING) &&
        (gFControl.state != F_STATE_LOCKING)) {
        return;
    }

    if (Tick_elapsed(gFControl.lastDKTick) > F_DK_TIMEOUT_MS) {
        relaySelectDirect();
        setFState(F_STATE_ERROR);
        UserUART_printf(
            "F_EVENT error=dk_timeout request=%lu timeout_ms=%u\r\n",
            (unsigned long)gFControl.requestId,
            (unsigned int)F_DK_TIMEOUT_MS);
    }
}

static void ensureDDS(void)
{
    if (!gDDSInitialized) {
        DDS_init();
        gDDSInitialized = true;
    }
}

static void forceDDSReinitialize(void)
{
    /*
     * The AD9959 has no usable connection-status flag in the present
     * one-wire write-only interface. The MCU-side boolean therefore cannot
     * detect that the DDS board was power-cycled independently. Repeating
     * the hardware reset and reference-clock setup is the recovery path.
    */
    gDDSInitialized = false;
    DDS_init();
    gDDSInitialized = true;
    ddsAllOff();
    /*
     * A full reset/clock sequence also satisfies any recovery request left by
     * an earlier DAC session (for example the explicit "dds" command).
     */
    (void)DacOutput_takeDDSRecoveryRequest();
}

static bool recoverDDSAfterDAC(void)
{
    if (!DacOutput_takeDDSRecoveryRequest()) {
        return false;
    }

    /*
     * DacOutput isolated all five control pins before starting PA15. Keep the
     * relay direct until DAC/DMA are quiescent, then rebuild the DDS bus,
     * clock configuration and channel state exactly once before selecting
     * the DDS output path.
     */
    DacOutput_stop();
    DL_GPIO_clearPins(GPIO_RELAY_PORT, GPIO_RELAY_CTRL_PIN);
    gRelayDDSSelected = false;
    Tick_delay(F_RELAY_SETTLE_MS);
    forceDDSReinitialize();
    gDDSRecoveryCount++;
    UserUART_printf(
        "DDS_RECOVER source=dac guard=bus_hiz reset=1 "
        "clock_mode=refclk_bypass count=%lu\r\n",
        (unsigned long)gDDSRecoveryCount);
    return true;
}

static void ddsAllOff(void)
{
    uint8_t index;

    if (gDDSInitialized) {
        AD9959_setAmp(
            AD9959_CH0 | AD9959_CH1 | AD9959_CH2 | AD9959_CH3, 0U);
        DDS_update();
    }

    for (index = 0U; index < 4U; index++) {
        gFControl.outputAmp[index] = 0U;
    }
}

static void relaySelectDirect(void)
{
    DacOutput_stop();
    ddsAllOff();
    DL_GPIO_clearPins(GPIO_RELAY_PORT, GPIO_RELAY_CTRL_PIN);
    gRelayDDSSelected = false;
    Tick_delay(F_RELAY_SETTLE_MS);
}

static void relaySelectDDS(void)
{
    DacOutput_stop();
    if (gRelayDDSSelected) {
        return;
    }

    ddsAllOff();
    DL_GPIO_setPins(GPIO_RELAY_PORT, GPIO_RELAY_CTRL_PIN);
    gRelayDDSSelected = true;
    Tick_delay(F_RELAY_SETTLE_MS);
}

static void applySingleOutput(uint32_t freqHz, uint16_t amp, uint16_t phase)
{
    DDS_SingleToneParam_t tone;
    uint8_t index;
    bool recovered;

    recovered = recoverDDSAfterDAC();
    relaySelectDDS();
    ensureDDS();

    AD9959_setAmp(AD9959_CH1 | AD9959_CH2 | AD9959_CH3, 0U);
    tone.freq = (float)freqHz;
    tone.amp = amp;
    tone.phase = phase;
    DDS_singleTone(AD9959_CH0, &tone);
    DDS_update();
    if (recovered) {
        /*
         * Repeat the complete channel write after the relay settles. This is
         * cheap on a DAC-to-DDS transition and removes dependence on the first
         * update immediately following clock-path start-up.
         */
        Tick_delay(2U);
        DDS_singleTone(AD9959_CH0, &tone);
        DDS_update();
    }

    gFControl.outputFreqHz[0] = freqHz;
    gFControl.outputAmp[0] = amp;
    gFControl.outputPhase[0] = phase;
    for (index = 1U; index < 4U; index++) {
        gFControl.outputFreqHz[index] = 0U;
        gFControl.outputAmp[index] = 0U;
        gFControl.outputPhase[index] = 0U;
    }
}

static void applyPFDReference(uint32_t freqHz, uint16_t amp, uint16_t phase)
{
    uint32_t ftw;
    uint8_t index;
    bool recovered;

    /*
     * CH1 is wired only to the external phase/frequency detector.  Keep the
     * relay released so this diagnostic reference cannot replace the direct
     * signal shown on scope CH2.
     */
    relaySelectDirect();
    recovered = recoverDDSAfterDAC();
    ensureDDS();
    AD9959_setAmp(AD9959_CH0 | AD9959_CH2 | AD9959_CH3, 0U);
    ftw = (uint32_t)((float)freqHz * AD9959_FTW_PER_HZ + 0.5F);
    AD9959_setFTW(AD9959_CH0 | AD9959_CH1, ftw);
    AD9959_setPhase(AD9959_CH0 | AD9959_CH1, phase);
    AD9959_setAmp(AD9959_CH1, amp);
    AD9959_syncPhaseAccumulators();
    if (recovered) {
        Tick_delay(2U);
        AD9959_setFTW(AD9959_CH0 | AD9959_CH1, ftw);
        AD9959_setPhase(AD9959_CH0 | AD9959_CH1, phase);
        AD9959_setAmp(AD9959_CH1, amp);
        AD9959_syncPhaseAccumulators();
    }

    for (index = 0U; index < 4U; index++) {
        gFControl.outputFreqHz[index] = 0U;
        gFControl.outputAmp[index] = 0U;
        gFControl.outputPhase[index] = 0U;
    }
    gFControl.outputFreqHz[1] = freqHz;
    gFControl.outputAmp[1] = amp;
    gFControl.outputPhase[1] = phase;
}

static bool applyManualMode(FMode mode)
{
    uint32_t outputFreqHz;
    uint16_t phase;

    if ((gFControl.inputFreqHz < F_FREQ_MIN_HZ) ||
        (gFControl.inputFreqHz > F_FREQ_MAX_HZ)) {
        UserUART_write("ERR no valid input frequency; use ffreq first\r\n");
        return false;
    }

    outputFreqHz = gFControl.inputFreqHz;
    phase = gFControl.singlePhaseWord;

    if (mode == F_MODE_SAME) {
        phase = gFControl.singlePhaseWord;
    } else if (mode == F_MODE_QUAD) {
        phase = gFControl.singlePhaseWord;
    } else if (mode == F_MODE_DOUBLE) {
        outputFreqHz *= 2U;
        phase = gFControl.singlePhaseWord;
    } else {
        return false;
    }

    gFControl.mode = mode;
    gFControl.autoTarget = F_MODE_OFF;
    applySingleOutput(
        outputFreqHz,
        ampFromDiv(gFControl.targetDiv, outputFreqHz),
        phase);
    setFState(F_STATE_READY);
    UserUART_printf(
        "OK fmode %s input_hz=%lu output_hz=%lu div=%u asf=%u phase=%u\r\n",
        fModeName(mode), (unsigned long)gFControl.inputFreqHz,
        (unsigned long)outputFreqHz, (unsigned int)gFControl.targetDiv,
        (unsigned int)ampFromDiv(gFControl.targetDiv, outputFreqHz),
        (unsigned int)phase);
    return true;
}

static bool applyProbe4(const uint32_t freqHz[4], uint16_t amp)
{
    static const uint8_t channels[4] = {
        AD9959_CH0, AD9959_CH1, AD9959_CH2, AD9959_CH3
    };
    DDS_SingleToneParam_t tone;
    uint8_t index;
    uint8_t pass;
    bool recovered;

    if (freqHz == NULL) return false;
    for (index = 0U; index < 4U; index++) {
        if ((freqHz[index] < F_FREQ_MIN_HZ) ||
            (freqHz[index] > F_FREQ_MAX_HZ)) {
            return false;
        }
    }

    recovered = recoverDDSAfterDAC();
    relaySelectDDS();
    ensureDDS();
    for (pass = 0U; pass < (recovered ? 2U : 1U); pass++) {
        if (pass != 0U) {
            Tick_delay(2U);
        }
        for (index = 0U; index < 4U; index++) {
            tone.freq = (float)freqHz[index];
            tone.amp = amp;
            tone.phase = 0U;
            DDS_singleTone(channels[index], &tone);
            gFControl.outputFreqHz[index] = freqHz[index];
            gFControl.outputAmp[index] = amp;
            gFControl.outputPhase[index] = 0U;
        }
        DDS_update();
    }
    gFControl.mode = F_MODE_PROBE4;
    return true;
}

static void startAutoMode(FMode mode)
{
    if ((mode != F_MODE_AUTO_LINE) &&
        (mode != F_MODE_AUTO_CIRCLE) &&
        (mode != F_MODE_AUTO_INFINITY)) {
        return;
    }

    gFControl.mode = mode;
    gFControl.autoTarget = mode;
    gFControl.requestId++;
    gFControl.lastDKTick = Tick_now();
    if (!runCCDAutoMode(mode)) {
        relaySelectDirect();
        setFState(F_STATE_ERROR);
        APP_UART_PRINTF(
            "F_EVENT visual_auto=failed request=%lu\r\n",
            (unsigned long)gFControl.requestId);
    }
}

static void requestDKAutoMode(FMode mode, const char *keyName)
{
    const char *targetName;

    switch (mode) {
        case F_MODE_AUTO_LINE:
            targetName = "line";
            break;
        case F_MODE_AUTO_CIRCLE:
            targetName = "circle";
            break;
        case F_MODE_AUTO_INFINITY:
            targetName = "infinity";
            break;
        default:
            return;
    }

    /*
     * Requirement 5 has no electrical input at PA16.  Leave every local
     * measurement/output block in a neutral state and ask the DK camera
     * controller to run the complete blind optical pipeline.  The same three
     * keys retain their existing requirements 1--4 behavior whenever the
     * electrical input is present, so no legacy control mapping is replaced.
    */
    phaseLockStop(true);
    gFControl.mode = mode;
    gFControl.autoTarget = mode;
    gFControl.requestId++;
    gFControl.lastDKTick = Tick_now();
    setFState(F_STATE_WAIT_DK);
    SoundLight_play(SOUND_LIGHT_CUE_START);
    UserUART_printf(
        "F_EVENT requirement5_start=%s key=%s request=%lu "
        "relay=direct\r\n",
        targetName, keyName,
        (unsigned long)gFControl.requestId);
}

static void setFState(FState state)
{
    FState previousState = gFControl.state;

    gRainbowEnabled = false;
    gFControl.state = state;

    if (state != previousState) {
        switch (state) {
            case F_STATE_SEARCHING:
                SoundLight_play(SOUND_LIGHT_CUE_START);
                break;
            case F_STATE_STABLE:
                SoundLight_play(SOUND_LIGHT_CUE_SUCCESS);
                break;
            case F_STATE_ERROR:
                SoundLight_play(SOUND_LIGHT_CUE_ERROR);
                break;
            case F_STATE_READY:
            case F_STATE_WAIT_DK:
            case F_STATE_LOCKING:
            default:
                break;
        }
    }

    switch (state) {
        case F_STATE_READY:
            RGBLED_setColor(0U, 24U, 0U);
            break;
        case F_STATE_WAIT_DK:
            RGBLED_setColor(0U, 0U, 64U);
            break;
        case F_STATE_SEARCHING:
            RGBLED_setColor(0U, 48U, 48U);
            break;
        case F_STATE_LOCKING:
            RGBLED_setColor(64U, 40U, 0U);
            break;
        case F_STATE_STABLE:
            RGBLED_setColor(0U, 96U, 0U);
            break;
        case F_STATE_ERROR:
        default:
            RGBLED_setColor(96U, 0U, 0U);
            break;
    }
}

static uint16_t ampFromDiv(uint8_t div, uint32_t outputFreqHz)
{
    uint8_t row;
    uint8_t index;

    switch (div) {
        case 2U:
            row = 0U;
            break;
        case 4U:
            row = 1U;
            break;
        case 6U:
            row = 2U;
            break;
        case 8U:
        default:
            row = 3U;
            break;
    }

    if (outputFreqHz <= gAmpAnchorHz[0]) {
        return gAmpAnchorAsf[row][0];
    }
    for (index = 1U; index < F_AMP_ANCHOR_COUNT; index++) {
        if (outputFreqHz <= gAmpAnchorHz[index]) {
            uint32_t frequencySpan =
                gAmpAnchorHz[index] - gAmpAnchorHz[index - 1U];
            uint32_t frequencyOffset =
                outputFreqHz - gAmpAnchorHz[index - 1U];
            int32_t codeSpan =
                (int32_t)gAmpAnchorAsf[row][index] -
                (int32_t)gAmpAnchorAsf[row][index - 1U];
            int32_t code = (int32_t)gAmpAnchorAsf[row][index - 1U] +
                (codeSpan * (int32_t)frequencyOffset) /
                    (int32_t)frequencySpan;
            return (uint16_t)code;
        }
    }
    return gAmpAnchorAsf[row][F_AMP_ANCHOR_COUNT - 1U];
}

static uint16_t circlePhaseOffset(uint32_t inputFreqHz)
{
    uint8_t index;
    int32_t phase;

    if (inputFreqHz <= gCirclePhaseAnchorHz[0]) {
        phase = (int32_t)gCirclePhaseAnchorWord[0];
    } else {
        phase = (int32_t)
            gCirclePhaseAnchorWord[F_CIRCLE_PHASE_ANCHOR_COUNT - 1U];
        for (index = 1U;
             index < F_CIRCLE_PHASE_ANCHOR_COUNT;
             index++) {
            if (inputFreqHz <= gCirclePhaseAnchorHz[index]) {
                uint32_t frequencySpan =
                    gCirclePhaseAnchorHz[index] -
                    gCirclePhaseAnchorHz[index - 1U];
                uint32_t frequencyOffset =
                    inputFreqHz - gCirclePhaseAnchorHz[index - 1U];
                int32_t phaseSpan =
                    (int32_t)gCirclePhaseAnchorWord[index] -
                    (int32_t)gCirclePhaseAnchorWord[index - 1U];

                phase = (int32_t)gCirclePhaseAnchorWord[index - 1U] +
                    (phaseSpan * (int32_t)frequencyOffset) /
                        (int32_t)frequencySpan;
                break;
            }
        }
    }

    return phaseLockWrapWord(phase);
}

static const char *fModeName(FMode mode)
{
    switch (mode) {
        case F_MODE_OFF:
            return "off";
        case F_MODE_THRU:
            return "thru";
        case F_MODE_SAME:
            return "same";
        case F_MODE_QUAD:
            return "quad";
        case F_MODE_DOUBLE:
            return "double";
        case F_MODE_PROBE4:
            return "probe4";
        case F_MODE_RAMP:
            return "ramp";
        case F_MODE_AUTO_LINE:
            return "auto_line";
        case F_MODE_AUTO_CIRCLE:
            return "auto_circle";
        case F_MODE_AUTO_INFINITY:
            return "auto_infinity";
        default:
            return "unknown";
    }
}

static const char *fStateName(FState state)
{
    switch (state) {
        case F_STATE_READY:
            return "ready";
        case F_STATE_WAIT_DK:
            return "wait_dk";
        case F_STATE_SEARCHING:
            return "searching";
        case F_STATE_LOCKING:
            return "locking";
        case F_STATE_STABLE:
            return "stable";
        case F_STATE_ERROR:
            return "error";
        default:
            return "unknown";
    }
}

static bool parseU32(const char **text, uint32_t *value)
{
    char *end;
    unsigned long parsed;
    const char *start;

    if ((text == NULL) || (*text == NULL) || (value == NULL)) {
        return false;
    }

    start = skipSpaces(*text);
    if ((*start < '0') || (*start > '9')) {
        return false;
    }

    parsed = strtoul(start, &end, 10);
    if ((end == start) || (parsed > 0xFFFFFFFFUL)) {
        return false;
    }

    *value = (uint32_t)parsed;
    *text = end;
    return true;
}

static const char *skipSpaces(const char *text)
{
    while ((text != NULL) && (*text == ' ')) {
        text++;
    }
    return text;
}
