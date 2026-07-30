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
#define PLOCK_FTW_TRIM_TOTAL_LIMIT 512
/*
 * At a 500 MHz DDS clock and a 20 ms control period:
 *   delta_FTW = phase_step * 262144 / 10000000.
 */
#define PLOCK_FTW_TRIM_NUMERATOR 262144L
#define PLOCK_FTW_TRIM_DENOMINATOR 10000000L
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
#define CCD_AUTO_THRESHOLD_PERCENT 25U
#define CCD_AUTO_SEARCH_THRESHOLD_PERCENT 40U
#define CCD_PHASE_THRESHOLD_PERCENT 40U
#define CCD_AUTO_SETTLE_MS 35U
#define CCD_AUTO_FREQ_SETTLE_MS 100U
#define CCD_AUTO_SEARCH_EXPOSURE_MS 40U
#define CCD_FINE_SEARCH_RADIUS_HZ 600U
#define CCD_FINE_PRIME_MS 500U
#define CCD_FINE_MIN_WIDTH 18U
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
#define CCD_FREQ_SETTLE_MS 250U
#define CCD_FREQ_SAMPLE_FRAMES 3U
#define CCD_FREQ_MIN_TRAIN_SPAN 60U
/*
 * Calibrated vertical span painted by PA15 codes 256..3840 on the present
 * scope/camera geometry. At 1 kHz with a 5 ms ramp, ten zero crossings occupy
 * nine intervals over 74 pixels, giving a full active span of 82.2 pixels.
 */
#define CCD_FREQ_ACTIVE_SPAN_PIXELS 82U
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
static char gCommand[APP_COMMAND_SIZE];
static FControl gFControl;
static PhaseLockControl gPhaseLock;
static uint16_t gPhaseScanCoarse[PLOCK_SCAN_COARSE_COUNT];
static uint16_t gPhaseScanFine[PLOCK_SCAN_FINE_COUNT];
static uint16_t gCCDPixels[TSL1401_PIXEL_COUNT];
static uint16_t gCCDBackground[TSL1401_PIXEL_COUNT];
static uint16_t gCCDCorrected[TSL1401_PIXEL_COUNT];
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
static bool runCCDAutoMode(FMode mode);
static bool ccdEstimateFrequency(uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint32_t *frequencyHz, uint32_t *windowUs, uint8_t *peakCount);
static uint32_t ccdEvaluateFrequency(uint32_t frequencyHz,
    uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT]);
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
static uint32_t roundFrequency100(uint32_t frequencyHz);
static bool measureInputFrequency(uint32_t *frequencyHz);
static bool captureInputFrequency(
    uint32_t sampleRateHz, uint32_t *frequencyHz);
static void ensureDDS(void);
static void forceDDSReinitialize(void);
static void ddsAllOff(void);
static void relaySelectDirect(void);
static void relaySelectDDS(void);
static void applySingleOutput(uint32_t freqHz, uint16_t amp, uint16_t phase);
static void applyPFDReference(uint32_t freqHz, uint16_t amp, uint16_t phase);
static bool applyManualMode(FMode mode);
static bool applyProbe4(const uint32_t freqHz[4], uint16_t amp);
static void startAutoMode(FMode mode);
static void setFState(FState state);
static uint16_t ampFromDiv(uint8_t div, uint32_t outputFreqHz);
static const char *fModeName(FMode mode);
static const char *fStateName(FState state);
static bool parseU32(const char **text, uint32_t *value);
static const char *skipSpaces(const char *text);

void App_init(void)
{
    BTN_init();
    ENC_init();
    AdcCapture_init();
    DacOutput_init();
    TSL1401_init();

    /*
     * Put the external DDS bus in an idle state without resetting or enabling
     * the AD9959. Actual DDS initialization is deferred until the "dds" command.
     */
    AD9959_IOInit();

    gRainbowEnabled = false;
    gDDSInitialized = false;
    gRelayDDSSelected = false;
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
    DacOutput_service();
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
                "ccd freq | ccd status\r\n");
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
                "OK dds reinitialized power_wait_ms=100 pll_wait_ms=50\r\n");
            return;
        } else if (strcmp(args, "off") == 0) {
            relaySelectDirect();
            UserUART_write("OK dds off relay=direct\r\n");
            return;
        }

        second = 512U;
        third = 0U;
        if (!parseU32(&args, &first) || (first > 200000000U)) {
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
         * This command is also the bench recovery path after the DDS module
         * is powered later than the MCU. Always repeat the hardware reset and
         * PLL setup before producing the requested diagnostic tone.
         */
        relaySelectDirect();
        forceDDSReinitialize();
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
        if (!DacOutput_playWindowedRamp(value[0],
                CCD_AUTO_FRAME_US, F_RAMP_MIN_CODE, F_RAMP_MAX_CODE,
                DAC_OUTPUT_MIDCODE)) {
            DacOutput_stop();
            setFState(F_STATE_ERROR);
            UserUART_write("ERR fwindow start failed\r\n");
            return true;
        }
        gFControl.mode = F_MODE_RAMP;
        gFControl.lastDKTick = Tick_now();
        setFState(F_STATE_SEARCHING);
        UserUART_printf(
            "OK fwindow window_us=%lu frame_us=%u relay=direct\r\n",
            (unsigned long)value[0], (unsigned int)CCD_AUTO_FRAME_US);
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
    UserUART_write("help | status | led R G B | rainbow on|off\r\n");
    UserUART_write("adc once [1000..1000000] | dac CODE(0..4095)\r\n");
    UserUART_write(
        "ccd capture [1..100ms] [10..90pct] | ccd dump [...] | "
        "ccd freq | ccd status\r\n");
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
     * Store the fixed grid with the DAC at its midpoint. The windowed ramp
     * idles at the same midpoint, so AC-coupled scope display places the
     * rising segment symmetrically around the screen center.
     */
    relaySelectDirect();
    DacOutput_setCode(DAC_OUTPUT_MIDCODE);
    Tick_delay(80U);
    if (!TSL1401_captureAuto(gCCDBackground, &gCCDExposureMs)) {
        DacOutput_stop();
        relaySelectDirect();
        setFState(F_STATE_ERROR);
        APP_UART_PRINTF(
            "CCD_FREQ error=background_capture "
            "check=ccd_power_and_alignment\r\n");
        return false;
    }
    APP_UART_PRINTF("CCD_FREQ exposure_ms=%lu\r\n",
        (unsigned long)gCCDExposureMs);

    if (!ccdEstimateFrequency(gCCDExposureMs, gCCDBackground,
            &coarseFrequencyHz, &probeWindowUs, &coarsePeaks)) {
        DacOutput_stop();
        relaySelectDirect();
        setFState(F_STATE_ERROR);
        APP_UART_PRINTF(
            "CCD_FREQ error=coarse_frequency "
            "check=camera_vertical_off_center\r\n");
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

    DacOutput_stop();
    outputAmplitude = ampFromDiv(8U, detectedFrequencyHz);
    applySingleOutput(detectedFrequencyHz, outputAmplitude, 0U);
    gFControl.inputFreqHz = detectedFrequencyHz;
    gFControl.targetDiv = 8U;
    gFControl.singlePhaseWord = 0U;
    gFControl.mode = F_MODE_SAME;
    gFControl.autoTarget = F_MODE_OFF;
    gFControl.lastDKTick = Tick_now();
    setFState(F_STATE_READY);

    APP_UART_PRINTF(
        "CCD_FREQ complete detected_hz=%lu coarse_hz=%lu fine_hz=%lu "
        "dds_hz=%lu amp=%u lock=off elapsed_ms=%lu\r\n",
        (unsigned long)detectedFrequencyHz,
        (unsigned long)coarseFrequencyHz,
        (unsigned long)fineFrequencyHz,
        (unsigned long)detectedFrequencyHz,
        (unsigned int)outputAmplitude,
        (unsigned long)Tick_elapsed(startTick));
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
     * Capture the fixed grid and the collapsed Y=midpoint trace first.
     * Subsequent ramp frames use the same exposure and subtract this line.
     */
    relaySelectDirect();
    DacOutput_setCode(DAC_OUTPUT_MIDCODE);
    Tick_delay(80U);
    if (!TSL1401_captureAuto(gCCDBackground, &gCCDExposureMs)) {
        APP_UART_PRINTF("CCD_AUTO error=background_capture\r\n");
        return false;
    }
    APP_UART_PRINTF("CCD_AUTO exposure_ms=%lu\r\n",
        (unsigned long)gCCDExposureMs);

    if (!ccdEstimateFrequency(gCCDExposureMs, gCCDBackground,
            &coarseFrequencyHz, &probeWindowUs, &coarsePeaks)) {
        DacOutput_stop();
        APP_UART_PRINTF(
            "CCD_AUTO error=coarse_frequency "
            "check=camera_vertical_off_center\r\n");
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

static bool ccdEstimateFrequency(uint32_t exposureMs,
    const uint16_t background[TSL1401_PIXEL_COUNT],
    uint32_t *frequencyHz, uint32_t *windowUs, uint8_t *peakCount)
{
    /*
     * Latency-optimized probe order. The 1 ms window covers most of the band;
     * 5 ms resolves 1..3 kHz, 200 us covers the upper middle band, and 50 us
     * covers the top end. The remaining windows are fallbacks for merged or
     * clipped optical trains.
     */
    static const uint16_t probeWindowsUs[] = {
        1000U, 5000U, 200U, 50U, 500U,
        10000U, 100U, 2000U, 20U, 10U
    };
    TSL1401_Stats stats;
    TSL1401_Stats frameStats;
    uint8_t localCenters[TSL1401_MAX_PEAKS];
    uint8_t frameCenters[TSL1401_MAX_PEAKS];
    uint8_t clusterCenters[TSL1401_MAX_PEAKS];
    uint8_t frameClusterCenters[TSL1401_MAX_PEAKS];
    uint32_t estimated;
    uint32_t bestFrequency = 0U;
    uint32_t bestWindow = 0U;
    uint32_t width;
    uint32_t frameWidth;
    uint32_t trainSpan;
    uint32_t clusterTrainSpan;
    uint32_t frameClusterTrainSpan;
    uint64_t estimateNumerator;
    uint64_t estimateDenominator;
    uint16_t prominenceThreshold;
    uint16_t frameProminenceThreshold;
    uint8_t rawLocalPointCount;
    uint8_t frameRawLocalPointCount;
    uint8_t localPointCount;
    uint8_t frameLocalPointCount;
    uint8_t clusterCount;
    uint8_t frameClusterCount;
    uint8_t effectivePointCount;
    uint8_t minimumGap;
    uint8_t maximumGap;
    uint8_t frameMinimumGap;
    uint8_t frameMaximumGap;
    uint8_t clusterMinimumGap;
    uint8_t clusterMaximumGap;
    uint8_t frameClusterMinimumGap;
    uint8_t frameClusterMaximumGap;
    uint8_t bestPeaks = 0U;
    uint8_t index;
    uint8_t point;
    uint8_t frame;

    (void)background;
    for (index = 0U;
         index < (sizeof(probeWindowsUs) / sizeof(probeWindowsUs[0]));
         index++) {
        if (!DacOutput_playWindowedRamp(probeWindowsUs[index],
                CCD_AUTO_FRAME_US, F_RAMP_MIN_CODE, F_RAMP_MAX_CODE,
                DAC_OUTPUT_MIDCODE)) {
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
                DAC_OUTPUT_MIDCODE)) {
            continue;
        }
        /*
         * The oscilloscope intensity display needs several refreshes after
         * changing the ramp window. A short 35 ms delay leaves remnants from
         * the preceding wider window and merges real crossings.
         */
        Tick_delay(CCD_FREQ_SETTLE_MS);
        rawLocalPointCount = 0U;
        localPointCount = 0U;
        clusterCount = 0U;
        prominenceThreshold = 0U;
        minimumGap = 0U;
        maximumGap = 0U;
        clusterMinimumGap = 0U;
        clusterMaximumGap = 0U;
        clusterTrainSpan = 0U;
        width = 0U;
        memset(&stats, 0, sizeof(stats));
        for (frame = 0U; frame < CCD_FREQ_SAMPLE_FRAMES; frame++) {
            /*
             * Evaluate independent frames. Pixel-wise median filtering erases
             * narrow optical peaks when the scope trace moves by one pixel.
             */
            if (!TSL1401_capture(gCCDPixels, exposureMs)) {
                DacOutput_stop();
                return false;
            }
            TSL1401_analyze(gCCDPixels,
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
                gCCDPixels, frameCenters, &frameProminenceThreshold);
            frameLocalPointCount = ccdSelectRegularPeaks(
                frameCenters, frameRawLocalPointCount,
                &frameMinimumGap, &frameMaximumGap);
            trainSpan = (frameLocalPointCount >= 2U) ?
                (uint32_t)(frameCenters[frameLocalPointCount - 1U] -
                    frameCenters[0]) : 0U;

            APP_UART_PRINTF(
                "CCD_FRAME window_us=%u frame=%u peaks=%u span=%u clusters=%u "
                "cluster_gap=%u..%u cluster_span=%lu raw=%u "
                "regular=%u gap=%u..%u train_span=%lu\r\n",
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
                (unsigned long)trainSpan);

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

        /*
         * With the present camera focus, the two mathematical crossings in
         * one sine period form one broad optical cluster. These clusters are
         * much more stable than their internal shoulders. Their spacing, not
         * their integer count, gives the frequency without a window-edge
         * quantization error:
         *
         * f = (N - 1) * active_pixels / train_span / window_seconds
         */
        if ((clusterCount >= CCD_FREQ_MIN_POINTS) &&
            (clusterCount <= CCD_FREQ_MAX_POINTS)) {
            if (clusterTrainSpan >= CCD_FREQ_MIN_TRAIN_SPAN) {
                estimateNumerator =
                    (uint64_t)(clusterCount - 1U) *
                    CCD_FREQ_ACTIVE_SPAN_PIXELS * 1000000ULL;
                estimateDenominator =
                    (uint64_t)clusterTrainSpan * probeWindowsUs[index];
                estimated = (uint32_t)(
                    (estimateNumerator + (estimateDenominator / 2ULL)) /
                    estimateDenominator);
                estimated = roundFrequency100(estimated);
                if ((estimated >= F_FREQ_MIN_HZ) &&
                    (estimated <= F_FREQ_MAX_HZ)) {
                    bestFrequency = estimated;
                    bestWindow = probeWindowsUs[index];
                    bestPeaks = clusterCount;
                    APP_UART_PRINTF(
                        "CCD_PROBE selected model=cycle_clusters "
                        "window_us=%lu clusters=%u estimate_hz=%lu\r\n",
                        (unsigned long)bestWindow,
                        (unsigned int)bestPeaks,
                        (unsigned long)bestFrequency);
                    break;
                }
            }
        }

        /*
         * The source is constrained to 100 Hz steps, so every 10 ms frame
         * starts at the same source phase. A vertical ramp maps time to the
         * scope Y axis. The CCD line through the middle therefore sees about
         * two crossings for each input cycle contained in the ramp window.
         *
         * Long windows produce too many crossings and merge into a bright
         * band. Shorten the ramp in the 10-5-2-1 sequence until 4..16
         * separated crossings remain. Reject a merged band by limiting the
         * width of each individual optical point.
         */
        if ((localPointCount < CCD_FREQ_MIN_POINTS) ||
            (localPointCount > CCD_FREQ_MAX_POINTS) ||
            (trainSpan < CCD_FREQ_MIN_TRAIN_SPAN) ||
            ((stats.maximum - stats.minimum) < 100U)) {
            continue;
        }

        /*
         * The outermost crossings can fall just outside the camera line.
         * Keep the effective-point check for rejecting implausible trains,
         * but derive frequency directly from the measured point spacing.
         * There are two zero crossings per input cycle.
         */
        effectivePointCount = (uint8_t)(
            (((uint32_t)CCD_FREQ_ACTIVE_SPAN_PIXELS *
              (uint32_t)(localPointCount - 1U)) +
             (trainSpan / 2U)) / trainSpan);
        if ((effectivePointCount < CCD_FREQ_MIN_POINTS) ||
            (effectivePointCount > CCD_FREQ_MAX_POINTS)) {
            continue;
        }
        estimateNumerator =
            (uint64_t)(localPointCount - 1U) *
            CCD_FREQ_ACTIVE_SPAN_PIXELS * 500000ULL;
        estimateDenominator =
            (uint64_t)trainSpan * probeWindowsUs[index];
        estimated = (uint32_t)(
            (estimateNumerator + (estimateDenominator / 2ULL)) /
            estimateDenominator);
        estimated = roundFrequency100(estimated);
        if ((estimated < F_FREQ_MIN_HZ) ||
            (estimated > F_FREQ_MAX_HZ)) {
            continue;
        }

        bestFrequency = estimated;
        bestWindow = probeWindowsUs[index];
        bestPeaks = effectivePointCount;
        APP_UART_PRINTF(
            "CCD_PROBE selected window_us=%lu visible_points=%u "
            "effective_points=%u "
            "estimate_hz=%lu\r\n",
            (unsigned long)bestWindow, (unsigned int)localPointCount,
            (unsigned int)bestPeaks,
            (unsigned long)bestFrequency);
        break;
    }

    DacOutput_setCode(DAC_OUTPUT_MIDCODE);
    if (bestFrequency == 0U) {
        return false;
    }
    *frequencyHz = bestFrequency;
    *windowUs = bestWindow;
    *peakCount = bestPeaks;
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

static uint32_t roundFrequency100(uint32_t frequencyHz)
{
    return ((frequencyHz + 50U) / 100U) * 100U;
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
    UserUART_write("pfdref FREQ_HZ AMP PHASE | pfdref off\r\n");
    UserUART_write("framp 1|2|5|10|off (milliseconds)\r\n");
    UserUART_write(
        "frampus 20|50|100|200|500|1000|2000|5000|10000 (microseconds)\r\n");
    UserUART_write(
        "fwindow 20|50|100|200|500|1000|2000|5000|10000|off "
        "(10ms frame)\r\n");
    UserUART_write("fprobe F0 F1 F2 F3 [AMP]\r\n");
    UserUART_write(
        "fauto line|circle|infinity (TSL1401 visual closed loop)\r\n");
    UserUART_write(
        "fstate ready|wait|search|lock|stable|error\r\n");
}

static void showFStatus(void)
{
    UserUART_printf(
        "F_STATUS mode=%s auto=%s state=%s input_hz=%lu target_div=%u "
        "phase_word=%u request=%lu dk_age_ms=%lu "
        "relay_path=%s pa13=%u\r\n",
        fModeName(gFControl.mode), fModeName(gFControl.autoTarget),
        fStateName(gFControl.state),
        (unsigned long)gFControl.inputFreqHz,
        (unsigned int)gFControl.targetDiv,
        (unsigned int)gFControl.singlePhaseWord,
        (unsigned long)gFControl.requestId,
        (unsigned long)Tick_elapsed(gFControl.lastDKTick),
        gRelayDDSSelected ? "dds" : "direct",
        gRelayDDSSelected ? 1U : 0U);
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
        "dds_init=%u rainbow=%u\r\n",
        (unsigned long)Tick_now(), AdcCapture_isBusy() ? 1U : 0U,
        AdcCapture_isReady() ? 1U : 0U,
        (unsigned long)AdcCapture_getSampleRate(),
        gDDSInitialized ? 1U : 0U, gRainbowEnabled ? 1U : 0U);
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

    BTN_getData(&buttons);
    if (buttons.left) {
        electricalInputPresent = gPhaseLock.enabled && gPhaseLock.locked;
        if (!electricalInputPresent) {
            electricalInputPresent =
                measureInputFrequency(&detectedFrequencyHz);
        }
        if (!electricalInputPresent) {
            UserUART_write(
                "F_EVENT key=left optical_auto=line "
                "reason=electrical_input_absent\r\n");
            startAutoMode(F_MODE_AUTO_LINE);
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
            UserUART_write(
                "F_EVENT key=right optical_auto=infinity "
                "reason=electrical_input_absent\r\n");
            startAutoMode(F_MODE_AUTO_INFINITY);
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
            UserUART_write(
                "F_EVENT key=mid optical_auto=circle "
                "reason=electrical_input_absent\r\n");
            startAutoMode(F_MODE_AUTO_CIRCLE);
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
    if (ENC_getSW()) {
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
     * The DDS module may be powered after, or independently from, the MCU.
     * Re-run its reset and PLL sequence on every new electrical-lock session.
     */
    forceDDSReinitialize();
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
     * the hardware reset and FR1/PLL sequence is the deterministic recovery.
     */
    gDDSInitialized = false;
    DDS_init();
    gDDSInitialized = true;
    ddsAllOff();
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

    relaySelectDDS();
    ensureDDS();

    AD9959_setAmp(AD9959_CH1 | AD9959_CH2 | AD9959_CH3, 0U);
    tone.freq = (float)freqHz;
    tone.amp = amp;
    tone.phase = phase;
    DDS_singleTone(AD9959_CH0, &tone);
    DDS_update();

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

    /*
     * CH1 is wired only to the external phase/frequency detector.  Keep the
     * relay released so this diagnostic reference cannot replace the direct
     * signal shown on scope CH2.
     */
    relaySelectDirect();
    ensureDDS();
    AD9959_setAmp(AD9959_CH0 | AD9959_CH2 | AD9959_CH3, 0U);
    ftw = (uint32_t)((float)freqHz * AD9959_FTW_PER_HZ + 0.5F);
    AD9959_setFTW(AD9959_CH0 | AD9959_CH1, ftw);
    AD9959_setPhase(AD9959_CH0 | AD9959_CH1, phase);
    AD9959_setAmp(AD9959_CH1, amp);
    AD9959_syncPhaseAccumulators();

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

    if (freqHz == NULL) return false;
    for (index = 0U; index < 4U; index++) {
        if ((freqHz[index] < F_FREQ_MIN_HZ) ||
            (freqHz[index] > F_FREQ_MAX_HZ)) {
            return false;
        }
    }

    relaySelectDDS();
    ensureDDS();
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

static void setFState(FState state)
{
    gRainbowEnabled = false;
    gFControl.state = state;

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
