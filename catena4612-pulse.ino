/*

Module:  catena4612-pulse.ino

Function:
        Code for the electric power sensor with Catena 4450-M101.

Copyright notice:
        See LICENSE file accompanying this project

Author:
        Terry Moore, MCCI Corporation	March 2017

*/

#include <cstdint>

#include <Catena.h>

#include <Catena_Led.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_TxBuffer.h>
#include <Catena_CommandStream.h>
#include <Catena_Si1133.h>
#include <Catena_Totalizer.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Arduino_LoRaWAN.h>
#include <lmic.h>
#include <hal/hal.h>
#include <mcciadk_baselib.h>

#include <cmath>
#include <type_traits>

/****************************************************************************\
|
|       Manifest constants & typedefs.
|
\****************************************************************************/

using namespace McciCatena;

/* how long do we wait between transmissions? (in seconds) */
enum    {
        // set this to interval between transmissions, in seconds
        // Actual time will be a little longer because have to
        // add measurement and broadcast time, but we attempt
        // to compensate for the gross effects below.
        CATCFG_T_CYCLE = 6 * 60,        // every 6 minutes
        CATCFG_T_CYCLE_TEST = 30,       // every 30 seconds
        CATCFG_T_CYCLE_INITIAL = 30,    // every 30 seconds initially
        CATCFG_INTERVAL_COUNT_INITIAL = 30,     // repeat for 15 minutes
        };

/* additional timing parameters; ususually you don't change these. */
enum    {
        CATCFG_T_WARMUP = 1,
        CATCFG_T_SETTLE = 5,
        CATCFG_T_OVERHEAD = (CATCFG_T_WARMUP + CATCFG_T_SETTLE + 4),
        CATCFG_T_MIN = CATCFG_T_OVERHEAD,
        CATCFG_T_MAX = CATCFG_T_CYCLE < 60 * 60 ? 60 * 60 : CATCFG_T_CYCLE,     // normally one hour max.
        CATCFG_INTERVAL_COUNT = 30,
        };

constexpr uint32_t CATCFG_GetInterval(uint32_t tCycle)
        {
        return (tCycle < CATCFG_T_OVERHEAD + 1)
                ? 1
                : tCycle - CATCFG_T_OVERHEAD
                ;
        }

enum    {
        CATCFG_T_INTERVAL = CATCFG_GetInterval(CATCFG_T_CYCLE),
        };

// forwards
static void settleDoneCb(osjob_t *pSendJob);
static void warmupDoneCb(osjob_t *pSendJob);
static void txNotProvisionedCb(osjob_t *pSendJob);
static void sleepDoneCb(osjob_t *pSendJob);
static Arduino_LoRaWAN::SendBufferCbFn sendBufferDoneCb;
void fillBuffer(TxBuffer_t &b);
void startSendingUplink(void);
void setTxCycleTime(unsigned txCycle, unsigned txCount);

// function for scaling pulse data
static uint16_t
dNdT_getFrac(
        uint32_t deltaC,
        uint32_t delta_ms
        );

/****************************************************************************\
|
|   pins and such. Shouldn't these be centralized?
|
\****************************************************************************/

constexpr uint8_t kPinBoost = D33;
constexpr uint8_t kPinExtVddEna = D11;
constexpr uint8_t kPinPulse1P1 = A1;

/****************************************************************************\
|
|   handy constexpr to extract the base name of a file
|
\****************************************************************************/

// two-argument version: first arg is what to return if we don't find
// a directory separator in the second part.
static constexpr const char *filebasename(const char *s, const char *p)
    {
    return p[0] == '\0'                     ? s                            :
           (p[0] == '/' || p[0] == '\\')    ? filebasename(p + 1, p + 1)   :
                                              filebasename(s, p + 1)       ;
    }

static constexpr const char *filebasename(const char *s)
    {
    return filebasename(s, s);
    }

/****************************************************************************\
|
|       Read-only data.
|
\****************************************************************************/

static const char sVersion[] = "0.1.0";

/****************************************************************************\
|
|       Variables.
|
\****************************************************************************/

// the Catena instance
Catena gCatena;

// SPI2: needed for the flash.
SPIClass gSPI2(
                Catena::PIN_SPI2_MOSI,
                Catena::PIN_SPI2_MISO,
                Catena::PIN_SPI2_SCK
                );


// the flash -- presently only here so we can power it down.
Catena_Mx25v8035f gFlash;
bool fFlash;

//
// the LoRaWAN backhaul.  Note that we use the
// Catena version so it can provide hardware-specific
// information to the base class.
//
Catena::LoRaWAN gLoRaWAN;

// the LED instance object
StatusLed gLed (Catena::PIN_STATUS_LED);

// The BME280 instance, for the temperature/humidity sensor
Adafruit_BME280 gBME280; // The default initalizer creates an I2C connection
bool fBme;               // true if BME280 is in use

//   The Light sensor
Catena_Si1133 gSi1133;
bool fLight;              // true if BH1750 is in use

//   The contact sensors
constexpr bool fHasPulse1 = true;

cTotalizer gPulse1P1;

//  the LMIC job that's used to synchronize us with the LMIC code
static osjob_t sensorJob;

// debug flag for throttling sleep prints
bool g_fPrintedSleeping = false;

// the cycle time to use
unsigned gTxCycle;

// remaining before we reset to default
unsigned gTxCycleCount;

static Arduino_LoRaWAN::ReceivePortBufferCbFn receiveMessage;

/****************************************************************************\
|
|	The command table
|
\****************************************************************************/

cCommandStream::CommandFn setTransmitPeriod;

static const cCommandStream::cEntry sApplicationCommmands[] =
        {
        { "tx-period", setTransmitPeriod },
        // other commands go here....
        };

/* create the top level structure for the command dispatch table */
static cCommandStream::cDispatch
sApplicationCommandDispatch(
        sApplicationCommmands,          /* this is the pointer to the table */
        sizeof(sApplicationCommmands),  /* this is the size of the table */
        "application"                   /* this is the "first word" for all the commands in this table*/
        );

/****************************************************************************\
|
|       Code.
|
\****************************************************************************/


/*

Name:	setup()

Function:
        Arduino setup function.

Definition:
        void setup(
            void
            );

Description:
        This function is called by the Arduino framework after
        basic framework has been initialized. We initialize the sensors
        that are present on the platform, set up the LoRaWAN connection,
        and (ultimately) return to the framework, which then calls loop()
        forever.

Returns:
        No explicit result.

*/

void setup(void)
        {
        gCatena.begin();

        setup_platform();
        setup_lorawan();
        setup_sensors();
        setup_flash();
        setup_commands();
        setup_uplink();
        }

void setup_commands()
        {
        /* add our application-specific commands */
        gCatena.addCommands(
                /* name of app dispatch table, passed by reference */
                sApplicationCommandDispatch,
                /*
                || optionally a context pointer using static_cast<void *>().
                || normally only libraries (needing to be reentrant) need
                || to use the context pointer.
                */
                nullptr
                );
        }

void setup_uplink()
        {
        /* for low clock rates, we need wider tolerances, it seems */
#if defined(ARDUINO_ARCH_STM32)
        LMIC_setClockError(10 * 65536 / 100);
#endif
        /* trigger a join by sending the first packet */
        if (!(gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fManufacturingTest)))
                {
                if (!gLoRaWAN.IsProvisioned())
                        gCatena.SafePrintf("LoRaWAN not provisioned yet. Use the commands to set it up.\n");
                else
                        {
                        gLed.Set(LedPattern::Joining);

                        /* warm up the BME280 by discarding a measurement */
                        if (fBme)
                                (void)gBME280.readTemperature();

                        /* trigger a join by sending the first packet */
                        startSendingUplink();
                        }
                }
        }

void setup_platform()
        {
        // turn on the boost converter

        pinMode(kPinBoost, OUTPUT);
        digitalWrite(kPinBoost, 1);

#ifdef USBCON
        // if running unattended, don't wait for USB connect.
        if (!(gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)))
                {
                while (!Serial)
                        /* wait for USB attach */
                        yield();
                }
#endif

        gCatena.SafePrintf("\n");
        gCatena.SafePrintf("-------------------------------------------------------------------------------\n");
        gCatena.SafePrintf("This is %s V%s.\n", filebasename(__FILE__), sVersion);
                {
                char sRegion[16];
                gCatena.SafePrintf("Target network: %s / %s\n",
                                gLoRaWAN.GetNetworkName(),
                                gLoRaWAN.GetRegionString(sRegion, sizeof(sRegion))
                                );
                }
        gCatena.SafePrintf("Enter 'help' for a list of commands.\n");
        gCatena.SafePrintf("(remember to select 'Line Ending: Newline' at the bottom of the monitor window.)\n");
        gCatena.SafePrintf("SYSCLK: %u MHz\n", unsigned(gCatena.GetSystemClockRate() / (1000 * 1000)));

#ifdef USBCON
        gCatena.SafePrintf("USB enabled\n");
#else
        gCatena.SafePrintf("USB disabled\n");
#endif

        gCatena.SafePrintf("--------------------------------------------------------------------------------\n");
        gCatena.SafePrintf("\n");

        // set up the LED
        gLed.begin();
        gCatena.registerObject(&gLed);
        gLed.Set(LedPattern::FastFlash);

        // display the CPU unique ID
        Catena::UniqueID_string_t CpuIDstring;

        gCatena.SafePrintf("CPU Unique ID: %s\n",
                gCatena.GetUniqueIDstring(&CpuIDstring)
                );

        /* find the platform */
        const Catena::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();

        uint32_t flags;
        const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();

        if (pPlatform)
                {
                gCatena.SafePrintf("EUI64: ");
                for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
                        {
                        gCatena.SafePrintf("%s%02x", i == 0 ? "" : "-", pSysEUI->b[i]);
                        }
                gCatena.SafePrintf("\n");
                flags = gCatena.GetPlatformFlags();
                gCatena.SafePrintf(
                        "Platform Flags:  %#010x\n",
                        flags
                        );
                gCatena.SafePrintf(
                        "Operating Flags:  %#010x\n",
                        gCatena.GetOperatingFlags()
                        );
                }
        else
                {
                gCatena.SafePrintf("**** no platform, check provisioning ****\n");
                flags = 0;
                }

        /* is it modded? */
        uint32_t modnumber = gCatena.PlatformFlags_GetModNumber(flags);

        if (modnumber != 0)
                {
                gCatena.SafePrintf("Catena 4612-M%u\n", modnumber);
                if (modnumber == 101)
                        {
                        }
                else
                        {
                        gCatena.SafePrintf("Catena 4612: unknown mod number %d\n", modnumber);
                        }
                }
        else
                {
                gCatena.SafePrintf("Standard Catena 4612\n");
                }
        }

void setup_flash(void)
        {
        if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
                {
                fFlash = true;
                gFlash.powerDown();
                gCatena.SafePrintf("FLASH found, put power down\n");
                }
        else
                {
                fFlash = false;
                gFlash.end();
                gSPI2.end();
                gCatena.SafePrintf("No FLASH found: check hardware\n");
                }
        }

void setup_lorawan()
        {
        gCatena.SafePrintf("LoRaWAN init: ");
        if (!gLoRaWAN.begin(&gCatena))
                {
                gCatena.SafePrintf("failed\n");
                gCatena.registerObject(&gLoRaWAN);
                }
        else
                {
                gCatena.SafePrintf("succeeded\n");
                gCatena.registerObject(&gLoRaWAN);
                }

        // hook up downlink handler
        gLoRaWAN.SetReceiveBufferBufferCb(receiveMessage);
        setTxCycleTime(CATCFG_T_CYCLE_INITIAL, CATCFG_INTERVAL_COUNT_INITIAL);
        }

void setup_sensors(void)
        {
        uint32_t const flags = gCatena.GetPlatformFlags();

        /* initialize the lux sensor */
        setup_light();

        /* initialize the BME280 */
        if (!gBME280.begin(BME280_ADDRESS, Adafruit_BME280::OPERATING_MODE::Sleep))
                {
                gCatena.SafePrintf("No BME280 found: defective board or incorrect platform\n");
                fBme = false;
                }
        else
                {
                fBme = true;
                }

        /* initialize the pulse counters */
        if (fHasPulse1)
                {
                pinMode(kPinExtVddEna, OUTPUT);
                digitalWrite(kPinExtVddEna, 1);

                if (! gPulse1P1.begin(kPinPulse1P1))
                        {
                        gCatena.SafePrintf("Pulse totalization failed to start: defective board or incorrect platform\n");
                        }

                pinMode(kPinPulse1P1, INPUT_PULLDOWN);
                }
        }

void setup_light(void)
        {
        if (gSi1133.begin())
                {
                fLight = true;
                auto const measConfig =	Catena_Si1133::ChannelConfiguration_t()
                        .setAdcMux(Catena_Si1133::InputLed_t::LargeWhite)
                        .set24bit(false);

                gSi1133.configure(0, measConfig, 0);
                }
        else
                {
                fLight = false;
                gCatena.SafePrintf("No Si1133 found: check hardware\n");
                }
        }


// The Arduino loop routine -- in our case, we just drive the other loops.
// If we try to do too much, we can break the LMIC radio. So the work is
// done by outcalls scheduled from the LMIC os loop.
void loop()
        {
        gCatena.poll();

        /* for mfg test, don't tx, just fill */
        if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fManufacturingTest))
                {
                TxBuffer_t b;
                fillBuffer(b);
                delay(1000);
                }
        }

static uint16_t dNdT_getFrac(
        uint32_t deltaC,
        uint32_t delta_ms
        )
        {
        if (delta_ms == 0 || deltaC == 0)
                return 0;

        // this is a value in [0,1)
        float dNdTperHour = float(deltaC * 250) / float(delta_ms);

        if (dNdTperHour <= 0)
                return 0;
        else if (dNdTperHour >= 1)
                return 0xFFFF;
        else
                {
                int iExp;
                float normalValue;
                normalValue = frexpf(dNdTperHour, &iExp);

                // dNdTperHour is supposed to be in [0..1), so useful exp
                // is [0..-15]
                iExp += 15;
                if (iExp < 0)
                        iExp = 0;
                if (iExp > 15)
                        return 0xFFFF;


                return (uint16_t)((iExp << 12u) + (unsigned) scalbnf(normalValue, 12));
                }
        }

void fillBuffer(TxBuffer_t &b)
        {
        uint32_t tLuxBegin;

        /* start a Lux measurement */
        if (fLight)
                gSi1133.start(true);

        /* meanwhile, fill the buffer */
        b.begin();
        FlagsSensor2 flag;

        flag = FlagsSensor2(0);

        b.put(FormatSensor2); /* the flag for this record format */
        uint8_t * const pFlag = b.getp();
        b.put(0x00); /* will be set to the flags */

        // vBat is sent as 5000 * v
        float vBat = gCatena.ReadVbat();
        gCatena.SafePrintf("vBat:    %d mV\n", (int)(vBat * 1000.0f));
        b.putV(vBat);
        flag |= FlagsSensor2::FlagVbat;

        uint32_t bootCount;
        if (gCatena.getBootCount(bootCount))
                {
                b.putBootCountLsb(bootCount);
                flag |= FlagsSensor2::FlagBoot;
                }

        if (fBme)
                {
                Adafruit_BME280::Measurements m = gBME280.readTemperaturePressureHumidity();
                // temperature is 2 bytes from -0x80.00 to +0x7F.FF degrees C
                // pressure is 2 bytes, hPa * 10.
                // humidity is one byte, where 0 == 0/256 and 0xFF == 255/256.
                gCatena.SafePrintf(
                        "BME280:  T: %d P: %d RH: %d\n",
                        (int)m.Temperature,
                        (int)m.Pressure,
                        (int)m.Humidity
                        );
                b.putT(m.Temperature);
                b.putP(m.Pressure);
                b.putRH(m.Humidity);

                flag |= FlagsSensor2::FlagTPH;
                }

        if (fLight)
                {
                /* Get a new sensor event */
                uint16_t data[3];
                auto const tNow = millis();

                while ((! gSi1133.isOneTimeReady()) && (millis() - tNow) < 10000)
                        {
                        auto const tWait = millis();

                        // leave the sensor alone for 100 ms.
                        while (millis() - tWait < 100)
                                gCatena.poll();
                        }

                if (millis() - tNow < 1000)
                        {
                        uint32_t data[1];

                        gSi1133.readMultiChannelData(data, 1);
                        gSi1133.stop();
                        gCatena.SafePrintf(
                                "Si1133:  %u White\n",
                                data[0]
                                );

                        b.putLux(uint16_t(data[0]));

                        flag |= FlagsSensor2::FlagLux;
                        }
                }

        if (fHasPulse1)
                {
                uint32_t pulse1in, pulse1out;
                uint32_t pulse1in_dc, pulse1in_dt;
                uint32_t pulse1out_dc, pulse1out_dt;

                pulse1in = gPulse1P1.getcurrent();
                gPulse1P1.getDeltaCountAndTime(pulse1in_dc, pulse1in_dt);
                gPulse1P1.setReference();

                pulse1out = 0;
                pulse1out_dc = 0; pulse1out_dt = pulse1in_dt;

                gCatena.SafePrintf(
                        "Pulses:   Pin1: %u Pin2: %u\n",
                        pulse1in,
                        pulse1out
                        );
                b.putWH(pulse1in);
                b.putWH(pulse1out);
                flag |= FlagsSensor2::FlagWattHours;

                // we know that we get at most 4 pulses per second, no matter
                // the scaling. Therefore, if we convert to pulses/hour, we'll
                // have a value that is no more than 3600 * 4, or 14,400.
                // This fits in 14 bits. At low pulse rates, there's more
                // info in the denominator than in the numerator. So FP is really
                // called for. We use a simple floating point format, since this
                // is unsigned: 4 bits of exponent, 12 bits of fraction, and we
                // don't bother to omit the MSB of the (normalized fraction);
                // and we multiply by 2^(exp+1) (so 0x0800 is 2 * 0.5 == 1,
                // 0xF8000 is 2^16 * 1 = 65536).
                uint16_t fracPulse1In = dNdT_getFrac(pulse1in_dc, pulse1in_dt);
                uint16_t fracPulse1Out = dNdT_getFrac(pulse1out_dc, pulse1out_dt);
                b.putPulseFraction(
                        fracPulse1In
                        );
                b.putPulseFraction(
                        fracPulse1Out
                        );

                gCatena.SafePrintf(
                        "Power:   IN: %u/%u (%04x) OUT: %u/%u (%04x)\n",
                        pulse1in_dc, pulse1in_dt,
                        fracPulse1In,
                        pulse1out_dc, pulse1out_dt,
                        fracPulse1Out
                        );

                flag |= FlagsSensor2::FlagPulsesPerHour;
                }

        *pFlag = uint8_t(flag);
        }

void startSendingUplink(void)
        {
        TxBuffer_t b;
        LedPattern savedLed = gLed.Set(LedPattern::Measuring);

        fillBuffer(b);
        if (savedLed != LedPattern::Joining)
                gLed.Set(LedPattern::Sending);
        else
                gLed.Set(LedPattern::Joining);

        bool fConfirmed = false;
        if (gCatena.GetOperatingFlags() & (1 << 16))
                {
                gCatena.SafePrintf("requesting confirmed tx\n");
                fConfirmed = true;
                }

        gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, NULL, fConfirmed);
        }

static void sendBufferDoneCb(
        void *pContext,
        bool fStatus
        )
        {
        osjobcb_t pFn;

        gLed.Set(LedPattern::Settling);

        pFn = settleDoneCb;
        if (!fStatus)
               {
                if (!gLoRaWAN.IsProvisioned())
                        {
                        // we'll talk about it at the callback.
                        pFn = txNotProvisionedCb;

                        // but prevent an attempt to join.
                        gLoRaWAN.Shutdown();
                        }
                else
                        gCatena.SafePrintf("send buffer failed\n");
                }

        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_SETTLE),
                pFn
                );
        }

static void txNotProvisionedCb(
        osjob_t *pSendJob
        )
        {
        gCatena.SafePrintf("not provisioned, idling\n");
        gLed.Set(LedPattern::NotProvisioned);
        }


//
// the following API is added to delay.c, .h in the BSP. It adjusts millis()
// forward after a deep sleep.
//
// extern "C" { void adjust_millis_forward(unsigned); };
//
// If you don't have it, make sure you're running the MCCI Board Support
// Package for MCCI Catenas, https://github.com/mcci-catena/arduino-boards
//

static void settleDoneCb(
        osjob_t *pSendJob
        )
        {
        bool const fDeepSleepTest = gCatena.GetOperatingFlags() & (1 << 19);
        bool fDeepSleep;

        if (fDeepSleepTest)
                {
                fDeepSleep = true;
                }
#ifdef USBCON
        else if (Serial.dtr())
                {
                fDeepSleep = false;
                }
#endif
      else if (fHasPulse1 || (gCatena.GetOperatingFlags() & (1 << 17)))
                {
                fDeepSleep = false;
                }
        else if ((gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)) != 0)
                {
                fDeepSleep = true;
                }
        else
                {
                fDeepSleep = false;
                }

        if (! g_fPrintedSleeping)
                {
                g_fPrintedSleeping = true;

                if (fDeepSleep)
                        {
                        const uint32_t deepSleepDelay = fDeepSleepTest ? 10 : 30;

                        gCatena.SafePrintf("using deep sleep in %u secs"
#ifdef USBCON
                                           " (USB will disconnect while asleep)"
#endif
                                           ": ",
                                           deepSleepDelay
                                           );

                        // sleep and print
                        gLed.Set(LedPattern::TwoShort);

                        for (auto n = deepSleepDelay; n > 0; --n)
                                {
                                uint32_t tNow = millis();

                                while (uint32_t(millis() - tNow) < 1000)
                                        {
                                        gCatena.poll();
                                        yield();
                                        }
                                gCatena.SafePrintf(".");
                                }
                        gCatena.SafePrintf("\nStarting deep sleep.\n");
                        uint32_t tNow = millis();
                        while (uint32_t(millis() - tNow) < 100)
                                {
                                gCatena.poll();
                                yield();
                                }
                        }
                else
                        gCatena.SafePrintf("using light sleep\n");
                }

        // update the sleep parameters
        if (gTxCycleCount > 1)
                {
                // values greater than one are decremented and ultimately reset to default.
                --gTxCycleCount;
                }
        else if (gTxCycleCount == 1)
                {
                // it's now one (otherwise we couldn't be here.)
                gCatena.SafePrintf("resetting tx cycle to default: %u\n", CATCFG_T_CYCLE);

                gTxCycleCount = 0;
                gTxCycle = CATCFG_T_CYCLE;
                }
        else
                {
                // it's zero. Leave it alone.
                }


        // if connected to USB, don't sleep
        // ditto if we're monitoring pulses.
        if (! fDeepSleep)
                {
                uint32_t interval = sec2osticks(CATCFG_GetInterval(gTxCycle));

                if (gCatena.GetOperatingFlags() & (1 << 18))
                        interval = 1;

                gLed.Set(LedPattern::Sleeping);
                os_setTimedCallback(
                        &sensorJob,
                        os_getTime() + interval,
                        sleepDoneCb
                        );
                return;
                }

        /*
        || ok... now it's time for a deep sleep.  Do the sleep here, since
        || the Arduino loop won't do it. Note that nothing will get polled
        || while we sleep. We can't poll if we're polling power.
        */
        gLed.Set(LedPattern::Off);
#ifdef ARDUINO_ARCH_SAMD
        USBDevice.detach();
#elif defined(ARDUINO_ARCH_STM32)
        Serial.end();
#endif
        Wire.end();
        SPI.end();

        uint32_t const sleepInterval = CATCFG_GetInterval(
                        fDeepSleepTest ? CATCFG_T_CYCLE_TEST : gTxCycle
                        );

        gCatena.Sleep(sleepInterval);

#ifdef ARDUINO_ARCH_SAMD
        USBDevice.attach();
#elif defined(ARDUINO_ARCH_STM32)
        Serial.begin();
#endif
        Wire.begin();
        SPI.begin();

        /* and now... we're awake again. Go to next state. */
        sleepDoneCb(pSendJob);
        }

static void sleepDoneCb(
        osjob_t *pJob
        )
        {
        gLed.Set(LedPattern::WarmingUp);

        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_WARMUP),
                warmupDoneCb
                );
        }

static void warmupDoneCb(
        osjob_t *pJob
        )
        {
        startSendingUplink();
        }

static void receiveMessage(
        void *pContext,
        uint8_t port,
        const uint8_t *pMessage,
        size_t nMessage
        )
        {
        unsigned txCycle;
        unsigned txCount;

        if (port == 0)
                {
                gCatena.SafePrintf("MAC message:");
                for (unsigned i = 0; i < LMIC.dataBeg; ++i)
                        {
                        gCatena.SafePrintf(" %02x", LMIC.frame[i]);
                        }
                gCatena.SafePrintf("\n");

                gCatena.SafePrintf("Transmit power now %d, ADR pow %d, datarate %d\n", LMIC.txpow, LMIC.adrTxPow, LMIC.datarate);

                return;
                }

        enum { kUnknown, kChangeCycle, kRejoin, kReboot } messageType;
        messageType = kUnknown;
        if (port == 1 && 2 <= nMessage && nMessage <= 3)
                messageType = kChangeCycle;
        else if (port == 2 && nMessage == 2)
                messageType = kRejoin;
        else if (port == 3 && nMessage == 2)
                messageType = kReboot;

        if (messageType == kChangeCycle)
                {
                txCycle = (pMessage[0] << 8) | pMessage[1];

                if (txCycle < CATCFG_T_MIN || txCycle > CATCFG_T_MAX)
                        {
                        gCatena.SafePrintf("tx cycle time out of range: %u\n", txCycle);
                        return;
                        }

                // byte [2], if present, is the repeat count.
                // explicitly sending zero causes it to stick.
                txCount = CATCFG_INTERVAL_COUNT;
                if (nMessage >= 3)
                        {
                        txCount = pMessage[2];
                        }

                setTxCycleTime(txCycle, txCount);
                }
        else if (messageType == kRejoin || messageType == kReboot)
                {
                uint32_t joinDelay = (pMessage[0] << 8) || pMessage[1];
                uint32_t tNow = millis();
                uint32_t joinDelayMillis = joinDelay * 1000;

                while (os_queryTimeCriticalJobs(sec2osticks(joinDelay)))
                        {
                        gCatena.poll();
                        }

                while (millis() - tNow < joinDelayMillis)
                        {
                        gCatena.poll();
                        }

                if (messageType == kRejoin)
                        LMIC_unjoinAndRejoin();
                else if (messageType == kReboot)
                        NVIC_SystemReset();
                }
        else /* (messageType == kUnknown) */
                {
                gCatena.SafePrintf("invalid message port(%02x)/length(%x)\n",
                        port, (unsigned) nMessage
                        );
                }
        }

void setTxCycleTime(
        unsigned txCycle,
        unsigned txCount
        )
        {
        if (txCount > 0)
                gCatena.SafePrintf(
                        "message cycle time %u seconds for %u messages\n",
                        txCycle, txCount
                        );
        else
                gCatena.SafePrintf(
                        "message cycle time %u seconds indefinitely\n",
                        txCycle
                        );

        gTxCycle = txCycle;
        gTxCycleCount = txCount;
        }

/* process "application tx-period" -- without args, display, with an arg set the value */
// argv[0] is "tx-period"
// argv[1] if present is the new value
cCommandStream::CommandStatus setTransmitPeriod(
        cCommandStream *pThis,
        void *pContext,
        int argc,
        char **argv
        )
        {
        if (argc > 2)
                return cCommandStream::CommandStatus::kInvalidParameter;

        if (argc < 2)
                {
                pThis->printf("%u s\n", (unsigned) gTxCycle);
                return cCommandStream::CommandStatus::kSuccess;
                }
        else
                {
                // convert argv[1] to unsigned
                cCommandStream::CommandStatus status;
                uint32_t newTxPeriod;

                // get arg 1 as tx period; default is irrelevant because
                // we know that we have argv[1] from above.
                status = cCommandStream::getuint32(argc, argv, 1, /* radix */ 0, newTxPeriod, 0);

                if (status != cCommandStream::CommandStatus::kSuccess)
                        return status;

                gTxCycle = newTxPeriod;
                return status;
                }
        }
