/*
    Authored 2024. Lucius Z Bligh

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    *	Redistributions of source code must retain the above
        copyright notice, this list of conditions and the following
        disclaimer.

    *	Redistributions in binary form must reproduce the above
        copyright notice, this list of conditions and the following
        disclaimer in the documentation and/or other materials
        provided with the distribution.

    *	Neither the name of the author nor the names of its
        contributors may be used to endorse or promote products
        derived from this software without specific prior written
        permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>
#include <math.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "gpio_pins.h"
#include "warp.h"
#include "devMMA8451Q.h"
#include "knock.h"

#define THRESHOLD 1400         // ABS Raw Accel Threshold for Knock
#define KNOCK_DEBOUNCE_MS 80   // Knock Debounce time in ms
#define KNOCKTIMESTDDEV 18.88F // Std Deviation of Knock time diffs from testing
#define SQRT2 1.414213562F     // Sqrt 2 for scaling erf

#define MAXKNOCKS 10         // Maximum number of knocks to listen for.
#define MAX_KNOCK_TIME 10000 // Longest time to wait for a knock before we assume that it's finished.

// Variables.
// Initial setup: "Shave and a Hair Cut, two bits."
uint8_t secretCode[MAXKNOCKS] = {2, 1, 1, 2, 4, 2, 0, 0, 0, 0};
int secretKnockCount = 6;

uint16_t knockReadings[MAXKNOCKS];   // array to store readings
uint16_t scaled_corrects[MAXKNOCKS]; // array to store scaled correct codes for comparison

uint32_t knockSensorValue = 0;

bool programButtonPressed = false;

uint32_t last_knock_time = 0;
int16_t accel[3];

uint16_t roundLuke(double x)
{
    uint16_t intPart = (uint16_t)x;
    intPart += (x - intPart >= 0.5) ? 1 : 0;
    return intPart;
}

#define greenOn() GPIO_DRV_ClearPinOutput(kWarpPinLED_GREEN)
#define greenOff() GPIO_DRV_SetPinOutput(kWarpPinLED_GREEN)
#define redOn() GPIO_DRV_ClearPinOutput(kWarpPinLED_RED)
#define redOff() GPIO_DRV_SetPinOutput(kWarpPinLED_RED)
#define blueOn() GPIO_DRV_ClearPinOutput(kWarpPinLED_BLUE)
#define blueOff() GPIO_DRV_SetPinOutput(kWarpPinLED_BLUE)

void setup_knocking()
{
    // set LED pins as OPs
    PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 11u, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 10u, kPortMuxAsGpio);

    // set button pin as IP
    PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);
    GPIO_HAL_SetPinDir(PORTB_BASE, 0u, kGpioDigitalInput);

    // button state is active low as pulled up - therefore invert to read
    programButtonPressed = !GPIO_DRV_ReadPinInput(progButtonPin);
    warpPrint("\nProgramming: %d\n", programButtonPressed);
}

void knock_loop()
{
    while (1)
    {
        // Get Abs Accel
        getAccelMMA8451Q(&accel[0]);
        knockSensorValue = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

        // check prog button
        programButtonPressed = !GPIO_DRV_ReadPinInput(progButtonPin);
        if (programButtonPressed)
        {
            redOn();
        }
        else
        {
            redOff();
        }

        if (knockSensorValue >= THRESHOLD && OSA_TimeGetMsec() - last_knock_time > KNOCK_DEBOUNCE_MS)
        {
            last_knock_time = OSA_TimeGetMsec();
            listenToSecretKnock();
        }
    }
}

// Records the timing of knocks.
void listenToSecretKnock()
{
    warpPrint("New sequence starting at %dms\n", last_knock_time);
    warpPrint((programButtonPressed) ? "Programming\n" : "Checking\n");

    // reset array
    for (int i = 0; i < MAXKNOCKS; i++)
    {
        knockReadings[i] = 0;
    }

    uint8_t currentKnockNumber = 0;       
    uint32_t startTime = last_knock_time; // Reference for when this knock started.
    uint32_t now = last_knock_time; // time so can check if 10s has elapsed without affecting debounce

    // blink LED once per knock - green if checking, yellow if prog
    greenOn();
    if (programButtonPressed)
    {
        redOn();
    }
    OSA_TimeDelay(10);
    greenOff();
    if (programButtonPressed)
    {
        redOff();
    }

    // loop till max time or max knocks reached
    do
    {
        // get accel
        getAccelMMA8451Q(&accel[0]);
        knockSensorValue = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

        if (knockSensorValue >= THRESHOLD && OSA_TimeGetMsec() - last_knock_time > KNOCK_DEBOUNCE_MS)
        {
            now = OSA_TimeGetMsec();

            knockReadings[currentKnockNumber] = now - last_knock_time;
            warpPrint("%d,", knockReadings[currentKnockNumber]);
            currentKnockNumber++;

            last_knock_time = now;

            // blink LED once per knock - green if checking, yellow if prog
            greenOn();
            if (programButtonPressed)
            {
                redOn();
            }
            OSA_TimeDelay(10);
            greenOff();
            if (programButtonPressed)
            {
                redOff();
            }
        }

        now = OSA_TimeGetMsec();

    } while ((now - startTime < MAX_KNOCK_TIME) && (currentKnockNumber < MAXKNOCKS)); 
    warpPrint("\b \n");

    // now check:
    if (!programButtonPressed)
    { 
        uint8_t likelihood_correct = validateKnock();
        warpPrint("P(Correct Sequence) = %d%%\n", likelihood_correct);

        if (likelihood_correct > 50)
        {
            warpPrint("Correct Sequence Entered!\n");
            for (int i = 0; i < 4; i++)
            {
                blueOn();
                OSA_TimeDelay(100);
                blueOff();
                OSA_TimeDelay(100);
            }
        }
        else // likelihood under 50%
        {
            warpPrint("Secret knock failed.\n");
            // greenOff(); // We didn't unlock, so blink the red LED as visual feedback.
            for (int i = 0; i < 4; i++)
            {
                redOn();
                OSA_TimeDelay(100);
                redOff();
                OSA_TimeDelay(100);
            }
        }
    }
    else
    { 
        // still check knock even when programming
        validateKnock();

        // blink the green and red alternately to show that programming is complete.
        warpPrint("New lock stored.\n");
        for (int i = 0; i < MAXKNOCKS; i++)
        {
            warpPrint("%d, ", secretCode[i]);
        }
        warpPrint("\n");
        redOff();
        greenOn();
        for (int i = 0; i < 3; i++)
        {
            OSA_TimeDelay(200);
            redOn();
            greenOff();
            OSA_TimeDelay(200);
            redOff();
            greenOn();
        }
        redOff();
        greenOff();
    }
}

// returns percentage liklihood it's a good knock.
uint8_t validateKnock()
{
    int i = 0;

    // simplest check first: Did we get the right number of knocks?
    int currentKnockCount = 0;
    int minKnockInterval = MAX_KNOCK_TIME; // We use this later to normalize the times.

    for (i = 0; i < MAXKNOCKS; i++)
    {
        if (knockReadings[i] > 0)
        {
            currentKnockCount++;
        }

        if (knockReadings[i] < minKnockInterval && knockReadings[i] != 0)
        { // collect normalization data while we're looping.
            minKnockInterval = knockReadings[i];
        }
    }

    if (programButtonPressed)
    {
        secretKnockCount = currentKnockCount;
        for (i = 0; i < MAXKNOCKS; i++)
        {
            // store normalised secret code.
            secretCode[i] = roundLuke((double)knockReadings[i] / (double)minKnockInterval);
        }

        // flash pattern in yellow
        redOff();
        greenOff();
        OSA_TimeDelay(1000);

        redOn(); // first knock is before the array - array stores gaps
        greenOn();
        OSA_TimeDelay(50);
        redOff();
        greenOff();

        for (i = 0; i < secretKnockCount; i++)
        {
            OSA_TimeDelay(secretCode[i] * 150); // Expand the time back out to what it was.  Roughly.
            redOn();
            greenOn();
            OSA_TimeDelay(50);
            redOff();
            greenOff();
        }
        return 0; // doesnt really matter cos called from a different path
    }

    if (currentKnockCount != secretKnockCount)
    {
        // 0% prob of correct if wrong number of knocks
        warpPrint("BAD NUMBER OF KNOCKS\n");
        return 0;
    }


    int timeDiff = 0;

    // do least squares to get LSE tempo
    // solving Ax = b for correct = a, measured = b
    // x = (A.T*A)^-1 * A.T * b
    double num = 0;
    double temp;
    int sum = 0;

    for (int i = 0; i < MAXKNOCKS; i++)
        num += secretCode[i] * secretCode[i]; // A.T * A

    num = 1 / num; // (A.T*A)^-1

    for (int i = 0; i < MAXKNOCKS; i++)
        sum += secretCode[i] * knockReadings[i]; // A.T * b

    sum = sum * num; //(A.T*A)^-1 * A.T * b
    num = 1;

    sum = (sum + knockReadings[0] / secretCode[0]) / 2; // weight first time difference higher

    int secretTime_ms;
    for (i = 0; i < max(secretKnockCount, currentKnockCount); i++)
    {
        secretTime_ms = secretCode[i] * sum;
        timeDiff = abs(knockReadings[i] - secretTime_ms);
        temp = erf((double)timeDiff / (KNOCKTIMESTDDEV * SQRT2));
        warpPrint("knock %d: time: %d, secretTime %d, timeDiff: %d, ind: ", i, knockReadings[i], secretTime_ms, timeDiff);
        floatPrint(temp + 0.0001);
        num *= (temp + 0.0001);
        warpPrint(", num: ");
        floatPrint(num);
        warpPrint("\n");
    }

    warpPrint("prob: ");
    floatPrint(1 - ((num>=1)?1:num));
    warpPrint("\n");

    int percent = (1 -  ((num>=1)?1:num)) * 100;
    return percent;
}
