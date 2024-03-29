/*
 Copyright (c) 2020 The Tech Ninja Team (https://ftc9929.com)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

/**
 * Applies a low pass filter to the given RangeInput to remove
 * noise (spikes).
 */
public class LowPassFilteredRangeInput implements RangeInput {
    private final RangeInput rawInput;

    private final float filterFactor;

    private float oldPosition = Float.MIN_VALUE;

    private float maxDeltaPosition = Float.MIN_VALUE;

    public LowPassFilteredRangeInput(RangeInput rawInput, float filterFactor) {
        this.rawInput = rawInput;
        this.filterFactor = filterFactor;
    }

    @Override
    public float getPosition() {
        float rawPosition = rawInput.getPosition();

        if (oldPosition == Float.MIN_VALUE) {
            oldPosition = rawPosition;
            return rawPosition;
        }

        float filteredPosition = (rawPosition - oldPosition) * filterFactor + oldPosition;

        oldPosition = filteredPosition;

        return oldPosition;
    }

    @Override
    public float getMaxPosition() {
        return rawInput.getMaxPosition();
    }

    @Override
    public float getMinPosition() {
        return rawInput.getMinPosition();
    }
}
