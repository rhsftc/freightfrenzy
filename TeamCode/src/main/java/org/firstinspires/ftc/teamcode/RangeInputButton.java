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

/**
 * An adapter that converts a RangeInputButton (commonly a gamepad trigger)
 * into an OnOffButton, that is considered "on" when the input reaches
 * the threshold value provided in the constructor.
 */
public class RangeInputButton implements OnOffButton {
    private final RangeInput input;

    private float threshold;

    private DebouncedButton debounced;

    public RangeInputButton(RangeInput originalInput, float threshold) {
        this.input = originalInput;
        this.threshold = threshold;
    }

    @Override
    public boolean isPressed() {
        if (input.getPosition() >= threshold) {
            return true;
        }

        return false;
    }

    @Override
    public DebouncedButton debounced() {
        if (debounced == null) {
            debounced = new DebouncedButton(this);
        }

        return debounced;
    }
}
