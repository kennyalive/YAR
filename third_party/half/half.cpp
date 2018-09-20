///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2002, Industrial Light & Magic, a division of Lucas
// Digital Ltd. LLC
// 
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// *       Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// *       Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// *       Neither the name of Industrial Light & Magic nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
//
// Implementation --
//
// Representation of a float:
//
//	We assume that a float, f, is an IEEE 754 single-precision
//	floating point number, whose bits are arranged as follows:
//
//	    31 (msb)
//	    | 
//	    | 30     23
//	    | |      | 
//	    | |      | 22                    0 (lsb)
//	    | |      | |                     |
//	    X XXXXXXXX XXXXXXXXXXXXXXXXXXXXXXX
//
//	    s e        m
//
//	S is the sign-bit, e is the exponent and m is the significand.
//
//	If e is between 1 and 254, f is a normalized number:
//
//	            s    e-127
//	    f = (-1)  * 2      * 1.m
//
//	If e is 0, and m is not zero, f is a denormalized number:
//
//	            s    -126
//	    f = (-1)  * 2      * 0.m
//
//	If e and m are both zero, f is zero:
//
//	    f = 0.0
//
//	If e is 255, f is an "infinity" or "not a number" (NAN),
//	depending on whether m is zero or not.
//
//	Examples:
//
//	    0 00000000 00000000000000000000000 = 0.0
//	    0 01111110 00000000000000000000000 = 0.5
//	    0 01111111 00000000000000000000000 = 1.0
//	    0 10000000 00000000000000000000000 = 2.0
//	    0 10000000 10000000000000000000000 = 3.0
//	    1 10000101 11110000010000000000000 = -124.0625
//	    0 11111111 00000000000000000000000 = +infinity
//	    1 11111111 00000000000000000000000 = -infinity
//	    0 11111111 10000000000000000000000 = NAN
//	    1 11111111 11111111111111111111111 = NAN
//
// Representation of a half:
//
//	Here is the bit-layout for a half number, h:
//
//	    15 (msb)
//	    | 
//	    | 14  10
//	    | |   |
//	    | |   | 9        0 (lsb)
//	    | |   | |        |
//	    X XXXXX XXXXXXXXXX
//
//	    s e     m
//
//	S is the sign-bit, e is the exponent and m is the significand.
//
//	If e is between 1 and 30, h is a normalized number:
//
//	            s    e-15
//	    h = (-1)  * 2     * 1.m
//
//	If e is 0, and m is not zero, h is a denormalized number:
//
//	            S    -14
//	    h = (-1)  * 2     * 0.m
//
//	If e and m are both zero, h is zero:
//
//	    h = 0.0
//
//	If e is 31, h is an "infinity" or "not a number" (NAN),
//	depending on whether m is zero or not.
//
//	Examples:
//
//	    0 00000 0000000000 = 0.0
//	    0 01110 0000000000 = 0.5
//	    0 01111 0000000000 = 1.0
//	    0 10000 0000000000 = 2.0
//	    0 10000 1000000000 = 3.0
//	    1 10101 1111000001 = -124.0625
//	    0 11111 0000000000 = +infinity
//	    1 11111 0000000000 = -infinity
//	    0 11111 1000000000 = NAN
//	    1 11111 1111111111 = NAN
//
// Conversion:
//
//	Converting from a float to a half requires some non-trivial bit
//	manipulations.  In some cases, this makes conversion relatively
//	slow, but the most common case is accelerated via table lookups.
//
//	Converting back from a half to a float is easier because we don't
//	have to do any rounding.  In addition, there are only 65536
//	different half numbers; we can convert each of those numbers once
//	and store the results in a table.  Later, all conversions can be
//	done using only simple table lookups.
//
//---------------------------------------------------------------------------


static const unsigned short   eLut[1 << 9] = 
{
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,  1024,  2048,  3072,  4096,  5120,  6144,  7168, 
     8192,  9216, 10240, 11264, 12288, 13312, 14336, 15360, 
    16384, 17408, 18432, 19456, 20480, 21504, 22528, 23552, 
    24576, 25600, 26624, 27648, 28672, 29696,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0, 33792, 34816, 35840, 36864, 37888, 38912, 39936, 
    40960, 41984, 43008, 44032, 45056, 46080, 47104, 48128, 
    49152, 50176, 51200, 52224, 53248, 54272, 55296, 56320, 
    57344, 58368, 59392, 60416, 61440, 62464,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
        0,     0,     0,     0,     0,     0,     0,     0, 
};

//-----------------------------------------------
// Overflow handler for float-to-half conversion;
// generates a hardware floating-point overflow,
// which may be trapped by the operating system.
//-----------------------------------------------

static float overflow() {
    volatile float f = 1e10;

    for (int i = 0; i < 10; i++)
        f *= f;				// this will overflow before
                        // the for­loop terminates
    return f;
}

//-----------------------------------------------------
// Float-to-half conversion -- general case, including
// zeroes, denormalized numbers and exponent overflows.
//-----------------------------------------------------

static short convert(int i) {
    //
    // Our floating point number, f, is represented by the bit
    // pattern in integer i.  Disassemble that bit pattern into
    // the sign, s, the exponent, e, and the significand, m.
    // Shift s into the position where it will go in in the
    // resulting half number.
    // Adjust e, accounting for the different exponent bias
    // of float and half (127 versus 15).
    //

    int s = (i >> 16) & 0x00008000;
    int e = ((i >> 23) & 0x000000ff) - (127 - 15);
    int m = i & 0x007fffff;

    //
    // Now reassemble s, e and m into a half:
    //

    if (e <= 0) {
        if (e < -10) {
            //
            // E is less than -10.  The absolute value of f is
            // less than HALF_MIN (f may be a small normalized
            // float, a denormalized float or a zero).
            //
            // We convert f to a half zero with the same sign as f.
            //

            return s;
        }

        //
        // E is between -10 and 0.  F is a normalized float
        // whose magnitude is less than HALF_NRM_MIN.
        //
        // We convert f to a denormalized half.
        //

        //
        // Add an explicit leading 1 to the significand.
        // 

        m = m | 0x00800000;

        //
        // Round to m to the nearest (10+e)-bit value (with e between
        // -10 and 0); in case of a tie, round to the nearest even value.
        //
        // Rounding may cause the significand to overflow and make
        // our number normalized.  Because of the way a half's bits
        // are laid out, we don't have to treat this case separately;
        // the code below will handle it correctly.
        // 

        int t = 14 - e;
        int a = (1 << (t - 1)) - 1;
        int b = (m >> t) & 1;

        m = (m + a + b) >> t;

        //
        // Assemble the half from s, e (zero) and m.
        //

        return s | m;
    }
    else if (e == 0xff - (127 - 15)) {
        if (m == 0) {
            //
            // F is an infinity; convert f to a half
            // infinity with the same sign as f.
            //

            return s | 0x7c00;
        }
        else {
            //
            // F is a NAN; we produce a half NAN that preserves
            // the sign bit and the 10 leftmost bits of the
            // significand of f, with one exception: If the 10
            // leftmost bits are all zero, the NAN would turn 
            // into an infinity, so we have to set at least one
            // bit in the significand.
            //

            m >>= 13;
            return s | 0x7c00 | m | (m == 0);
        }
    }
    else {
        //
        // E is greater than zero.  F is a normalized float.
        // We try to convert f to a normalized half.
        //

        //
        // Round to m to the nearest 10-bit value.  In case of
        // a tie, round to the nearest even value.
        //

        m = m + 0x00000fff + ((m >> 13) & 1);

        if (m & 0x00800000) {
            m = 0;		// overflow in significand,
            e += 1;		// adjust exponent
        }

        //
        // Handle exponent overflow
        //

        if (e > 30) {
            overflow();	// Cause a hardware floating point overflow;
            return s | 0x7c00;	// if this returns, the half becomes an
        }   			// infinity with the same sign as f.

        //
        // Assemble the half from s, e and m.
        //

        return s | (e << 10) | (m >> 13);
    }
}

unsigned short float_to_half(float f) {
    union uif {
        unsigned int i;
        float        f;
    };

    uif x;
    unsigned short h;


    x.f = f;

    if (f == 0) {
        //
        // Common special case - zero.
        // Preserve the zero's sign bit.
        //

        h = (x.i >> 16);
    }
    else {
        //
        // We extract the combined sign and exponent, e, from our
        // floating-point number, f.  Then we convert e to the sign
        // and exponent of the half number via a table lookup.
        //
        // For the most common case, where a normalized half is produced,
        // the table lookup returns a non-zero value; in this case, all
        // we have to do is round f's significand to 10 bits and combine
        // the result with e.
        //
        // For all other cases (overflow, zeroes, denormalized numbers
        // resulting from underflow, infinities and NANs), the table
        // lookup returns zero, and we call a longer, non-inline function
        // to do the float-to-half conversion.
        //

        int e = (x.i >> 23) & 0x000001ff;

        e = eLut[e];

        if (e) {
            //
            // Simple case - round the significand, m, to 10
            // bits and combine it with the sign and exponent.
            //

            int m = x.i & 0x007fffff;
            h = e + ((m + 0x00000fff + ((m >> 13) & 1)) >> 13);
        }
        else {
            //
            // Difficult case - call a function.
            //

            h = convert(x.i);
        }
    }

    return h;
}
