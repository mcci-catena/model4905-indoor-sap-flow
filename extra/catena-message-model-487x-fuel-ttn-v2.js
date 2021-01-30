// This function decodes the records (port 1, format 0x11, 0x14, 0x15, 0x16, 0x17)
// sent by the MCCI Catena 4410/4450/4551 soil/water and power applications.
// For use with console.thethingsnetwork.org
// 2017-09-19 add dewpoints.
// 2017-12-13 fix commments, fix negative soil/water temp, add test vectors.
// 2017-12-15 add format 0x11.
// 2018-04-24 add format 0x16.
// 2018-04-01 add format 0x17.
// 2018-06-13 add air quality.
// 2019-02-13 add simple sensor format (port 2).
// 2019-06-28 add port 3 (no barometric pressure)

// calculate dewpoint (degrees C) given temperature (C) and relative humidity (0..100)
// from http://andrew.rsmas.miami.edu/bmcnoldy/Humidity.html
// rearranged for efficiency and to deal sanely with very low (< 1%) RH
function dewpoint(t, rh) {
    var c1 = 243.04;
    var c2 = 17.625;
    var h = rh / 100;
    if (h <= 0.01)
        h = 0.01;
    else if (h > 1.0)
        h = 1.0;

    var lnh = Math.log(h);
    var tpc1 = t + c1;
    var txc2 = t * c2;
    var txc2_tpc1 = txc2 / tpc1;

    var tdew = c1 * (lnh + txc2_tpc1) / (c2 - lnh - txc2_tpc1);
    return tdew;
}

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};

    if (port === 1) {
        cmd = bytes[0];
        if (cmd == 0x14) {
            // decode Catena 4450 M101 pulse data

            // test vectors:
            //  14 01 18 00 ==> Vbat = 1.5
            //  14 01 F8 00 ==> Vbat = -0.5
            //  14 05 F8 00 42 ==> boot: 66, Vbat: -0.5
            //  14 0D F8 00 42 17 80 59 35 80 ==> adds one temp of 23.5, rh = 50, p = 913.48

            // i is used as the index into the message. Start with the flag byte.
            var i = 1;
            // fetch the bitmap.
            var flags = bytes[i++];

            if (flags & 0x1) {
                // set Vraw to a uint16, and increment pointer
                var Vraw = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                // interpret uint16 as an int16 instead.
                if (Vraw & 0x8000)
                    Vraw += -0x10000;
                // scale and save in decoded.
                decoded.Vbat = Vraw / 4096.0;
            }

            if (flags & 0x2) {
                var VrawBus = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                if (VrawBus & 0x8000)
                    VrawBus += -0x10000;
                decoded.Vbus = VrawBus / 4096.0;
            }

            if (flags & 0x4) {
                var iBoot = bytes[i];
                i += 1;
                decoded.boot = iBoot;
            }

            if (flags & 0x8) {
                // we have temp, pressure, RH
                var tRaw = (bytes[i] << 8) + bytes[i + 1];
                if (tRaw & 0x8000)
                    tRaw = -0x10000 + tRaw;
                i += 2;
                var pRaw = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                var hRaw = bytes[i++];

                decoded.tempC = tRaw / 256;
                decoded.p = pRaw * 4 / 100.0;
                decoded.rh = hRaw / 256 * 100;
                decoded.tDewC = dewpoint(decoded.tempC, decoded.rh);
            }

            if (flags & 0x10) {
                // we have lux
                var lightRaw = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                var irradiance = {};
                decoded.irradiance = irradiance;
                irradiance.White = lightRaw;
            }

            if (flags & 0x20) {
                // fuel, spare
                var pulseIn = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                var pulseOut = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                decoded.fuelUsedCount = pulseIn;
                decoded.fuelUsedLiters = pulseIn * 0.46e-3;
            }

            if (flags & 0x40) {
                // normalize floating pulses per hour
                var floatIn = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                var floatOut = (bytes[i] << 8) + bytes[i + 1];
                i += 2;

                var exp1 = floatIn >> 12;
                var mant1 = (floatIn & 0xFFF) / 4096.0;
                var pulsePerHourIn = mant1 * Math.pow(2, exp1 - 15) * 60 * 60 * 4;
                decoded.fuelUsedPulsesPerHour = pulsePerHourIn;
                decoded.fuelUsedLitersPerHour = pulsePerHourIn * 0.46e-3;
            }
        } else {
            // cmd value not recognized.
        }
    }

    // at this point, decoded has the real values.
    return decoded;
}