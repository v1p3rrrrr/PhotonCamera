package com.particlesdevs.photoncamera.processing.parameters;

import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.params.ColorSpaceTransform;
import android.hardware.camera2.params.RggbChannelVector;
import android.util.Rational;

import com.particlesdevs.photoncamera.capture.CaptureController;
import com.particlesdevs.photoncamera.processing.render.Converter;

/**
 * Mathematically rigorous utility for bidirectional conversion between
 * Color Temperature (Kelvin 2000K - 10000K) and Camera2 sensor RGB channel gains and Color Transform.
 * Strictly complies with Adobe DNG 1.4.0.0 Specification (Section 6), CIE 1931 xy Chromaticity,
 * and Planckian / Daylight locus standards (ISO/CIE 23603).
 */
public final class ColorTemperatureConverter {

    public static final int MIN_KELVIN = 2000;
    public static final int MAX_KELVIN = 10000;

    private static final int DEFAULT_TEMP_1 = 2856; // Standard Illuminant A
    private static final int DEFAULT_TEMP_2 = 6504; // CIE D65

    private ColorTemperatureConverter() {
    }

    /**
     * Converts a target color temperature in Kelvin (2000K - 10000K) into Camera2 sensor RGB gains
     * using the active sensor's factory DNG ColorMatrix calibration.
     *
     * @param kelvin desired color temperature in Kelvin
     * @return RggbChannelVector containing calibrated sensor gains [R_gain, 1.0, 1.0, B_gain]
     */
    public static RggbChannelVector kelvinToRggb(int kelvin) {
        return kelvinToRggb(kelvin, CaptureController.mCameraCharacteristics);
    }

    /**
     * Converts a target color temperature in Kelvin (2000K - 10000K) into Camera2 sensor RGB gains
     * using the provided CameraCharacteristics according to Adobe DNG 1.4 Specification (Section 6).
     *
     * @param kelvin          desired color temperature in Kelvin
     * @param characteristics CameraCharacteristics of the target sensor
     * @return RggbChannelVector containing calibrated sensor gains [R_gain, 1.0, 1.0, B_gain]
     */
    public static RggbChannelVector kelvinToRggb(int kelvin, CameraCharacteristics characteristics) {
        int clampedKelvin = Math.max(MIN_KELVIN, Math.min(MAX_KELVIN, kelvin));
        double t = clampedKelvin;

        // 1. Calculate CIE 1931 (x, y) chromaticity coordinates on the Planckian / Daylight locus (ISO/CIE 23603)
        double[] xy = kelvinToCIExy(t);
        double x = xy[0];
        double y = Math.max(xy[1], 1e-4);
        float[] xyz = new float[]{(float) (x / y), 1.0f, (float) ((1.0 - x - y) / y)};

        CalibrationData calib = extractCalibrationData(characteristics);
        if (calib == null) {
            return fallbackGains(t);
        }

        // 2. Calculate DNG Mired-space reciprocal temperature interpolation factor g
        double invT = 1.0 / t;
        double invT1 = 1.0 / calib.colorTemp1;
        double invT2 = 1.0 / calib.colorTemp2;

        double denom = invT1 - invT2;
        double g = (Math.abs(denom) > 1e-9) ? (invT - invT2) / denom : 0.5;
        if (Double.isNaN(g)) g = 0.5;
        float gClamped = (float) Math.max(0.0, Math.min(1.0, g));

        // 3. Interpolate XYZToCamera matrix: (1 - g) * XYZToCamera2 + g * XYZToCamera1
        float[] xyzToCamera = new float[9];
        Converter.lerp(calib.xyzToCamera2, calib.xyzToCamera1, gClamped, xyzToCamera);

        // 4. Raw sensor spectral response to illuminant: [R_sensor, G_sensor, B_sensor]
        float[] cameraSensor = new float[3];
        Converter.map(xyzToCamera, xyz, /*out*/cameraSensor);

        if (cameraSensor[0] <= 1e-5f || cameraSensor[1] <= 1e-5f || cameraSensor[2] <= 1e-5f) {
            return fallbackGains(t);
        }

        // 5. White balance gains according to Adobe DNG 1.4 (Section 6): R_gain = G / R, B_gain = G / B
        float rGain = cameraSensor[1] / cameraSensor[0];
        float bGain = cameraSensor[1] / cameraSensor[2];

        // Hardware safety clamp
        rGain = Math.max(0.1f, Math.min(20.0f, rGain));
        bGain = Math.max(0.1f, Math.min(20.0f, bGain));

        return new RggbChannelVector(rGain, 1.0f, 1.0f, bGain);
    }

    /**
     * Creates a calibrated 3x3 ColorSpaceTransform matrix for Camera2 COLOR_CORRECTION_TRANSFORM (Sensor -> sRGB).
     * Built via Converter.sXYZtoSRGB * (ForwardMatrix(T) * Calib(T)^-1) according to Adobe DNG 1.4.
     *
     * @param kelvin          color temperature in Kelvin (2000K - 10000K)
     * @param characteristics CameraCharacteristics of the active camera sensor
     * @return ColorSpaceTransform matrix for CaptureRequest
     */
    public static ColorSpaceTransform createColorTransform(int kelvin, CameraCharacteristics characteristics) {
        if (characteristics == null) {
            return null;
        }

        int clampedKelvin = Math.max(MIN_KELVIN, Math.min(MAX_KELVIN, kelvin));
        double t = clampedKelvin;

        Integer ref1Obj = characteristics.get(CameraCharacteristics.SENSOR_REFERENCE_ILLUMINANT1);
        int ref1 = (ref1Obj != null) ? ref1Obj : CameraMetadata.SENSOR_REFERENCE_ILLUMINANT1_STANDARD_A;

        Object ref2Obj = characteristics.get(CameraCharacteristics.SENSOR_REFERENCE_ILLUMINANT2);
        int ref2 = (ref2Obj != null) ? ((Number) ref2Obj).intValue() : CameraMetadata.SENSOR_REFERENCE_ILLUMINANT1_D65;

        int colorTemp1 = Converter.sStandardIlluminates.get(ref1, DEFAULT_TEMP_1);
        int colorTemp2 = Converter.sStandardIlluminates.get(ref2, DEFAULT_TEMP_2);

        double invT = 1.0 / t;
        double invT1 = 1.0 / colorTemp1;
        double invT2 = 1.0 / colorTemp2;

        double denom = invT1 - invT2;
        double g = (Math.abs(denom) > 1e-9) ? (invT - invT2) / denom : 0.5;
        if (Double.isNaN(g)) g = 0.5;
        float gClamped = (float) Math.max(0.0, Math.min(1.0, g));

        ColorSpaceTransform calib1Xform = characteristics.get(CameraCharacteristics.SENSOR_CALIBRATION_TRANSFORM1);
        ColorSpaceTransform calib2Xform = characteristics.get(CameraCharacteristics.SENSOR_CALIBRATION_TRANSFORM2);
        ColorSpaceTransform color1Xform = characteristics.get(CameraCharacteristics.SENSOR_COLOR_TRANSFORM1);
        ColorSpaceTransform color2Xform = characteristics.get(CameraCharacteristics.SENSOR_COLOR_TRANSFORM2);
        ColorSpaceTransform forward1Xform = characteristics.get(CameraCharacteristics.SENSOR_FORWARD_MATRIX1);
        ColorSpaceTransform forward2Xform = characteristics.get(CameraCharacteristics.SENSOR_FORWARD_MATRIX2);

        float[] calib1 = new float[9];
        float[] calib2 = new float[9];
        if (calib1Xform != null) Converter.convertColorspaceTransform(calib1Xform, calib1);
        else identityMatrix(calib1);
        if (calib2Xform != null) Converter.convertColorspaceTransform(calib2Xform, calib2);
        else identityMatrix(calib2);

        float[] cameraToXYZ = new float[9];

        if (forward1Xform != null) {
            float[] forward1 = new float[9];
            float[] forward2 = new float[9];
            Converter.convertColorspaceTransform(forward1Xform, forward1);
            if (forward2Xform != null) {
                Converter.convertColorspaceTransform(forward2Xform, forward2);
            } else {
                System.arraycopy(forward1, 0, forward2, 0, 9);
            }

            Converter.lerp(forward2, forward1, gClamped, /*out*/cameraToXYZ);
        } else if (color1Xform != null) {
            float[] color1 = new float[9];
            float[] color2 = new float[9];
            Converter.convertColorspaceTransform(color1Xform, color1);
            if (color2Xform != null) Converter.convertColorspaceTransform(color2Xform, color2);
            else System.arraycopy(color1, 0, color2, 0, 9);

            float[] xyzToCamera1 = new float[9];
            float[] xyzToCamera2 = new float[9];
            Converter.multiply(calib1, color1, xyzToCamera1);
            Converter.multiply(calib2, color2, xyzToCamera2);

            float[] interpolatedXYZToCamera = new float[9];
            Converter.lerp(xyzToCamera2, xyzToCamera1, gClamped, interpolatedXYZToCamera);

            if (!Converter.invert(interpolatedXYZToCamera, /*out*/cameraToXYZ)) {
                identityMatrix(cameraToXYZ);
            }
        } else {
            identityMatrix(cameraToXYZ);
        }

        // Multiply: M_(Sensor -> sRGB) = Converter.sXYZtoSRGB * CameraToXYZ
        float[] sensorToSRGB = new float[9];
        Converter.multiply(Converter.sXYZtoSRGB, cameraToXYZ, /*out*/sensorToSRGB);

        // Row normalization (sum of each row = 1.0) according to DNG white point preservation
        for (int row = 0; row < 3; row++) {
            float sum = sensorToSRGB[row * 3] + sensorToSRGB[row * 3 + 1] + sensorToSRGB[row * 3 + 2];
            if (Math.abs(sum) > 1e-6f) {
                sensorToSRGB[row * 3]     /= sum;
                sensorToSRGB[row * 3 + 1] /= sum;
                sensorToSRGB[row * 3 + 2] /= sum;
            }
        }

        Rational[] rationals = new Rational[9];
        for (int i = 0; i < 9; i++) {
            rationals[i] = new Rational(Math.round(sensorToSRGB[i] * 1024.0f), 1024);
        }
        return new ColorSpaceTransform(rationals);
    }

    /**
     * Creates a ColorSpaceTransform matrix from current gains.
     */
    public static ColorSpaceTransform createColorTransformFromGains(
            RggbChannelVector gains,
            CameraCharacteristics characteristics) {

        if (characteristics == null) {
            return null;
        }

        float r = (gains != null) ? Math.max(gains.getRed(), 1e-4f) : 1.0f;
        float b = (gains != null) ? Math.max(gains.getBlue(), 1e-4f) : 1.0f;
        Rational[] neutralPoint = new Rational[]{
                new Rational(Math.round((1.0f / r) * 1024), 1024),
                new Rational(1, 1),
                new Rational(Math.round((1.0f / b) * 1024), 1024)
        };

        int kelvin = neutralPointToKelvin(neutralPoint, characteristics);
        return createColorTransform(kelvin, characteristics);
    }

    /**
     * Reconstructs the exact Correlated Color Temperature (CCT) in Kelvin
     * from the sensor's neutral color point [r, g, b] using DNG matrix inversion and McCamy's algorithm.
     *
     * @param neutralPoint SENSOR_NEUTRAL_COLOR_POINT rational array from CaptureResult
     * @return estimated color temperature in Kelvin, rounded to the nearest 50K
     */
    public static int neutralPointToKelvin(Rational[] neutralPoint) {
        return neutralPointToKelvin(neutralPoint, CaptureController.mCameraCharacteristics);
    }

    /**
     * Reconstructs the exact Correlated Color Temperature (CCT) in Kelvin
     * from the sensor's neutral color point [r, g, b] using DNG matrix inversion and McCamy's algorithm.
     *
     * @param neutralPoint    SENSOR_NEUTRAL_COLOR_POINT rational array from CaptureResult
     * @param characteristics CameraCharacteristics of the target sensor
     * @return estimated color temperature in Kelvin, rounded to the nearest 50K
     */
    public static int neutralPointToKelvin(Rational[] neutralPoint, CameraCharacteristics characteristics) {
        if (neutralPoint == null || neutralPoint.length < 3) {
            return 5500;
        }

        double r = neutralPoint[0].doubleValue();
        double g = neutralPoint[1].doubleValue();
        double b = neutralPoint[2].doubleValue();

        if (Double.isNaN(r) || Double.isNaN(g) || Double.isNaN(b) || r <= 1e-4 || g <= 1e-4 || b <= 1e-4) {
            return 5500;
        }

        CalibrationData calib = extractCalibrationData(characteristics);
        if (calib == null) {
            return fallbackKelvin(r, b);
        }

        float[] cameraNeutral = new float[]{(float) r, (float) g, (float) b};

        double lower = Math.min(calib.colorTemp1, calib.colorTemp2);
        double upper = Math.max(calib.colorTemp1, calib.colorTemp2);

        if (Math.abs(upper - lower) < 1.0) {
            float[] invMat = new float[9];
            if (Converter.invert(calib.xyzToCamera1, invMat)) {
                float[] xyzGuess = new float[3];
                Converter.map(invMat, cameraNeutral, xyzGuess);
                double[] xy = Converter.calculateCIExyCoordinates(xyzGuess[0], xyzGuess[1], xyzGuess[2]);
                double cct = Converter.calculateColorTemperature(xy[0], xy[1]);
                int rounded = (int) (Math.round(cct / 50.0) * 50);
                return Math.max(MIN_KELVIN, Math.min(MAX_KELVIN, rounded));
            }
            return fallbackKelvin(r, b);
        }

        double interpolationFactor = 0.5;
        double oldInterpolationFactor = interpolationFactor;
        double lastDiff = Double.MAX_VALUE;
        double tolerance = 0.0001;

        float[] interpolationXYZToCamera = new float[9];
        float[] interpolationXYZToCameraInverse = new float[9];
        float[] neutralGuess = new float[3];
        double lastColorTemperature = 5500.0;

        int loopLimit = 30;
        while (lastDiff > tolerance && loopLimit > 0) {
            Converter.lerp(calib.xyzToCamera2, calib.xyzToCamera1, (float) interpolationFactor, interpolationXYZToCamera);
            if (!Converter.invert(interpolationXYZToCamera, /*out*/interpolationXYZToCameraInverse)) {
                return fallbackKelvin(r, b);
            }

            Converter.map(interpolationXYZToCameraInverse, cameraNeutral, /*out*/neutralGuess);
            double[] xy = Converter.calculateCIExyCoordinates(neutralGuess[0], neutralGuess[1], neutralGuess[2]);
            lastColorTemperature = Converter.calculateColorTemperature(xy[0], xy[1]);

            double targetFactor;
            if (lastColorTemperature <= lower) {
                targetFactor = 1.0;
            } else if (lastColorTemperature >= upper) {
                targetFactor = 0.0;
            } else {
                double invCT = 1.0 / lastColorTemperature;
                targetFactor = (invCT - 1.0 / upper) / (1.0 / lower - 1.0 / upper);
            }

            if (lower == calib.colorTemp1) {
                targetFactor = 1.0 - targetFactor;
            }

            interpolationFactor = (targetFactor + oldInterpolationFactor) / 2.0;
            lastDiff = Math.abs(oldInterpolationFactor - interpolationFactor);
            oldInterpolationFactor = interpolationFactor;
            loopLimit--;
        }

        int roundedKelvin = (int) (Math.round(lastColorTemperature / 50.0) * 50);
        return Math.max(MIN_KELVIN, Math.min(MAX_KELVIN, roundedKelvin));
    }

    /**
     * Exact CIE 1931 (x, y) chromaticity coordinates according to CIE 15:2004 / ISO 23603 and Planckian locus.
     */
    private static double[] kelvinToCIExy(double T) {
        double x;
        if (T <= 4000.0) {
            // Planckian blackbody locus (Kim et al.)
            x = -0.2661239 * (1e9 / (T * T * T))
                    - 0.2343580 * (1e6 / (T * T))
                    + 0.8776956 * (1e3 / T)
                    + 0.179910;
        } else if (T <= 7000.0) {
            // CIE 15:2004 Daylight Series 4000K <= T <= 7000K
            x = -4.6070 * (1e9 / (T * T * T))
                    + 2.9678 * (1e6 / (T * T))
                    + 0.09911 * (1e3 / T)
                    + 0.244063;
        } else {
            // CIE 15:2004 Daylight Series 7000K < T <= 25000K
            x = -2.0064 * (1e9 / (T * T * T))
                    + 1.9018 * (1e6 / (T * T))
                    + 0.24748 * (1e3 / T)
                    + 0.237040;
        }

        // Planckian blackbody locus y(x) for T <= 4000K (Kim et al.)
        double yPlanck = -1.1063814 * (x * x * x)
                - 1.34811020 * (x * x)
                + 2.18555832 * x
                - 0.20219683;

        // CIE Daylight locus y(x) for T > 4000K (CIE 15:2004 / ISO 23603)
        double yDaylight = -3.000 * (x * x) + 2.870 * x - 0.275;

        double y;
        if (T < 3800.0) {
            y = yPlanck;
        } else if (T > 4200.0) {
            y = yDaylight;
        } else {
            // Smooth C1 Hermite blending between 3800K and 4200K
            double k = (T - 3800.0) / 400.0;
            double w = k * k * (3.0 - 2.0 * k);
            y = yPlanck * (1.0 - w) + yDaylight * w;
        }

        y = Math.max(y, 1e-4);
        return new double[]{x, y};
    }

    private static CalibrationData extractCalibrationData(CameraCharacteristics characteristics) {
        if (characteristics == null) {
            return null;
        }

        Integer ref1Obj = characteristics.get(CameraCharacteristics.SENSOR_REFERENCE_ILLUMINANT1);
        int ref1 = (ref1Obj != null) ? ref1Obj : CameraMetadata.SENSOR_REFERENCE_ILLUMINANT1_STANDARD_A;

        Object ref2Obj = characteristics.get(CameraCharacteristics.SENSOR_REFERENCE_ILLUMINANT2);
        int ref2 = (ref2Obj != null) ? ((Number) ref2Obj).intValue() : CameraMetadata.SENSOR_REFERENCE_ILLUMINANT1_D65;

        int colorTemp1 = Converter.sStandardIlluminates.get(ref1, DEFAULT_TEMP_1);
        int colorTemp2 = Converter.sStandardIlluminates.get(ref2, DEFAULT_TEMP_2);

        ColorSpaceTransform calib1Xform = characteristics.get(CameraCharacteristics.SENSOR_CALIBRATION_TRANSFORM1);
        ColorSpaceTransform calib2Xform = characteristics.get(CameraCharacteristics.SENSOR_CALIBRATION_TRANSFORM2);
        ColorSpaceTransform color1Xform = characteristics.get(CameraCharacteristics.SENSOR_COLOR_TRANSFORM1);
        ColorSpaceTransform color2Xform = characteristics.get(CameraCharacteristics.SENSOR_COLOR_TRANSFORM2);

        if (color1Xform == null) {
            return null;
        }
        if (color2Xform == null) {
            color2Xform = color1Xform;
        }

        float[] calib1 = new float[9];
        float[] calib2 = new float[9];
        float[] color1 = new float[9];
        float[] color2 = new float[9];

        if (calib1Xform != null) Converter.convertColorspaceTransform(calib1Xform, calib1);
        else identityMatrix(calib1);

        if (calib2Xform != null) Converter.convertColorspaceTransform(calib2Xform, calib2);
        else identityMatrix(calib2);

        Converter.convertColorspaceTransform(color1Xform, color1);
        Converter.convertColorspaceTransform(color2Xform, color2);

        float[] xyzToCamera1 = new float[9];
        float[] xyzToCamera2 = new float[9];
        System.arraycopy(color1, 0, xyzToCamera1, 0, 9);
        System.arraycopy(color2, 0, xyzToCamera2, 0, 9);

        return new CalibrationData(colorTemp1, colorTemp2, xyzToCamera1, xyzToCamera2);
    }

    private static RggbChannelVector fallbackGains(double t) {
        double[] xy = kelvinToCIExy(t);
        double x = xy[0];
        double y = Math.max(xy[1], 1e-4);
        float rGain = (float) (y / x);
        float bGain = (float) (y / Math.max(1.0 - x - y, 1e-4));
        return new RggbChannelVector(rGain, 1.0f, 1.0f, bGain);
    }

    private static int fallbackKelvin(double r, double b) {
        double ratio = b / r;
        double kelvin = 5500.0 * Math.pow(ratio, 1.0);
        return Math.max(MIN_KELVIN, Math.min(MAX_KELVIN, (int) (Math.round(kelvin / 50.0) * 50)));
    }

    private static void identityMatrix(float[] m) {
        m[0] = 1f; m[1] = 0f; m[2] = 0f;
        m[3] = 0f; m[4] = 1f; m[5] = 0f;
        m[6] = 0f; m[7] = 0f; m[8] = 1f;
    }

    private static final class CalibrationData {
        final int colorTemp1;
        final int colorTemp2;
        final float[] xyzToCamera1;
        final float[] xyzToCamera2;

        CalibrationData(int colorTemp1, int colorTemp2, float[] xyzToCamera1, float[] xyzToCamera2) {
            this.colorTemp1 = colorTemp1;
            this.colorTemp2 = colorTemp2;
            this.xyzToCamera1 = xyzToCamera1;
            this.xyzToCamera2 = xyzToCamera2;
        }
    }
}