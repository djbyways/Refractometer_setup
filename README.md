# Refractometer_setup
Experiment control and data acquisition

Matlab codes were written to control our experimental setup and collect data.

Gennadiy Derkachov, PhD
Kwasi Nyandey, MSc
Daniel Jakubczyk, PhD hab. Eng.
Anastasiya Derkachova, PhD

collaborated in writing the codes. GD created the concept and wrote the core functions.
The CalculatePacketDelay is written by MathWorks and is freely available on the net. We put it here for the completeness of the package.

As of Sep 2023, the code takes care of temperature measurement in the prism - reads CHY506R electronic thermometer (CHY Firemate Co.),
obtains images from GC651MP (Smartek Vision) camera, processes the image of the shadow (clipping, background subtraction, vertical summation, smoothing)
and facilitates the reading – the position of the shadow edge was represented as the minimum of the derivative of brightness
– corresponding to the inflection point on the shadow edge. The crosshair centre was determined by pointing at it in a magnified image at the beginning of the measurement series.
Thus, the measurement consisted of adjusting the derivative minimum to the crosshair centre. The raw value of the refractive index is read by the experimenter from the screen and
written into the appropriate field. Then the code converts the raw reading (Amicis prism compensator was removed from the refractometer) into a corrected value, using an appropriate formula.
The formula was found in the literature (Kedenburg, S.et al., Opt. Mater. Express 2, 1588, 2012) and adapted to our setup.

The functions for temperature stabilisation are not in use now due to the change of the setup.

