# Refractometer_setup
Experiment control and data acquisition

Matlab codes written to control our experimental setup and collect data.

Gennadiy Derkachov, PhD
Kwasi Nyandey, MSc
Daniel Jakubczyk, PhD hab. Eng.
Anastasiya Derkachova, PhD

collaborated in writing the codes. GD created the concept and wrote the core functions.

As of Sep 2023, the code takes care of temperature measurement in the prism,
processes the image of the shadow and facilitates the reading.
It also converts the raw reading (Amicis prism compensator was removed from the refractometer) into a corrected value,
using an appropriate formula. The formula was found in the literature and adapted to our setup.
