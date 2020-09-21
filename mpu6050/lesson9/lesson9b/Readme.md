# Lesson 9b - Restricting Pitch and Roll

Lesson 9a used equations 25 and 26 as an estimation of the rotation R<sub>xyz</sub> roll angle &phi; and pitch angle &theta from [Freescale](./datasheets/AN3461.pdf).

**Eqn 25** Roll angle: tan &phi;<sub>xyz</sub> = (Gpy)/(Gpz)

**Eqn 26** Pitch angle: tan &theta;<sub>xyz</sub> = (-Gpx)/(sqrt(Gpy<sup>2</sup> + Gpz<sup>2</sup>)

where Gpx, Gpy, Gpz are the accelerometer x, y, and z measurements, respectively.

Equations 28 and 29 may also be used to compute these angles, but will give different results, due to the different rotation order yxz. Remember matrix multiplication does not commute.

Both sets of equations provide duplicate solutions for some angles. Equation 8 is the matrix for the R<sub>xyz</sub> rotation.

**Eqn 8** 
| | 
| --- |
| -sin &theta; |
| cos &theta; sin &phi; |
| cos &theta; cos &phi; |
<br>

When the pitch angle is &pi; - &theta; and roll angle &phi; + &pi;, substituting the angles into the matrix and applying trig identities produces the same accelerometer measurements as equation 8

| | Eqn 30 | |
| --- | --- | --- |
| -sin (&pi; - &theta;) | | -sin &theta; |
| cos (&pi; - &theta;) sin (&phi; + &pi;) | = | cos &theta; sin &phi; |
| cos (&pi; - &theta;) cos (&phi; + &pi;)| | cos &theta; cos &phi; |

The same holds true for equation 11 of the R<sub>yxz</sub> rotation, when the pitch angle is &theta; + &pi; and the roll angle is &pi; - &phi;.

The recommended solution is to restrict either the roll or the pitch angle (but not both) to lie between -90 degrees and +90 degrees. The convention used in the aerospace sequence (xyz) is that the roll angle has the range [-180, +180] degrees, but the pitch angle is restricted to [-90, +90] degrees. This will produce a unique solution for roll with the atan2 function and a unique solution for pitch with the atan function.

> Note: Equations 25 and 29 have regions of instability when both the numerator and denominator are zero. For equation 25, this condition occurs when the device is aligned with its x-axis pointing vertically upwards or downwards so that a rotation about the gravitational field vector cannot be detected.

[lesson9b.ino](lesson9b.ino) restricts the pitch angle as suggested.

```
// Freescale, equations 25 and 26, Rotation order Rxyz
// roll may vary from -pi to +pi, use atan2
roll = atan2(accel.y, accel.z) * 180.0/PI;
// pitch is restricted to -pi/2 to +pi/2, use atan
pitch = atan(-accel.x / sqrt(pow(accel.y,2) + pow(accel.z,2))) * 180.0/PI;
```