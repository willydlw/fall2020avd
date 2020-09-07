# Digital Low-Pass IIR Filter based on Analog Passive RC Low-Pass Filter

## Analog Passive RC Low-Pass Filter

![RC Filter](./images/lowpass_rc_filter.png "Low-pass RC Filter")

[Image 1: Low-pass RC Filter][1]

[1]: https://www.electronics-tutorials.ws/wp-content/uploads/2013/08/fil5.gif?fit=326%2C161
<br>

## Time-Domain Analysis

Using Kirchhoff's Laws and the definition of capacitance:

1) v<sub>in</sub>(t) - v<sub>out</sub>(t) = R i(t)
2) Q<sub>c</sub>(t) = C v<sub>out</sub>(t)

3) i(t) = &delta;v<sub>out</sub> / &delta;t

where Q<sub>c</sub>(t) is the charge stored in the capacitory at time t.

Substituting equation 2 into 3,

4) i(t) = C &delta;v<sub>out</sub> / &delta;t

Substituting equation 4 into 1 for i(t),

5) v<sub>in</sub>(t) - v<sub>out</sub>(t) = RC &delta;v<sub>out</sub> / &delta;t

<br>

## Discretize Time Domain Equations

Assume that input and output are sampled at evenly spaced points in time, separated by &Delta;t time. Let the samples of v<sub>in</sub> be represented by the sequence (x<sub>1</sub>, x<sub>2</sub>, x<sub>3</sub>, ..., x<sub>n</sub>). Let v<sub>out</sub> be represented by the sequence (y<sub>1</sub>, y<sub>2</sub>, y<sub>3</sub>, ..., y<sub>n</sub>).
<br>

The discrete form of the continous equation 5 is

6) x<sub>i</sub> - y<sub>i</sub> = RC (y<sub>i</sub> - y<sub>i-1</sub>) / &Delta;t

<br>
Rearranging the terms gives the recurrence relation

7) y<sub>i</sub> = x<sub>i</sub> (&Delta;t)/(RC + &Delta;t) + y<sub>i-1</sub> (RC)/(RC + &Delta;t)

<br>
Let &alpha; = (&Delta;t)/(RC + &Delta;t).

The discrete time implementation of the RC low-pass filter is

8) y<sub>i</sub> = &alpha; x<sub>i</sub> + (1-&alpha;) y<sub>i-1</sub>

<br>
Recognize that this is in the form of an infinite impulse response filter. It is an exponentially weighted moving average. The &alpha; value is an exponential smoothing factor, whose function is to remove high-frequency noise.

By definition, the smoothing factor, &alpha; is subject to 0 <= &alpha; <= 1

Larger values of &alpha; reduce the level of smoothing. Values of &alpha; close to 1 give greater weight to recent changes in data, while values of &alpha; closer to zero have a greater smoothing effect and are less responsive to recent changes. There is no formally correct procedure for choosing &alpha; <br>
<br>

The time constant RC is 

9) RC = &Delta;t ((1-&alpha;)/&alpha;)

<br>
The cutoff frequency f<sub>c</sub> is

10) f<sub>c</sub> = 1 / (2 &pi; RC)

<br>
Rearranging equation 10,

11) RC = 1 / (2 &pi; f<sub>c</sub>)

<br>
Making &alpha; and f<sub>c</sub> related by

12) &alpha; = (2 &pi; &Delta;t f<sub>c</sub>) / (2 &pi; &Delta;t f<sub>c</sub> + 1)

13) f<sub>c</sub> = &alpha; / ( (1-&alpha;) 2&pi;&Delta;t)

