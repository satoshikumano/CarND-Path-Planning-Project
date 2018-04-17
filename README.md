# Udacity self driving car Path Planning Project

This repository is forked from [Udacity CarND Path Planning Project](https://github.com/udacity/CarND-Path-Planning-Project).

Steps of setup is written in [Original README](./REAMDE-ORG.md)

## State

Implemented following 3 state.

- KL : Keep lane.
- LCL : Change lane to left.
- LCR : Change lane to right.


Valid State transitions:
- KL -> LCL
- KL -> LCR
- LCL -> KL
- LCR -> KL

## Trajectory Generation Details.

### KL Trajectory generation

1. Pick following points on Frenet coordinate.

In case there's previously generated trajectory,
`ref_s` and `ref_d` is the last point of previously generated and not consumed yet.

```
{ prev_ref_s, prev_ref_d }
{ ref_s, ref_d }
{ ref_s + 15, ref_d }
{ ref_s + 30, ref_d }
{ ref_s + 60, ref_d }
{ ref_s + 90, ref_d }
```

`ref_s` is current car's s coordinate or last point previously given to simulator.
`ref_d` is lane center position calculated by `2 + 4 * lane`. (lane = 0, 1, 2);

2. Convert coordinate obtained in step 1. to vehicle coordinate.

3. Apply 3rd order polynomial fit to coordinates obtained in step 2.
    (using spline.h)

4. On vehicle coordinate, divide interval x [0, 30] into N steps in following process.

```
  double target_x = 30;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double N = target_dist / (0.02 * _ref_speed);
  double x_diff = target_x / N;
```

```
x(i) = 0 + x_diff * i (i = 0, 1, 2, ...,)
```

Calculate y(i) coordinate corresponds to x(i) with 3rd order polynomial obtained in step 3.

5. convert coordinate obtained in steps 4. to map coordinate.

### Lane change trajectory generation

1. Calculate frenet coordinate after 1, 2 and 3 second after from the reference point
`{ ref_s, ref_d }.

Reference point is current car position or last point of previously given to simulator if it has not been consumed yet.

In the calculation,

- Assume longitutional speed is constant.

- JMT to calculate s coordinate after 1sec, 2sec, 3 sec.
  - Assume s dot, s dot dot is in both start and end point.

```
auto coeffs = helper::JMT({car_d, 0, 0}, {target_d, 0, 0}, 3);
```

Given point is following:

```
{ prev_ref_s, prev_ref_d }
{ ref_s, ref_d }
{ ref_s + speed, jmt(1) }
{ ref_s + speed *  2, jmt(2) }
{ ref_s + speed * 3, jmt(2) }
```

where `speed` is reference speed of the car.
jmt(t) is `d` position at 5 given by jerk minimization function.

2. Apply same steps 2-5 in `KL Trajectory generation` to obtain smooth trajectory
 on map coordinate.

### Other notes.

PID Controller is used to

- Keep the distance of the front car 
- Keep the center of the lane.
 (Since the frenet/ map coordinate conversion is not very acculate due to map resolution and way of calculating s.)
