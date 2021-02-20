# Extended Kalman Filter
## Overview
This project contains an implementation of [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) in C++.

The project uses [uWebSockets](https://github.com/uNetworking/uWebSockets) to implement the server, which performs all necessary calculations.

There's a [simulator](https://github.com/udacity/self-driving-car-sim/releases) provided by Udacity. There're also testing data, provided by Udacity too.

The simulator communicates with the server and performs all the necessary calculations.

# Dependencies

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

# How to build

- Clone the repo and cd into it
- `mkdir build`
- `cd build`
- `cmake ..`
- `make`

## How to run

From the build directory, execute `./ExtendedKF`.
If everythin is OK, then the output must be:

```
Listening to port 4567
Connected!!!
```

## Results

Achieved accuracy:

- Dataset 1 : RMSE <= [0.0974, 0.0855, 0.4517, 0.4404]
- Dataset 2 : RMSE <= [0.0726, 0.0965, 0.4219, 0.4937]

Examples:

![alt](ds1_all.png)
![alt](ds2_all.png)
