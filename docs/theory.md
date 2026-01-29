# Theoretical Background & Filter Math

This document outlines the mathematical principles and algorithms used in the **Sensor Fusion Lite** library.

## 1. Terminology

| Term | Symbol | Description |
| :--- | :---: | :--- |
| **State Vector** | $x$ | The vector representing the system's current status (e.g., position, velocity, orientation). |
| **State Covariance** | $P$ | A matrix representing the uncertainty/error in the state estimate. |
| **Process Noise** | $Q$ | Uncertainty in the system model (e.g., "how much does the robot wobble unexpectedly?"). |
| **Measurement Noise** | $R$ | Uncertainty in sensor readings (e.g., "how noisy is the GPS?"). |
| **Transition Matrix** | $F$ | Describes how the state evolves over time without external measurements. |
| **Observation Matrix** | $H$ | Describes how to map the state vector $x$ to a measurement $z$. |
| **Kalman Gain** | $K$ | The weighting factor that determines how much to trust the measurement vs. the prediction. |

---

## 2. Complementary Filter

The **Complementary Filter** is a simple, computationally efficient algorithm often used for orientation estimation. It fuses a signal that is good in the short term (high frequency) with a signal that is good in the long term (low frequency).

### Theory

In our context:
*   **Gyroscope (High Frequency)**: Accurate for short durations but drifts over time (integration error).
*   **Accelerometer/Magnetometer (Low Frequency)**: Noisy in specific instances but provides a stable absolute reference (gravity/north).

The filter combines these two:

$$ \hat{\theta}_{k} = \alpha \cdot (\hat{\theta}_{k-1} + \omega \cdot \Delta t) + (1 - \alpha) \cdot \theta_{acc} $$

Where:
*   $\alpha$ is the filter coefficient (typically $0.90 - 0.98$).
*   $(\hat{\theta}_{k-1} + \omega \cdot \Delta t)$ is the prediction from the gyroscope integration.
*   $\theta_{acc}$ is the orientation calculated from the accelerometer (e.g., `atan2(ay, az)`).

### Application in Fusion
We use this primarily for **Orientation**.
*   **Prediction**: Integrate angular velocity from IMU.
*   **Correction**: Pull the orientation towards the "Gravity Vector" measured by the accelerometer.

---

## 3. Extended Kalman Filter (EKF)

The **EKF** is the industry standard for non-linear state estimation. It linearizes the non-linear system equations around the current estimate using Jacobian matrices.

### The System Model (Constant Velocity)

We assume a constant velocity model for prediction.
State vector $x = [p_x, p_y, p_z, v_x, v_y, v_z]^T$.

$$ x_k = F x_{k-1} + w_k $$

The transition matrix $F$ for a time step $\Delta t$:

$$
F = \begin{bmatrix}
I & I \cdot \Delta t \\
0 & I
\end{bmatrix}
$$

### Prediction Step (Time Update)

Using the IMU / Time delta:

1.  **Project State**: $\hat{x}_k^- = f(\hat{x}_{k-1}, u_k)$
2.  **Project Error Covariance**: $P_k^- = F_k P_{k-1} F_k^T + Q_k$

### Update Step (Measurement Update)

When a sensor (Odom, GPS) arrives ($z_k$):

1.  **Compute Kalman Gain**:
    $$ K_k = P_k^- H_k^T (H_k P_k^- H_k^T + R_k)^{-1} $$

2.  **Update Estimate**:
    $$ \hat{x}_k = \hat{x}_k^- + K_k (z_k - h(\hat{x}_k^-)) $$

3.  **Update Covariance**:
    $$ P_k = (I - K_k H_k) P_k^- $$

### Application in Fusion
*   **IMU**: Used often for the **Prediction** step (integrating acceleration/velocity) or as a measurement for orientation.
*   **Odometry**: Provides a measurement of velocity ($v_x, v_y$) and relative position update. $H$ matrix maps just the velocity states.
*   **GPS**: Provides absolute position ($p_x, p_y$). $H$ matrix maps just the position states.

---

## 4. Unscented Kalman Filter (UKF)

The **UKF** addresses the linearization errors of the EKF by using a deterministic sampling approach called the **Unscented Transform**.

### Theory

Instead of calculating Jacobians ($F, H$), the UKF:
1.  Generates a set of **Sigma Points** around the mean state that capture the spread (covariance).
2.  Passes these points through the non-linear functions $\mathcal{X}' = f(\mathcal{X})$.
3.  Re-calculates the mean and covariance from the transformed points.

### Pros & Cons vs. EKF
*   **Pros**: captures higher-order non-linearities; no need to derive Jacobians (complex math).
*   **Cons**: Slightly higher computational cost ($2L+1$ sigma points effectively multiply the cost of the transform).

### Application in Fusion
Used when the motion model is highly non-linear (e.g., complex Ackerman steering dynamics or high-speed maneuvers) where the linear approximation of the EKF breaks down.
