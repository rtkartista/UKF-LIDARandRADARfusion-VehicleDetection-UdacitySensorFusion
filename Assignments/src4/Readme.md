## Algorithm For UKF
1. Given: Inital mean and covariance of the state X and the Radar Measurement Znew
### Predict
2. Calculate the Augumented Sigma Points with the noise
3. Apply Non linear transformation on the above sigma points with respective noise elements
4. Calculate the predicted mean and covariance in X
### Update
5. With the given X augumented sigma points, calculate the corresponding Z sigma points. Additionally, calculate the mean and covariance in Z (zpred)
6. Finally, with the new Radar measurment, calculate the corresponding Kalman Gain and update the X mean and covariance.

- This is a part of Udacity's coding assignments from the lesson UKF.