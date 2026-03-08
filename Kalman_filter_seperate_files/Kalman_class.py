import numpy as np

class KalmanFilter:
    def __init__(self, A, B, H, Q, R):
        self.A = A  # State transition matrix
        self.B = B  # Control input matrix
        self.H = H  # Measurement matrix
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.x = np.zeros((A.shape[0], 1))  # Initial state estimate
        self.P = np.eye(A.shape[0])  # Initial estimate error covariance

    def predict(self, u):
        # Predict the next state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        # Predict the error covariance
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

    def update(self, z):
        # Compute Kalman Gain
        y = z - np.dot(self.H, self.x)  # Measurement residual
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R  # Residual covariance
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))  # Kalman Gain

        # Update state estimate and error covariance
        self.x += np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = (I - np.dot(K, self.H)).dot(self.P)