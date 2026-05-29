"""
Kalman Filter for sensor noise reduction.

This module implements a 1D Kalman filter to smooth noisy depth readings
from the RealSense D435 camera before they are used for occupancy grid mapping.

The Kalman filter maintains a state estimate and covariance, and updates both
using process and measurement models.
"""

import numpy as np


class KalmanFilter1D:
    """
    1D Kalman Filter for smoothing depth sensor readings.
    
    Attributes:
        q: Process noise variance (uncertainty in motion model)
        r: Measurement noise variance (sensor noise)
        x: Current state estimate
        p: Current covariance (uncertainty)
    """
    
    def __init__(self, process_noise=0.01, measurement_noise=0.1, initial_state=0.0):
        """
        Initialize the Kalman filter.
        
        Args:
            process_noise: Process noise variance (q). Higher = less confidence in motion model.
            measurement_noise: Measurement noise variance (r). Higher = less confidence in sensor.
            initial_state: Initial state estimate (meters).
        """
        self.q = process_noise
        self.r = measurement_noise
        self.x = initial_state
        self.p = 1.0  # Initial uncertainty (covariance)
        
    def predict(self):
        """Prediction step: predict next state based on motion model (identity)."""
        # For a stationary target, x doesn't change in predict
        # p increases due to process uncertainty
        self.p = self.p + self.q
        
    def update(self, measurement):
        """
        Update step: incorporate measurement to refine state estimate.
        
        Args:
            measurement: Raw sensor measurement (meters).
            
        Returns:
            Filtered estimate of the state.
        """
        # Kalman gain
        k = self.p / (self.p + self.r)
        
        # Update state estimate
        self.x = self.x + k * (measurement - self.x)
        
        # Update covariance
        self.p = (1 - k) * self.p
        
        return self.x
    
    def filter_measurement(self, measurement):
        """
        Filter a single measurement using one predict-update cycle.
        
        Args:
            measurement: Raw sensor reading.
            
        Returns:
            Filtered estimate.
        """
        self.predict()
        return self.update(measurement)


class KalmanFilterDepthArray:
    """
    Kalman filter array for smoothing multiple depth readings (e.g., from LiDAR scan).
    
    Each beam/pixel gets its own 1D filter state, but they share hyperparameters.
    """
    
    def __init__(self, num_readings=360, process_noise=0.01, measurement_noise=0.1):
        """
        Initialize array of Kalman filters.
        
        Args:
            num_readings: Number of independent depth measurements (e.g., 360 for LIDAR).
            process_noise: Process noise variance (shared across all filters).
            measurement_noise: Measurement noise variance (shared across all filters).
        """
        self.num_readings = num_readings
        self.q = process_noise
        self.r = measurement_noise
        self.x = np.zeros(num_readings)  # State for each reading
        self.p = np.ones(num_readings)   # Covariance for each reading
        
    def predict(self):
        """Prediction step for all filters."""
        self.p = self.p + self.q
        
    def update(self, measurements):
        """
        Update all filters with new measurements.
        
        Args:
            measurements: Array of measurements (shape: num_readings,).
            
        Returns:
            Filtered estimates (shape: num_readings,).
        """
        measurements = np.asarray(measurements)
        
        # Kalman gain per filter
        k = self.p / (self.p + self.r)
        
        # Update state
        self.x = self.x + k * (measurements - self.x)
        
        # Update covariance
        self.p = (1 - k) * self.p
        
        return self.x.copy()
    
    def filter_measurements(self, measurements):
        """
        Filter an array of measurements.
        
        Args:
            measurements: Array of raw readings.
            
        Returns:
            Filtered array.
        """
        self.predict()
        return self.update(measurements)
