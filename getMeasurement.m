%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Copyright 2010 Randolph Voorhies
%  This program is free software: you can redistribute it and/or modify
%  it under the terms of the GNU General Public License as published by
%  the Free Software Foundation, either version 3 of the License, or
%  (at your option) any later version.
%
%  This program is distributed in the hope that it will be useful,
%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  GNU General Public License for more details.
%
%  You should have received a copy of the GNU General Public License
%  along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
function [z, H] = getMeasurement(pos, landmark_pos, observation_variance)
%   Given a landmark position and a robot position, this method will return a
%   "measurement" z that contains the distance and the angle to the landmark.
%   Gaussian random noise is added to both based on the variances given in the
%   diagonal of the observation_variance matrix. Note that this method is used
%   both to take a "real" measurement in the simulation, as well as to assess 
%   what kind of measurement each of our hypothetical particles would take.
%   This method also computes the Jacobian of the measurent function for use
%   in an extended Kalman filter.


  % Compute the distance from the current position to the landmark, and add
  % some Gaussian noise to make things interesting. Note that we are using
  % a smaller variance in this Gaussian distribution, as the algorithm seems
  % to work better when it underestimates the quality of the sensor. 
  vector_to_landmark = [landmark_pos(1) - pos(1); landmark_pos(2) - pos(2)];
  landmark_distance = norm(vector_to_landmark);
  landmark_distance = landmark_distance + normrnd(0, observation_variance(1)*.25);

  % Compute the angle from the given pos to the landmark
  landmark_angle = atan2(vector_to_landmark(2), vector_to_landmark(1));
  landmark_angle = landmark_angle + normrnd(0, observation_variance(2)*.25);


  % Compute the Jacobian of this measurement function
  q = landmark_distance^2.0;
  H = [-(landmark_pos(1) - pos(1))/sqrt(q), -(landmark_pos(2) - pos(2))/sqrt(q), 0.0;
        (landmark_pos(2) - pos(2))/q,       -(landmark_pos(1) - pos(1))/q,      -1.0;
        0.0,                                0.0,                               1.0];

  z = [landmark_distance; 
       landmark_angle;
       0];
end
