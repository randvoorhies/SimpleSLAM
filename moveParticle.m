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
function newpos = updateMovement(pos, movement, variance)
% Compute how the robot should move from "pos" given the requested movement and
% some Gaussian random noise using a very simple motion model. This method is
% used to move the simulated robot as well as each of the hypothetical
% particles.

  % Add some Gaussian random noise to the movement. Note that we are using
  % a smaller variance in this Gaussian distribution, as the algorithm seems
  % to work better when it underestimates the quality of the robot plant. 
  speed    = normrnd(movement(1), variance(1)*.25);
  rotation = normrnd(movement(2), variance(2)*.25);

  delta = zeros(3,1);
  delta(1,1) = cos(pos(3)+rotation)*speed;
  delta(2,1) = sin(pos(3)+rotation)*speed;
  delta(3,1) = rotation;

  newpos = pos+delta;
end
