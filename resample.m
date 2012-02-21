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
function [newParticles] = resample(oldParticles)
% Resample a group of particles such that those with higher weights have a
% higher chance of being replicated, and those with low weights have a high
% chance of disappearing
  
  weightSum = 0;
  for i=1:length(oldParticles)
    weightSum = weightSum + oldParticles(i).w;
  end
  for i=1:length(oldParticles)
    oldParticles(i).w = oldParticles(i).w/weightSum;
  end

  M = length(oldParticles);

  newParticles = [];

  r = rand / M;

  c = oldParticles(1).w;

  i = 1;

  for m=1:M
    U = r + (m-1) * M^(-1);

    while U > c
      i = i+1;
      c = c+oldParticles(i).w;
    end
    newParticles = [newParticles, oldParticles(i)];
  end

  for i=1:length(newParticles)
    newParticles(i).w = 1.0/length(newParticles);
  end

end
