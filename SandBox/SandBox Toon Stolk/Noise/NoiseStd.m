function std = NoiseStd(betaAngle, minStd, maxStd)
%NOISESTD Summary of this function goes here
%   Detailed explanation goes here
amplitude = maxStd - minStd; 

std = -amplitude/2 * sin(betaAngle) + minStd + amplitude/2;
end

