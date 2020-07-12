function std = NoiseStd(betaAngle, minStd, maxStd)
%NOISESTD 
%   This function returns the magnitude of the noise as a function of the
%   betaAngle, the minimum noise and the maximum noise.
amplitude = maxStd - minStd; 

std = -amplitude/2 * cos(betaAngle) + minStd + amplitude/2;
end

