function bool = NoiseBoolean(betaAngle, eclipseCone, FOVcone)
%NOISEBOOLEAN Summary of this function goes here
%   Detailed explanation goes here
if (pi-eclipseCone/2 < betaAngle) && (betaAngle < pi+eclipseCone/2)
    bool = 0;
elseif (1.5*pi-FOVcone/2 < betaAngle) && (betaAngle < 1.5*pi+FOVcone/2)
    bool = 0;
else
    bool = 1;
end
end

