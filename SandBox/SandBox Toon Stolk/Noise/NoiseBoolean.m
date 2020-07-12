function bool = NoiseBoolean(betaAngle, eclipseCone, FOVcone)
%NOISEBOOLEAN
%   This function returns a 0 or a 1 for a specific betaAngle. A zero if
%   the betaAngle lies in either the eclipse or if the Field of View is
%   pointing the sun. And a 1 otherwise.
if (pi/2-eclipseCone/2 <= betaAngle) && (betaAngle <= pi/2+eclipseCone/2)
    bool = 0;
elseif (pi-FOVcone/2 <= betaAngle) && (betaAngle <= pi+FOVcone/2)
    bool = 0;
else
    bool = 1;
end
end
