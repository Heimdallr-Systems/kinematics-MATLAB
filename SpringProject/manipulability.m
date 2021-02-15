function [muFR, muFL, muBR, muBL] = manipulability(state)

state(19:36) = zeros(1,18);

[GeoJc_FR,GeoJc_FL,GeoJc_BR,GeoJc_BL] = contactJacobians(state);

% Find SVD
[U_FR,SIG_FR,V_FR] = svd(GeoJc_FR);
[U_FL,SIG_FL,V_FL] = svd(GeoJc_FL);
[U_BR,SIG_BR,V_BR] = svd(GeoJc_BR);
[U_BL,SIG_BL,V_BL] = svd(GeoJc_BL);

sig_size = size(SIG_FR);
sig_size = sig_size(1);

sigFR = zeros(6,1);
sigFL = zeros(6,1);
sigBR = zeros(6,1);
sigBL = zeros(6,1);

for ii = 1:1:sig_size
    sigFR(ii) = SIG_FR(ii,ii);
    sigFL(ii) = SIG_FL(ii,ii);
    sigBR(ii) = SIG_BR(ii,ii);
    sigBL(ii) = SIG_BL(ii,ii);
end

muFR = prod(sigFR);
muFL = prod(sigFL);
muBR = prod(sigBR);
muBL = prod(sigBL);

end