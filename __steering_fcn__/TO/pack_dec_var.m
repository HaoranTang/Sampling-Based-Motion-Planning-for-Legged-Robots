function [z,pack] = pack_dec_var(t,Xi,co,X,U,p)

nTime = length(t);
nX = size(X,1);
nU = size(U,1);

tSpan = [t(1);t(end)];

indz = reshape(2 + nX + 2*p.nco + (1:numel(X) + numel(U)),nX+nU,nTime);

tIdx = 1:2;
xiIdx = 3:6;
coIdx = 6 + [(1:p.nco);(1+p.nco):2*p.nco];
xIdx = indz(1:nX,:);
uIdx = indz(nX+(1:nU),:);

z = zeros(6+2*p.nco+numel(indz),1);
z(tIdx(:),1) = tSpan;
z(xiIdx(:),1) = Xi(:);
z(coIdx(:),1) = co(:);
z(xIdx(:),1) = X(:);
z(uIdx(:),1) = U(:);

pack.nt = nTime;
pack.nco = p.nco;
pack.nX = nX;
pack.nU = nU;
pack.tIdx = tIdx;
pack.xiIdx = xiIdx;
pack.coIdx = coIdx;
pack.xIdx = xIdx;
pack.uIdx = uIdx;


end