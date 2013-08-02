function [X]=TwoDLSQ(RefPos,Dist)
if size(RefPos,1)~=size(Dist,1)
    error('Dimension should be match');
end
if size(RefPos,1)<=3
    error('Too few equation');
end
A=RefPos(2:end,:)-repmat([RefPos(1,:)],length(Dist)-1,1);
B=(Dist(2:end).^2-RefPos(2:end,1).^2-RefPos(2:end,2).^2-repmat([Dist(1)^2-RefPos(1,1)^2-RefPos(1,2)^2],length(Dist)-1,1))/-2;
NA=A'*A;
NB=A'*B;
X=inv(NA)*NB;
X=X'
end
