function PX=CalCDF(X,NumBins)
%Calculat the cumulated distribution funtion of X
%
%Usage:
% [PX]= CalCDF (X,NumBins)
%Inputs: 
% X = data need to be stated 
%Output:
% PX = probability that x<X  
MinX = min(X);
MaxX = max(X);
bins=linspace(MinX,MaxX,NumBins);
PX=zeros(length(bins),1);
for i=1:length(bins)
   PX(i)=sum(X<=bins(i));
end
PX=PX./length(X);
end
