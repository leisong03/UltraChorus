function [BPSDeployment]=GenBPSDeployment(NumBPS,DeploymentType,BPSRadius,SizeX,SizeY)
% BPS deployment generator 
% Usage:
% [BPSDeployment]=GenBPSDeployment(NumBPS,DeploymentType,BPSRadius,SizeX,SizeY)
% Inputs:
% NumBPS		= number of BPS
% DeploymentType	= Type of deployment, could be 'rand' or 'uniform'
% BPSRadius		= sensing range of BPS
% SizeX			= X Size of deployment range
% SizeY			= Y Size of deployment range
% Outputs:
% BPSDeployment= structure with following fileds
%	.num	    = number of BPS
%	.X	    = X coordinator (num x 1 )
%	.Y	    = Y coordinator (num x 1 )
%	.R	    = Radius of BPS (num x 1 )
%	.XBound	    = X Boundary of deployment
%	.YBound	    = Y Boundary of deployment
BPSDeployment.num = NumBPS;
BPSDeployment.R=repmat([BPSRadius],NumBPS,1);
BPSDeployment.XBound=SizeX;
BPSDeployment.YBound=SizeY;
switch lower(DeploymentType)
    case 'rand'
	BPSDeployment.X=rand(NumBPS,1)*SizeX;
	BPSDeployment.Y=rand(NumBPS,1)*SizeY;
    case 'uniform'
	ratio= SizeX/SizeY; 
	YNum=floor(sqrt(NumBPS/ratio));
	XNum=floor(NumBPS/YNum);
	BPSDeployment.num=XNum*YNum;
	BPSDeployment.X=zeros(BPSDeployment.num,1);
	BPSDeployment.Y=zeros(BPSDeployment.num,1);
	Xcord=linspace(0+BPSRadius,SizeX-BPSRadius,XNum);
	%BPSDeployment.Y=linspace(0+BPSRadius,SizeY-BPSRadius,YNum);
	Ycord=linspace(0+BPSRadius,SizeY-BPSRadius,YNum);
	for i=1:XNum
	    for j=1:YNum
		idx=(i-1)*YNum+j;
		BPSDeployment.X(idx)=Xcord(i);
		BPSDeployment.Y(idx)=Xcord(j);
	    end
	end
	BPSDeployment.R=repmat([BPSRadius],BPSDeployment.num,1);
    otherwise 
	error('DeploymentType is incorrect');
end

end
