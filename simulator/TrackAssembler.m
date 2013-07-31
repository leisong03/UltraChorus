function [FinalX,FinalY]=TrackAssembler(EstTrackX,EstTrackY,AssignToID,NumAgent)
FinalX=ones(size(EstTrackX,1),NumAgent)*inf;
FinalY=ones(size(EstTrackY,1),NumAgent)*inf;
for i=1:size(EstTrackX,1)
    for j=1:size(AssignToID,2)
	if AssignToID(i,j)~=0
	    FinalX(i,AssignToID(i,j))=EstTrackX(i,j);
	    FinalY(i,AssignToID(i,j))=EstTrackY(i,j);
	end
    end
end
lastX=0;
lastY=0;
for j=1:size(FinalX,2)
    for i=1:size(FinalX,1)
	i,j
	blockstart=find(FinalX(:,j)==inf & [1:size(FinalX,1)]'>=i,1,'first')
	blockend=find(FinalX(:,j)~=inf & [1:size(FinalX,1)]'>=i+1,1,'first')
	if isempty(blockstart)
	    break;
	else
	    if blockstart~=1
		lastX=FinalX(blockstart-1,j);
		lastY=FinalY(blockstart-1,j);
	    end
	    if isempty(blockend)
		FinalX(blockstart:end,j)=lastX;
		FinalY(blockstart:end,j)=lastY;
		break;
	    else
		nextX=FinalX(blockend-1);
		nextY=FinalY(blockend-1);
		FinalX(blockstart:(blockend-1),j)=linspace(lastX,nextX,blockend-blockstart);
		%FinalY(blockstart:(blockend-1),j)=linspace(lastY,nextY,blockend-blockstart);
	    end
	end
	i=blockend+1;
    end
end
end
