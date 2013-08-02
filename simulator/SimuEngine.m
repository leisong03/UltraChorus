function SimuEngine(XSize,YSize,NumBPS,rangerr,NumAgent,DepMode)
close all;
%XSize=100;
%YSize=100;
%NumBPS=100;
%radius=4;
%ThisUDG=UniDiskGraph(BMSPos);
Tw=5;
Velocity=330;
Dw=Tw/1000*Velocity;
OutputSeq=0;
OutputUI=1;
OutputCountingSeq=0;
BPSDeployment=GenBPSDeployment(NumBPS,DepMode,1,XSize,YSize);   %return the BPSDeployment structure
delta=.05;  %interval of angle
theta=0:delta:2*pi;
OutlineX=zeros(size(theta,1),length(theta));
OutlineY=zeros(size(theta,1),length(theta));
for i=1: BPSDeployment.num
    %thisx=BPSDeployment.X(i)
    OutlineX(i,:)=BPSDeployment.R(i).*cos(theta)+BPSDeployment.X(i);
    OutlineY(i,:)=BPSDeployment.R(i).*sin(theta)+BPSDeployment.Y(i);
end

T=30; %length of time 
deltaT=.1; %interval of simulation
%NumAgent=10; %number of Agent
AgentVel=1+rand(NumAgent,1); %velocity of agent
LastPos=rand(NumAgent,2).*repmat([XSize,YSize],NumAgent,1); %Initial position of agetn 
dir=2*pi*rand(NumAgent,1); %initial moving direction
%split the UI into 2 subfigure
ChannelLive=figure(); %plot Live into left window
%UDG=subplot(1,2,2); %plot UDG in right window

RealTrackX=zeros(length([0:deltaT:T]),NumAgent);
RealTrackY=zeros(length([0:deltaT:T]),NumAgent);
EstTrackX=zeros(length([0:deltaT:T]),NumAgent*2);
EstTrackY=zeros(length([0:deltaT:T]),NumAgent*2);
AssignToID=zeros(length([0:deltaT:T]),NumAgent*2);
LocErrVec=[];

Tidx=1;
for t=0:deltaT:T % for each time
    t
    Tidx
    SensorReading=zeros(BPSDeployment.num,1); % initial readig of sensor is 0 
    if mod(Tidx,10)==0
	% every 5 seconds, change the direction of agent 
	dir=2*pi*rand(NumAgent,1);
    end
    [DeltaX, DeltaY]=pol2cart(dir,AgentVel*deltaT); 
    AgentPos=LastPos+[DeltaX,DeltaY]; %Current position of agent

    idx=find(AgentPos(:,1)<0); %if agent across the bound
    AgentPos(idx,1)=0;
    AgentPos(idx,1)=-1*AgentPos(idx,1); %reflect
    dir(idx)=2*pi*rand(length(idx),1); % re peak the direction

    idx=find(AgentPos(:,1)>XSize);
    dir(idx)=2*pi*rand(length(idx),1);
    AgentPos(idx,1)=2*XSize-1*AgentPos(idx,1);

    idx=find(AgentPos(:,2)<0);
    AgentPos(idx,2)=-1*AgentPos(idx,2);
    dir(idx)=2*pi*rand(length(idx),1);

    idx=find(AgentPos(:,2)>YSize);
    AgentPos(idx,2)=2*YSize-1*AgentPos(idx,2);
    dir(idx)=2*pi*rand(length(idx),1);

    dist=ones(BPSDeployment.num,size(AgentPos,1)).*-1; %m x n matrix, each row is the distance from receiver i to emitter [1:m]

    for i=1:BPSDeployment.num
	thisdist=repmat([BPSDeployment.X(i),BPSDeployment.Y(i)],size(AgentPos,1),1)-AgentPos;
	thisdist=sqrt(thisdist(:,1).^2+thisdist(:,2).^2);
	thisdist=sort(thisdist);
	ptr=1;
	for j=1:size(AgentPos,1)
	    size(AgentPos,1)
	    j,Dw
	    if j==1 
		dist(i,ptr)=thisdist(j);
		ptr=ptr+1;
	    elseif thisdist(j)-thisdist(j-1)>Dw
		dist(i,ptr)=thisdist(j);
		ptr=ptr+1;
	    end
	end
	if ~isempty(find( dist(i,:)==-1))
	    disp('too close');
	    dist(i,:)
	end
    end

    LastPos=AgentPos;
    RealTrackX(Tidx,:)=AgentPos(:,1)';
    RealTrackY(Tidx,:)=AgentPos(:,2)';
    %update the UDG

    EstAgentPos=zeros(size(AgentPos,1),2);

    %add tracking algorithm here 
    for i=1:size(AgentPos,1)
	FeasibleDist=ones(BPSDeployment.num,size(AgentPos,1))*-1;
	PrecDist=repmat(AgentPos(i,:),BPSDeployment.num,1)-[BPSDeployment.X,BPSDeployment.Y];
	PrecDist=sqrt(PrecDist(:,1).^2+PrecDist(:,2).^2);
	LowerBound=PrecDist-Dw
	UpperBound=PrecDist+Dw
	for j=1:BPSDeployment.num
	    idx=find(dist(j,:)>=LowerBound(j)&dist(j,:)<=UpperBound(j));
	    FeasibleDist(j,1:length(idx))=dist(j,idx);
	end
	FeasibleDist;
	ValidIdx=find(FeasibleDist~=-1)
	AgentPos(i,:)
	AX=TwoDLSQ([BPSDeployment.X(ValidIdx),BPSDeployment.Y(ValidIdx)],FeasibleDist(ValidIdx))
	EstAgentPos(i,:)=AX;
    end
    EstTrackX(Tidx,:)=EstAgentPos(:,1)';
    EstTrackX(Tidx,:)=EstInBoxSeq(:,1)';
    %tracking algorithm finished

    if OutputCountingSeq==1
	if mod(Tidx,10)==0
	    NumSeq=figure();
	    %axis([0 t 0 max([max(RealInBoxSeq),max(EstInBoxSeq)])]);
	    TimeIdx=linspace(0,t,Tidx);
	    %plot(TimeIdx,RealInBoxSeq(1:Tidx),'r-',TimeIdx,EstInBoxSeq(1:Tidx),'b-');
	    xlabel('time (sec)');
	    ylabel('Number(#)');
	    legend('Ground truth','Estimated num.');
	    set(gca,'FontSize',20)
	    xlhand = get(gca,'xlabel')
	    set(xlhand,'fontsize',20) 
	    xlhand = get(gca,'ylabel')
	    set(xlhand,'fontsize',20) 
	    print(NumSeq,'-dpdf',['numseq',num2str(t),'.pdf']);
	    close(NumSeq);
	end
    end
    
    %Plot state of BPS according to the     
    if OutputSeq==1
    HandlePos=figure();
    hold on;
    axis([0 XSize 0 YSize]);
    xlabel('X(m)');
    ylabel('Y(m)');
    for i=1:size(AgentPos,1); 
	plot(RealTrackX(1:Tidx,i),RealTrackY(1:Tidx,i),'-d');
    end
    for i=1:n
	plot(center(:,1),center(:,2),'ro');
    end
    hold off;
	print(HandlePos,'-dpdf',[num2str(t),'.pdf']);
    close(HandlePos);
    end

    
    if OutputUI==1
    figure(ChannelLive);
    clf();
    text(XSize,YSize,['Time=',num2str(t)]);
    %plot(OutlineX(1,:),OutlineY(1,:),'-');
    hold on;
    %for i=2:BPSDeployment.num
	%plot(OutlineX(i,:),OutlineY(i,:),'-');
    %end

    %for i=1:BPSDeployment.num
	%if SensorReading(i)==1
	    %fill(OutlineX(i,:),OutlineY(i,:),'red');
	%end
    %end
    plot(AgentPos(:,1),AgentPos(:,2),'kp');
    plot(EstAgentPos(:,1),EstAgentPos(:,2),'go');
    plot(BPSDeployment.X,BPSDeployment.Y,'rd');
    hold off;
    axis equal;
    axis ([0 XSize 0 YSize ]);
    xlabel('X(m)');
    ylabel('Y(m)');
    %title(['time',num2str(t)]);
    print('-dpdf',['ui',num2str(t),'.pdf']);
    %pause(deltaT/100);
    end
    Tidx=Tidx+1;
end
assignin('base','LocErr',LocErrVec);
assignin('base','RealTrackX',RealTrackX);
assignin('base','RealTrackY',RealTrackY);
assignin('base','EstTrackX',EstTrackX);
assignin('base','EstTrackY',EstTrackY);
assignin('base','AssignToID',AssignToID);
%Mation loop, the position of the agent
