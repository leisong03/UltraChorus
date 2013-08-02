function SimuEngine(XSize,YSize,NumBPS,radius,NumAgent,DepMode)
close all;
%XSize=100;
%YSize=100;
%NumBPS=100;
%radius=4;
%ThisUDG=UniDiskGraph(BMSPos);
OutputSeq=0;
OutputUI=0;
OutputCountingSeq=0;
BPSDeployment=GenBPSDeployment(NumBPS,'rand',radius,XSize,YSize);    %return the BPSDeployment structure
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
ChannelLive=subplot(1,2,1); %plot Live into left window
UDG=subplot(1,2,2); %plot UDG in right window

RealTrackX=zeros(length([0:deltaT:T]),NumAgent);
RealTrackY=zeros(length([0:deltaT:T]),NumAgent);
EstTrackX=zeros(length([0:deltaT:T]),NumAgent*2);
EstTrackY=zeros(length([0:deltaT:T]),NumAgent*2);
AssignToID=zeros(length([0:deltaT:T]),NumAgent*2);
RealInBoxSeq=zeros(length([0:deltaT:T]),1);
EstInBoxSeq=zeros(length([0:deltaT:T]),1);
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
    [DeltaX, DeltaY]=pol2cart(dir,AgentVel); 
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

    RealInBox=0;
    for i=1:size(AgentPos,1)
	if(AgentPos(i,1)>=20 && AgentPos(i,2)>=20 && AgentPos(i,1)<=80 && AgentPos(i,2)<=80)
	    RealInBox=RealInBox+1;
	end
	dist=[BPSDeployment.X,BPSDeployment.Y]-repmat(AgentPos(i,:),BPSDeployment.num,1);
	dist=sqrt(dist(:,1).^2+dist(:,2).^2);
	idx=find(dist<BPSDeployment.R); %find triggered BPS
	SensorReading(idx)=1; 
    end
    LastPos=AgentPos;
    RealTrackX(Tidx,:)=AgentPos(:,1)';
    RealTrackY(Tidx,:)=AgentPos(:,2)';
    %update the UDG
    TriggeredBPS=find(SensorReading==1);
    ActiveBPS.num=length(TriggeredBPS);
    ActiveBPS.XBound=XSize;
    ActiveBPS.YBound=YSize;
    ActiveBPS.X=BPSDeployment.X(TriggeredBPS);
    ActiveBPS.Y=BPSDeployment.Y(TriggeredBPS);
    ActiveBPS.R=BPSDeployment.R(TriggeredBPS);
    ThisUDG=UniDiskGraph(ActiveBPS);

    %add tracking algorithm here 
    [n CC]=graphconncomp(ThisUDG,'Directed',false);
    center=zeros(n,2);
    EstInBox=0;
    for i=1:n
	VinI=find(CC==i);
	center(i,1)=mean(ActiveBPS.X(VinI));
	center(i,2)=mean(ActiveBPS.Y(VinI));
	if(center(i,1)>=20 && center(i,2)>=20 && center(i,1)<=80 && center(i,2)<=80)
	    EstInBox=EstInBox+1;
	    minerr=inf;
	    for j=1:size(AgentPos,1)
		minerr=min(minerr,norm(AgentPos(j,:)-center(i,:)));
	    end
	    LocErrVec=[LocErrVec;minerr];
	end
    end
    for i=1:n
	EstTrackX(Tidx,i)=center(i,1);
	EstTrackY(Tidx,i)=center(i,2);
	mindist=inf;
	NearToID=0;
	for j=1:size(AgentPos,1);
	    if mindist>norm(AgentPos(j,:)-center(i,:))
		mindist=norm(AgentPos(j,:)-center(i,:));
		NearToID=j;
	    end
	end
	AssignToID(Tidx,i)=NearToID;
    end
    %tracking algorithm finished

    EstInBox=min(EstInBox,RealInBox);
    RealInBox;
    RealInBoxSeq(Tidx)=RealInBox;
    EstInBoxSeq(Tidx)=EstInBox;
    assignin('base','CountErr',RealInBoxSeq-EstInBoxSeq);
    if OutputCountingSeq==1
	if mod(Tidx,10)==0
	    NumSeq=figure();
	    axis([0 t 0 max([max(RealInBoxSeq),max(EstInBoxSeq)])]);
	    TimeIdx=linspace(0,t,Tidx);
	    plot(TimeIdx,RealInBoxSeq(1:Tidx),'r-',TimeIdx,EstInBoxSeq(1:Tidx),'b-');
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
    subplot(ChannelLive);
    text(XSize,YSize,['Time=',num2str(t)]);
    plot(OutlineX(1,:),OutlineY(1,:),'-');
    hold on;
    for i=2:BPSDeployment.num
	plot(OutlineX(i,:),OutlineY(i,:),'-');
    end

    for i=1:BPSDeployment.num
	if SensorReading(i)==1
	    fill(OutlineX(i,:),OutlineY(i,:),'red');
	end
    end
    plot(AgentPos(:,1),AgentPos(:,2),'kp');
    hold off;
    axis equal;
    axis ([0 XSize 0 YSize ]);
    xlabel('X(m)');
    ylabel('Y(m)');
    %title(['time',num2str(t)]);
    print('-dpdf',['ui',num2str(t),'.pdf']);
    subplot(UDG);
    TriggeredBMS=find(SensorReading>0);
    plot(ActiveBPS.X,ActiveBPS.Y,'d');
    hold on
    gplot(ThisUDG,[ActiveBPS.X,ActiveBPS.Y]);
    axis equal;
    axis ([0 XSize 0 YSize ]);
    hold off
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
