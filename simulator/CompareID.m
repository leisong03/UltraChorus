function validpoint=CompareID(FinalX,FinalY,RealTrackX,RealTrackY,tint,idset)
close all;
hold on
TrackError=zeros(length(tint)*length(idset));
j=0;
for i=1:length(idset)
    id=idset(i);
    plot(FinalX(tint,id),FinalY(tint,id),'b-');
    plot(RealTrackX(tint,id),RealTrackY(tint,id),'r-');
    for k=1:length(tint);
	time=tint(k);
	if(FinalX(time,id)==inf || FinalY(time,id)==inf)
	   continue; 
	end
	j=j+1;
	TrackError(j)=norm([RealTrackX(time,id),RealTrackY(time,id)]-[FinalX(time,id),FinalY(time,id)]);
    end
end
hold off
assignin('base','TrackError',TrackError(1:j));
validpoint=j;

bins=[0:.1:5];
freq=zeros(length(bins),1);
p=zeros(length(bins),1);
hold on;
y=TrackError(1: validpoint);
for i=1:length(bins)
        p(i)=sum(y<bins(i));
	    freq(i)=freq(i)/length(y);
	end
	p=p./length(y);
	length(bins);


figure();
plot(bins,p,'r-');
xlabel('Tracking error(m)');
ylabel('CDF');
legend('0.2/m^2');
end
